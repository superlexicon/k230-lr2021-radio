#!/usr/bin/env python3
"""sim_device.py — a software stand-in for the LR2021 radio device.

Implements the host↔device byte-tag contract (docs/protocol.md) over a PTY
so the REAL consensus node (superlexicon's radio.rs) can be integration-
tested on x86 with no hardware:

    python3 host/sim_device.py --symlink /tmp/lr2021-sim
    # then point lr2021.serial_port at /tmp/lr2021-sim and start the node

Behaviors implemented (mirrors the K230 service / nRF firmware):
  - 'X'           → bare 'G' byte, then a fresh READY (port survives; the
                    sim tolerates the host closing/reopening the slave)
  - 'C' (15/17)   → ack 'G' + max_payload; 0xFFFFFFFF scan magic triggers a
                    fake sweep (D events 50-53 at dwell cadence)
  - 'W'           → ack 'G'
  - 'S'           → 68-byte stats frame (plus a 10 s periodic cadence)
  - 'Q'           → fake sweep events (no ack, per contract)
  - 'T' frames    → accepted into a 2-deep queue, EXACTLY ONE terminal per
                    frame ('G' after a simulated air-time), and the payload
                    is echoed back as [FB]-chunked 'R' frames so the host's
                    burst reassembly is exercised
  - periodic fake RX: [FB]-chunked SLRF-prefixed payloads to exercise
                    reassembly + the engine's dispatch path

This is a test rig, NOT the production device: the K230 service (service/)
is the real implementation.
"""

import argparse
import os
import pty
import select
import struct
import sys
import threading
import time

MAX_PAYLOAD = 12288
PACKET_PAYLOAD = 252
CHUNK = PACKET_PAYLOAD - 16
FB_MAGIC = 0x4642

# parser states
TAG, TXMAGIC, SETWINDOW, LENLO, LENHI, CONFIG, PAYLOAD, SCANCOUNT, SCANBODY = (
    "TAG", "TXMAGIC", "SETWINDOW", "LENLO", "LENHI", "CONFIG", "PAYLOAD",
    "SCANCOUNT", "SCANBODY")


def crc32(data: bytes) -> int:
    if not data:
        return 0
    crc = 0xFFFFFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1))
    return crc ^ 0xFFFFFFFF


def now_ms() -> int:
    return int(time.monotonic() * 1000)


class SimDevice:
    def __init__(self, args):
        self.args = args
        self.master_fd, self.slave_fd = pty.openpty()
        slave_name = os.ttyname(self.slave_fd)
        if args.symlink:
            try:
                os.unlink(args.symlink)
            except FileNotFoundError:
                pass
            os.symlink(slave_name, args.symlink)
            print(f"[sim] node should open: {args.symlink} (→ {slave_name})")
        else:
            print(f"[sim] node should open: {slave_name}")
        self.stats = dict.fromkeys(
            ("bursts_tx bursts_rx packets_tx packets_rx crc_errors "
             "host_tx_dropped uart_ring_drops staging_overwrites "
             "burst_id_conflicts hw_rx hw_crc hw_len hw_ok killed").split(), 0)
        self.burst_id = 1
        self.tx_q = []
        self.tx_mu = threading.Lock()
        self.mode = "rx"          # rx | scan
        self.freq = args.freq
        self.running = True

    # ---------- framing ----------
    def send(self, data: bytes):
        try:
            os.write(self.master_fd, data)
        except OSError:
            pass  # slave closed — host is mid-reset; keep running

    def frame(self, tag: bytes, payload: bytes = b""):
        self.send(tag + struct.pack("<H", len(payload)) + payload)

    def ready(self, max_payload: int = PACKET_PAYLOAD):
        # 'G' is NOT length-prefixed: 'G' + max_payload:u16 LE directly
        self.send(b"G" + struct.pack("<H", max_payload))

    def error(self, code: int, msg: str):
        m = msg.encode()[:64]
        self.frame(b"E", bytes([code, len(m)]) + m)

    def diag(self, event: int, val: int):
        self.frame(b"D", struct.pack("<BIB", event, val, 0))

    def stats_frame(self):
        s = self.stats
        body = struct.pack(
            "<6I 5B 3x 9I",
            s["bursts_tx"], s["bursts_rx"], s["packets_tx"], s["packets_rx"],
            s["crc_errors"], s["host_tx_dropped"],
            0 if self.mode == "rx" else 2,      # burst_mode
            0,                                   # zero25
            1 if self.tx_q else 0,               # pending_tx_valid
            0,                                   # restart_rx_pending
            0,                                   # zero28
            now_ms(),
            s["uart_ring_drops"], s["staging_overwrites"],
            s["burst_id_conflicts"], s["hw_rx"], s["hw_crc"],
            s["hw_len"], s["hw_ok"], s["killed"])
        self.frame(b"S", body)

    def rx_packet(self, air_payload: bytes, rssi: int = -40):
        # 'R' length covers ONLY the payload; the RSSI i16 follows after it.
        self.send(b"R" + struct.pack("<H", len(air_payload)) + air_payload
                  + struct.pack("<h", rssi))

    def send_burst(self, payload: bytes):
        total = (len(payload) + CHUNK - 1) // CHUNK
        self.burst_id = (self.burst_id % 65535) + 1
        self.stats["bursts_rx"] += 1
        for idx in range(total):
            hdr = struct.pack("<HHHHII", FB_MAGIC, self.burst_id, idx, total,
                              len(payload), crc32(payload))
            pkt = hdr + payload[idx * CHUNK:(idx + 1) * CHUNK]
            pkt += b"\x00" * (PACKET_PAYLOAD - len(pkt))
            self.rx_packet(pkt)
            self.stats["packets_rx"] += 1

    # ---------- fake sweeps ----------
    def run_fake_sweep(self, start_hz, step_hz, count, dwell_ms):
        if getattr(self, "sweep_active", False):
            return
        self.sweep_active = True

        def sweep():
            self.mode = "scan"
            for i in range(count):
                time.sleep(max(dwell_ms, 20) / 1000.0)
                freq = start_hz + i * (step_hz or 500000)
                frames = 40 if freq == 916500000 else 0
                self.diag(50, freq)
                self.diag(51, frames)
                self.diag(52, frames)
                self.diag(53, 128 - 35)
            self.mode = "rx"
            self.sweep_active = False
            print("[sim] fake sweep complete")
        threading.Thread(target=sweep, daemon=True).start()

    # ---------- RX generator ----------
    def rx_generator(self):
        payload = bytes.fromhex("534C5246") + b"\x00" * (300 - 4)  # SLRF…
        while self.running:
            if self.args.rx_burst_every > 0 and self.mode == "rx":
                self.send_burst(payload)
            time.sleep(self.args.rx_burst_every)

    # ---------- TX queue ----------
    def tx_worker(self):
        while self.running:
            item = None
            with self.tx_mu:
                if self.tx_q:
                    item = self.tx_q.pop(0)
            if item is None:
                time.sleep(0.005)
                continue
            payload = item
            air_ms = max(30, len(payload) * 46 // 1000)
            time.sleep(air_ms / 1000.0)
            self.stats["bursts_tx"] += 1
            self.ready()  # 'G' terminal = confirmed aired
            if self.args.echo_tx:
                self.send_burst(payload)

    # ---------- command handlers ----------
    def handle_config(self, body: bytes):
        freq = struct.unpack("<I", body[0:4])[0]
        if freq == 0xFFFFFFFF:
            start = struct.unpack("<I", body[10:14])[0]
            step = struct.unpack("<I", body[4:8])[0]
            count = body[8] or 16
            dwell = body[9] * 100
            self.run_fake_sweep(start, step, min(count, 16), dwell)
        else:
            self.freq = freq
        self.ready()

    def handle_tx(self, payload: bytes):
        with self.tx_mu:
            if len(self.tx_q) >= 2:
                self.error(0x04, "tx queue full")
                self.stats["host_tx_dropped"] += 1
                return
            self.tx_q.append(payload)
        self.diag(47, len(payload))

    # ---------- parser ----------
    def parse(self):
        fd = self.master_fd
        state = TAG
        tag = 0
        frame_len = 0
        buf = b""
        cfg_body = bytearray()
        win_body = bytearray()
        qbuf = b""
        scan_count = scan_have = 0
        last_arrival = now_ms()

        while self.running:
            r, _, _ = select.select([fd], [], [], 0.05)
            now = now_ms()

            if state != TAG and now - last_arrival > 100:
                state = TAG  # idle resync
                buf = b""

            if not r:
                continue
            try:
                b = os.read(fd, 1)[0]
            except OSError:
                # slave closed (host cold-reset path) — wait for reopen
                time.sleep(0.05)
                last_arrival = now_ms()
                state = TAG
                continue
            last_arrival = now_ms()

            if state == TAG:
                tag = b
                if b == 0x54:
                    state = TXMAGIC
                elif b == 0x43:
                    state = LENLO
                elif b == 0x57:
                    state = SETWINDOW
                elif b == 0x53:
                    self.stats_frame()
                elif b == 0x51:
                    state = SCANCOUNT
                elif b == 0x58:
                    self.send(b"G")
                    time.sleep(0.2)
                    self.ready()
            elif state == TXMAGIC:
                state = LENLO if b == 0x58 else TAG
            elif state == SETWINDOW:
                win_body.append(b)
                if len(win_body) == 8:
                    win_body.clear()
                    self.ready()
                    state = TAG
            elif state == LENLO:
                frame_len = b
                state = LENHI
            elif state == LENHI:
                frame_len |= b << 8
                if tag == 0x43:
                    if frame_len not in (15, 17):
                        self.error(0x03, "bad len")
                        state = TAG
                    else:
                        cfg_body.clear()
                        state = CONFIG
                elif tag == 0x54:
                    if frame_len == 0 or frame_len > MAX_PAYLOAD:
                        self.error(0x02, "bad tx len")
                        state = TAG
                    else:
                        buf = b""
                        state = PAYLOAD
                else:
                    state = TAG
            elif state == CONFIG:
                cfg_body.append(b)
                if len(cfg_body) == frame_len:
                    self.handle_config(bytes(cfg_body))
                    state = TAG
            elif state == PAYLOAD:
                buf += bytes([b])
                if len(buf) == frame_len:
                    self.handle_tx(buf)
                    buf = b""
                    state = TAG
            elif state == SCANCOUNT:
                scan_count = b
                scan_have = 0
                qbuf = b""
                state = SCANBODY if 1 <= scan_count <= 16 else TAG
            elif state == SCANBODY:
                qbuf += bytes([b])
                scan_have += 1
                need = 4 + scan_count * 4
                if scan_have == need:
                    dwell = struct.unpack("<I", qbuf[:4])[0]
                    freqs = struct.unpack(f"<{scan_count}I", qbuf[4:need])
                    self.run_fake_sweep(freqs[0], 0, scan_count, dwell)
                    state = TAG

    def run(self):
        threading.Thread(target=self.tx_worker, daemon=True).start()
        threading.Thread(target=self.rx_generator, daemon=True).start()
        last_stats = now_ms()
        while self.running:
            r, _, _ = select.select([self.master_fd], [], [], 0.5)
            if r:
                self.parse()
            if now_ms() - last_stats >= 10000:
                last_stats = now_ms()
                self.stats_frame()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--symlink", default=None,
                    help="stable path to symlink to the PTY slave")
    ap.add_argument("--freq", type=lambda x: int(x, 0), default=916500000)
    ap.add_argument("--rx-burst-every", type=int, default=15,
                    help="seconds between fake [FB] RX bursts (0 = off)")
    ap.add_argument("--echo-tx", action="store_true", default=True)
    args = ap.parse_args()
    SimDevice(args).run()
