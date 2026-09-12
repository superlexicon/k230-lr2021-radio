# Host ↔ Radio Service Wire Contract (byte-exact)

This is the contract between the consensus node (host, Linux core) and this
radio service (RT-SMART core). It is **byte-identical** to the nRF54L15
firmware contract (`lr2021-flrc-firmware/src/main.c`), so the host's serial
implementation (`superlexicon/src/lr2021_transport/radio.rs`) works against
either device unchanged. All multi-byte integers are **little-endian**.

Transport: a serial stream (cross-core UART or USB-CDC). The host opens it at
the configured baud, asserts DTR/RTS, sleeps 500 ms, purges stale bytes, sends
`'X'`, waits for the device to re-establish (≤15 s), then handshakes.

## 1. Host → device commands

The device runs a byte-at-a-time tag state machine (states:
`TAG → TX_MAGIC | SET_WINDOW | LEN_LO | SCAN_COUNT`, `LEN_LO → LEN_HI`,
`LEN_HI → CONFIG | PAYLOAD`, `SCAN_COUNT → SCAN_BODY`).

**Resync rules (must match exactly):**
- If state ≠ TAG and **100 ms** elapsed since the last byte *arrival*, the
  partial frame is killed: state returns to TAG; if the state was
  PAYLOAD / CONFIG / SET_WINDOW / LEN_HI, `parser_frames_killed++` (stats
  offset 64) and `skip_remain = expected − have` bytes are silently discarded
  before tag scanning resumes. `expected` = 15 for CONFIG (always 15, even for
  a 17-byte v2 body — historical quirk), 8 for SET_WINDOW, else `frame_len`.
- CONFIG length ∉ {15, 17} → `E` code 0x03 "bad len".
- TX length == 0 or > 12288 → `E` code 0x02 "bad tx len".

### 'T' — transmit (2-byte magic)
```
54 58 | len:u16 LE | payload[len]        (len 1..12288)
```
Second byte must be `'X'` or the frame is dropped silently (resume at TAG).
No ack at accept time (only `D` event 47). **Exactly one terminal** is sent
later per accepted frame: `'G'` (confirmed aired) or `'E'` 0x05/0x06.
If both queue slots are occupied → `E` 0x04 "tx queue full" and the frame is
NOT taken.

### 'C' — config
```
43 | len:u16 LE (=15 or 17) | body[len]   → always acked with 'G'
```
15-byte body:

| offset | size | field |
|---|---|---|
| 0-3 | u32 | frequency_hz — **0xFFFFFFFF = scan magic** |
| 4-7 | u32 | bandwidth_hz (stored; in scan mode = step Hz) |
| 8 | u8 | coding_rate (stored; air CR is fixed 1/1) |
| 9 | i8 | tx_power_dbm |
| 10-13 | u32 | sync_word (stored; air sync fixed `90 56 34 12`) |
| 14 | u8 | role: 0=BOTH, 1=TX_ONLY, 2=RX_ONLY (gates TX trigger only) |

17-byte v2 additions: byte 15 = rate selector (0=1.3 Mbps, 1=2.6 Mbps), byte
16 = packet-payload selector (0=252 B, 1=511 B); values >1 keep current. The
K230 service accepts v2 but may fix 252 B in v1.

**Scan magic 0xFFFFFFFF** — body reinterpreted: bw = step Hz, cr = channel
count (0 or >16 → 16), pwr = dwell ms÷100, sync = start Hz. Grid =
`start + i*step`. Behaviors to replicate: first dwell listens on the
pre-scan frequency while reporting `start` Hz as channel 0; no
return-to-home (the host retunes afterwards); TX is suppressed during scan.

### 'W' — set window
```
57 | slot_offset_ms:u32 LE | slot_period_ms:u32 LE      (8 bytes) → acked 'G'
```
Stored but unused — the device transmits ASAP (the host schedules).

### 'S' — stats request (single byte `53`)
Responds with the `S` stats frame.

### 'Q' — scan (alternate entry)
```
51 | count:u8 | dwell_ms:u32 LE | freq_hz:u32 LE × count
```
count must be 1..16 (else silently ignored); dwell clamped ≥50 ms. No ack —
results arrive as `D` events 50-53.

### 'X' — reset (single byte `58`)
Write a **single bare `'G'` byte (no payload)**, then cold-restart the radio
service state (on the nRF this reboots the MCU; the host expects the port to
drop/re-establish within 15 s).

## 2. Device → host frames

- **'G' READY**: `47 | max_payload:u16 LE` — value is the **on-air packet
  size** (252, or 511 after a v2 CONFIG with payload_sel=1), not the burst
  cap. Sent: after radio init at boot, after every TX completion, as the ack
  for `C` and `W`.
- **'R' RX**: `52 | len:u16 LE | payload[len] | rssi:i16 LE` — one packet per
  frame, len ≤ 12288 (host guard rejects >24600). The K230 service forwards
  **raw air packets including their `[FB]` headers**; the host reassembles.
- **'E' ERROR**: `45 | code:u8 | msg_len:u8 | msg[msg_len]` (1-byte length).
  Codes: 0x01 radio init, 0x02 payload too big, 0x03 bad config,
  0x04 TX_BUSY (not taken), 0x05 TX_FAILED (uncertain), 0x06 TX_DROPPED.
- **'S' STATS**: `53 | 68:u16 LE | body[68]` — u32 LE offsets:
  0 bursts_tx, 4 bursts_rx, 8 packets_tx, 12 packets_rx, 16 crc_errors,
  20 host_tx_dropped, 24 burst_mode u8 (0=RX/1=TX_ABORTING/2=TX),
  26 pending_tx_valid u8, 27 restart_rx_pending u8, 32 uptime ms,
  36 uart ring drops, 40 staging overwrites, 44 burst_id_conflicts,
  48 hw received_packets, 52 hw crc_errors, 56 hw length_errors,
  60 hw crc_ok, 64 parser_frames_killed. Emitted every **10 s** and on
  request — the host kills the session after **60 s of silence**.
- **'D' FW-DIAG**: exactly `44 | 06 00 | event_id:u8 | val:u32 LE` (9 bytes).
  Events: 30 stall (val=ms, then reset), 32/33 crashlog, 41 TX-never-aired
  chip fault bits, 42 burst truncated, 43 TX_DONE ms, 44 RX re-arm
  turnaround ms, 45 TX ring depth, 46 TX launch, 47 TX frame accepted
  (val=len), 48 TX trigger, **50-53 scan results** (50 freq Hz, 51 frames,
  52 consensus frames — payload starting `53 4C 52 46` "SLRF" or `42 46` "BF"
  chunk —, 53 best RSSI + 128).

## 3. On-air burst protocol (`[FB]`)

16-byte packet header, packed, all LE:

| offset | size | field |
|---|---|---|
| 0 | u16 | magic 0x4642 (on-air bytes `42 46`) |
| 2 | u16 | burst_id |
| 4 | u16 | packet_idx (0-based) |
| 6 | u16 | total_packets |
| 8 | u32 | total_payload_len |
| 12 | u32 | payload_crc32 (over the whole reassembled payload) |

- Max total payload **12288 B**; packet payload 252 B (chunk = 236) or 511 B
  (chunk = 495); ≤53 packets per burst; every packet padded to the full
  packet size with zeros; chunk i sits at payload offset `i × chunk`.
- RX validity: total_packets 1..=53, total_len 1..=12288, idx < total.
- CRC32: IEEE 802.3 reflected (init 0xFFFFFFFF, poly 0xEDB88320 shifted
  right per LSB, final inversion; returns 0 for empty input).
- FLRC air parameters (fixed): sync word bytes `90 56 34 12`, coding rate 1/1,
  2.6 Mbps raw, min inter-frame gap 2000 µs.
- Discovery payloads begin `SLMT`/`TMLS`; consensus payloads begin
  `SLRF 01 01 5F 76 [msg_type]` (full magic `53 4C 52 46 01 01 5F 76`).

## 4. Host-side expectations the service must satisfy

1. Port opens at any baud (host-configured); tolerate DTR/RTS asserted.
2. `'X'` on every session open: reset radio state; if the transport is a real
   UART the port stays open — simply re-send `'G'` READY when ready (≤15 s).
3. Handshake: accept `'C'` re-sent every ~1 s; answer `'G'` within 15 s.
4. TX: 2-deep queue, one terminal per accepted frame, or pipeline credits
   leak and the host's 15 s TX watchdog cold-resets the session.
5. `'R'` frames ≤24600 B; forward raw packets (host reassembles `[FB]`).
6. At least one frame every <60 s (the 10 s stats covers this).
7. Scan events 50-53 only if roaming/scan is used (it is — the host's roamer
   uses them).

## 5. Cross-core transport

Primary: a K230 UART routed between the RT-SMART core and Linux (appears on
Linux as `/dev/ttyS*`; host config `lr2021.serial_port` points at it).
Fallbacks: USB-CDC from RT-SMART; `/sharefs` (last resort). See
`docs/bringup.md` for the measured choice.
