# k230-lr2021-radio

Headless **radio driver service** for the LILYGO T-Display K230 (LR2021 variant).
Runs on the K230's **dedicated RT-SMART core (CPU0)** and exposes the SuperLexicon
byte-tag radio protocol over a cross-core channel, so the consensus node running on
the Linux core (CPU1) connects to it as just another serial port.

**This repo contains no UI code.** The device UI lives in `k230-device-ui`;
the consensus node lives in `superlexicon`. One concern per repo.

## Architecture

```
┌─────────────────────────── K230 SoC ───────────────────────────┐
│ CPU1 — Linux (CanMV)              │ CPU0 — RT-SMART (dedicated)│
│                                   │                            │
│  superlexicon node (Rust)  ───────┼──▶  this service (C++)     │
│  + k230-device-ui (LVGL)          │     RadioLib LR20xx        │
│         ▲ RPC localhost           │     byte-tag serial server │
│         └─────────────────────────┘     │                      │
└─────────────────────────────────────────┼──────────────────────┘
                                          ▼ SPI
                                    Semtech LR2021
```

- **CPU0 is reserved for the radio.** Nothing else runs there — UI, DB and
  network load on CPU1 physically cannot affect radio timing. This property
  is what makes validator operation on this device sound.
- The on-air protocol is **byte-identical** to the nRF54L15 nodes
  (`[FB]` burst format, `SLRF` envelopes) — peers cannot tell the hardware apart.

## Status / phases

| Phase | Scope | Gate |
|---|---|---|
| 0 | Repo, SDK/toolchain, out-of-tree build | `app.elf` builds and boots on RT-SMART |
| 1 | Radio spike (SPI probe, FLRC vs bench node) | ≥99% burst capture, polled-IRQ worker |
| 2 | Byte-protocol server (full contract, see `docs/protocol.md`) | node connects and roams |
| 3 | Node on K230 Linux (riscv64 superlexicon) | roamer end-to-end |
| 4 | (in `k230-device-ui`) panel + dashboard | — |
| 5 | Validator promotion | soak vs nRF-node baseline |
| 6 | RTToF ranging | — |

## Building

Requirements: the LILYGO SDK tree cloned as a sibling
(`git clone --depth 1 https://github.com/Xinyuan-LilyGO/T-Display-K230_canmv_rt`,
pinned upstream commit `abb07090ad8a666ed7a5e097b3c714b918731645`), and the
Canaan musl toolchain `riscv64-linux-musleabi_for_x86_64-pc-linux-gnu`
(see `scripts/build.sh` for the expected layout). Everything cross-compiles
from x86.

```sh
scripts/build.sh    # → build/lora_radio.elf
scripts/deploy.sh   # → copy to SD card as app.elf (auto-runs on RT-SMART)
```

## Documentation

- `docs/protocol.md` — the byte-exact host↔device wire contract this
  service implements (mirrors the nRF54L15 firmware contract).
- `docs/bringup.md` — SPI/IRQ bring-up measurements and the cross-core
  channel findings.

## No-hardware integration test (PTY simulator)

`host/sim_device.py` implements this repo's contract in Python over a PTY —
the real node connects to it as its serial port, exercising the host's
parser, handshake, stats cadence, scan flow and `[FB]` reassembly on x86:

```sh
python3 host/sim_device.py --symlink /tmp/lr2021-sim &
cd ../superlexicon
# sim-config.json: lr2021.serial_port=/tmp/lr2021-sim, reset_on_open=false,
# tx_chunk_gap_ms=0, fresh data dir, roamer (non-validator)
./target/release/superlexicon-app --config sim-config.json
```

Pass criteria (bench 2026-09-12): READY handshake at 252 B, scan sweep
command → D 50-53 → best-channel tune, zero session teardowns past the
60 s silence timeout, and `FLRC burst reassembled ... CRC ok` for the
sim's padded `[FB]` bursts (which found a real host bug — see the
superlexicon commit "padded-burst reassembly").
