# Bring-up log (Phase 0-2 measurements)

## Phase 0 — build bring-up
- [x] LILYGO SDK cloned (pinned abb07090), Canaan musl toolchain
      (riscv64-unknown-linux-musl-gcc 12.0.1) installed at
      canmv_k230/toolchain/ (from kendryte-download toolchain tarball
      riscv64-unknown-linux-musl-rv64imafdcv-lp64d-20230420)
- [x] Out-of-tree build of service/ produces lora_radio.elf
      (static rv64gc ELF, 916 KB unstripped) — 2026-09-12
      Notes: the kernel rt-smart.mk's include paths don't resolve in the
      checkout layout (cconfig.h lives at mpp/kernel/include; rtconfig.h is
      firmware-build-generated and NOT needed by userspace LWP apps — the
      vendored k230Hal.cpp was patched to drop kernel includes).
- [ ] app.elf boots on the RT-SMART core (serial console shows [main]/[radio] lines)

## Phase 1 — radio spike (hardware gate)
- [ ] GPIO44 power → beginFLRC() → chip version readable over SPI
- [ ] FLRC air-format interop vs nRF node: sync word byte order
      (RadioLib setFLRCSyncWord(0x12345690) vs on-air 90 56 34 12), preamble,
      CRC config — tune setPacketParamsFLRC/setFLRCSyncWord until a raw packet
      from a bench nRF decodes
- [ ] Burst capture: stream 50-packet bursts from an nRF node; count captured
      packets. GATE: ≥99% (polled-IRQ turnaround budget)
- [ ] Cross-core channel identified: which K230 UART reaches Linux as
      /dev/ttyS* (candidates UART1-4; Canaan UART driver, 128 KB buffers)

## Phase 2 — protocol server
- [ ] Host connects (config lr2021.serial_port = /dev/ttyS*): READY handshake
      within 15 s, CONFIG re-send every 1 s answered with 'G'
- [ ] Roamer run: scan sweeps emit D 50-53, node tunes and records world map
- [ ] 10 s 'S' stats cadence seen by the host (no 60 s session kills)
