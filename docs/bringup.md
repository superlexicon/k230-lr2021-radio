# Bring-up log (Phase 0-2 measurements)

## Phase 0 — build bring-up
- [ ] LILYGO SDK cloned (pinned abb07090), musl toolchain in place
- [ ] Out-of-tree build of service/ produces lora_radio.elf
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
