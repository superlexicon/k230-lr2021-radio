#!/usr/bin/env bash
# Copy the built ELF to the device's SD card (mount point as arg 1).
# RT-SMART auto-runs app.elf from the SD card at boot.
set -euo pipefail
MOUNT="${1:-/run/media/$USER/YES}"
ELF="$(dirname "$0")/../build/lora_radio.elf"
cp -f "$ELF" "$MOUNT/app.elf"
sync
echo "deployed $ELF → $MOUNT/app.elf"
