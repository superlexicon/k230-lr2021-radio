#!/usr/bin/env bash
# Drop our app into the LILYGO tree the same way built-in apps are laid out
# (only needed if building IN-tree rather than out-of-tree).
set -euo pipefail
SDK="${LILYGO_SDK:-$HOME/Projects/T-Display-K230_canmv_rt}"
APPS="$SDK/canmv_k230/src/rtsmart/mpp/userapps/sample/sample_display/ui_brookesia/src/app_examples"
REPO="$(cd "$(dirname "$0")/.." && pwd)"
mkdir -p "$APPS/lora_radio"
cp -r "$REPO/app"/* "$APPS/lora_radio/"
echo "installed → $APPS/lora_radio (register it in ui_brookesia.mk if needed)"
