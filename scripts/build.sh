#!/usr/bin/env bash
# Cross-compile the radio service from x86.
# Env / layout expectations:
#   LILYGO_SDK (default: ~/Projects/T-Display-K230_canmv_rt) — the SDK checkout
#   The Canaan musl toolchain at:
#     $LILYGO_SDK/canmv_k230/toolchain/riscv64-linux-musleabi_for_x86_64-pc-linux-gnu/bin
#   (obtained via the canmv-k230 `repo` manifest flow — see the SDK's BUILD.md;
#    any riscv64-linux-musleabi gcc works if it provides the RT-SMART-compatible
#    sysroot headers the mpp.mk expects)
set -euo pipefail

SDK="${LILYGO_SDK:-$HOME/Projects/T-Display-K230_canmv_rt}"
MPP="$SDK/canmv_k230/src/rtsmart/mpp"
REPO="$(cd "$(dirname "$0")/.." && pwd)"

if [ ! -d "$MPP" ]; then
  echo "ERROR: LILYGO SDK not found at $SDK" >&2
  exit 1
fi

# shellcheck disable=SC1091
source "$MPP/build_env.sh"

make -C "$REPO/service" LILYGO_SDK="$SDK" "$@"
echo "→ $REPO/build/lora_radio.elf"
