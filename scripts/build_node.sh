#!/usr/bin/env bash
# Cross-compile the superlexicon consensus node for the K230's Linux core
# (riscv64, glibc) from this x86 machine. Proven 2026-09-12.
#
# Requires: rustup target riscv64gc-unknown-linux-gnu, the Xuantie-900
# glibc toolchain (toolchain/Xuantie-900-gcc-linux-5.10.4-glibc-x86_64-V2.6.0,
# from kendryte-download), lld on the host, and cmake.
set -euo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"
SDK="${LILYGO_SDK:-$HOME/Projects/T-Display-K230_canmv_rt}"
TC="$SDK/canmv_k230/toolchain/Xuantie-900-gcc-linux-5.10.4-glibc-x86_64-V2.6.0"
SHIM="$REPO/toolchain-shim"
NODE="${2:-$HOME/Projects/superlexicon}"

[ -x "$TC/bin/riscv64-unknown-linux-gnu-gcc" ] || { echo "Xuantie toolchain missing at $TC" >&2; exit 1; }
[ -x "$SHIM/ld" ] || { mkdir -p "$SHIM"; ln -sf /usr/bin/ld.lld "$SHIM/ld"; ln -sf /usr/bin/ld.lld "$SHIM/ld.lld"; }

# Ensure the rust std for the target
rustup target add riscv64gc-unknown-linux-gnu 2>/dev/null || true

cd "$NODE"
env -u RUSTC -u RUSTC_WRAPPER -u RUSTC_WORKSPACE_WRAPPER \
  PATH="$SHIM:$PATH" \
  CARGO_TARGET_RISCV64GC_UNKNOWN_LINUX_GNU_LINKER="$TC/bin/riscv64-unknown-linux-gnu-gcc" \
  CC_riscv64gc_unknown_linux-gnu="$TC/bin/riscv64-unknown-linux-gnu-gcc" \
  CXX_riscv64gc_unknown_linux-gnu="$TC/bin/riscv64-unknown-linux-gnu-g++" \
  AR_riscv64gc_unknown_linux-gnu="$TC/bin/riscv64-unknown-linux-gnu-ar" \
  TARGET_CC="$TC/bin/riscv64-unknown-linux-gnu-gcc" \
  TARGET_AR="$TC/bin/riscv64-unknown-linux-gnu-ar" \
  RUSTFLAGS="-C link-arg=--sysroot=$TC/sysroot -C link-arg=-march=rv64gc -C link-arg=-mabi=lp64d -C link-arg=-fuse-ld=lld -C link-arg=-B$SHIM" \
  cargo build --release --target riscv64gc-unknown-linux-gnu

echo "→ $NODE/target/riscv64gc-unknown-linux-gnu/release/superlexicon-app"
echo "  deploy to the K230 Linux side (scp to the device, chmod +x)"
