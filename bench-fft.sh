#!/usr/bin/env bash
# bench-fft.sh — Run deluge-fft benchmarks in all three configurations.
#
# Usage:
#   ./bench-fft.sh [native|native-cpu|qemu|all]
#
# Configurations:
#   native      — x86-64, generic codegen (baseline; no AVX etc.)
#   native-cpu  — x86-64, -C target-cpu=native (enables AVX2/AVX-512)
#   qemu        — Cortex-A9 NEON via qemu-arm (representative of hardware)
#   all         — run all three in sequence (default)
#
# Results land in:
#   target/criterion/          (native and native-cpu share this dir)
#   target/criterion-qemu/     (qemu run uses CRITERION_BASELINE=qemu)
#
# Prerequisites:
#   rustup target add armv7-unknown-linux-gnueabihf
#   apt install qemu-user gcc-arm-linux-gnueabihf

set -euo pipefail
MANIFEST="$(dirname "$0")/deluge-fft/Cargo.toml"
MODE="${1:-all}"

run_native() {
  echo "=== Native x86-64 (generic) ==="
  cargo bench \
    --manifest-path "$MANIFEST" \
    --target x86_64-unknown-linux-gnu \
    -- --output-format bencher 2>&1 | tee /tmp/bench-native.txt
}

run_native_cpu() {
  echo "=== Native x86-64 (target-cpu=native) ==="
  RUSTFLAGS="-C target-cpu=native" \
  cargo bench \
    --manifest-path "$MANIFEST" \
    --target x86_64-unknown-linux-gnu \
    -- --output-format bencher 2>&1 | tee /tmp/bench-native-cpu.txt
}

run_qemu() {
  echo "=== QEMU Cortex-A9 NEON (armv7-unknown-linux-gnueabihf) ==="
  cargo bench \
    --manifest-path "$MANIFEST" \
    --target armv7-unknown-linux-gnueabihf \
    -- --output-format bencher 2>&1 | tee /tmp/bench-qemu.txt
}

case "$MODE" in
  native)     run_native ;;
  native-cpu) run_native_cpu ;;
  qemu)       run_qemu ;;
  all)
    run_native
    run_native_cpu
    run_qemu
    echo ""
    echo "=== Summary ==="
    echo "--- native ---"
    grep "^test " /tmp/bench-native.txt || true
    echo "--- native-cpu ---"
    grep "^test " /tmp/bench-native-cpu.txt || true
    echo "--- qemu cortex-a9 ---"
    grep "^test " /tmp/bench-qemu.txt || true
    ;;
  *)
    echo "Unknown mode: $MODE. Use native|native-cpu|qemu|all" >&2
    exit 1
    ;;
esac
