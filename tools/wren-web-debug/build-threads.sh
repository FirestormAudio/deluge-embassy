#!/usr/bin/env bash
# Task 3.2: build the threaded debug wasm (wasm32-wasip1-threads).
#
# Reuses the spike's recipe verbatim (task-3.2a-spike-report.md): the target
# spec already supplies the shared-memory/atomics linker args, and wren-core's
# build.rs (commit d36c2c4) compiles the C VM with matching wasm atomics for
# this target. -Zbuild-std rebuilds std for the threads target.
#
# We build the `dbg_threads` BIN (a command module with a real `_start` that
# bootstraps the main thread's TLS — see src/bin/dbg_threads.rs) rather than the
# cdylib. A command module exports only `_start`/`memory` by default, so we
# force-export the `dbg_*` control functions the host calls after init.
set -euo pipefail

: "${WASI_SYSROOT:=/home/kate/.local/wasi-sysroot-25.0}"
export WASI_SYSROOT

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"

rustup component add rust-src --toolchain nightly >/dev/null 2>&1 || true

export RUSTFLAGS="-C target-feature=+atomics,+bulk-memory,+mutable-globals \
-C link-arg=--export=dbg_alloc \
-C link-arg=--export=dbg_sab_ptr \
-C link-arg=--export=dbg_launch"

cargo +nightly build \
  -Zbuild-std=std,panic_abort \
  --target wasm32-wasip1-threads \
  --release \
  --features threads \
  --bin dbg_threads

echo "built: $HERE/target/wasm32-wasip1-threads/release/dbg_threads.wasm"
