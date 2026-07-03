#!/usr/bin/env bash
# Task 3.3: build the threaded debug wasm and place it in public/.
#
# Delegates to tools/wren-web-debug/build-threads.sh (nightly + -Zbuild-std,
# wasm32-wasip1-threads) then copies the artifact to
# public/wren-debug-threads.wasm, which the DebugController fetches at
# ${import.meta.env.BASE_URL}wren-debug-threads.wasm. Vite serves public/ under
# the COOP/COEP headers vite.config.ts already sets (cross-origin isolation is
# required for SharedArrayBuffer).
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
APP="$(cd "$HERE/.." && pwd)"
DEBUG="$(cd "$APP/../../wren-web-debug" && pwd)"

bash "$DEBUG/build-threads.sh"
cp "$DEBUG/target/wasm32-wasip1-threads/release/dbg_threads.wasm" "$APP/public/wren-debug-threads.wasm"
echo "copied -> $APP/public/wren-debug-threads.wasm"
