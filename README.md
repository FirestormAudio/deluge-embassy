# deluge-embassy

Demo Rust firmware for the [Synthstrom Deluge], built on
[Embassy] and targeting the onboard Renesas RZ/A1L (ARM Cortex-A9,
R7S721001).

---

## Repository layout

| Crate | Description |
|-------|-------------|
| [`demo-firmware`](demo-firmware/) | Top-level firmware binary — Embassy executor, USB stack, audio, task orchestration |
| [`rza1l-hal`](rza1l-hal/) | Register-level HAL for the RZ/A1L SoC (MMU, caches, GIC, timers, DMA, RSPI, SSI, SCUX, SDHI, …) |
| [`deluge-bsp`](deluge-bsp/) | Board support package — SDRAM, audio codec, OLED, PIC co-processor, CV/gate, MIDI, SD card, USB |
| [`deluge-fft`](deluge-fft/) | `no_std` FFT library with RZ/A1L-tuned radix-4/8 paths and a real-FFT spectrum analyser |

---

## Target

| Property      | Value |
|---------------|-------|
| CPU           | ARM Cortex-A9, ARMv7-A, 400 MHz |
| SoC           | Renesas RZ/A1L (R7S721001) |
| RAM           | 3 MB on-chip SRAM + 64 MB SDRAM |
| Rust target   | `armv7a-none-eabihf` |
| Rust channel  | nightly (see [`rust-toolchain.toml`](rust-toolchain.toml)) |
| Async runtime | [Embassy] |

---

## Prerequisites

- Rust nightly with the `armv7a-none-eabihf` target and `llvm-tools-preview`
  (managed automatically by `rust-toolchain.toml`):
  ```sh
  rustup show   # installs the toolchain on first run
  ```
- [`cargo-binutils`] for the `build-fw-bin` alias:
  ```sh
  cargo install cargo-binutils
  ```
- A J-Link probe and `JLinkGDBServerCLExe` on `$PATH` for flashing/debugging.

---

## Building

`.cargo/config.toml` sets `target = "armv7a-none-eabihf"` and defines these
aliases — always use them instead of bare `cargo build` so the required
`-Zbuild-std=core` nightly flags are passed correctly:

| Command | Output | Notes |
|---------|--------|-------|
| `cargo build-fw` | `target/armv7a-none-eabihf/debug/deluge-firmware` | Debug ELF, RTT enabled |
| `cargo build-fw-rel` | `target/armv7a-none-eabihf/release/deluge-firmware` | Release ELF, RTT disabled |
| `cargo build-fw-bin` | `target/armv7a-none-eabihf/release/deluge-firmware.bin` | Raw binary for flashing |

---

## Debugging

Two probe backends are supported: J-Link (stable, recommended) and a custom
`probe-rs` fork with Cortex-A9 trace support.

### J-Link

Open the project in VS Code with the [Cortex-Debug] extension installed, then
use one of the launch configurations in [`.vscode/launch.json`](.vscode/launch.json):

- **Rust firmware (debug)** — loads the debug ELF via J-Link and attaches RTT.
- **Rust firmware (release)** — loads the release ELF.

Both configurations use [`rza1_debug.JLinkScript`](rza1_debug.JLinkScript) to
halt the CPU and configure it for direct SRAM execution without going through
the ROM bootloader. Requires `JLinkGDBServerCLExe` on `$PATH`.

RTT log output is written to `rtt.log` in the workspace root and streamed to
the Cortex-Debug terminal.

### probe-rs (`trace-a9` fork)

A fork of probe-rs with Cortex-A9 PTM/ETF trace support for the RZ/A1L is
available at [`stellar-aria/probe-rs`][probe-rs-fork], `trace-a9` branch.
A vendored copy is kept in [`tools/probe-rs/`](tools/probe-rs/).

Build and install from the fork:

```sh
cd ~/GitHub/probe-rs          # or tools/probe-rs/
cargo install --path probe-rs-tools --locked
```

**Flash and run** (streams RTT to the terminal):

```sh
probe-rs run --chip R7S721020 target/armv7a-none-eabihf/debug/deluge-firmware
```

**Capture PTM instruction trace** from the on-chip ETF buffer:

```sh
# Dump raw bytes for offline decoding (ptm2human / Trace Compass):
probe-rs read-trace --chip R7S721020 --duration-ms 2000 --output trace.bin

# Decode inline and print packets:
probe-rs read-trace --chip R7S721020 --duration-ms 2000 --decode \
    --elf target/armv7a-none-eabihf/debug/deluge-firmware

# Compact execution-flow view (ISync + branch targets with symbols):
probe-rs read-trace --chip R7S721020 --duration-ms 2000 --flow \
    --elf target/armv7a-none-eabihf/debug/deluge-firmware

# JSON Lines output for programmatic processing:
probe-rs read-trace --chip R7S721020 --duration-ms 2000 --decode \
    --elf target/armv7a-none-eabihf/debug/deluge-firmware \
    --output-format json 2>&1 | jq 'select(.type == "Branch")'
```

---

## CI

The [CI workflow](.github/workflows/ci.yml) builds the release firmware binary
on every push to `main` and on pull requests. The resulting
`firmware-release` artifact contains `deluge-firmware.bin`.

[Synthstrom Deluge]: https://synthstrom.com/product/deluge/
[Embassy]: https://embassy.dev
[`cargo-binutils`]: https://github.com/rust-embedded/cargo-binutils
[Cortex-Debug]: https://marketplace.visualstudio.com/items?itemName=marus25.cortex-debug
[probe-rs-fork]: https://github.com/stellar-aria/probe-rs/tree/trace-a9
