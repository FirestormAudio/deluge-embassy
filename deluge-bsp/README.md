# deluge-bsp

Board Support Package for the Synthstrom Deluge controller, targeting the
Renesas RZ/A1L (Arm Cortex-A9, 400 MHz, 3 MB on-chip SRAM + 64 MB SDRAM).

This crate sits on top of [`rza1l-hal`](../rza1l-hal) and provides
ready-to-use drivers and initialization routines for every peripheral on the
Deluge board.  It is `no_std` when compiled for the bare-metal target
(`armv7a-none-eabihf`), and can also be compiled for the host for unit
testing.

## Modules

| Module | Hardware |
|---|---|
| [`system`] | Clock gating (CPG / `StbConfig`), DMA channel map, single `init_clocks()` boot entry point |
| [`sdram`] | 64 MB SDRAM (Micron MT48LC16M16A2P-75) — pin-mux, BSC registers, JEDEC init |
| [`audio`] | SSI0 stereo codec interface; codec power sequencing and master-clock enable |
| [`scux_dvu_path`] | SCUX DVU path — CPU SRAM → DMA → SCUX → SSIF0 TX (2048-frame buffer, software volume / fade) |
| [`scux_src_path`] | SCUX async SRC path — asynchronous sample-rate conversion between two independent clock domains (e.g. 44.1 kHz ↔ 48 kHz) |
| [`cv_gate`] | MAX5136 quad 16-bit SPI CV DAC (2 channels, ~6552 counts/V) + 4 V-trig gate GPIOs |
| [`midi_gate`] | MTU2 one-shot timer for sub-millisecond precise gate-off scheduling (~1.92 µs resolution) |
| [`oled`] | SSD1309 128×48 OLED — `FrameBuffer`, pixel API, DMA frame send via RSPI0; CS/RST handshaked through the PIC co-processor |
| [`pic`] | PIC32 co-processor — 144-pad matrix, 36 buttons, 6 encoders, 36 LEDs, 7-segment display, gold knob indicators; dual-baud UART handshake (31 250 → 200 000 bps) |
| [`uart`] | SCIF0 MIDI DIN (31 250 bps) + SCIF1 PIC UART; DMA-backed RX/TX |
| [`sd`] | SDHI SD v2 card — full JEDEC init, block read/write, SDHC/SDXC auto-detect, DMA bounce buffer |
| [`fat`] | `embedded-sdmmc` wrapper — `DelugeVolumeManager`, `DelugeBlockDevice`, FAT filesystem access |
| [`usb`] | RUSB1 USB port — device mode (`embassy-usb-driver`) and host mode (`embassy-usb-host`) with compile-time mode tracking |

## Peripheral sharing

RSPI0 is shared between the OLED DMA path (`oled`) and the CV DAC
(`cv_gate`).  The global `RSPI0_DMA_ACTIVE` atomic flag serialises access:
`cv_gate::cv_set_blocking` spins until the OLED DMA transfer completes before
reconfiguring the bus.

## USB modes

`usb::init_device_mode` and `usb::init_host_mode` return a typed
`UsbPort<Device>` / `UsbPort<Host>` handle, preventing mode confusion at
compile time.  The port can be switched at runtime via
`into_device_mode()` / `into_host_mode()`.

## Feature flags

There are currently no Cargo feature flags.  Peripherals that require
Embassy (timers, USB, SCUX async paths) are gated on
`#[cfg(target_os = "none")]` and are excluded from host builds.

## Usage

```toml
[dependencies]
deluge-bsp = { path = "../deluge-bsp" }
```

A typical boot sequence:

```rust
deluge_bsp::system::init_clocks();
deluge_bsp::sdram::init();
deluge_bsp::audio::init();
deluge_bsp::uart::init_pic(31_250);
deluge_bsp::cv_gate::init();
```
