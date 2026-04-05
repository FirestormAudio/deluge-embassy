# rza1l-hal

Register-level HAL for the **Renesas RZ/A1L** (Cortex-A9, R7S721001),
written in `no_std` Rust with async support via [Embassy].

Developed for the [Synthstrom Deluge] open-source firmware port, but kept
board-agnostic where possible — board-specific configuration is passed in
at runtime rather than compiled in.

---

## Target

| Property       | Value                              |
|----------------|------------------------------------|
| CPU            | ARM Cortex-A9, ARMv7-A             |
| Chip           | Renesas RZ/A1L (R7S721001)         |
| Rust target    | `armv7a-none-eabihf`               |
| Async runtime  | [Embassy]                          |
| `no_std`       | yes (host-target unit tests only)  |

---

## Modules

### System

| Module        | Description |
|---------------|-------------|
| `startup`     | ARM vector table and reset handler (`_start`). Includes the bootloader signature words at fixed offsets so the Deluge boot ROM accepts the image. |
| `mmu`         | Flat-map L1 Translation Table (VA = PA, 1 MB sections) covering the full 4 GB address space with correct cache attributes per region (SRAM cached, I/O strongly-ordered, uncached mirror at +0x4000_0000). |
| `cache`       | L1 D/I-cache + Cortex-A9 branch predictor enable; PL310 L2 cache init. Call `mmu::init_and_enable` first. |
| `stb`         | CPG Standby Control Register (STBCR2–STBCR12) init — ungate module clocks for all required peripherals via a `StbConfig` struct. |
| `gic`         | ARM GIC-400 (GICD/GICC) driver. `gic::init` → `gic::register(id, handler)` → `gic::enable(id)`. The IRQ assembly shim calls `gic::dispatch`. |
| `bsc`         | Bus State Controller CS0/CS1 timing (NOR flash). |

### Memory

| Module      | Description |
|-------------|-------------|
| `allocator` | Two independent `linked_list_allocator`-backed heaps: `allocator::SRAM` and `allocator::SDRAM`. Each is protected by a critical-section lock. Call `CsHeap::init` once during startup before any allocation in that arena. |

The RZ/A1L provides **uncached mirror aliases** of SRAM and SDRAM at
`physical_address + 0x4000_0000` (see `UNCACHED_MIRROR_OFFSET`). DMA-shared
buffers must be accessed through these aliases to avoid stale cache lines
without explicit cache maintenance.

### Timers

| Module        | Description |
|---------------|-------------|
| `ostm`        | OS Timer — two 32-bit channels clocked from P0 (33.33 MHz). Supports free-running and interval modes. Used as the Embassy time-driver tick source. |
| `time_driver`  | Embassy `time-driver` implementation backed by OSTM0 (free-running) + OSTM1 (alarm). |
| `mtu2`        | Multi-Function Timer Pulse Unit 2 — five 16-bit channels. Used for bare-metal timing where OSTM is unavailable. |

### DMA

| Module | Description |
|--------|-------------|
| `dmac` | 16-channel DMAC (channels 0–15). Register-mode (one-shot) and link-descriptor (circular) transfers. `init_register_mode` / `start_transfer` for memory→peripheral; `init_register_mode_rx` / `start_transfer_rx` for peripheral→memory; `init_with_link_descriptor` for circular audio streaming. |

DMAC resource selectors (DMARS values) are peripheral-specific constants
documented in each peripheral module.

### Audio

| Module | Description |
|--------|-------------|
| `ssi`  | SSIF (I²S) stereo 44.1 kHz driver. TX and RX each run in self-referential circular link-descriptor DMA mode so the hardware re-arms automatically. Buffer pointers and current-position helpers are exposed for the audio task. |
| `scux` | Sample Rate Conversion Unit — FFD (CPU→SCUX), IPC, 2SRC (async SRC), DVU (digital volume), MIX, OPC, FFU (SCUX→CPU) blocks. Exposes `init_ffd_dma`, `init_ffu_dma`, `set_volume`, `set_src_ratio`, etc. |

### Peripherals

| Module  | Description |
|---------|-------------|
| `sdhi`  | SD Host Interface — two ports (SDHI0/SDHI1). Async `send_cmd`, `read_blocks_sw` / `write_blocks_sw` (PIO with `AtomicWaker`), `read_blocks_dma` / `write_blocks_dma` (DMAC-backed), and polling variants for use outside the executor. |
| `uart`  | SCIF async serial — five channels (SCIF0–4). TX via TXI interrupts; RX via DMA ring buffer (`init_dma_rx`) or interrupt (`register_txi_for`). |
| `rspi`  | SPI master (RSPI0–4). 32-bit frame mode; interrupt-driven transfer-complete. Used for CV DAC and OLED. |
| `gpio`  | GPIO port registers (P1–P11, PMC0–PMC11). `set_pin_mux`, `set_output`, `read_input`. Implements `embedded-hal` `InputPin` / `OutputPin`. |
| `rusb1` | USB200/USB201 GIC interrupt helpers and clock-gate wrappers. Register-level USB logic lives in the BSP layer. |

---

## Getting started

Add to `Cargo.toml`:

```toml
[dependencies]
rza1l-hal = { path = "../rza1l-hal" }
embassy-executor = { version = "0.6", features = ["arch-cortex-ar"] }
embassy-time    = "0.5"
```

Minimal bare-metal init sequence:

```rust,ignore
#[no_mangle]
unsafe extern "C" fn board_init() {
    // 1. Ungate peripheral clocks
    rza1l_hal::stb::init(&MY_STB_CONFIG);
    // 2. MMU + caches
    rza1l_hal::mmu::init_and_enable();
    rza1l_hal::cache::l1_enable();
    rza1l_hal::cache::l2_init();
    // 3. GIC
    rza1l_hal::gic::init();
    // 4. Embassy time driver (OSTM0 free-run + OSTM1 alarm)
    rza1l_hal::ostm::enable_clock();
    rza1l_hal::time_driver::init();
}
```

All peripheral init functions (`sdhi::init`, `ssi::init`, `uart::init`, …)
are `unsafe` and must be called before interrupts are enabled on that
peripheral.

---

## Features

| Feature | Description |
|---------|-------------|
| `rtt`   | Enables RTT (SEGGER Real-Time Transfer) logging via `rtt-target`. |

---

## Safety

Nearly all functions in this crate are `unsafe`. The caller is responsible for:

- Calling init functions **exactly once**, from a single-threaded boot context.
- Not calling peripheral functions **concurrently** on the same channel.
- Ensuring DMA buffers reside in **uncached memory** (use `UNCACHED_MIRROR_OFFSET`).
- Registering GIC IRQs before enabling the corresponding interrupt source.

---

## Crate structure policy

This crate is board-agnostic — it does not hard-code DMA channel numbers,
GPIO pin assignments, or clock configurations. Those belong in a board
support layer (BSP). Peripheral drivers accept configuration by
value (e.g. `SsiConfig`, `StbConfig`) so the same HAL binary can target
different board layouts.

[Embassy]: https://embassy.dev
