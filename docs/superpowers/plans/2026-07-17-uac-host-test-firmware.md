# UAC Host Test Firmware Implementation Plan

> **⚠️ SUPERSEDED (2026-07-17).** This plan built the firmware over embassy-usb-host's
> output-only `UacHandler`. The project is instead building its own
> `deluge_bsp::usb::host::uac` (capture + playback) per
> [`2026-07-16-usb-uac-host-design.md`](../specs/2026-07-16-usb-uac-host-design.md),
> staged capture-first. See the Phase-1 (capture) plan
> `2026-07-17-uac-host-capture.md`. Tasks 1–3 here (crate scaffold, `is_connected`
> accessor, blink/pic/OLED bring-up) remain reusable for the eventual validation
> firmware; Tasks 4–5 (the `UacHandler` stream) are obsolete.

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a dedicated `uac-host-firmware` image that brings USB0 up in host mode, enumerates a class-compliant USB audio output device, opens the ISO OUT pipe via `embassy-usb-host`'s `UacHandler`, and streams a 440 Hz tone — with every stage logged over RTT and mirrored to the OLED so the host path is observable even with nothing plugged in.

**Architecture:** A single-purpose firmware crate modeled on `msc-firmware`. The pure test-tone logic (`SineGen`) lives in a platform-independent module that host-tests under `x86_64`; all embassy/HAL/BSP code is gated behind `cfg(target_os = "none")` so the crate still compiles (and runs its unit tests) natively. One small reusable accessor — `Rusb1HostDriver::is_connected()` — is added to the HAL to drive the stream's connection guard.

**Tech Stack:** Rust (edition 2024, nightly), `embassy-executor`/`embassy-time`/`embassy-usb-host` 0.1.0, `rza1l-hal` (RUSB1 host driver), `deluge-bsp` (OLED/PIC), `deluge-alloc`, `libm`, RTT via `rtt-target`.

## Global Constraints

- **Spec:** `docs/superpowers/specs/2026-07-17-uac-host-test-firmware-design.md`. This is a bring-up/validation firmware, not the production feature (`2026-07-16-usb-uac-host-design.md`).
- **Bare-metal target** `armv7a-none-eabihf` (`target_os = "none"`); it is the default in `.cargo/config.toml`. Device builds use `cargo build-uac` (adds `-Zbuild-std=core`).
- **Host tests** compile the crate natively; run them with exactly:
  `cargo test -p uac-host-firmware --target x86_64-unknown-linux-gnu --no-default-features`
  (`--no-default-features` disables the `rtt` feature, whose deps are `target_os = "none"`-only.)
- **32-bit `usize` on device** (`usize == u32`); review any offset/index math for `u32` overflow by hand — x86-64 host tests will not catch it.
- **Host driver is alloc-free**; `SineGen` is `no_std`, `core` + `libm` only, no heap.
- **No probe-rs.** Flashing is the user's existing dev-mode upload (`cargo deluge run --release`); RTT capture is the user's own J-Link flow (writes `rtt.log`).
- **USB identity / product-impersonation:** not applicable (host mode advertises no device descriptors).

---

### Task 1: Crate scaffold + host-tested `SineGen`

Creates the crate so it links for the device (stub `main`) and its pure tone module passes native unit tests.

**Files:**
- Create: `firmwares/uac-host-firmware/Cargo.toml`
- Create: `firmwares/uac-host-firmware/build.rs`
- Create: `firmwares/uac-host-firmware/memory.x`
- Create: `firmwares/uac-host-firmware/memory_rtt.x`
- Create: `firmwares/uac-host-firmware/src/main.rs`
- Create: `firmwares/uac-host-firmware/src/firmware.rs` (stub boot)
- Create: `firmwares/uac-host-firmware/src/sine.rs` (logic + tests)
- Modify: `Cargo.toml` (workspace `members`)
- Modify: `.cargo/config.toml` (add `build-uac` / `build-uac-bin` aliases)

**Interfaces:**
- Produces: `crate::sine::Format { sample_rate: u32, channels: u8, bytes_per_sample: u8 }` (Copy); `Format::frame_bytes(&self) -> usize`; `sine::frames_per_packet(len: usize, fmt: &Format) -> usize`; `sine::encode_sample(sample: f32, out: &mut [u8])`; `sine::SineGen` with `SineGen::new(freq_hz: f32, fmt: Format) -> Self` and `SineGen::fill(&mut self, buf: &mut [u8])`.

- [ ] **Step 1: Write `src/sine.rs` with the logic and failing tests**

```rust
//! Format-aware test-tone generator for the UAC host firmware.
//!
//! Pure `core` + `libm` math with no platform dependencies, so it host-tests
//! natively (the crate's platform deps are `cfg(target_os = "none")`-only).

use core::f32::consts::TAU;

/// Negotiated PCM stream format, captured from the device's UAC descriptors.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Format {
    /// Sample rate in Hz (from the clock-source SAMPLING_FREQ control).
    pub sample_rate: u32,
    /// Channels per frame (UAC `bNrChannels`).
    pub channels: u8,
    /// Bytes per sample subslot (UAC Type-I `bSubslotSize`): 2, 3, or 4.
    pub bytes_per_sample: u8,
}

impl Format {
    /// Bytes in one interleaved frame (all channels).
    #[inline]
    pub fn frame_bytes(&self) -> usize {
        self.channels as usize * self.bytes_per_sample as usize
    }
}

/// Number of whole frames that fit in an ISO packet of `len` bytes.
#[inline]
pub fn frames_per_packet(len: usize, fmt: &Format) -> usize {
    let fb = fmt.frame_bytes();
    if fb == 0 { 0 } else { len / fb }
}

/// Peak amplitude: -6 dBFS of full scale — clearly audible, never clips.
const AMP: f32 = 0.5;

/// Encode one sample in `[-1.0, 1.0]` into `out`; `out.len()` selects the width:
/// 2 -> i16 LE, 3 -> 24-bit signed packed LE, 4 -> i32 LE (full scale).
/// Other lengths write nothing.
#[inline]
pub fn encode_sample(sample: f32, out: &mut [u8]) {
    let s = (sample * AMP).clamp(-1.0, 1.0);
    match out.len() {
        2 => {
            let v = (s * i16::MAX as f32) as i16;
            out.copy_from_slice(&v.to_le_bytes());
        }
        3 => {
            let v = (s * 8_388_607.0) as i32; // 2^23 - 1
            out[0] = v as u8;
            out[1] = (v >> 8) as u8;
            out[2] = (v >> 16) as u8;
        }
        4 => {
            let v = (s * i32::MAX as f32) as i32;
            out.copy_from_slice(&v.to_le_bytes());
        }
        _ => {}
    }
}

/// Phase-accumulator sine generator that fills raw ISO OUT packets.
pub struct SineGen {
    phase: f32,  // radians, in [0, TAU)
    dphase: f32, // radians per frame
    fmt: Format,
}

impl SineGen {
    /// Create a generator for `freq_hz` at the device's negotiated `fmt`.
    pub fn new(freq_hz: f32, fmt: Format) -> Self {
        let dphase = if fmt.sample_rate == 0 {
            0.0
        } else {
            TAU * freq_hz / fmt.sample_rate as f32
        };
        Self { phase: 0.0, dphase, fmt }
    }

    /// Fill `buf` with interleaved frames of the tone. Writes only whole frames;
    /// any trailing partial frame is left untouched (the UAC layer always hands
    /// whole-frame packets).
    pub fn fill(&mut self, buf: &mut [u8]) {
        let fb = self.fmt.frame_bytes();
        let bps = self.fmt.bytes_per_sample as usize;
        if fb == 0 || bps == 0 {
            return;
        }
        for frame in buf.chunks_mut(fb) {
            if frame.len() < fb {
                break;
            }
            let s = libm::sinf(self.phase);
            self.phase += self.dphase;
            if self.phase >= TAU {
                self.phase -= TAU;
            }
            for ch in frame.chunks_mut(bps) {
                encode_sample(s, ch);
            }
        }
    }
}

#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::*;

    #[test]
    fn frames_per_packet_divides_by_frame_bytes() {
        let fmt = Format { sample_rate: 44_100, channels: 2, bytes_per_sample: 3 };
        assert_eq!(fmt.frame_bytes(), 6);
        assert_eq!(frames_per_packet(24, &fmt), 4);
        assert_eq!(frames_per_packet(25, &fmt), 4); // trailing partial ignored
        assert_eq!(frames_per_packet(0, &fmt), 0);
    }

    #[test]
    fn encode_16bit_le_at_minus_6dbfs() {
        let mut out = [0u8; 2];
        encode_sample(1.0, &mut out);
        assert_eq!(out, 16_383i16.to_le_bytes()); // 1.0 * 0.5 * 32767
        encode_sample(-1.0, &mut out);
        assert_eq!(out, (-16_383i16).to_le_bytes());
        encode_sample(0.0, &mut out);
        assert_eq!(out, [0, 0]);
    }

    #[test]
    fn encode_24bit_packed_le() {
        let mut out = [0u8; 3];
        encode_sample(1.0, &mut out);
        assert_eq!(out, [0xFF, 0xFF, 0x3F]); // (0.5 * 8388607) = 4194303 = 0x3FFFFF
    }

    #[test]
    fn encode_32bit_le() {
        let mut out = [0u8; 4];
        encode_sample(1.0, &mut out);
        assert_eq!(out, ((0.5 * i32::MAX as f32) as i32).to_le_bytes());
    }

    #[test]
    fn fill_replicates_sample_across_channels_within_frame() {
        let fmt = Format { sample_rate: 48_000, channels: 2, bytes_per_sample: 2 };
        let mut gen = SineGen::new(440.0, fmt);
        let mut buf = [0u8; 8]; // 2 frames
        gen.fill(&mut buf);
        assert_eq!(&buf[0..2], &buf[2..4]); // frame 0: ch0 == ch1
        assert_eq!(&buf[4..6], &buf[6..8]); // frame 1: ch0 == ch1
    }

    #[test]
    fn fill_starts_at_phase_zero_so_first_frame_is_silence() {
        let fmt = Format { sample_rate: 48_000, channels: 1, bytes_per_sample: 2 };
        let mut gen = SineGen::new(440.0, fmt);
        let mut buf = [0xAAu8; 2];
        gen.fill(&mut buf);
        assert_eq!(buf, [0, 0]); // sin(0) == 0
    }

    #[test]
    fn fill_leaves_trailing_partial_frame_untouched() {
        let fmt = Format { sample_rate: 48_000, channels: 2, bytes_per_sample: 2 };
        let mut gen = SineGen::new(440.0, fmt);
        let mut buf = [0x5Au8; 6]; // 1 whole frame (4B) + 2B partial
        gen.fill(&mut buf);
        assert_eq!(&buf[4..6], &[0x5A, 0x5A]); // partial untouched
    }
}
```

- [ ] **Step 2: Create `src/firmware.rs` (stub boot — replaced in Task 3)**

```rust
//! Device entry point. Stub for scaffolding; replaced with the full executor
//! bring-up in Task 3.

use core::panic::PanicInfo;
use log::{error, info};

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    error!("PANIC: {}", info);
    loop {
        core::hint::spin_loop();
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn main() -> ! {
    #[cfg(feature = "rtt")]
    {
        let channels = rtt_target::rtt_init! {
            up: { 0: { size: 16384, name: "Terminal", section: ".rtt_buffer" } }
            section_cb: ".rtt_buffer"
        };
        rtt_target::set_print_channel(channels.up.0);
        rtt_target::init_logger_with_level(log::LevelFilter::Debug);
    }
    info!("UAC host test firmware (scaffold) — waiting");
    loop {
        core::hint::spin_loop();
    }
}
```

- [ ] **Step 3: Create `src/main.rs`**

```rust
//! Deluge UAC host test firmware — see
//! `docs/superpowers/specs/2026-07-17-uac-host-test-firmware-design.md`.
//!
//! Brings USB0 up in host mode and streams a 440 Hz tone to a class-compliant
//! USB audio output device, logging every stage over RTT and the OLED.

#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]
#![cfg_attr(target_os = "none", feature(impl_trait_in_assoc_type))]

mod sine;

#[cfg(target_os = "none")]
mod firmware;

// Native builds (used only to run `sine`'s unit tests) need a `main`.
#[cfg(all(not(target_os = "none"), not(test)))]
fn main() {}
```

- [ ] **Step 4: Create `build.rs` (verbatim from `msc-firmware/build.rs`)**

```rust
use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());

    // Select the memory layout and linker script that match the rtt feature.
    // rza1l-hal's build.rs places rza1l.x / rza1l_rtt.x on the search path;
    // we just need to put the matching memory.x alongside them.
    let rtt = env::var("CARGO_FEATURE_RTT").is_ok();
    let (memory_src, linker_script) = if rtt {
        ("memory_rtt.x", "rza1l_rtt.x")
    } else {
        ("memory.x", "rza1l.x")
    };

    fs::copy(manifest_dir.join(memory_src), out_dir.join("memory.x")).unwrap();

    println!("cargo:rustc-link-search={}", out_dir.display());
    println!("cargo:rustc-link-arg=-T{linker_script}");
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=memory_rtt.x");
}
```

- [ ] **Step 5: Create `memory.x` and `memory_rtt.x` (verbatim copies)**

Run:
```bash
cp firmwares/msc-firmware/memory.x     firmwares/uac-host-firmware/memory.x
cp firmwares/msc-firmware/memory_rtt.x firmwares/uac-host-firmware/memory_rtt.x
```

- [ ] **Step 6: Create `Cargo.toml`**

```toml
[package]
publish = false  # binary crate — not for crates.io
name = "uac-host-firmware"
version = "0.1.0"
edition = "2024"
license.workspace = true

[[bin]]
name = "uac-host-firmware"
# `test` is left enabled so the pure `sine` module host-tests via:
#   cargo test -p uac-host-firmware --target x86_64-unknown-linux-gnu --no-default-features

# Host-buildable deps (needed natively so `sine`'s tests compile).
[dependencies]
log = { workspace = true }
libm = { version = "0.2", default-features = false }

# Platform deps: only compiled for the bare-metal target, so `cargo test`
# on the host never pulls embassy/cortex-ar (which don't build for x86_64).
[target.'cfg(target_os = "none")'.dependencies]
rza1l-hal = { path = "../../crates/rza1l-hal" }
deluge-bsp = { path = "../../crates/deluge-bsp" }
deluge-alloc = { path = "../../crates/deluge-alloc" }
cortex-ar = { workspace = true }
embassy-executor = { workspace = true, features = ["nightly", "platform-cortex-ar", "executor-thread"] }
embassy-time = { workspace = true, features = ["tick-hz-1_000_000"] }
embassy-sync = { workspace = true }
embassy-futures = { workspace = true }
embassy-usb-driver = { workspace = true }
# `log` feature surfaces the UAC class's own `[UAC] …` debug/error lines on RTT.
embassy-usb-host = { workspace = true, features = ["log"] }
rtt-target = { workspace = true, optional = true }

[features]
## RTT (SEGGER Real-Time Transfer) debug logging — default on for dev builds.
## Release/native builds disable it: `--no-default-features`.
default = ["rtt"]
rtt = ["dep:rtt-target", "rza1l-hal/rtt"]
```

- [ ] **Step 7: Add the crate to the workspace**

In `Cargo.toml` (workspace root), add to the `members` list after the `"firmwares/wp-probe",` line:

```toml
  "firmwares/uac-host-firmware",
```

- [ ] **Step 8: Add cargo build aliases**

In `.cargo/config.toml`, after the `build-msc-bin` line, add:

```toml
build-uac            = "build -Zbuild-std=core -Zbuild-std-features=compiler-builtins-mem -p uac-host-firmware"
build-uac-bin        = "objcopy -Zbuild-std=core -Zbuild-std-features=compiler-builtins-mem --release --no-default-features -p uac-host-firmware -- -O binary target/armv7a-none-eabihf/release/uac-host-firmware.bin"
```

- [ ] **Step 9: Run the host tests to verify they pass**

Run: `cargo test -p uac-host-firmware --target x86_64-unknown-linux-gnu --no-default-features`
Expected: PASS — 7 tests in `sine::tests` all green.

- [ ] **Step 10: Verify the device crate links**

Run: `cargo build-uac`
Expected: compiles and links `uac-host-firmware` (stub boot) with no errors.

- [ ] **Step 11: Commit**

```bash
git add firmwares/uac-host-firmware Cargo.toml .cargo/config.toml
git commit -m "feat(uac-host-firmware): scaffold crate + host-tested SineGen"
```

---

### Task 2: HAL `Rusb1HostDriver::is_connected()` accessor

Adds a connection flag the streaming loop polls, set/cleared on the ATTCH/DTCH ISR paths.

**Files:**
- Modify: `crates/rza1l-hal/src/usb/host.rs`

**Interfaces:**
- Produces: `rza1l_hal::usb::Rusb1HostDriver::is_connected(&self) -> bool`.

- [ ] **Step 1: Import `AtomicBool`**

In `crates/rza1l-hal/src/usb/host.rs`, change the atomics import (currently line 51):

```rust
use core::sync::atomic::{AtomicU8, AtomicU16, Ordering};
```

to:

```rust
use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU16, Ordering};
```

- [ ] **Step 2: Add the `HCD_CONNECTED` static**

Directly below the `HCD_EVENT_WAKER` static (near line 171):

```rust
static HCD_EVENTS: AtomicU8 = AtomicU8::new(0);
static HCD_EVENT_WAKER: AtomicWaker = AtomicWaker::new();
/// Root-port connection state: `true` between the ATTCH and DTCH interrupts.
/// Read by `Rusb1HostDriver::is_connected` to guard isochronous streaming.
static HCD_CONNECTED: AtomicBool = AtomicBool::new(false);
```

- [ ] **Step 3: Set the flag on ATTCH**

In the ATTCH branch of the ISR, after the existing `HCD_EVENTS.fetch_or(EVT_ATTACH, Ordering::Release);` and `HCD_EVENT_WAKER.wake();` lines:

```rust
            HCD_EVENTS.fetch_or(EVT_ATTACH, Ordering::Release);
            HCD_EVENT_WAKER.wake();
            HCD_CONNECTED.store(true, Ordering::Release);
```

- [ ] **Step 4: Clear the flag on DTCH**

In the DTCH branch (`if is1 & INTSTS1_DTCH != 0 {`), after the first line `rmw(core::ptr::addr_of_mut!(e.dvstctr0), DVSTCTR0_UACT, 0);`:

```rust
        if is1 & INTSTS1_DTCH != 0 {
            rmw(core::ptr::addr_of_mut!(e.dvstctr0), DVSTCTR0_UACT, 0);
            HCD_CONNECTED.store(false, Ordering::Release);
```

- [ ] **Step 5: Add the `is_connected` method**

In the `impl Rusb1HostDriver` block, directly above the existing `pub fn port_speed(&self)` method (near line 569):

```rust
    /// Whether a device is currently attached to the root port.
    ///
    /// Tracks the ATTCH/DTCH interrupts. Used as the connection guard for
    /// `embassy_usb_host`'s `UacOut::output_stream`, so an unplug ends the
    /// stream promptly rather than only when the next transfer errors.
    pub fn is_connected(&self) -> bool {
        HCD_CONNECTED.load(Ordering::Acquire)
    }
```

- [ ] **Step 6: Verify the HAL host tests still pass**

Run: `cargo test -p rza1l-hal --target x86_64-unknown-linux-gnu`
Expected: PASS — the existing HAL host tests (14) are unaffected.

- [ ] **Step 7: Verify the device HAL build**

Run: `cargo build-fw -p rza1l-hal`
Expected: compiles with no errors.

- [ ] **Step 8: Commit**

```bash
git add crates/rza1l-hal/src/usb/host.rs
git commit -m "feat(rza1l-hal): Rusb1HostDriver::is_connected() root-port flag"
```

---

### Task 3: Executor bring-up + blink/pic tasks (host + oled stubbed)

Replaces the stub boot with the full executor setup and wires all four tasks. `uac_host` and `oled` are minimal stubs here (implemented in Tasks 4–5) so the firmware links, boots, and spawns.

**Files:**
- Modify: `firmwares/uac-host-firmware/src/main.rs` (add `mod tasks;`)
- Replace: `firmwares/uac-host-firmware/src/firmware.rs`
- Create: `firmwares/uac-host-firmware/src/tasks/mod.rs`
- Create: `firmwares/uac-host-firmware/src/tasks/blink.rs`
- Create: `firmwares/uac-host-firmware/src/tasks/pic.rs`
- Create: `firmwares/uac-host-firmware/src/tasks/oled.rs` (stub)
- Create: `firmwares/uac-host-firmware/src/tasks/uac_host.rs` (stub)

**Interfaces:**
- Consumes: `rza1l_hal::usb::{hcd_int_handler, init_host_mode, USB0_IRQ, Rusb1HostDriver}`.
- Produces: `crate::tasks::blink::blink_task()`, `crate::tasks::pic::pic_task()`, `crate::tasks::oled::oled_task()`, `crate::tasks::uac_host::uac_host_task(driver: Rusb1HostDriver)` — all `#[embassy_executor::task]`.

- [ ] **Step 1: Add `mod tasks;` to `src/main.rs`**

After the `#[cfg(target_os = "none")] mod firmware;` line, add:

```rust
#[cfg(target_os = "none")]
mod firmware;

#[cfg(target_os = "none")]
mod tasks;
```

- [ ] **Step 2: Replace `src/firmware.rs` with the full bring-up**

```rust
//! Device entry point: platform bring-up + Embassy executor.

use core::mem::MaybeUninit;
use core::panic::PanicInfo;

use embassy_executor::{Executor, Spawner};
use log::{error, info};

use deluge_alloc as allocator;
use deluge_bsp::{cv_gate, uart as bsp_uart};
use rza1l_hal::gic;
use rza1l_hal::usb::{hcd_int_handler, init_host_mode};

use crate::tasks;

unsafe extern "C" {
    /// Start of the free SRAM heap region (set by the linker script).
    static __sram_heap_start: u8;
    /// End of the free SRAM heap region (start of RTT/stack reservation).
    static __sram_heap_end: u8;
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    error!("PANIC: {}", info);
    loop {
        core::hint::spin_loop();
    }
}

static mut EXECUTOR: MaybeUninit<Executor> = MaybeUninit::uninit();

#[unsafe(no_mangle)]
pub extern "C" fn main() -> ! {
    #[cfg(feature = "rtt")]
    {
        let channels = rtt_target::rtt_init! {
            up: { 0: { size: 16384, name: "Terminal", section: ".rtt_buffer" } }
            section_cb: ".rtt_buffer"
        };
        rtt_target::set_print_channel(channels.up.0);
        rtt_target::init_logger_with_level(log::LevelFilter::Debug);
    }
    info!("UAC host test firmware starting");

    // SRAM heap (from internal RAM), before any allocation.
    unsafe {
        let start = core::ptr::addr_of!(__sram_heap_start) as *mut u8;
        let size = core::ptr::addr_of!(__sram_heap_end) as usize - start as usize;
        allocator::SRAM.init(start, size);
    }

    // Module clocks, MMU, cache, SDRAM window, GIC, OSTM, time driver.
    unsafe { deluge_bsp::system::init_clocks() };
    unsafe { allocator::SDRAM.init(0x0C00_0000 as *mut u8, 64 * 1024 * 1024) };

    // Heartbeat LED + PIC transport (for the OLED chip-select echo) + RSPI0.
    unsafe { rza1l_hal::gpio::set_as_output(6, 7) };
    unsafe { bsp_uart::init_pic(31_250) };
    unsafe { cv_gate::init() };

    // USB0 host-mode ISR — registered before IRQ is globally enabled.
    unsafe {
        gic::register(rza1l_hal::usb::USB0_IRQ, || {
            hcd_int_handler(0);
        });
    }

    let (_port, host_driver) = unsafe { init_host_mode(0) };
    info!("USB: host driver ready (USB0 host mode)");

    unsafe { cortex_ar::interrupt::enable() };
    info!("IRQ enabled — starting tasks");

    #[allow(static_mut_refs)]
    let executor = unsafe {
        EXECUTOR.write(Executor::new());
        EXECUTOR.assume_init_mut()
    };
    executor.run(|spawner: Spawner| {
        spawner.spawn(tasks::blink::blink_task().unwrap());
        spawner.spawn(tasks::pic::pic_task().unwrap());
        spawner.spawn(tasks::oled::oled_task().unwrap());
        spawner.spawn(tasks::uac_host::uac_host_task(host_driver).unwrap());
    })
}
```

- [ ] **Step 3: Create `src/tasks/mod.rs`**

```rust
pub(crate) mod blink;
pub(crate) mod oled;
pub(crate) mod pic;
pub(crate) mod uac_host;
```

- [ ] **Step 4: Create `src/tasks/blink.rs` (verbatim from `msc-firmware`)**

```rust
use embassy_time::Timer;
use log::debug;

/// Heartbeat blink — P6_7 plays a double-pulse heartbeat pattern so it is
/// visually obvious that the firmware is alive even when idle.
#[embassy_executor::task]
pub(crate) async fn blink_task() {
    debug!("blink_task: started");
    loop {
        unsafe { rza1l_hal::gpio::write(6, 7, true) };
        Timer::after_millis(80).await;
        unsafe { rza1l_hal::gpio::write(6, 7, false) };
        Timer::after_millis(120).await;
        unsafe { rza1l_hal::gpio::write(6, 7, true) };
        Timer::after_millis(80).await;
        unsafe { rza1l_hal::gpio::write(6, 7, false) };
        Timer::after_millis(720).await;
    }
}
```

- [ ] **Step 5: Create `src/tasks/pic.rs` (verbatim from `msc-firmware`)**

```rust
use log::info;

use deluge_bsp::pic;

/// Minimal PIC32 transport task.
///
/// This firmware does not use the pads, buttons or encoders, so this task only
/// performs the PIC baud-rate handshake and relays the OLED chip-select echo
/// (`OledSelected` / `OledDeselected`) that [`deluge_bsp::oled`] waits on during
/// init and frame writes. All other PIC events are discarded.
#[embassy_executor::task]
pub(crate) async fn pic_task() {
    info!("PIC: init (31 250 → 200 000 baud)");
    pic::init().await;
    info!("PIC: ready");

    let mut parser = pic::Parser::new();
    loop {
        let byte = rza1l_hal::uart::read_byte(pic::UART_CH).await;
        let Some(event) = parser.push(byte) else {
            continue;
        };

        match event {
            pic::Event::OledSelected => pic::notify_oled_selected(),
            pic::Event::OledDeselected => pic::notify_oled_deselected(),
            _ => {}
        }
    }
}
```

- [ ] **Step 6: Create `src/tasks/oled.rs` (stub — implemented in Task 5)**

```rust
use embassy_time::Timer;

/// Placeholder OLED task; the status display is implemented in Task 5.
#[embassy_executor::task]
pub(crate) async fn oled_task() {
    loop {
        Timer::after_millis(1000).await;
    }
}
```

- [ ] **Step 7: Create `src/tasks/uac_host.rs` (stub — implemented in Task 4)**

```rust
use embassy_time::Timer;
use log::info;

use rza1l_hal::usb::Rusb1HostDriver;

/// Placeholder host task; the enumerate/stream loop is implemented in Task 4.
#[embassy_executor::task]
pub(crate) async fn uac_host_task(_driver: Rusb1HostDriver) {
    info!("uac_host_task: stub");
    loop {
        Timer::after_millis(1000).await;
    }
}
```

- [ ] **Step 8: Verify the full firmware links**

Run: `cargo build-uac`
Expected: compiles and links; boots, blinks P6_7, completes the PIC handshake, logs "starting"/"host driver ready"/"starting tasks".

- [ ] **Step 9: Verify host tests still pass**

Run: `cargo test -p uac-host-firmware --target x86_64-unknown-linux-gnu --no-default-features`
Expected: PASS — the `mod tasks` / `mod firmware` code is `cfg(target_os = "none")`-gated, so the native build is unchanged and the 7 `sine` tests stay green.

- [ ] **Step 10: Commit**

```bash
git add firmwares/uac-host-firmware/src
git commit -m "feat(uac-host-firmware): executor bring-up + blink/pic tasks"
```

---

### Task 4: `uac_host_task` — enumerate → register → stream

Implements the full host audio loop and the shared state the OLED task reads.

**Files:**
- Replace: `firmwares/uac-host-firmware/src/tasks/uac_host.rs`

**Interfaces:**
- Consumes: `crate::sine::{Format, SineGen}`; `rza1l_hal::usb::Rusb1HostDriver::is_connected()`; `embassy_usb_host::{bus, BusState, handler::BusRoute, class::uac::{UacHandler, descriptors::FormatTypeDescriptor}}`.
- Produces (read by Task 5): module statics `STAGE: AtomicU8`, `VID/PID: AtomicU16`, `SAMPLE_RATE/PACKETS: AtomicU32`, `CHANNELS/BITS: AtomicU8`, and the `STAGE_*` constants (`STAGE_WAITING=0`, `STAGE_ENUMERATING=1`, `STAGE_CONNECTED=2`, `STAGE_STREAMING=3`, `STAGE_ERROR=4`), all `pub(crate)`.

- [ ] **Step 1: Replace `src/tasks/uac_host.rs` with the full implementation**

```rust
//! USB host audio loop: enumerate a class-compliant USB audio output device,
//! register it, and stream a 440 Hz tone to its isochronous OUT endpoint.

use core::sync::atomic::{AtomicU8, AtomicU16, AtomicU32, Ordering};

use embassy_usb_host::class::uac::{descriptors::FormatTypeDescriptor, UacHandler};
use embassy_usb_host::handler::BusRoute;
use embassy_usb_host::{bus, BusState};
use log::{error, info};

use rza1l_hal::usb::Rusb1HostDriver;

use crate::sine::{Format, SineGen};

// ── Shared UI state (read by the OLED task) ──────────────────────────────
pub(crate) const STAGE_WAITING: u8 = 0;
pub(crate) const STAGE_ENUMERATING: u8 = 1;
pub(crate) const STAGE_CONNECTED: u8 = 2;
pub(crate) const STAGE_STREAMING: u8 = 3;
pub(crate) const STAGE_ERROR: u8 = 4;

pub(crate) static STAGE: AtomicU8 = AtomicU8::new(STAGE_WAITING);
pub(crate) static VID: AtomicU16 = AtomicU16::new(0);
pub(crate) static PID: AtomicU16 = AtomicU16::new(0);
pub(crate) static SAMPLE_RATE: AtomicU32 = AtomicU32::new(0);
pub(crate) static CHANNELS: AtomicU8 = AtomicU8::new(0);
pub(crate) static BITS: AtomicU8 = AtomicU8::new(0);
pub(crate) static PACKETS: AtomicU32 = AtomicU32::new(0);

const TONE_HZ: f32 = 440.0;

#[embassy_executor::task]
pub(crate) async fn uac_host_task(driver: Rusb1HostDriver) {
    info!("uac_host_task: running (USB0 host mode)");
    // Bus-wide state (address allocator + enumeration lock); `'static` so it
    // satisfies `bus()`.
    static BUS_STATE: BusState = BusState::new();
    let (mut controller, handle) = bus(driver, &BUS_STATE);
    let mut cfg_buf = [0u8; 512];

    loop {
        STAGE.store(STAGE_WAITING, Ordering::Relaxed);
        info!("UAC host: waiting for device…");
        let speed = controller.wait_for_connection().await;
        info!("UAC host: connected ({:?})", speed);
        STAGE.store(STAGE_ENUMERATING, Ordering::Relaxed);

        let (enum_info, _num_cfg) =
            match handle.enumerate(BusRoute::Direct(speed), &mut cfg_buf).await {
                Ok(v) => v,
                Err(e) => {
                    error!("UAC host: enumeration failed: {:?}", e);
                    STAGE.store(STAGE_ERROR, Ordering::Relaxed);
                    continue;
                }
            };
        let addr = enum_info.device_address;
        let vid = enum_info.device_desc.vendor_id;
        let pid = enum_info.device_desc.product_id;
        VID.store(vid, Ordering::Relaxed);
        PID.store(pid, Ordering::Relaxed);
        info!("UAC host: VID={:04x} PID={:04x} addr={}", vid, pid, addr);

        match UacHandler::try_register(&handle, enum_info).await {
            Ok(mut uac) => {
                // Snapshot the negotiated format from the streaming descriptors.
                // Copy primitives out so the immutable borrow of `uac` ends
                // before the `&mut uac` sampling-frequency request below.
                let (channels, subslot, bits, max_pkt, has_fb);
                {
                    let iface = uac.output_interface();
                    channels = iface.class_descriptor.num_channels;
                    (subslot, bits) = match &iface.format_type_descriptor {
                        Some(FormatTypeDescriptor::I(f)) => (f.subslot_size, f.bit_resolution),
                        _ => (0, 0),
                    };
                    max_pkt = iface
                        .endpoint_descriptor
                        .as_ref()
                        .map(|e| e.max_packet_size)
                        .unwrap_or(0);
                    has_fb = uac.feedback_channel.is_some();
                    info!(
                        "UAC host: input-term id={} type={:?} clock={} | out {}ch {}B/{}bit maxpkt={} feedback={}",
                        uac.input_terminal().terminal_id(),
                        uac.input_terminal().terminal_type(),
                        uac.input_terminal().clock_source_id(),
                        channels,
                        subslot,
                        bits,
                        max_pkt,
                        has_fb,
                    );
                }

                let clock_id = uac.input_terminal().clock_source_id();
                let sample_rate = match uac.get_sampling_freq(clock_id).await {
                    Ok(sr) => sr,
                    Err(e) => {
                        error!("UAC host: get_sampling_freq failed: {:?}", e);
                        STAGE.store(STAGE_ERROR, Ordering::Relaxed);
                        handle.free_address(addr);
                        continue;
                    }
                };
                info!("UAC host: sample_rate={} Hz", sample_rate);

                let fmt = Format { sample_rate, channels, bytes_per_sample: subslot };
                SAMPLE_RATE.store(sample_rate, Ordering::Relaxed);
                CHANNELS.store(channels, Ordering::Relaxed);
                BITS.store(bits, Ordering::Relaxed);
                PACKETS.store(0, Ordering::Relaxed);
                STAGE.store(STAGE_CONNECTED, Ordering::Relaxed);

                let mut out = match uac.output().await {
                    Ok(o) => o,
                    Err(e) => {
                        error!("UAC host: output() failed: {:?}", e);
                        STAGE.store(STAGE_ERROR, Ordering::Relaxed);
                        handle.free_address(addr);
                        continue;
                    }
                };

                info!("UAC host: streaming {} Hz tone…", TONE_HZ);
                STAGE.store(STAGE_STREAMING, Ordering::Relaxed);
                let mut osc = SineGen::new(TONE_HZ, fmt);
                let mut pkts: u32 = 0;
                let r = out
                    .output_stream(
                        || controller.controller().is_connected(),
                        |buf| {
                            osc.fill(buf);
                            pkts = pkts.wrapping_add(1);
                            PACKETS.store(pkts, Ordering::Relaxed);
                        },
                    )
                    .await;
                info!("UAC host: stream ended: {:?}", r);
            }
            Err(e) => {
                error!("UAC host: not a supported UAC output device: {:?}", e);
                STAGE.store(STAGE_ERROR, Ordering::Relaxed);
            }
        }

        handle.free_address(addr);
    }
}
```

- [ ] **Step 2: Verify the firmware links**

Run: `cargo build-uac`
Expected: compiles and links with no errors.

> If the compiler reports that `(subslot, bits) = match …` (destructuring assignment) is unstable or rejected on this toolchain, replace that line with a `let (subslot, bits) = match … ;` binding and remove `subslot, bits` from the outer `let (...)` declaration; keep `channels`, `max_pkt`, `has_fb` as shown. Re-run `cargo build-uac`.

- [ ] **Step 3: Verify host tests still pass**

Run: `cargo test -p uac-host-firmware --target x86_64-unknown-linux-gnu --no-default-features`
Expected: PASS — 7 `sine` tests (this file is `cfg(target_os = "none")`-gated out natively).

- [ ] **Step 4: Commit**

```bash
git add firmwares/uac-host-firmware/src/tasks/uac_host.rs
git commit -m "feat(uac-host-firmware): UAC enumerate + register + tone stream"
```

---

### Task 5: OLED status display + final validation

Renders the shared connection/format state to the panel and validates the whole build.

**Files:**
- Replace: `firmwares/uac-host-firmware/src/tasks/oled.rs`

**Interfaces:**
- Consumes: `deluge_bsp::oled::{self, text}`, `deluge_bsp::pic`; `crate::tasks::uac_host::{STAGE, STAGE_WAITING, STAGE_ENUMERATING, STAGE_CONNECTED, STAGE_STREAMING, STAGE_ERROR, VID, PID, SAMPLE_RATE, CHANNELS, BITS, PACKETS}`.

- [ ] **Step 1: Replace `src/tasks/oled.rs` with the status display**

```rust
//! OLED status display for the UAC host test firmware.
//!
//! ```text
//! UAC HOST            UAC HOST            UAC HOST
//! waiting for device  1234:5678           1234:5678
//!                     44100Hz 2ch 24bit   44100Hz 2ch 24bit
//!                                         streaming 1234pk
//! ```

use core::sync::atomic::Ordering;

use embassy_time::Timer;

use deluge_bsp::oled::{self, text};
use deluge_bsp::pic;

use crate::tasks::uac_host::{
    BITS, CHANNELS, PACKETS, PID, SAMPLE_RATE, STAGE, STAGE_CONNECTED, STAGE_ENUMERATING,
    STAGE_ERROR, STAGE_STREAMING, STAGE_WAITING, VID,
};

/// Refresh interval.
const INTERVAL_MS: u64 = 250;
/// First on-screen pixel row (top rows sit off the visible area).
const TOP: usize = 10;

#[embassy_executor::task]
pub(crate) async fn oled_task() {
    // oled::init() drives the panel over RSPI0 and waits on the PIC
    // chip-select echo, so the PIC handshake must finish first.
    pic::wait_ready().await;
    oled::init().await;

    let mut fb = oled::FrameBuffer::new();
    let mut line = [0u8; 24];

    loop {
        fb.fill(0x00);
        text::draw_str(&mut fb, 0, TOP, b"UAC HOST");

        match STAGE.load(Ordering::Relaxed) {
            STAGE_WAITING => {
                text::draw_str(&mut fb, 0, TOP + 14, b"waiting for device");
            }
            STAGE_ENUMERATING => {
                text::draw_str(&mut fb, 0, TOP + 14, b"enumerating...");
            }
            stage @ (STAGE_CONNECTED | STAGE_STREAMING) => {
                let n = fmt_vidpid(
                    &mut line,
                    VID.load(Ordering::Relaxed),
                    PID.load(Ordering::Relaxed),
                );
                text::draw_str(&mut fb, 0, TOP + 12, &line[..n]);

                let n = fmt_format(
                    &mut line,
                    SAMPLE_RATE.load(Ordering::Relaxed),
                    CHANNELS.load(Ordering::Relaxed),
                    BITS.load(Ordering::Relaxed),
                );
                text::draw_str(&mut fb, 0, TOP + 24, &line[..n]);

                if stage == STAGE_STREAMING {
                    let n = fmt_pkts(&mut line, PACKETS.load(Ordering::Relaxed));
                    text::draw_str(&mut fb, 0, TOP + 36, &line[..n]);
                }
            }
            STAGE_ERROR => {
                text::draw_str(&mut fb, 0, TOP + 14, b"error - see RTT");
            }
            _ => {}
        }

        oled::send_frame(&fb).await;
        Timer::after_millis(INTERVAL_MS).await;
    }
}

/// `"<vid>:<pid>"` in lowercase hex, 4 digits each. Returns the length written.
fn fmt_vidpid(out: &mut [u8], vid: u16, pid: u16) -> usize {
    let mut p = 0;
    push_hex4(out, &mut p, vid);
    push(out, &mut p, b':');
    push_hex4(out, &mut p, pid);
    p
}

/// `"<rate>Hz <ch>ch <bits>bit"`. Returns the length written.
fn fmt_format(out: &mut [u8], rate: u32, channels: u8, bits: u8) -> usize {
    let mut p = 0;
    push_u32(out, &mut p, rate);
    for &b in b"Hz " {
        push(out, &mut p, b);
    }
    push_u32(out, &mut p, channels as u32);
    for &b in b"ch " {
        push(out, &mut p, b);
    }
    push_u32(out, &mut p, bits as u32);
    for &b in b"bit" {
        push(out, &mut p, b);
    }
    p
}

/// `"streaming <n>pk"`. Returns the length written.
fn fmt_pkts(out: &mut [u8], packets: u32) -> usize {
    let mut p = 0;
    for &b in b"streaming " {
        push(out, &mut p, b);
    }
    push_u32(out, &mut p, packets);
    for &b in b"pk" {
        push(out, &mut p, b);
    }
    p
}

#[inline]
fn push(out: &mut [u8], p: &mut usize, b: u8) {
    if *p < out.len() {
        out[*p] = b;
        *p += 1;
    }
}

fn push_hex4(out: &mut [u8], p: &mut usize, v: u16) {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    for shift in [12u32, 8, 4, 0] {
        push(out, p, HEX[((v >> shift) & 0xF) as usize]);
    }
}

fn push_u32(out: &mut [u8], p: &mut usize, mut v: u32) {
    if v == 0 {
        push(out, p, b'0');
        return;
    }
    let mut tmp = [0u8; 10];
    let mut i = 0;
    while v > 0 {
        tmp[i] = b'0' + (v % 10) as u8;
        v /= 10;
        i += 1;
    }
    while i > 0 {
        i -= 1;
        push(out, p, tmp[i]);
    }
}
```

- [ ] **Step 2: Verify the firmware links (debug + RTT)**

Run: `cargo build-uac`
Expected: compiles and links with no errors.

- [ ] **Step 3: Verify the release build links (RTT disabled)**

Run: `cargo build-fw-rel -p uac-host-firmware`
Expected: compiles and links with `--release --no-default-features` (no `rtt`).

- [ ] **Step 4: Verify host tests still pass**

Run: `cargo test -p uac-host-firmware --target x86_64-unknown-linux-gnu --no-default-features`
Expected: PASS — 7 `sine` tests.

- [ ] **Step 5: Commit**

```bash
git add firmwares/uac-host-firmware/src/tasks/oled.rs
git commit -m "feat(uac-host-firmware): OLED host status + format display"
```

- [ ] **Step 6: On-hardware validation (manual — user-driven)**

Flash via the user's dev-mode flow and capture RTT. Two checks:

1. **No device plugged in:** RTT shows `UAC host test firmware starting` → `USB: host driver ready` → `IRQ enabled — starting tasks` → `UAC host: waiting for device…`; the OLED shows `UAC HOST` / `waiting for device`; P6_7 blinks the heartbeat. This confirms host bring-up + ISR/executor liveness with no USB traffic.
2. **With a class-compliant USB DAC:** RTT shows the connect/enumerate lines, the `input-term … | out Nch …` topology dump, `sample_rate=… Hz`, then `streaming 440 Hz tone…`; the OLED shows `VID:PID`, the `…Hz …ch …bit` format line, and a rising `streaming <n>pk` counter; the DAC plays a clean 440 Hz tone. Unplugging logs `stream ended: Ok(())`/`DeviceDisconnected` and returns to `waiting for device`.

This step has no automated gate; record the observed RTT lines in the PR description.

---

## Notes for the implementer

- **Why the `cfg(target_os = "none")` split:** the default build target is `armv7a-none-eabihf`. `embassy-executor` (`platform-cortex-ar`) and `cortex-ar` do not build for `x86_64`, so they live under `[target.'cfg(target_os = "none")'.dependencies]`. Native `cargo test` (target `x86_64`, `--no-default-features`) then compiles only `log` + `libm` + the `sine` module, matching how `rza1l-hal` host-tests.
- **`bus()` / `UacHandler` ownership:** `bus(driver, &BUS_STATE)` returns `(BusController, BusHandle)`. `BusHandle` implements `UsbHostAllocator`, so `&handle` is passed straight to `UacHandler::try_register`. `controller` and `handle` are independent, so the `|| controller.controller().is_connected()` closure can borrow `controller` while `out.output_stream(...)` runs.
- **Device address:** captured (`let addr = enum_info.device_address;`) before `try_register` consumes `enum_info`; freed on every loop exit.
