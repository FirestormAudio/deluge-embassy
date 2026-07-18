# UAC Host — Validation Firmware (Phase 3) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a dedicated host-mode firmware that runs `usb_host_supervisor` (auto-binds our `host::uac` driver), loops captured USB audio back to playback, and shows status + input level on OLED/RTT — the first real-hardware exercise of the Phase 1–2 host UAC stack.

**Architecture:** New `firmwares/uac-host-firmware` (msc-firmware pattern: `#![no_std]`/`#![no_main]`, RTT, `build.rs` + memory scripts). Host-only USB: `main` brings up the platform, wires the USB0 ISR to `hcd_int_handler(0)`, calls `init_host_mode(0)`, and spawns `deluge_bsp::usb::host::usb_host_supervisor(hd, spawner)` plus app tasks. The supervisor does enumeration + UAC binding; the firmware's `loopback` task just calls `capture_read` → `playback_write`, and `oled` renders shared status. One small `deluge-bsp` addition exposes `playback_channels()`.

**Tech Stack:** Rust (edition 2024), `embassy-executor`/`embassy-time`, `rza1l-hal` (host driver, host-mode init), `deluge-bsp` (USB host supervisor + UAC app API, OLED, PIC), `deluge-alloc`, RTT.

## Global Constraints

- **Design of record:** `docs/superpowers/specs/2026-07-17-uac-host-validation-firmware-design.md`. Builds on the merged-in-spirit Phase 1 capture + Phase 2 playback (`deluge_bsp::usb::host::uac`) on branch `feat/usb-uac-host`.
- **Host-only USB.** No device-mode `embassy_usb::Builder`, no descriptor statics. The supervisor (`deluge_bsp::usb::host::usb_host_supervisor(driver: Rusb1HostDriver, spawner: Spawner)`) owns enumeration + class binding.
- **App API (device-gated, from `deluge_bsp::usb::host::uac`):** `capture_channels() -> u8` (0 until a UAC device binds), `capture_read(&mut [f32]) -> usize` (interleaved f32, `capture_channels` wide, short-returns on underrun), `playback_write(&[f32]) -> usize` (short-returns when the ring is full, never blocks). Task 1 adds `playback_channels() -> u8`.
- **Never block:** `capture_read`/`playback_write` short-return; the loopback task batches with a short `Timer`, never busy-spins.
- **Bare-metal target** `armv7a-none-eabihf`; device build is `cargo build-uac` (adds `-Zbuild-std=core`), release `cargo build-fw-rel -p uac-host-firmware` (`--no-default-features`). The firmware is bring-up glue with no host-testable pure logic; verification is the device build + the on-hardware checklist.
- **RTT** via the `rtt` feature (default on); release disables it (`--no-default-features`).
- 32-bit `usize` on device.
- **Cross-branch:** both iso pipes are claimed by full-duplex UAC; the `TX_BULK_PIPE=1` collision (`feat/usb-midi-tx-multiplex`) remains a merge gate — not touched here.

---

### Task 1: Expose `playback_channels()`

The loopback task needs the playback channel count to adapt when it differs from capture. Mirror `capture_channels`.

**Files:**
- Modify: `crates/deluge-bsp/src/usb/host/uac/mod.rs`

**Interfaces:**
- Produces: `deluge_bsp::usb::host::uac::playback_channels() -> u8` (0 if capture-only / no device).

- [ ] **Step 1: Add the accessor in the `shared` bridge**

In the `#[cfg(target_os = "none")] pub(crate) mod shared` block, next to the playback helpers, add:

```rust
    /// Playback channels the hosted device declared, or 0 if none / capture-only.
    pub fn playback_channels() -> u8 {
        PLAYBACK.lock(|c| c.borrow().channels)
    }
```

- [ ] **Step 2: Re-export it**

Extend the device-gated re-export:

```rust
pub use shared::{capture_channels, capture_read, playback_channels, playback_write};
```

- [ ] **Step 3: Verify**

Run: `cargo build-fw -p deluge-bsp` (device build links) and `cargo test -p deluge-bsp --lib --target armv7-unknown-linux-gnueabihf` (all 131 host tests still pass; this is device-gated code).
Expected: links; PASS.

- [ ] **Step 4: Commit**

```bash
git add crates/deluge-bsp/src/usb/host/uac/mod.rs
git commit -m "feat(deluge-bsp): host::uac expose playback_channels()"
```

---

### Task 2: Scaffold the firmware — bring-up + supervisor + blink/pic

Create the crate; `main` brings up the platform and spawns the supervisor + tasks. `loopback`/`oled` start as minimal stubs so it links and boots.

**Files:**
- Create: `firmwares/uac-host-firmware/{Cargo.toml, build.rs, memory.x, memory_rtt.x}`
- Create: `firmwares/uac-host-firmware/src/main.rs`
- Create: `firmwares/uac-host-firmware/src/tasks/{mod,blink,pic,oled,loopback}.rs`
- Modify: `Cargo.toml` (workspace `members`), `.cargo/config.toml` (aliases)

**Interfaces:**
- Consumes: `rza1l_hal::usb::{hcd_int_handler, init_host_mode, USB0_IRQ}`, `deluge_bsp::usb::host::usb_host_supervisor`.
- Produces: `crate::tasks::{blink::blink_task, pic::pic_task, oled::oled_task, loopback::loopback_task}` (all `#[embassy_executor::task]`).

- [ ] **Step 1: `Cargo.toml`**

```toml
[package]
publish = false
name = "uac-host-firmware"
version = "0.1.0"
edition = "2024"
license.workspace = true

[[bin]]
name = "uac-host-firmware"
test = false

[dependencies]
rza1l-hal = { path = "../../crates/rza1l-hal" }
deluge-bsp = { path = "../../crates/deluge-bsp" }
deluge-alloc = { path = "../../crates/deluge-alloc" }
cortex-ar = { workspace = true }
embassy-executor = { workspace = true, features = ["nightly", "platform-cortex-ar", "executor-thread"] }
embassy-time = { workspace = true, features = ["tick-hz-1_000_000"] }
embassy-sync = { workspace = true }
embassy-futures = { workspace = true }
log = { workspace = true }
rtt-target = { workspace = true, optional = true }

[features]
default = ["rtt"]
rtt = ["dep:rtt-target", "rza1l-hal/rtt", "deluge-bsp/rtt"]
```

> Confirm `deluge-bsp` exposes an `rtt` feature (the demo/controller firmwares enable `deluge/rtt` via the SDK; deluge-bsp's own feature name may differ). If `deluge-bsp/rtt` doesn't exist, drop it — `rza1l-hal/rtt` + `dep:rtt-target` is what defines the RTT control block.

- [ ] **Step 2: `build.rs`, `memory.x`, `memory_rtt.x` (verbatim from msc-firmware)**

```bash
cp firmwares/msc-firmware/build.rs      firmwares/uac-host-firmware/build.rs
cp firmwares/msc-firmware/memory.x      firmwares/uac-host-firmware/memory.x
cp firmwares/msc-firmware/memory_rtt.x  firmwares/uac-host-firmware/memory_rtt.x
```

- [ ] **Step 3: `src/main.rs`**

```rust
//! Deluge UAC host **validation** firmware.
//!
//! Brings USB0 up in host mode, runs `usb_host_supervisor` (which auto-binds the
//! `deluge_bsp::usb::host::uac` driver on connect), and loops captured audio back
//! to playback. First real-hardware exercise of the host UAC capture+playback
//! stack. See `docs/superpowers/specs/2026-07-17-uac-host-validation-firmware-design.md`.

#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

mod tasks;

use core::mem::MaybeUninit;
use core::panic::PanicInfo;

use embassy_executor::{Executor, Spawner};
use log::{error, info};

use deluge_alloc as allocator;
use deluge_bsp::{cv_gate, uart as bsp_uart};
use rza1l_hal::gic;
use rza1l_hal::usb::{hcd_int_handler, init_host_mode};

unsafe extern "C" {
    static __sram_heap_start: u8;
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
    info!("UAC host validation firmware starting");

    unsafe {
        let start = core::ptr::addr_of!(__sram_heap_start) as *mut u8;
        let size = core::ptr::addr_of!(__sram_heap_end) as usize - start as usize;
        allocator::SRAM.init(start, size);
    }
    unsafe { deluge_bsp::system::init_clocks() };
    unsafe { allocator::SDRAM.init(0x0C00_0000 as *mut u8, 64 * 1024 * 1024) };

    unsafe { rza1l_hal::gpio::set_as_output(6, 7) };
    unsafe { bsp_uart::init_pic(31_250) };
    unsafe { cv_gate::init() };

    // USB0 host-mode ISR.
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
        spawner.spawn(tasks::loopback::loopback_task().unwrap());
        // The supervisor enumerates + binds the UAC driver, which spawns the
        // capture/playback pump. It needs the spawner to spawn that child task.
        spawner
            .spawn(deluge_bsp::usb::host::usb_host_supervisor(host_driver, spawner))
            .unwrap();
    })
}
```

> `Spawner` is `Copy`, so passing it into `usb_host_supervisor` and still using it for the other spawns is fine. If `usb_host_supervisor`'s spawn ordering matters (it shouldn't — it awaits a connection first), spawn it last as shown.

- [ ] **Step 4: `src/tasks/mod.rs`**

```rust
pub(crate) mod blink;
pub(crate) mod loopback;
pub(crate) mod oled;
pub(crate) mod pic;
```

- [ ] **Step 5: `src/tasks/blink.rs` and `src/tasks/pic.rs` (verbatim from msc-firmware)**

Copy `firmwares/msc-firmware/src/tasks/blink.rs` and `firmwares/msc-firmware/src/tasks/pic.rs` unchanged (P6_7 heartbeat; PIC baud handshake + OLED chip-select echo).

- [ ] **Step 6: Stub `src/tasks/oled.rs` and `src/tasks/loopback.rs`**

```rust
// oled.rs
use embassy_time::Timer;
/// Placeholder; the status display is implemented in Task 4.
#[embassy_executor::task]
pub(crate) async fn oled_task() {
    loop {
        Timer::after_millis(1000).await;
    }
}
```

```rust
// loopback.rs
use embassy_time::Timer;
/// Placeholder; the capture->playback loop is implemented in Task 3.
#[embassy_executor::task]
pub(crate) async fn loopback_task() {
    loop {
        Timer::after_millis(1000).await;
    }
}
```

- [ ] **Step 7: Workspace member + cargo aliases**

In workspace `Cargo.toml` `members`, after `"firmwares/wp-probe",`:
```toml
  "firmwares/uac-host-firmware",
```
In `.cargo/config.toml`, after the `build-msc-bin` line:
```toml
build-uac            = "build -Zbuild-std=core -Zbuild-std-features=compiler-builtins-mem -p uac-host-firmware"
build-uac-bin        = "objcopy -Zbuild-std=core -Zbuild-std-features=compiler-builtins-mem --release --no-default-features -p uac-host-firmware -- -O binary target/armv7a-none-eabihf/release/uac-host-firmware.bin"
```

- [ ] **Step 8: Verify the firmware links**

Run: `cargo build-uac`
Expected: compiles and links — boots host mode, blinks P6_7, logs "starting"/"host driver ready"/"starting tasks", and the supervisor waits for a device.

- [ ] **Step 9: Commit**

```bash
git add firmwares/uac-host-firmware Cargo.toml .cargo/config.toml
git commit -m "feat(uac-host-firmware): host bring-up + supervisor + blink/pic scaffold"
```

---

### Task 3: Loopback task — capture → playback + stats

The core app task: drain captured frames, meter, remap channels, write to playback, publish status.

**Files:**
- Replace: `firmwares/uac-host-firmware/src/tasks/loopback.rs`

**Interfaces:**
- Consumes: `deluge_bsp::usb::host::uac::{capture_channels, capture_read, playback_channels, playback_write}`.
- Produces (read by Task 4): `loopback::{STAGE, STAGE_WAITING, STAGE_STREAMING, CAP_CH, PLAY_CH, PEAK_MILLI, IN_FPS, OUT_FPS}` (`pub(crate)` atomics/consts).

- [ ] **Step 1: Implement `loopback.rs`**

```rust
//! Capture -> playback loopback + metering.
//!
//! Reads captured interleaved `f32` frames from the hosted UAC device and writes
//! them straight back to playback (round-trip), adapting channel counts and
//! publishing a status snapshot for the OLED task.

use core::sync::atomic::{AtomicU8, AtomicU32, Ordering};

use embassy_time::{Instant, Timer};
use log::info;

use deluge_bsp::usb::host::uac;

// ── Shared status (read by the OLED task) ────────────────────────────────
pub(crate) const STAGE_WAITING: u8 = 0;
pub(crate) const STAGE_STREAMING: u8 = 1;

pub(crate) static STAGE: AtomicU8 = AtomicU8::new(STAGE_WAITING);
pub(crate) static CAP_CH: AtomicU8 = AtomicU8::new(0);
pub(crate) static PLAY_CH: AtomicU8 = AtomicU8::new(0);
/// Input peak over the last window, in milli-units of full scale (0..=1000).
pub(crate) static PEAK_MILLI: AtomicU32 = AtomicU32::new(0);
pub(crate) static IN_FPS: AtomicU32 = AtomicU32::new(0);
pub(crate) static OUT_FPS: AtomicU32 = AtomicU32::new(0);

const MAX_CH: usize = 8;
/// Scratch frame budget per read (samples). 256 frames * 8ch.
const SCRATCH: usize = 256 * MAX_CH;

/// Peak absolute value over a sample slice, as milli-full-scale (0..=1000).
fn peak_milli(samples: &[f32]) -> u32 {
    let mut peak = 0.0f32;
    for &s in samples {
        let a = if s < 0.0 { -s } else { s };
        if a > peak {
            peak = a;
        }
    }
    (peak.min(1.0) * 1000.0) as u32
}

/// Remap `in_` (interleaved, `cap_ch`) into `out` (interleaved, `play_ch`),
/// returning samples written. `cap_ch == play_ch` copies through; mono capture
/// duplicates to all playback channels; otherwise copies `min(cap_ch, play_ch)`
/// channels per frame and zero-fills the rest.
fn remap(cap_ch: usize, play_ch: usize, in_: &[f32], out: &mut [f32]) -> usize {
    let frames = in_.len() / cap_ch;
    let mut w = 0;
    for f in 0..frames {
        let src = &in_[f * cap_ch..f * cap_ch + cap_ch];
        for c in 0..play_ch {
            let v = if cap_ch == 1 {
                src[0]
            } else if c < cap_ch {
                src[c]
            } else {
                0.0
            };
            out[w] = v;
            w += 1;
        }
    }
    w
}

#[embassy_executor::task]
pub(crate) async fn loopback_task() {
    info!("loopback: running");
    let mut scratch = [0.0f32; SCRATCH];
    let mut out = [0.0f32; SCRATCH];
    let mut in_frames: u32 = 0;
    let mut out_frames: u32 = 0;
    let mut window_peak: u32 = 0;
    let mut last_stats = Instant::now();
    let mut was_connected = false;

    loop {
        let cap_ch = uac::capture_channels() as usize;
        let play_ch = uac::playback_channels() as usize;

        if cap_ch == 0 {
            if was_connected {
                info!("loopback: device disconnected");
                was_connected = false;
            }
            STAGE.store(STAGE_WAITING, Ordering::Relaxed);
            CAP_CH.store(0, Ordering::Relaxed);
            PLAY_CH.store(0, Ordering::Relaxed);
            Timer::after_millis(100).await;
            continue;
        }
        if !was_connected {
            info!("loopback: device connected cap={}ch play={}ch", cap_ch, play_ch);
            was_connected = true;
        }
        STAGE.store(STAGE_STREAMING, Ordering::Relaxed);
        CAP_CH.store(cap_ch as u8, Ordering::Relaxed);
        PLAY_CH.store(play_ch as u8, Ordering::Relaxed);

        // Bound the read so both the input (`scratch`) and remap's output
        // (`frames * play_ch`, into `out`) stay within SCRATCH. Whole cap-frames
        // only, so the read stays frame-aligned. Capture-only (`play_ch == 0`)
        // skips remap/playback below, so clamp it to 1 just for this sizing.
        let out_ch = play_ch.max(1);
        let max_frames = (SCRATCH / cap_ch).min(SCRATCH / out_ch);
        let budget = max_frames * cap_ch;
        let n = uac::capture_read(&mut scratch[..budget]);
        if n > 0 {
            let p = peak_milli(&scratch[..n]);
            if p > window_peak {
                window_peak = p;
            }
            in_frames += (n / cap_ch) as u32;
            if play_ch > 0 {
                let w = remap(cap_ch, play_ch, &scratch[..n], &mut out);
                let written = uac::playback_write(&out[..w]);
                out_frames += (written / play_ch) as u32;
            }
        }

        // ~1 Hz stats window.
        if last_stats.elapsed().as_millis() >= 1000 {
            PEAK_MILLI.store(window_peak, Ordering::Relaxed);
            IN_FPS.store(in_frames, Ordering::Relaxed);
            OUT_FPS.store(out_frames, Ordering::Relaxed);
            info!(
                "loopback: in={}f/s out={}f/s peak={}m/1000 ({}ch->{}ch)",
                in_frames, out_frames, window_peak, cap_ch, play_ch
            );
            in_frames = 0;
            out_frames = 0;
            window_peak = 0;
            last_stats = Instant::now();
        }

        // Batch roughly one engine block; capture_read is non-blocking.
        Timer::after_micros(1000).await;
    }
}
```

> `remap`/`peak_milli` are trivial pure helpers; the whole task is bring-up glue, so no host tests (the acceptance test is the hardware run). If a channel-mismatch device misbehaves, the RTT `cap->play` counts make it visible.

- [ ] **Step 2: Verify the firmware links**

Run: `cargo build-uac`
Expected: compiles and links.

- [ ] **Step 3: Commit**

```bash
git add firmwares/uac-host-firmware/src/tasks/loopback.rs
git commit -m "feat(uac-host-firmware): capture->playback loopback task + metering"
```

---

### Task 4: OLED status display + build validation

Render the shared status; validate debug + release builds.

**Files:**
- Replace: `firmwares/uac-host-firmware/src/tasks/oled.rs`

**Interfaces:**
- Consumes: `deluge_bsp::oled::{self, text}`, `deluge_bsp::pic`; `crate::tasks::loopback::{STAGE, STAGE_WAITING, STAGE_STREAMING, CAP_CH, PLAY_CH, PEAK_MILLI, IN_FPS, OUT_FPS}`.

- [ ] **Step 1: Implement `oled.rs`**

```rust
//! OLED status for the UAC host validation firmware.
//!
//! ```text
//! UAC HOST                UAC HOST
//! waiting for device      44100Hz 2>2ch 24bit
//!                         loop  in ####----
//! ```

use core::sync::atomic::Ordering;

use embassy_time::Timer;

use deluge_bsp::oled::{self, text};
use deluge_bsp::pic;

use crate::tasks::loopback::{
    CAP_CH, PEAK_MILLI, PLAY_CH, STAGE, STAGE_STREAMING, STAGE_WAITING,
};

const INTERVAL_MS: u64 = 250;
const TOP: usize = 10;

#[embassy_executor::task]
pub(crate) async fn oled_task() {
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
            STAGE_STREAMING => {
                let cap = CAP_CH.load(Ordering::Relaxed);
                let play = PLAY_CH.load(Ordering::Relaxed);
                let n = fmt_format(&mut line, cap, play);
                text::draw_str(&mut fb, 0, TOP + 12, &line[..n]);

                // "loop  in <bar>" — 8-cell peak bar from PEAK_MILLI (0..=1000).
                let peak = PEAK_MILLI.load(Ordering::Relaxed);
                let n = fmt_meter(&mut line, peak);
                text::draw_str(&mut fb, 0, TOP + 26, &line[..n]);
            }
            _ => {}
        }

        oled::send_frame(&fb).await;
        Timer::after_millis(INTERVAL_MS).await;
    }
}

/// `"44100Hz <cap>><play>ch 24bit"`.
fn fmt_format(out: &mut [u8], cap: u8, play: u8) -> usize {
    let mut p = 0;
    for &b in b"44100Hz " {
        push(out, &mut p, b);
    }
    push_u8(out, &mut p, cap);
    push(out, &mut p, b'>');
    push_u8(out, &mut p, play);
    for &b in b"ch 24bit" {
        push(out, &mut p, b);
    }
    p
}

/// `"loop  in ########"` — an 8-cell bar filled proportional to `peak` (0..=1000).
fn fmt_meter(out: &mut [u8], peak: u32) -> usize {
    let mut p = 0;
    for &b in b"loop  in " {
        push(out, &mut p, b);
    }
    let filled = ((peak.min(1000) * 8) / 1000) as usize;
    for i in 0..8 {
        push(out, &mut p, if i < filled { b'#' } else { b'-' });
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

fn push_u8(out: &mut [u8], p: &mut usize, mut v: u8) {
    if v == 0 {
        push(out, p, b'0');
        return;
    }
    let mut tmp = [0u8; 3];
    let mut i = 0;
    while v > 0 {
        tmp[i] = b'0' + (v % 10);
        v /= 10;
        i += 1;
    }
    while i > 0 {
        i -= 1;
        push(out, p, tmp[i]);
    }
}
```

- [ ] **Step 2: Verify debug + release builds**

Run: `cargo build-uac` and `cargo build-fw-rel -p uac-host-firmware`
Expected: both link (release with `--no-default-features` = no RTT).

- [ ] **Step 3: Commit**

```bash
git add firmwares/uac-host-firmware/src/tasks/oled.rs
git commit -m "feat(uac-host-firmware): OLED loopback status + input meter"
```

- [ ] **Step 4: On-hardware validation (manual — user-driven)**

Flash via the dev-mode upload flow and capture RTT. Checks:

1. **No device:** RTT shows `UAC host validation firmware starting` → `USB: host driver ready` → `IRQ enabled — starting tasks`; OLED shows `UAC HOST` / `waiting for device`; P6_7 blinks.
2. **Full-duplex USB audio interface plugged in:** RTT shows the supervisor enumerate + bind the UAC device (`usb_host: UAC capture device …`), then `loopback: device connected cap=Nch play=Mch` and ~1 Hz `in=…f/s out=…f/s peak=…` lines. OLED shows `44100Hz N>Mch 24bit` and a moving input meter. Feeding audio into the interface's input is heard back out its output (round-trip), and the meter tracks it.
3. **Unplug:** RTT `loopback: device disconnected`; OLED reverts to `waiting for device`; **no hang** (this is the Phase-2 DTCH/detach concern — if the pump hangs here, capture the RTT and reconcile the HAL DTCH path per the Phase-2 note).

Record the observed RTT lines in the PR/commit description; this step is the acceptance test for the whole Phase 1–3 UAC host stack.

---

## Notes for the implementer

- **Order:** Task 1 (`playback_channels`) unblocks Task 3. Task 2 scaffolds + boots; Task 3 is the core loop; Task 4 is the display + build gate.
- **No host tests:** this firmware is bring-up glue over the already-host-tested `deluge-bsp` driver; `remap`/`peak_milli`/formatters are trivial and self-evident. The acceptance test is the on-hardware run (Task 4 Step 4).
- **Deferred:** automated HW testing, DAC-only playback, capture-as-engine-input, and any HAL DTCH reconciliation the hardware run turns out to require. Carry the `TX_BULK_PIPE=1` cross-branch gate forward.
