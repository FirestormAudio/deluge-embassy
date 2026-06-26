//! Deluge Wren scripting firmware.
//!
//! **M1:** a live-coding REPL over USB-CDC-ACM plus SD-card persistence, on top
//! of the M0 VM bring-up. The upstream wren C VM (compiled stock in `wren-sys`,
//! including its on-device compiler) runs in a single Embassy task that owns the
//! VM; its `System.print`/error output is bridged to the CDC endpoints through
//! two byte rings (mirroring `crow-firmware`).
//!
//! ## REPL protocol (line-oriented over CDC)
//! - A plain line is evaluated immediately in the `main` module (module-level
//!   `var`s persist across lines, so the REPL is stateful).
//! - `^^s` begins a multi-line script upload; subsequent lines accumulate.
//! - `^^e` ends the upload and runs the accumulated script (no save).
//! - `^^w` ends the upload, runs it, and persists it to `/MAIN.WREN`.
//! - `^^c` cancels/clears the accumulator.
//! - `^^v` prints a version banner.
//! - `^^m` prints the VM heap's SRAM/SDRAM spill split (live / peak per tier).
//!
//! `/MAIN.WREN` is loaded and run at boot.
//!
//! ## Tasks
//! - `usb_task` — drives the `embassy_usb` device state machine.
//! - `cdc_rx_task` — host → [`RX`] ring (bulk-OUT packets, newline-framed).
//! - `cdc_tx_task` — [`TX`] ring → host (bulk-IN packets).
//! - `vm_task` — owns the VM: drains the REPL, runs the boot script.
//! - `flash_task` — SD load at boot + persist on `^^w`.
//!
//! Later milestones add the `deluge` foreign bindings (CV/gate, MIDI, timing,
//! pads/encoders/OLED/LEDs) and the native DSP audio engine.

#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]
#![feature(impl_trait_in_assoc_type)]

use core::cell::RefCell;
use core::ffi::c_char;
use core::ptr::addr_of_mut;
use core::sync::atomic::{AtomicBool, AtomicPtr, AtomicU16, AtomicUsize, Ordering};

use deluge::deluge_bsp;
use deluge::prelude::*;
use embassy_futures::select::{Either, select};
use embassy_sync::blocking_mutex::{Mutex, raw::CriticalSectionRawMutex};
use embassy_time::{Duration, Instant, Timer};
use wren_sys::{Vm, WrenVM};

// The USB-CDC REPL transport is device-only — the desktop simulator has no USB,
// so the host build drives the REPL over stdin/stdout instead (see `host_repl`
// and `host_tx_task`). `rza1l_hal` is only re-exported by the SDK on the device.
#[cfg(target_os = "none")]
use deluge::rza1l_hal;
#[cfg(target_os = "none")]
use embassy_usb::class::cdc_acm::{CdcAcmClass, Receiver, Sender, State};
#[cfg(target_os = "none")]
use embassy_usb::{Builder, Config, UsbDevice};
#[cfg(target_os = "none")]
use rza1l_hal::usb::{Rusb1Driver, USB0_IRQ, dcd_int_handler, init_device_mode};

mod audio;
mod host;

// Boot, heaps, clocks, the executor, and the panic handler are all provided by
// `#[deluge::app]` (see `main` below); this crate owns product behaviour only.

/// Max persisted/uploaded script size.
const SCRIPT_MAX: usize = 16 * 1024;

// ── REPL byte rings ─────────────────────────────────────────────────────────

/// wren → host (bulk-IN). Filled by the host hooks, drained by `cdc_tx_task`.
static TX: Mutex<CriticalSectionRawMutex, RefCell<ByteRing<4096>>> =
    Mutex::new(RefCell::new(ByteRing::new()));
/// host → wren (bulk-OUT). Filled by `cdc_rx_task`, drained by `vm_task`.
static RX: Mutex<CriticalSectionRawMutex, RefCell<ByteRing<4096>>> =
    Mutex::new(RefCell::new(ByteRing::new()));
/// wren → MIDI DIN out. Filled by the `Midi.*` tx bindings (in `vm_task`),
/// drained by `midi_task`.
static MIDI_TX: Mutex<CriticalSectionRawMutex, RefCell<ByteRing<256>>> =
    Mutex::new(RefCell::new(ByteRing::new()));
/// MIDI DIN in → wren. Filled by `midi_task` (from the SDK `Midi` handle),
/// drained + parsed by `vm_task`.
static MIDI_RX: Mutex<CriticalSectionRawMutex, RefCell<ByteRing<256>>> =
    Mutex::new(RefCell::new(ByteRing::new()));

// ── UI (pads/buttons/encoders in; LEDs/OLED out) ─────────────────────────────

/// Input events (`input_task` → `vm_task`), 3 bytes each: `[kind, a, b]`.
/// kind 0/1 = pad press/release (`a`=x, `b`=y); 2/3 = button press/release
/// (`a`=id); 4 = encoder (`a`=index, `b`=delta as `i8`).
static INPUT_EVENTS: Mutex<CriticalSectionRawMutex, RefCell<ByteRing<256>>> =
    Mutex::new(RefCell::new(ByteRing::new()));
/// LED commands (`vm_task` bindings → `ui_task`), 2 bytes each: `[id, on]`.
static UI_LED: Mutex<CriticalSectionRawMutex, RefCell<ByteRing<128>>> =
    Mutex::new(RefCell::new(ByteRing::new()));
/// OLED frame buffer: drawn by the `Oled.*` bindings (vm_task), rendered by
/// `ui_task` on `OLED_SHOW`.
static OLED_FB: Mutex<CriticalSectionRawMutex, RefCell<deluge_bsp::oled::FrameBuffer>> =
    Mutex::new(RefCell::new(deluge_bsp::oled::FrameBuffer::new()));
static OLED_SHOW: AtomicBool = AtomicBool::new(false);

// ── VM handle + persistence handshakes ──────────────────────────────────────

/// The live VM, published by `main` once booted; consumed by `vm_task`.
static VM_PTR: AtomicPtr<WrenVM> = AtomicPtr::new(core::ptr::null_mut());

/// Boot script staged by `flash_task` (read from `/MAIN.WREN`), run once by
/// `vm_task`. NUL space reserved at the end for in-place C-string termination.
static mut BOOT_BUF: [u8; SCRIPT_MAX + 1] = [0; SCRIPT_MAX + 1];
static BOOT_LEN: AtomicUsize = AtomicUsize::new(0);
static BOOT_PENDING: AtomicBool = AtomicBool::new(false);

/// Save buffer filled by `vm_task` on `^^w`, written to `/MAIN.WREN` by
/// `flash_task`.
static mut SAVE_BUF: [u8; SCRIPT_MAX] = [0; SCRIPT_MAX];
static SAVE_LEN: AtomicUsize = AtomicUsize::new(0);
static SAVE_PENDING: AtomicBool = AtomicBool::new(false);

/// Simple SPSC byte ring (one producer task, one consumer) guarded by a
/// critical-section mutex so the host hooks and the USB tasks can share it.
/// (Lifted verbatim from `crow-firmware`.)
struct ByteRing<const N: usize> {
    buf: [u8; N],
    head: usize,
    tail: usize,
    full: bool,
}

impl<const N: usize> ByteRing<N> {
    const fn new() -> Self {
        Self { buf: [0; N], head: 0, tail: 0, full: false }
    }
    #[allow(dead_code)] // part of the lifted ring API; used by `space`
    fn len(&self) -> usize {
        if self.full {
            N
        } else if self.tail >= self.head {
            self.tail - self.head
        } else {
            N - self.head + self.tail
        }
    }
    #[allow(dead_code)] // part of the lifted ring API
    fn space(&self) -> usize {
        N - self.len()
    }
    /// Push as many bytes as fit; returns the count written.
    fn push(&mut self, data: &[u8]) -> usize {
        let mut n = 0;
        for &b in data {
            if self.full {
                break;
            }
            self.buf[self.tail] = b;
            self.tail = (self.tail + 1) % N;
            if self.tail == self.head {
                self.full = true;
            }
            n += 1;
        }
        n
    }
    /// Pop up to `dst.len()` bytes; returns the count read.
    fn pop(&mut self, dst: &mut [u8]) -> usize {
        let mut n = 0;
        while n < dst.len() {
            if self.head == self.tail && !self.full {
                break;
            }
            dst[n] = self.buf[self.head];
            self.head = (self.head + 1) % N;
            self.full = false;
            n += 1;
        }
        n
    }
}

/// Push bytes toward the host REPL (best-effort; drops if the ring is full).
fn tx_push(data: &[u8]) {
    TX.lock(|r| r.borrow_mut().push(data));
}

/// Tiny fixed-capacity `core::fmt::Write` sink for formatting short REPL replies
/// (e.g. the `^^m` heap report) without an allocator. Overflow is silently
/// truncated — replies are well under the buffer.
struct TxFmt {
    buf: [u8; 128],
    len: usize,
}

impl TxFmt {
    fn new() -> Self {
        Self { buf: [0; 128], len: 0 }
    }
    fn as_bytes(&self) -> &[u8] {
        &self.buf[..self.len]
    }
}

impl core::fmt::Write for TxFmt {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let b = s.as_bytes();
        let n = b.len().min(self.buf.len() - self.len);
        self.buf[self.len..self.len + n].copy_from_slice(&b[..n]);
        self.len += n;
        Ok(())
    }
}

/// Push a MIDI message toward the DIN TX ring (called by the `Midi.*` bindings).
pub(crate) fn midi_tx_push(data: &[u8]) {
    MIDI_TX.lock(|r| r.borrow_mut().push(data));
}

// ── UI output hooks (called by bindings, run in vm_task) ─────────────────────

/// Queue an LED on/off command for `ui_task`.
pub(crate) fn led_cmd(id: u8, on: bool) {
    UI_LED.lock(|r| r.borrow_mut().push(&[id, on as u8]));
}
/// Clear the OLED frame buffer.
pub(crate) fn oled_clear() {
    OLED_FB.lock(|r| r.borrow_mut().fill(0));
}
/// Draw a string into the OLED frame buffer (5×7 font; `x`,`y` in pixels).
pub(crate) fn oled_text(x: usize, y: usize, s: &[u8]) {
    OLED_FB.lock(|r| deluge_bsp::oled::text::draw_str(&mut r.borrow_mut(), x, y, s));
}
/// Set/clear one OLED pixel.
pub(crate) fn oled_pixel(x: usize, y: usize, on: bool) {
    OLED_FB.lock(|r| r.borrow_mut().set_pixel(x, y, on));
}
/// Request the OLED be re-rendered from the frame buffer (`ui_task` picks it up).
pub(crate) fn oled_show() {
    OLED_SHOW.store(true, Ordering::Release);
}

/// Push an input event (`[kind, a, b]`) for `vm_task` to dispatch.
fn input_push(kind: u8, a: u8, b: u8) {
    INPUT_EVENTS.lock(|r| r.borrow_mut().push(&[kind, a, b]));
}

// ── CV/gate output targets (bindings in vm_task → cv_task) ───────────────────
//
// `bindings::tick` computes per-channel CV slew + gate state at control rate and
// writes the resulting DAC code / gate level here; `cv_task` owns the `Cv`/`Gate`
// SDK handles and mirrors these to hardware (the SPI write is async, so it cannot
// run inside the VM task).

const N_CV: usize = Cv::CHANNELS;
const N_GATE: usize = Gate::CHANNELS;

static CV_TARGET: [AtomicU16; N_CV] = [const { AtomicU16::new(0) }; N_CV];
static GATE_STATE: [AtomicBool; N_GATE] = [const { AtomicBool::new(false) }; N_GATE];

/// Set the target DAC code for CV channel `ch` (consumed by `cv_task`).
pub(crate) fn cv_set_target(ch: u8, code: u16) {
    if (ch as usize) < N_CV {
        CV_TARGET[ch as usize].store(code, Ordering::Relaxed);
    }
}
/// Set the level for gate channel `ch` (consumed by `cv_task`).
pub(crate) fn gate_set_target(ch: u8, on: bool) {
    if (ch as usize) < N_GATE {
        GATE_STATE[ch as usize].store(on, Ordering::Relaxed);
    }
}

// ── Host hooks the wren VM calls back into (see wren-sys) ────────────────────

/// Max bytes we scan/log for a single C string. Caps the scan so a bad/wild
/// pointer can never make *this* sink walk off the end of RAM.
const CSTR_CAP: usize = 4096;

/// Convert a NUL-terminated C string to a `&str`, capped at [`CSTR_CAP`].
/// Returns the slice and whether the cap was hit (no NUL found — a bad pointer).
unsafe fn cstr(ptr: *const c_char) -> (&'static str, bool) {
    if ptr.is_null() {
        return ("", false);
    }
    let mut len = 0usize;
    // SAFETY: bounded scan; stops at NUL or the cap, whichever comes first.
    while len < CSTR_CAP && unsafe { *ptr.add(len) } != 0 {
        len += 1;
    }
    let bytes = unsafe { core::slice::from_raw_parts(ptr as *const u8, len) };
    (core::str::from_utf8(bytes).unwrap_or("<non-utf8>"), len == CSTR_CAP)
}

/// `System.print` output sink → host REPL (+ RTT mirror).
#[unsafe(no_mangle)]
extern "C" fn wren_host_write(text: *const c_char) {
    let (s, capped) = unsafe { cstr(text) };
    if capped {
        error!("wren> <string not NUL-terminated within {} B @ {:p}>", CSTR_CAP, text);
        return;
    }
    // To the host terminal, with CR before any LF so lines don't stair-step.
    // wren calls this once with the content and again with a lone "\n".
    if s == "\n" {
        tx_push(b"\r\n");
    } else {
        tx_push(s.as_bytes());
    }
    // Trimmed, non-empty mirror to RTT.
    let t = s.trim_end_matches(['\r', '\n']);
    if !t.is_empty() {
        info!("wren> {}", t);
    }
}

/// VM error sink (compile + runtime) → host REPL (+ RTT mirror).
#[unsafe(no_mangle)]
extern "C" fn wren_host_error(line: i32, message: *const c_char) {
    let (s, _capped) = unsafe { cstr(message) };
    tx_push(b"!! ");
    tx_push(s.as_bytes());
    tx_push(b"\r\n");
    if line >= 0 {
        error!("wren error [line {}]: {}", line, s);
    } else {
        error!("wren error: {}", s);
    }
}

/// Diagnostic numeric trace from wren-sys (bring-up only). Only the device VM
/// heap reports through this (see wren-sys), so it's device-only here too.
#[cfg(target_os = "none")]
#[unsafe(no_mangle)]
extern "C" fn wren_host_debug(tag: i32, value: usize) {
    error!("wren-dbg: tag={} value=0x{:x} ({})", tag, value, value);
}

// ── USB device static buffers (need 'static for embassy_usb::Builder) ───────

#[cfg(target_os = "none")]
static mut USB_CONFIG_DESC: [u8; 256] = [0; 256];
#[cfg(target_os = "none")]
static mut USB_BOS_DESC: [u8; 64] = [0; 64];
#[cfg(target_os = "none")]
static mut USB_MSOS_DESC: [u8; 0] = [];
#[cfg(target_os = "none")]
static mut USB_CONTROL_BUF: [u8; 64] = [0; 64];
#[cfg(target_os = "none")]
static mut CDC_ACM_STATE: State<'static> = State::new();

// ── Pre-interrupt setup (runs with IRQs masked) ──────────────────────────────

/// Synchronous bring-up that must complete before interrupts are enabled.
///
/// The SDK has already initialised heaps + clocks (see
/// [`#[deluge::app]`](deluge::app)); here we register the USB0 ISR and boot the
/// Wren VM (pure allocation + compilation, so safe with IRQs masked). All
/// peripheral init is owned by the SDK capability handles acquired in `main`.
fn setup() {
    info!("wren-firmware: starting (M1)");

    // USB0 device-mode interrupt. Registering here (before global enable) is
    // sufficient; the interrupt source is only raised once `usb_task` runs.
    // Device-only — the host REPL uses stdin/stdout, not USB.
    #[cfg(target_os = "none")]
    unsafe {
        rza1l_hal::gic::register(USB0_IRQ, || dcd_int_handler(0))
    };

    // Host: start the blocking stdin reader that feeds the REPL `RX` ring. Runs
    // on the brain thread before the executor, alongside the GUI on the main
    // thread (see the SDK host runtime).
    #[cfg(not(target_os = "none"))]
    host_repl::spawn_stdin();

    // Register the hardware host the portable bindings call back into (CV/gate,
    // MIDI, LEDs, OLED, audio). Must happen before any script runs.
    deluge_wren_core::set_host(unsafe { &mut *addr_of_mut!(host::FW_HOST) });

    // Boot the VM and load the prelude (declares the foreign classes + the
    // `output[]` / `gate[]` accessors). ~0.6 s; mirrors crow's ordering.
    info!("wren: booting VM...");
    let vm =
        unsafe { wren_sys::boot_with_foreign(deluge_wren_core::METHODS, deluge_wren_core::CLASSES) };
    if vm.is_null() {
        error!("wren: wrenNewVM returned NULL — out of SDRAM?");
        loop {
            core::hint::spin_loop();
        }
    }
    VM_PTR.store(vm, Ordering::Release);
    info!(
        "wren: VM up — peak SRAM {} B / SDRAM {} B",
        wren_sys::peak_sram_bytes(),
        wren_sys::peak_sdram_bytes()
    );
    info!("wren: loading prelude...");
    run_source(vm, deluge_wren_core::prelude_ptr(), "prelude");
}

#[deluge::app(setup = setup)]
async fn main(dlg: Deluge) {
    let spawner = dlg.spawner();

    // ── REPL transport ───────────────────────────────────────────────────────
    // Device: a USB-CDC-ACM port (ISR registered in `setup`). Host: stdin/stdout
    // (reader started in `setup`; `host_tx_task` drains the TX ring to stdout).
    #[cfg(target_os = "none")]
    {
        let (device, cdc) = unsafe {
            let (_port, driver) = init_device_mode(0);
            let mut config = Config::new(0x1209, 0x5741);
            config.manufacturer = Some("skyline");
            config.product = Some("wren: deluge");
            config.serial_number = Some("deluge-wren");
            config.self_powered = false;
            config.max_power = 250;

            let mut builder = Builder::new(
                driver,
                config,
                &mut *addr_of_mut!(USB_CONFIG_DESC),
                &mut *addr_of_mut!(USB_BOS_DESC),
                &mut *addr_of_mut!(USB_MSOS_DESC),
                &mut *addr_of_mut!(USB_CONTROL_BUF),
            );
            // 512-byte bulk endpoints: the RUSB1 PHY negotiates high speed, where
            // USB 2.0 requires HS bulk wMaxPacketSize 512.
            let cdc = CdcAcmClass::new(&mut builder, &mut *addr_of_mut!(CDC_ACM_STATE), 512);
            (builder.build(), cdc)
        };
        let (tx, rx) = cdc.split();
        info!("USB: CDC-ACM device built (1209:5741)");
        spawner.spawn(usb_task(device).unwrap());
        spawner.spawn(cdc_tx_task(tx).unwrap());
        spawner.spawn(cdc_rx_task(rx).unwrap());
    }
    #[cfg(not(target_os = "none"))]
    {
        spawner.spawn(host_tx_task().unwrap());
        info!("REPL: stdin/stdout — type wren at the terminal");
    }

    // ── Acquire SDK capability handles (each take-once; one owner per task) ───
    let audio = dlg.audio();
    let input = dlg.input();
    let oled = dlg.oled().await;
    let leds = dlg.leds().await;
    let cv = dlg.cv();
    let gate = dlg.gate();
    let midi = dlg.midi();
    info!("deluge: capabilities acquired");

    // ── Spawn tasks (REPL transport tasks were spawned above, per target) ─────
    spawner.spawn(vm_task().unwrap());
    spawner.spawn(midi_task(midi).unwrap());
    spawner.spawn(input_task(input).unwrap());
    spawner.spawn(ui_task(oled, leds).unwrap());
    spawner.spawn(cv_task(cv, gate).unwrap());
    spawner.spawn(audio::audio_task(audio).unwrap());

    // SD-card persistence is best-effort: if the card is absent/unformatted we
    // simply don't spawn the task (REPL-only, no save/load).
    match dlg.sd().await {
        Ok(sd) => spawner.spawn(flash_task(sd).unwrap()),
        Err(e) => warn!("flash: SD unavailable ({:?}); persistence disabled", e),
    }
}

// ── Tasks ───────────────────────────────────────────────────────────────────

// ── REPL transport: device (USB-CDC) ─────────────────────────────────────────

#[cfg(target_os = "none")]
#[embassy_executor::task]
async fn usb_task(mut device: UsbDevice<'static, Rusb1Driver>) {
    device.run().await;
}

#[cfg(target_os = "none")]
#[embassy_executor::task]
async fn cdc_rx_task(mut rx: Receiver<'static, Rusb1Driver>) {
    let mut buf = [0u8; 512];
    loop {
        match rx.read_packet(&mut buf).await {
            Ok(n) if n > 0 => {
                RX.lock(|r| r.borrow_mut().push(&buf[..n]));
            }
            Ok(_) => {}
            Err(_) => Timer::after(Duration::from_millis(5)).await,
        }
    }
}

#[cfg(target_os = "none")]
#[embassy_executor::task]
async fn cdc_tx_task(mut tx: Sender<'static, Rusb1Driver>) {
    let mut buf = [0u8; 64];
    loop {
        tx.wait_connection().await;
        loop {
            let n = TX.lock(|r| r.borrow_mut().pop(&mut buf));
            if n == 0 {
                Timer::after(Duration::from_millis(2)).await;
                continue;
            }
            if tx.write_packet(&buf[..n]).await.is_err() {
                break; // host disconnected
            }
        }
    }
}

// ── REPL transport: host (stdin/stdout, for `cargo deluge sim`) ───────────────

/// Host REPL input: a blocking stdin reader thread feeds typed bytes into the
/// `RX` ring (the same ring `cdc_rx_task` fills on the device). It lives on its
/// own std thread because a blocking read would otherwise stall the executor.
#[cfg(not(target_os = "none"))]
mod host_repl {
    use super::RX;
    use std::io::Read;

    pub fn spawn_stdin() {
        std::thread::Builder::new()
            .name("wren-stdin".into())
            .spawn(|| {
                let mut stdin = std::io::stdin().lock();
                let mut byte = [0u8; 1];
                while let Ok(1) = stdin.read(&mut byte) {
                    RX.lock(|r| r.borrow_mut().push(&byte));
                }
            })
            .expect("spawning wren-stdin thread");
    }
}

/// Host REPL output: drain the `TX` ring to stdout (mirrors `cdc_tx_task`).
#[cfg(not(target_os = "none"))]
#[embassy_executor::task]
async fn host_tx_task() {
    use std::io::Write;
    let mut buf = [0u8; 256];
    loop {
        let n = TX.lock(|r| r.borrow_mut().pop(&mut buf));
        if n == 0 {
            Timer::after(Duration::from_millis(5)).await;
            continue;
        }
        let mut out = std::io::stdout().lock();
        let _ = out.write_all(&buf[..n]);
        let _ = out.flush();
    }
}

/// SD-card script persistence (SDK `Sd`). Reads `MAIN.WREN` at boot (staged for
/// `vm_task` to run), and writes it back on `^^w`. Only spawned when the card
/// mounts (see `main`); the FAT volume/dir handling lives inside the SDK.
#[embassy_executor::task]
async fn flash_task(mut sd: Sd) {
    // ── Boot load: read MAIN.WREN, stage it for vm_task ──
    // SAFETY: BOOT_BUF is written only here, before BOOT_PENDING is set.
    let buf = unsafe { &mut *addr_of_mut!(BOOT_BUF) };
    match sd.read("MAIN.WREN", &mut buf[..SCRIPT_MAX]) {
        Ok(len) => {
            buf[len] = 0; // NUL-terminate for the VM
            BOOT_LEN.store(len, Ordering::Relaxed);
            BOOT_PENDING.store(true, Ordering::Release);
            info!("flash: loaded MAIN.WREN ({} bytes)", len);
        }
        Err(_) => info!("flash: no MAIN.WREN on card; REPL only"),
    }

    // ── Persistence loop: drain script saves from vm_task ──
    loop {
        Timer::after(Duration::from_millis(250)).await;
        if !SAVE_PENDING.swap(false, Ordering::Acquire) {
            continue;
        }
        let n = SAVE_LEN.load(Ordering::Relaxed);
        // SAFETY: SAVE_BUF was fully written before SAVE_PENDING was set.
        let save_buf: &[u8; SCRIPT_MAX] = unsafe { &*addr_of_mut!(SAVE_BUF) };
        match sd.write("MAIN.WREN", &save_buf[..n]) {
            Ok(()) => info!("flash: saved MAIN.WREN ({} bytes)", n),
            Err(e) => warn!("flash: write failed ({:?})", e),
        }
    }
}

/// Owns the SDK `Midi` handle: forwards the DIN-out ring (`MIDI_TX`, filled by
/// the `Midi.*` bindings) to the UART, and pushes received DIN bytes into
/// `MIDI_RX` for `vm_task` to parse. One handle does both directions.
#[embassy_executor::task]
async fn midi_task(midi: Midi) {
    let mut buf = [0u8; 16];
    loop {
        // Wait for either an inbound byte or a short tick to flush queued TX.
        match select(midi.recv(), Timer::after(Duration::from_millis(1))).await {
            Either::First(b) => {
                MIDI_RX.lock(|r| r.borrow_mut().push(&[b]));
            }
            Either::Second(()) => {}
        }
        let n = MIDI_TX.lock(|r| r.borrow_mut().pop(&mut buf));
        if n > 0 {
            midi.send(&buf[..n]).await;
        }
    }
}

/// Owns the SDK `Input` handle (unified pad/button/encoder stream — the PIC
/// service + encoder IRQs are brought up by the SDK). Normalises each event into
/// the `INPUT_EVENTS` ring (`[kind, a, b]`) for `vm_task` to dispatch to the wren
/// callbacks. Pad coordinates arrive already decoded, so no `pad_coords` lookup.
#[embassy_executor::task]
async fn input_task(input: Input) {
    loop {
        match input.next().await {
            Event::Pad { x, y, pressed } => input_push(if pressed { 0 } else { 1 }, x, y),
            Event::Button { id, pressed } => input_push(if pressed { 2 } else { 3 }, id, 0),
            Event::Encoder { index, delta } => input_push(4, index, delta as u8),
            _ => {}
        }
    }
}

/// UI output task: owns the SDK `Oled` + `Leds` handles. Drains LED commands and
/// re-renders the OLED frame buffer. (Both are async, so they can't run inside a
/// sync foreign method — the bindings queue into the rings consumed here.)
#[embassy_executor::task]
async fn ui_task(mut oled: Oled, mut leds: Leds) {
    let mut led = [0u8; 2];
    loop {
        // LED commands.
        loop {
            let n = UI_LED.lock(|r| r.borrow_mut().pop(&mut led));
            if n < 2 {
                break;
            }
            if led[1] != 0 {
                leds.on(led[0]).await;
            } else {
                leds.off(led[0]).await;
            }
        }
        // OLED re-render: copy the bindings' frame buffer into the SDK frame and
        // flush it.
        if OLED_SHOW.swap(false, Ordering::Acquire) {
            OLED_FB.lock(|r| *oled.frame() = r.borrow().clone());
            oled.flush().await;
        }
        Timer::after(Duration::from_millis(5)).await;
    }
}

/// CV/gate output task: owns the SDK `Cv` + `Gate` handles and mirrors the
/// control-rate targets written by `bindings::tick` (`CV_TARGET`/`GATE_STATE`) to
/// hardware. The CV write is an async SPI transfer, so it lives here rather than
/// in the VM task.
#[embassy_executor::task]
async fn cv_task(mut cv: Cv, mut gate: Gate) {
    let mut last_cv = [u16::MAX; N_CV];
    let mut last_gate = [false; N_GATE];
    // Seed gates so the first real value always writes through.
    for g in &mut last_gate {
        *g = true;
    }
    loop {
        for ch in 0..N_CV {
            let code = CV_TARGET[ch].load(Ordering::Relaxed);
            if code != last_cv[ch] {
                last_cv[ch] = code;
                cv.set(ch as u8, code).await;
            }
        }
        for ch in 0..N_GATE {
            let on = GATE_STATE[ch].load(Ordering::Relaxed);
            if on != last_gate[ch] {
                last_gate[ch] = on;
                gate.set(ch as u8, on);
            }
        }
        Timer::after(Duration::from_millis(1)).await;
    }
}

/// Incremental MIDI parser for channel-voice messages, honouring running status.
/// System-common cancels running status; realtime bytes are ignored (and don't
/// disturb a message in progress). (Lifted from `crow-firmware`.)
struct MidiParser {
    status: u8,
    needed: u8,
    have: u8,
    data: [u8; 2],
}

impl MidiParser {
    const fn new() -> Self {
        Self { status: 0, needed: 0, have: 0, data: [0; 2] }
    }

    /// Feed one raw MIDI byte; yields a complete `(status, d1, d2)` when ready.
    fn push(&mut self, b: u8) -> Option<(u8, u8, u8)> {
        if b >= 0xF8 {
            return None; // system realtime: single byte, transparent
        }
        if b >= 0x80 {
            if b >= 0xF0 {
                self.status = 0; // system common cancels running status
                self.have = 0;
                return None;
            }
            self.status = b;
            self.needed = if matches!(b & 0xF0, 0xC0 | 0xD0) { 1 } else { 2 };
            self.have = 0;
            return None;
        }
        if self.status == 0 {
            return None; // data byte with no status
        }
        self.data[self.have as usize] = b;
        self.have += 1;
        if self.have >= self.needed {
            self.have = 0; // keep running status
            let d2 = if self.needed == 2 { self.data[1] } else { 0 };
            return Some((self.status, self.data[0], d2));
        }
        None
    }
}

/// The VM-owning task: runs the boot script, then services the REPL.
#[embassy_executor::task]
async fn vm_task() {
    let vm = VM_PTR.load(Ordering::Acquire);
    let vmw = Vm(vm);
    info!("wren: REPL live");

    let mut repl = Repl::new();
    let mut frame = [0u8; 256];
    let mut last = Instant::now();
    let mut midi = MidiParser::new();

    loop {
        // Render tick: advance CV slew (→ cv_task), fire due metro callbacks.
        // Runs every loop iteration (~every 2 ms when idle, faster under REPL I/O).
        let now = Instant::now();
        let dt_s = (now - last).as_micros() as f32 / 1_000_000.0;
        last = now;
        deluge_wren_core::tick(vmw, now.as_millis(), dt_s);

        // MIDI DIN RX: drain bytes staged by `midi_task`, parse, and dispatch to
        // the wren callbacks. Runs here (the VM-owning task) so it never re-enters
        // wren mid-evaluation.
        let mut mb = [0u8; 32];
        loop {
            let n = MIDI_RX.lock(|r| r.borrow_mut().pop(&mut mb));
            if n == 0 {
                break;
            }
            for &b in &mb[..n] {
                if let Some((s, d1, d2)) = midi.push(b) {
                    deluge_wren_core::midi_rx(vmw, s, d1, d2);
                }
            }
        }

        // Input events (`[kind, a, b]` from `input_task`) → wren callbacks.
        let mut ev = [0u8; 3];
        loop {
            let n = INPUT_EVENTS.lock(|r| r.borrow_mut().pop(&mut ev));
            if n < 3 {
                break;
            }
            if ev[0] == 4 {
                deluge_wren_core::enc_turn(vmw, ev[1], ev[2] as i8);
            } else {
                deluge_wren_core::input_dispatch(vmw, ev[0], ev[1], ev[2]);
            }
        }

        // Run the SD-persisted boot script once flash_task has staged it.
        if BOOT_PENDING.swap(false, Ordering::Acquire) {
            let len = BOOT_LEN.load(Ordering::Relaxed);
            info!("wren: running MAIN.WREN ({} bytes)", len);
            // SAFETY: BOOT_BUF[..=len] is staged and NUL-terminated.
            let ptr = unsafe { (*addr_of_mut!(BOOT_BUF)).as_ptr() as *const c_char };
            run_source(vm, ptr, "MAIN.WREN");
        }

        // Drain received bytes and process completed REPL lines.
        let n = RX.lock(|r| r.borrow_mut().pop(&mut frame));
        if n == 0 {
            Timer::after(Duration::from_millis(2)).await;
            continue;
        }
        for &b in &frame[..n] {
            repl.push_byte(vm, b);
        }
    }
}

// ── REPL ─────────────────────────────────────────────────────────────────────

/// Interpret `source` (NUL-terminated) under `label`, reporting the result over
/// the REPL transport + RTT. Output/errors flow through the host hooks.
fn run_source(vm: *mut WrenVM, source: *const c_char, label: &str) {
    let result = unsafe { wren_sys::interpret(vm, c"main".as_ptr(), source) };
    match result {
        wren_sys::WREN_RESULT_SUCCESS => {}
        wren_sys::WREN_RESULT_COMPILE_ERROR => {
            warn!("wren: {} compile error", label);
        }
        wren_sys::WREN_RESULT_RUNTIME_ERROR => {
            warn!("wren: {} runtime error", label);
        }
        other => warn!("wren: {} unexpected result {}", label, other),
    }
}

/// Line-assembling REPL state machine. Bytes arrive in arbitrary chunks; we
/// buffer until a newline, then dispatch a complete line.
struct Repl {
    /// Current line being assembled (NUL space reserved at the end).
    line: [u8; 512],
    line_len: usize,
    /// Multi-line `^^s` upload accumulator (NUL space reserved at the end).
    script: [u8; SCRIPT_MAX + 1],
    script_len: usize,
    uploading: bool,
}

impl Repl {
    fn new() -> Self {
        Self {
            line: [0; 512],
            line_len: 0,
            script: [0; SCRIPT_MAX + 1],
            script_len: 0,
            uploading: false,
        }
    }

    /// Feed one received byte; dispatches a line on CR, LF, or CRLF. Terminals
    /// differ (picocom sends CR, others LF or CRLF), so treat any of them as a
    /// line terminator and skip the resulting empty line (so CRLF doesn't
    /// dispatch twice and blank Enters are no-ops).
    fn push_byte(&mut self, vm: *mut WrenVM, b: u8) {
        if b == b'\n' || b == b'\r' {
            let len = self.line_len;
            self.line_len = 0;
            if len > 0 {
                self.dispatch_line(vm, len);
            }
        } else if b == 0x08 || b == 0x7f {
            // Backspace (BS) / Delete (DEL): drop the last buffered char so
            // typo corrections don't end up as literal bytes in the source.
            self.line_len = self.line_len.saturating_sub(1);
        } else if b >= 0x20 && self.line_len < self.line.len() - 1 {
            // Printable byte; control chars other than CR/LF/BS are ignored.
            self.line[self.line_len] = b;
            self.line_len += 1;
        }
        // Bytes past the line buffer are dropped (lines > 511 B are unusual in
        // the REPL; large programs come via `^^s` accumulation, line by line).
    }

    fn dispatch_line(&mut self, vm: *mut WrenVM, len: usize) {
        if self.line[..len].starts_with(b"^^") {
            // Copy the short command token out so we don't hold a borrow of
            // `self.line` across the `&mut self` call below.
            let mut cmd = [0u8; 16];
            let clen = (len - 2).min(cmd.len());
            cmd[..clen].copy_from_slice(&self.line[2..2 + clen]);
            self.handle_command(vm, &cmd[..clen]);
            return;
        }
        if self.uploading {
            // `self.script` (mut) and `self.line` (shared) are disjoint fields.
            let room = SCRIPT_MAX - self.script_len;
            let n = len.min(room);
            let at = self.script_len;
            self.script[at..at + n].copy_from_slice(&self.line[..n]);
            self.script_len += n;
            if self.script_len < SCRIPT_MAX {
                self.script[self.script_len] = b'\n';
                self.script_len += 1;
            }
        } else {
            // Single-line eval: NUL-terminate in place and run.
            self.line[len] = 0;
            let ptr = self.line.as_ptr() as *const c_char;
            run_source(vm, ptr, "repl");
        }
    }

    fn handle_command(&mut self, vm: *mut WrenVM, cmd: &[u8]) {
        match cmd {
            b"s" => {
                self.uploading = true;
                self.script_len = 0;
                tx_push(b"-- upload: send lines, then ^^e (run) or ^^w (run+save)\r\n");
            }
            b"e" => {
                self.uploading = false;
                self.run_script(vm);
            }
            b"w" => {
                self.uploading = false;
                self.run_script(vm);
                self.save_script();
            }
            b"c" => {
                self.uploading = false;
                self.script_len = 0;
                tx_push(b"-- cleared\r\n");
            }
            b"v" => {
                tx_push(b"wren: deluge (M1)\r\n");
            }
            b"m" => {
                // Heap report: the SRAM/SDRAM spill split (live / peak per tier).
                use core::fmt::Write;
                let mut f = TxFmt::new();
                let _ = write!(
                    f,
                    "-- heap: SRAM {}/{} B  SDRAM {}/{} B  (live/peak)\r\n",
                    wren_sys::sram_bytes(),
                    wren_sys::peak_sram_bytes(),
                    wren_sys::sdram_bytes(),
                    wren_sys::peak_sdram_bytes(),
                );
                tx_push(f.as_bytes());
            }
            other => {
                tx_push(b"-- unknown command ^^");
                tx_push(other);
                tx_push(b"\r\n");
            }
        }
    }

    fn run_script(&mut self, vm: *mut WrenVM) {
        self.script[self.script_len] = 0;
        let ptr = self.script.as_ptr() as *const c_char;
        run_source(vm, ptr, "script");
    }

    fn save_script(&mut self) {
        let n = self.script_len.min(SCRIPT_MAX);
        // SAFETY: vm_task is the sole writer of SAVE_BUF; flash_task reads only
        // after SAVE_PENDING is observed.
        let save_buf: &mut [u8; SCRIPT_MAX] = unsafe { &mut *addr_of_mut!(SAVE_BUF) };
        save_buf[..n].copy_from_slice(&self.script[..n]);
        SAVE_LEN.store(n, Ordering::Relaxed);
        SAVE_PENDING.store(true, Ordering::Release);
        tx_push(b"-- saving to MAIN.WREN\r\n");
    }
}
