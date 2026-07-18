//! USB **dev-mode upload** listener: a background CDC-ACM serial endpoint that
//! receives a framed ELF straight from the host (`cargo deluge run`) and loads +
//! launches it to RAM — the same hand-off the SD `/APPS/` boot path uses, but
//! sourced from USB and with no SD shuffling.
//!
//! Unlike [`crate::usbmsc`], this is **not a mode the user enters**: when dev
//! mode is on, `boot_task` races [`listen`] against the menu selector while the
//! boot menu is shown.  [`listen`] brings up the CDC device but draws nothing
//! until a valid upload header arrives; the moment a complete, CRC-checked image
//! is received it performs the launch itself and never returns.  A bad frame is
//! resynced and listening continues.
//!
//! ## Wire protocol (host → device, little-endian)
//! ```text
//! magic b"DLUP" | version u8 | flags u8 | len u32 | crc32 u32 | <len ELF bytes>
//! ```
//! `crc32` is the shared [`deluge_image::crc32`] of the `len` ELF bytes.  The
//! image is streamed into a high-SDRAM scratch window (clear of both the SDRAM
//! load region and the SRAM staging window the loader uses), validated, then
//! handed to [`crate::elf::load_from_slice`].

use core::sync::atomic::{AtomicBool, Ordering};

use embassy_futures::join::join;
use embassy_time::{Duration, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, Receiver, State};
use log::{info, warn};

use rza1l_hal::gic;
use rza1l_hal::usb::{Rusb1Driver, USB0_IRQ, dcd_int_handler, disconnect, init_device_mode};

use crate::{elf, launcher, ui};

// ── Wire-protocol constants ───────────────────────────────────────────────────

/// Frame magic preceding every upload.
const MAGIC: [u8; 4] = *b"DLUP";
/// Protocol version this loader speaks.
const VERSION: u8 = 1;
/// Bytes of fixed header after the magic: `version | flags | len | crc32`.
const HEADER_TAIL: usize = 1 + 1 + 4 + 4;

/// CDC bulk-endpoint max packet size.  The RUSB1 PHY negotiates high speed, and
/// USB 2.0 requires HS bulk endpoints to advertise 512 (matching the proven
/// `usb_debug` / MSC paths).
const MAX_PACKET: u16 = 512;

// ── SDRAM scratch window for the received ELF ─────────────────────────────────
//
// The 64 MB SDRAM runs 0x0C000000..0x10000000.  The loader uses 0x0C000000..
// 0x0F000000 for direct SDRAM segment loads and 0x0F000000..~0x0F2E0000 as the
// SRAM staging window.  We stage the *raw uploaded ELF* above all of that so the
// later `load_from_slice` copies never overlap their own source.
/// Base of the raw-upload scratch window (above the SRAM staging window).
const SCRATCH_ADDR: u32 = 0x0F30_0000;
/// Length of the scratch window (`0x0F300000..0x0FF00000`, 12 MB, well inside
/// the 64 MB SDRAM).  Uploads larger than this are rejected.
const SCRATCH_LEN: u32 = 0x00C0_0000;

// ── USB descriptor / class `'static` backing storage ──────────────────────────

static mut USB_CONFIG_DESC: [u8; 256] = [0; 256];
static mut USB_BOS_DESC: [u8; 64] = [0; 64];
static mut USB_MSOS_DESC: [u8; 0] = [];
static mut USB_CONTROL_BUF: [u8; 64] = [0; 64];
static mut CDC_STATE: State = State::new();

/// Ensures the USB0 ISR is wired into the GIC exactly once across mode entries.
static USB_IRQ_REGISTERED: AtomicBool = AtomicBool::new(false);

/// A brought-up dev-upload USB device, ready to listen.
///
/// Created by [`prepare`] **before** the boot menu starts drawing: USB bring-up
/// reconfigures interrupts/clocks, and doing it while an OLED frame DMA (and its
/// PIC chip-select handshake) is in flight can wedge the display so the menu
/// never redraws. The proven [`crate::usbmsc`] path likewise builds USB before
/// starting its OLED loop. [`run`](Listener::run) then drives it concurrently
/// with the menu selector.
pub struct Listener {
    device: embassy_usb::UsbDevice<'static, Rusb1Driver>,
    rx: Receiver<'static, Rusb1Driver>,
}

/// Bring up the dev-upload CDC device. Call this **before** the menu selector
/// starts drawing (see [`Listener`]), then `.await` [`Listener::run`].
pub fn prepare() -> Listener {
    let (device, cdc) = unsafe { build_usb() };
    info!("devupload: CDC listener up (waiting for upload)");
    let (_tx, rx) = cdc.split();
    Listener { device, rx }
}

impl Listener {
    /// Run the CDC device alongside the frame receiver. The receiver only
    /// returns by loading and launching a received image, so this future never
    /// resolves — `boot_task` races it against the menu selector and treats its
    /// completion as "an upload happened".
    pub async fn run(self) -> ! {
        let Listener { mut device, rx } = self;
        join(device.run(), receive(rx)).await;
        // `receive` is `-> !`; the join can never resolve.
        unreachable!()
    }
}

/// Build the USB device in CDC-ACM configuration.  Mirrors
/// [`crate::usbmsc::build_usb`] but uses a distinct product string so the host
/// can pick the right `/dev/ttyACM*`.
///
/// # Safety
/// Mutates the module's `'static` descriptor buffers; one listener at a time.
unsafe fn build_usb() -> (
    embassy_usb::UsbDevice<'static, Rusb1Driver>,
    CdcAcmClass<'static, Rusb1Driver>,
) {
    unsafe {
        // Wire the USB0 ISR once (global IRQs are already enabled by boot_task).
        if !USB_IRQ_REGISTERED.swap(true, Ordering::AcqRel) {
            gic::register(USB0_IRQ, || dcd_int_handler(0));
        }

        let (_port, driver) = init_device_mode(0);
        let mut config = embassy_usb::Config::new(
            deluge_bsp::usb::ids::VID,
            deluge_bsp::usb::ids::PID_APP_LOADER_CDC,
        );
        config.manufacturer = Some("Synthstrom Audible");
        config.product = Some("Deluge Dev Upload");
        config.self_powered = false;
        config.max_power = 250; // 500 mA

        let mut builder = embassy_usb::Builder::new(
            driver,
            config,
            &mut *core::ptr::addr_of_mut!(USB_CONFIG_DESC),
            &mut *core::ptr::addr_of_mut!(USB_BOS_DESC),
            &mut *core::ptr::addr_of_mut!(USB_MSOS_DESC),
            &mut *core::ptr::addr_of_mut!(USB_CONTROL_BUF),
        );

        let cdc = CdcAcmClass::new(
            &mut builder,
            &mut *core::ptr::addr_of_mut!(CDC_STATE),
            MAX_PACKET,
        );

        (builder.build(), cdc)
    }
}

/// Receive framed uploads forever.  Returns only by loading and launching an
/// image (so the return type is `!`); a malformed/short/CRC-bad frame is logged,
/// reported on the OLED, and listening resumes.
async fn receive(rx: Receiver<'static, Rusb1Driver>) -> ! {
    let mut reader = PacketReader::new(rx);
    loop {
        // Resync to the frame magic, then read the fixed header tail.
        reader.sync_to_magic().await;
        let mut tail = [0u8; HEADER_TAIL];
        reader.read_exact(&mut tail).await;
        let version = tail[0];
        let _flags = tail[1];
        let len = u32::from_le_bytes([tail[2], tail[3], tail[4], tail[5]]);
        let expect_crc = u32::from_le_bytes([tail[6], tail[7], tail[8], tail[9]]);

        if version != VERSION || len == 0 || len > SCRATCH_LEN {
            warn!("devupload: bad header (version={version}, len={len}); resyncing");
            // Header looked plausible enough to reach here but is unusable; the
            // body bytes (if any) will be resynced past as non-magic noise.
            continue;
        }

        // A header has arrived: take the OLED from the menu selector and show
        // receive progress.
        ui::UPLOAD_ACTIVE.store(true, Ordering::Release);
        info!("devupload: receiving {len} byte image");

        let dst = SCRATCH_ADDR as *mut u8;
        reader.read_to_ptr(dst, len).await;

        let image = unsafe { core::slice::from_raw_parts(dst as *const u8, len as usize) };
        let crc = deluge_image::crc32(image);
        if crc != expect_crc {
            warn!("devupload: CRC mismatch (got {crc:#010x}, want {expect_crc:#010x})");
            ui::show_message(b"UPLOAD ERROR", b"BAD CRC").await;
            Timer::after(Duration::from_secs(2)).await;
            ui::UPLOAD_ACTIVE.store(false, Ordering::Release);
            continue;
        }

        match unsafe { elf::load_from_slice(image) } {
            Ok(result) => {
                info!("devupload: image loaded, entry={:#010x}", result.entry);
                handoff(result).await
            }
            Err(e) => {
                let line2: &[u8] = match e {
                    elf::ElfError::BadMagic => b"BAD MAGIC",
                    elf::ElfError::WrongFormat => b"WRONG FORMAT",
                    elf::ElfError::BadLoadAddress => b"BAD LOAD ADDR",
                    elf::ElfError::UnexpectedEof => b"TRUNCATED",
                    _ => b"SEE LOG",
                };
                warn!("devupload: image rejected: {e:?}");
                ui::show_message(b"UPLOAD ERROR", line2).await;
                Timer::after(Duration::from_secs(2)).await;
                ui::UPLOAD_ACTIVE.store(false, Ordering::Release);
            }
        }
    }
}

/// Final handoff: tear down USB cleanly (so the host re-enumerates the app's own
/// usb-log CDC) and launch the loaded image. Mirrors the SD ELF path in
/// `boot_task` (blank OLED, disable interrupts, quiesce, launch). Never returns.
async fn handoff(result: elf::LoadResult) -> ! {
    ui::show_message(b"LAUNCHING", b"FROM USB").await;

    // Draw the boot droplet before interrupts/DMA are quiesced (must run while
    // the executor + pic_rx_task are still live — see `crate::show_boot_logo`).
    crate::show_boot_logo().await;

    // Unplug from the host: it sees a clean disconnect and re-enumerates the
    // launched app's own USB stack (e.g. the usb-log CDC for `cargo deluge run
    // --log`).
    unsafe { disconnect(0) };

    cortex_ar::interrupt::disable();
    unsafe { crate::quiesce_for_handoff() };

    if result.n_sram > 0 {
        unsafe {
            launcher::launch_via_trampoline(&result.sram_descs[..result.n_sram], result.entry)
        }
    } else {
        unsafe { launcher::launch(result.entry) }
    }
}

/// Buffered reader over the CDC OUT endpoint: `read_packet` only yields whole
/// packets, so this re-packetises into byte / fixed-length / bulk reads and
/// drives the OLED progress bar during the bulk copy.
struct PacketReader {
    rx: Receiver<'static, Rusb1Driver>,
    buf: [u8; MAX_PACKET as usize],
    pos: usize,
    fill: usize,
    connected: bool,
}

impl PacketReader {
    fn new(rx: Receiver<'static, Rusb1Driver>) -> Self {
        Self {
            rx,
            buf: [0u8; MAX_PACKET as usize],
            pos: 0,
            fill: 0,
            connected: false,
        }
    }

    /// Refill the internal buffer with the next non-empty packet, (re)waiting for
    /// the host to open the port across disconnects.
    async fn refill(&mut self) {
        loop {
            if !self.connected {
                self.rx.wait_connection().await;
                self.connected = true;
            }
            match self.rx.read_packet(&mut self.buf).await {
                Ok(n) if n > 0 => {
                    self.pos = 0;
                    self.fill = n;
                    return;
                }
                Ok(_) => {}                       // zero-length packet; keep reading
                Err(_) => self.connected = false, // host went away; rewait
            }
        }
    }

    /// Read one byte.
    async fn byte(&mut self) -> u8 {
        if self.pos >= self.fill {
            self.refill().await;
        }
        let b = self.buf[self.pos];
        self.pos += 1;
        b
    }

    /// Read exactly `dst.len()` bytes.
    async fn read_exact(&mut self, dst: &mut [u8]) {
        let mut i = 0;
        while i < dst.len() {
            if self.pos >= self.fill {
                self.refill().await;
            }
            let take = (self.fill - self.pos).min(dst.len() - i);
            dst[i..i + take].copy_from_slice(&self.buf[self.pos..self.pos + take]);
            self.pos += take;
            i += take;
        }
    }

    /// Slide a 4-byte window until it matches the frame magic.
    async fn sync_to_magic(&mut self) {
        let mut window = [0u8; 4];
        // Prime the window with the first four bytes.
        for slot in window.iter_mut() {
            *slot = self.byte().await;
        }
        while window != MAGIC {
            window.rotate_left(1);
            window[3] = self.byte().await;
        }
    }

    /// Stream exactly `len` bytes into the raw destination at `dst`, updating the
    /// OLED progress bar as it goes.
    ///
    /// # Safety-ish
    /// `dst` must point at `len` writable bytes (the SDRAM scratch window).
    async fn read_to_ptr(&mut self, dst: *mut u8, len: u32) {
        let len = len as usize;
        let mut received = 0usize;
        let mut last_pct = u8::MAX;
        while received < len {
            if self.pos >= self.fill {
                self.refill().await;
            }
            let take = (self.fill - self.pos).min(len - received);
            unsafe {
                core::ptr::copy_nonoverlapping(
                    self.buf.as_ptr().add(self.pos),
                    dst.add(received),
                    take,
                );
            }
            self.pos += take;
            received += take;

            let pct = ((received as u64) * 100 / len as u64) as u8;
            if pct != last_pct {
                ui::show_progress(b"RECEIVING", pct).await;
                last_pct = pct;
            }
        }
    }
}
