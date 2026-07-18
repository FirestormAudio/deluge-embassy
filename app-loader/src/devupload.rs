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
//! image is streamed straight to each `PT_LOAD` segment's load / SRAM-staging
//! address as it arrives, checksummed incrementally, and launched once the CRC
//! verifies.

use core::sync::atomic::{AtomicBool, Ordering};

use embassy_futures::join::join;
use embassy_time::{Duration, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, Receiver, State};
use log::{info, warn};

use rza1l_hal::gic;
use rza1l_hal::usb::{Rusb1Driver, USB0_IRQ, dcd_int_handler, disconnect, init_device_mode};

use deluge_image::Crc32;
use deluge_image::elf::{MAX_PHDRS, SDRAM_HI, SDRAM_LO, StreamRouter};

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

// ── Streaming loader window ───────────────────────────────────────────────────
//
// The upload is routed straight to each segment's load / SRAM-staging address as
// it arrives — there is no whole-image scratch buffer. Only the front matter
// (ELF header + program-header table) is buffered, and the app-segment ceiling
// bounds a sane maximum upload length.
/// Bytes of front matter buffered to build the [`StreamRouter`]: ELF header plus
/// the largest program-header table.
const HEADER_BUF: usize = 52 + MAX_PHDRS * 32;
/// Largest upload accepted (the SDRAM app-region span). Per-segment placement is
/// still validated; this only rejects absurd lengths early.
const MAX_UPLOAD: u32 = SDRAM_HI - SDRAM_LO;

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

/// Copy a contiguous run of upload bytes starting at file offset `off` to their
/// segment destinations, discarding bytes that fall in gaps.
///
/// # Safety
/// Writes physical RAM at addresses the [`StreamRouter`] validated via
/// `place_segment`; the caller must ensure no live data occupies those regions
/// (true during the boot menu, like the SD loader).
unsafe fn route_bytes(router: &StreamRouter, mut off: u32, mut data: &[u8]) {
    while !data.is_empty() {
        let step = router.route_at(off);
        let take = (step.run as usize).min(data.len());
        if let Some(dst) = step.dst {
            unsafe { core::ptr::copy_nonoverlapping(data.as_ptr(), dst as *mut u8, take) };
        }
        off += take as u32;
        data = &data[take..];
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

        if version != VERSION || len < 52 || len > MAX_UPLOAD {
            warn!("devupload: bad header (version={version}, len={len}); resyncing");
            continue;
        }

        ui::UPLOAD_ACTIVE.store(true, Ordering::Release);
        info!("devupload: streaming {len} byte image");

        let mut crc = Crc32::new();

        // 1. Buffer + CRC the front matter (ELF header + program headers).
        let mut prefix = [0u8; HEADER_BUF];
        reader.read_exact(&mut prefix[..52]).await;
        let e_phoff = deluge_image::elf::le32(&prefix, 28);
        let e_phnum = deluge_image::elf::le16(&prefix, 44) as usize;
        if e_phoff < 52 || e_phnum > MAX_PHDRS {
            warn!("devupload: bad phdr table (e_phoff={e_phoff}, e_phnum={e_phnum})");
            ui::show_message(b"UPLOAD ERROR", b"BAD LAYOUT").await;
            Timer::after(Duration::from_secs(2)).await;
            ui::UPLOAD_ACTIVE.store(false, Ordering::Release);
            continue;
        }
        let header_end = match (e_phoff as usize).checked_add(e_phnum * 32) {
            Some(h) => h,
            None => {
                warn!("devupload: header offset overflow (e_phoff={e_phoff})");
                ui::show_message(b"UPLOAD ERROR", b"BAD LAYOUT").await;
                Timer::after(Duration::from_secs(2)).await;
                ui::UPLOAD_ACTIVE.store(false, Ordering::Release);
                continue;
            }
        };
        if header_end > HEADER_BUF || header_end as u32 > len {
            warn!("devupload: phdr table outside header window (header_end={header_end})");
            ui::show_message(b"UPLOAD ERROR", b"BAD LAYOUT").await;
            Timer::after(Duration::from_secs(2)).await;
            ui::UPLOAD_ACTIVE.store(false, Ordering::Release);
            continue;
        }
        if header_end > 52 {
            reader.read_exact(&mut prefix[52..header_end]).await;
        }
        crc.update(&prefix[..header_end]);

        // 2. Build + validate the route plan.
        let router = match StreamRouter::new(&prefix[..header_end]) {
            Ok(r) => r,
            Err(e) => {
                let line2: &[u8] = match elf::ElfError::from(e) {
                    elf::ElfError::BadMagic => b"BAD MAGIC",
                    elf::ElfError::WrongFormat => b"WRONG FORMAT",
                    elf::ElfError::BadLoadAddress => b"BAD LOAD ADDR",
                    elf::ElfError::Unstreamable => b"BAD LAYOUT",
                    _ => b"SEE LOG",
                };
                warn!("devupload: image rejected: {e:?}");
                ui::show_message(b"UPLOAD ERROR", line2).await;
                Timer::after(Duration::from_secs(2)).await;
                ui::UPLOAD_ACTIVE.store(false, Ordering::Release);
                continue;
            }
        };

        // 3. Replay the buffered front matter through the router (handles a
        //    first segment whose p_offset is 0), then stream the remainder.
        unsafe { route_bytes(&router, 0, &prefix[..header_end]) };
        let total = len as usize;
        let mut off = header_end as u32;
        let mut remaining = total - header_end;
        let mut last_pct = u8::MAX;
        while remaining > 0 {
            let chunk = reader.next_chunk(remaining).await;
            crc.update(chunk);
            unsafe { route_bytes(&router, off, chunk) };
            off += chunk.len() as u32;
            remaining -= chunk.len();
            let pct = (((total - remaining) as u64) * 100 / total as u64) as u8;
            if pct != last_pct {
                ui::show_progress(b"RECEIVING", pct).await;
                last_pct = pct;
            }
        }

        // 4. Verify integrity before anything irreversible.
        if crc.finalize() != expect_crc {
            warn!("devupload: CRC mismatch");
            ui::show_message(b"UPLOAD ERROR", b"BAD CRC").await;
            Timer::after(Duration::from_secs(2)).await;
            ui::UPLOAD_ACTIVE.store(false, Ordering::Release);
            continue;
        }

        // 5. Zero-extend BSS tails and collect SRAM descriptors for handoff.
        let mut sram_descs = [elf::SramSegDesc::default(); MAX_PHDRS];
        let mut n_sram = 0usize;
        for seg in router.segments() {
            let filesz = seg.file_end - seg.file_start;
            let zero_extra = seg.memsz - filesz;
            if zero_extra > 0 {
                unsafe {
                    core::ptr::write_bytes(
                        (seg.write_addr + filesz) as *mut u8,
                        0,
                        zero_extra as usize,
                    )
                };
            }
            if seg.sram {
                sram_descs[n_sram] = elf::SramSegDesc {
                    src: seg.write_addr,
                    dst: seg.final_dst,
                    filesz,
                    zero_extra,
                };
                n_sram += 1;
            }
        }

        let result = elf::LoadResult {
            entry: router.entry(),
            sram_descs,
            n_sram,
        };
        info!("devupload: image loaded, entry={:#010x}", result.entry);
        handoff(result).await
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

    /// Borrow the next run of received bytes (up to `remaining`), refilling from
    /// USB as needed. Never returns empty until the caller has taken `remaining`
    /// bytes across calls.
    async fn next_chunk(&mut self, remaining: usize) -> &[u8] {
        if self.pos >= self.fill {
            self.refill().await;
        }
        let take = (self.fill - self.pos).min(remaining);
        let chunk = &self.buf[self.pos..self.pos + take];
        self.pos += take;
        chunk
    }
}
