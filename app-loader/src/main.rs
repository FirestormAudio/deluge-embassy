//! App loader (second-stage bootloader) for the Synthstrom Deluge (RZ/A1L).
//!
//! Loaded by the Deluge first-stage bootloader from SPI flash into SRAM at
//! `0x20020000`.  On boot it:
//!   1. Initialises the platform (MMU, caches, SDRAM, GIC, OSTM).
//!   2. Mounts the SD card FAT filesystem.
//!   3. Lists ELF application images from `/APPS/` on the card.
//!   4. If more than one image is found, presents an OLED + encoder-wheel
//!      file-selector; otherwise auto-launches the only image.
//!   5. Seeks through the selected ELF file, loads PT_LOAD segments to their
//!      physical addresses, flushes all caches, and branches to `e_entry`.

#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

mod devupload;
mod elf;
mod file_browser;
mod flashboot;
mod launcher;
mod settings;
mod ui;
mod usbmsc;

/// Label for the synthetic menu entry that enters SD-card USB mass-storage mode.
const DATA_MENU_LABEL: &[u8] = b"DATA TRANSFER";

/// Label for the synthetic entry that opens the settings screen.
const SETTINGS_MENU_LABEL: &[u8] = b"SETTINGS";

use core::mem::MaybeUninit;
use core::sync::atomic::AtomicBool;

/// Set by `pic_rx_task` when the BACK button is pressed.  USB modes poll this to
/// exit back to the boot menu (see [`usbmsc`]).
pub(crate) static BACK_PRESSED: AtomicBool = AtomicBool::new(false);

/// Latched by `pic_rx_task` on the first LOAD press — including a LOAD held
/// down at power-on, which the PIC reports in response to the
/// `CMD_RESEND_BUTTON_STATES` that `pic::init` sends.
///
/// `boot_task` samples this once, at the boot decision, as the **recovery
/// gesture**: it forces the boot menu no matter what the persisted auto-boot
/// setting says.  A latch rather than a level check, so the gesture is forgiving
/// — any LOAD press between reset and the decision counts, and the user does
/// not have to guess when the loader looks.  LOAD rather than SELECT, so the
/// gesture cannot be confused with the SELECT presses that drive the menu
/// itself.  Without it, a unit set to `AUTO-BOOT: INSTANT` with broken firmware
/// in its flash slot would be unrecoverable.
pub(crate) static LOAD_SEEN: AtomicBool = AtomicBool::new(false);
use core::panic::PanicInfo;

use deluge_alloc as allocator;
use embassy_executor::{Executor, Spawner};
use log::{error, info, warn};

use deluge_bsp::{oled, sd};

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
            up: {
                0: {
                    size: 16384,
                    name: "Terminal",
                    section: ".rtt_buffer"
                }
            }
            section_cb: ".rtt_buffer"
        };
        rtt_target::set_print_channel(channels.up.0);
        rtt_target::init_logger_with_level(log::LevelFilter::Debug);
    }
    info!("app-loader: starting");

    unsafe {
        let start = core::ptr::addr_of!(__sram_heap_start) as *mut u8;
        let size = core::ptr::addr_of!(__sram_heap_end) as usize - start as usize;
        allocator::SRAM.init(start, size);
    }

    unsafe { deluge_bsp::system::init_clocks() };
    info!("platform: clocks, MMU, cache, SDRAM, GIC ready");

    // SDRAM heap initialised but not used by the bootloader itself;
    // kept for consistency with the BSP init pattern.
    unsafe { allocator::SDRAM.init(0x0C00_0000 as *mut u8, 64 * 1024 * 1024) };

    // SCIF1 (PIC32 link): configures pins, baud, TX, and — crucially — the
    // DMA-RX ring that `uart::read_byte` reads from.  Must run before the
    // executor starts (global IRQs still masked).  Without it the OLED
    // chip-select echo handshake can never be received and `oled::init` hangs.
    unsafe { deluge_bsp::uart::init_pic(31_250) };

    unsafe { deluge_bsp::cv_gate::init() };

    #[allow(static_mut_refs)]
    let executor = unsafe {
        EXECUTOR.write(Executor::new());
        EXECUTOR.assume_init_mut()
    };
    executor.run(|spawner| {
        spawner.spawn(boot_task(spawner).unwrap());
    });
}

/// PIC32 receive pump.
///
/// Owns the PIC init handshake (`pic::init`, including the 31 250 → 200 000 bps
/// baud switch) and then continuously parses incoming PIC bytes.  It must be
/// live before `oled::init()` runs: the OLED chip-select handshake
/// (`pic::wait_oled_selected`) only unblocks when the PIC's echoed SELECT /
/// DESELECT bytes (248 / 249) are decoded here and forwarded to the matching
/// `notify_*` signal.  Button presses confirm the file selector.
///
/// Runs until the bootloader hands off to the app (interrupts are disabled at
/// that point, so the underlying DMA-RX waker never fires again).
#[embassy_executor::task]
async fn pic_rx_task() {
    use core::sync::atomic::Ordering;
    use deluge_bsp::{
        controls,
        pic::{self, Event},
    };

    // Run the PIC init sequence here (mirrors the firmware's `pic_task`), then
    // parse RX.  `boot_task` waits on `pic::wait_ready()` for this to finish.
    pic::init().await;

    let mut parser = pic::Parser::new();
    loop {
        let byte = rza1l_hal::uart::read_byte(deluge_bsp::uart::PIC_CH).await;
        match parser.push(byte) {
            Some(Event::OledSelected) => pic::notify_oled_selected(),
            Some(Event::OledDeselected) => pic::notify_oled_deselected(),
            // Track the SELECT button held-state so the selector can tell a
            // short tap (confirm) from a long-press (write-to-flash).
            Some(Event::ButtonPress { id }) if id == controls::encoder_button::SELECT => {
                ui::SELECT_DOWN.store(true, Ordering::Release)
            }
            Some(Event::ButtonRelease { id }) if id == controls::encoder_button::SELECT => {
                ui::SELECT_DOWN.store(false, Ordering::Release)
            }
            // Latch the first LOAD press as the recovery gesture (see LOAD_SEEN).
            Some(Event::ButtonPress { id }) if id == controls::button::LOAD => {
                crate::LOAD_SEEN.store(true, Ordering::Release)
            }
            // BACK exits an active USB mode back to the boot menu.
            Some(Event::ButtonPress { id }) if id == controls::button::BACK => {
                crate::BACK_PRESSED.store(true, Ordering::Release)
            }
            _ => {}
        }
    }
}

/// Stop all autonomous peripheral activity before branching to the app.
///
/// Must run after `cortex_ar::interrupt::disable()` and before the launch.
/// The app re-initialises every controller from scratch, so it only needs the
/// hardware to be *quiet*: no DMA channel still transferring into RAM, no timer
/// or GIC source able to fire.  The critical one is the SCIF1/PIC circular
/// receive DMA — see the call site.
pub(crate) unsafe fn quiesce_for_handoff() {
    unsafe {
        // Stop + software-reset every DMA channel and clear its DMARS request
        // route (covers PIC RX/TX, SD, and OLED channels). Clearing DMARS here
        // is what lets Linux's rz-dmac claim SCIF1-RX cleanly — previously
        // U-Boot's deluge_reset_dmac() had to do it after us.
        for ch in 0..16u8 {
            rza1l_hal::dmac::stop(ch);
        }
        // Stop the Embassy time-driver timers (OSTM0 free-run + OSTM1 alarm).
        rza1l_hal::ostm::stop(0);
        rza1l_hal::ostm::stop(1);
        // Turn the interrupt controller off entirely.
        rza1l_hal::gic::shutdown();
    }
}

/// Draw the Deluge droplet right before handing off to a launched image, so the
/// panel holds the boot logo (not the loader's menu) all the way through U-Boot
/// and early kernel until the image's own display code takes over. Replaces the
/// former blank so early boot is never a dark panel.
///
/// Must be called while interrupts, the executor, and `pic_rx_task` are still
/// live: `send_frame` awaits the OLED DMA-completion IRQ and the PIC chip-select
/// echo, so it cannot complete once the machine has been quiesced for handoff.
pub(crate) async fn show_boot_logo() {
    oled::send_frame(&oled::boot_logo()).await;
}

/// Write `byte` as two uppercase hex digits into `out[..2]` (for on-OLED
/// diagnostics, since the loader has no other display channel by default).
fn hex2(out: &mut [u8], byte: u8) {
    const HEX: &[u8; 16] = b"0123456789ABCDEF";
    out[0] = HEX[(byte >> 4) as usize];
    out[1] = HEX[(byte & 0xF) as usize];
}

/// Store a selected SD `/APPS` ELF into the flash app slot.
///
/// Opens the file, flattens its `PT_LOAD` segments into the SDRAM staging window
/// ([`elf::flatten_to_flash_staging`]), then validates + programs it into the
/// slot ([`flashboot::store_image_to_slot`]).  Progress and the final result are
/// shown on the OLED.  On success the next boot-menu pass re-probes the slot and
/// the image appears as the default `BOOT FLASH` entry — no extra wiring needed.
///
/// Returns to the menu on any error; the slot is only ever touched after the
/// image passes FSB validation, so a bad ELF cannot brick an existing image.
async fn write_app_to_flash(
    vm: &mut deluge_bsp::fat::DelugeVolumeManager,
    root: deluge_bsp::fat::RawDirectory,
    entry: &deluge_bsp::fat::DirEntry,
    label: &[u8],
) {
    use embassy_time::{Duration, Timer};

    ui::show_progress(label, 0).await;

    let file = match file_browser::open_app(vm, root, entry) {
        Ok(f) => f,
        Err(e) => {
            error!("Flash store: open failed: {:?}", e);
            ui::show_message(b"FLASH ERROR", b"OPEN FAILED").await;
            Timer::after(Duration::from_secs(2)).await;
            return;
        }
    };

    // Flatten the ELF into SDRAM staging — first half of the progress bar.
    let stage = match unsafe {
        elf::flatten_to_flash_staging(vm, file, |done, total| async move {
            let pct = done.saturating_mul(50).checked_div(total).unwrap_or(50) as u8;
            ui::show_progress(label, pct).await;
        })
        .await
    } {
        Ok(s) => s,
        Err(e) => {
            let line2: &[u8] = match e {
                elf::ElfError::BadMagic => b"BAD MAGIC",
                elf::ElfError::WrongFormat => b"WRONG FORMAT",
                elf::ElfError::NotFlashable => b"NOT FLASHABLE",
                elf::ElfError::TooLarge => b"TOO LARGE",
                _ => b"SEE LOG",
            };
            error!("Flash store: flatten failed: {:?}", e);
            let _ = vm.close_file(file);
            ui::show_message(b"FLASH ERROR", line2).await;
            Timer::after(Duration::from_secs(2)).await;
            return;
        }
    };

    let _ = vm.close_file(file);

    // Validate + program into the flash slot — second half of the progress bar.
    match unsafe {
        flashboot::store_image_to_slot(&stage, |done, total| async move {
            let pct = 50 + done.saturating_mul(50).checked_div(total).unwrap_or(50) as u8;
            ui::show_progress(label, pct.min(100)).await;
        })
        .await
    } {
        Ok(()) => {
            info!("Flash store: programmed {} bytes into the slot", stage.len);
            ui::show_message(b"FLASHED OK", b"NOW DEFAULT").await;
            Timer::after(Duration::from_secs(2)).await;
        }
        Err(e) => {
            error!("Flash store: image rejected: {:?}", e);
            let line2: &[u8] = match e {
                deluge_image::elf::FsbError::VerifyFailed(_) => b"VERIFY FAILED",
                deluge_image::elf::FsbError::TooLargeForSlot => b"TOO LARGE",
                _ => b"BAD IMAGE",
            };
            ui::show_message(b"FLASH ERROR", line2).await;
            Timer::after(Duration::from_secs(2)).await;
        }
    }
}

#[embassy_executor::task]
async fn boot_task(spawner: Spawner) {
    // Global IRQs must be unmasked *before* the first interrupt-driven await.
    // pic::init() and oled::init() use embassy_time `Timer`s, which are woken
    // only by the OSTM1 GIC interrupt (see rza1l_hal::time_driver); with
    // CPSR.I still masked that interrupt can never be taken and the executor
    // parks in wfe() forever.  encoder::irq_init() registers the encoder GPIO
    // IRQ and, per its contract, must run before the global unmask.
    unsafe { deluge_bsp::encoder::irq_init() };
    unsafe { cortex_ar::interrupt::enable() };

    // Start the PIC receive pump *before* driving the OLED.  It owns
    // `pic::init()` and keeps parsing RX, so the OLED chip-select echo
    // handshake inside `oled::init()` / `send_frame()` has a live parser to
    // unblock it.  Wait for PIC init to finish before issuing OLED commands.
    spawner.spawn(pic_rx_task().unwrap());
    deluge_bsp::pic::wait_ready().await;

    oled::init().await;
    info!("OLED: ready");

    // Recovery gesture: LOAD pressed or held at any point between reset and
    // here (see LOAD_SEEN).  Sampled once — later menu passes are user-driven
    // and must not re-trigger it.  The splash matters most on an INSTANT unit,
    // which otherwise gives no sign that the *hold* — rather than a failed boot —
    // is why the menu appeared.  Nothing is persisted: the user's setting stands.
    let recovery = LOAD_SEEN.load(core::sync::atomic::Ordering::Acquire);
    if recovery {
        info!("Recovery: LOAD held at boot — forcing the boot menu");
        ui::show_message(b"RECOVERY", b"BOOT MENU").await;
        embassy_time::Timer::after(embassy_time::Duration::from_millis(700)).await;
    }

    // Auto-boot (countdown *or* instant) only ever fires on the first pass of the
    // loop below.  See `settings::boot_mode`.
    let mut auto_boot_allowed = true;

    // Boot menu loop.  A bootable selection launches and never returns; the
    // DATA TRANSFER USB mode returns here when BACK is pressed, and a
    // write-to-flash also returns here, so the menu is rebuilt from fresh state
    // (re-probing the flash slot and re-listing the SD card).
    //
    // Only the very first pass shows the "DELUGE BOOT / INIT SD..." splash:
    // re-showing it when returning from a USB mode or a DEV MODE toggle looks
    // like the unit rebooted instead of simply going back to the menu.  Later
    // passes rebuild quietly and let `run_selector` redraw the menu directly.
    let mut first_pass = true;
    loop {
        // Read persisted settings each pass so a DEV MODE toggle takes effect on
        // the very next menu rebuild (dev mode adds a background USB listener and
        // disables the auto-boot countdown).
        let cfg = settings::read();

        // Probe the on-flash firmware slot.  Independent of the SD card, so a
        // unit with valid flash firmware can boot with no card inserted.
        let flash_img = flashboot::probe();
        if flash_img.is_some() {
            info!("Flash: valid firmware image present in slot");
        }

        if first_pass {
            ui::show_message(b"DELUGE BOOT", b"INIT SD...").await;
        }
        first_pass = false;

        // Try the SD card.  A missing/failed card is not fatal.
        let sd_ok = match sd::init().await {
            Ok(()) => {
                info!("SD: card ready (HC={})", sd::is_hc());
                true
            }
            Err(e) => {
                warn!("SD: init failed: {:?}", e);
                false
            }
        };

        let mut vm = deluge_bsp::fat::new_volume_manager();
        let sd_listing = if sd_ok {
            match file_browser::list_apps(&mut vm) {
                Ok(result) => Some(result),
                Err(e) => {
                    error!("FAT: failed to list /APPS: {:?}", e);
                    None
                }
            }
        } else {
            None
        };

        // Build the menu: on-flash firmware (default) + SD `/APPS/` images, then
        // trailing "DATA TRANSFER" and "DEV MODE" entries.
        let has_flash = flash_img.is_some();
        let flash_offset = has_flash as usize;
        let sd_count = sd_listing.as_ref().map_or(0, |(_, _, e)| e.len());
        let boot_total = flash_offset + sd_count; // real boot targets
        // DATA TRANSFER only needs a *card*, not a working filesystem, and the
        // Deluge's card-detect pin is unreliable (it reads "no card" even with
        // one inserted — see the SSB RTT logs), so gating on detection hides the
        // recovery path exactly when it's needed.  Offer it unconditionally: the
        // SdBlock backend retries `sd::init` on demand when the host probes it,
        // so an unformatted/corrupt/initially-unresponsive card can still be
        // accessed or reformatted from the host; with no card the host simply
        // sees an empty drive.  This also guarantees the menu always has ≥1
        // entry.
        let data_idx = boot_total; // DATA TRANSFER follows the boot targets
        let settings_idx = boot_total + 1; // SETTINGS follows DATA TRANSFER
        let menu_total = boot_total + 2;

        let mut name_refs = [b"".as_slice(); file_browser::MAX_APPS + 3];
        if has_flash {
            name_refs[0] = b"BOOT FLASH";
        }
        if let Some((_, _, entries)) = sd_listing.as_ref() {
            for i in 0..sd_count {
                name_refs[flash_offset + i] = entries.display_name(i);
            }
        }
        name_refs[data_idx] = DATA_MENU_LABEL;
        name_refs[settings_idx] = SETTINGS_MENU_LABEL;

        info!(
            "Boot menu: {} boot entry(ies) (flash={}, dev_mode={})",
            boot_total, has_flash, cfg.dev_mode
        );

        // How to boot: skip the menu entirely (INSTANT), run it with a countdown,
        // or sit on it.  All the precedence lives in the host-tested decision fn.
        let mode = settings::boot_mode(&cfg, boot_total, recovery, auto_boot_allowed);
        // Every pass after this one is user-driven — never auto-boot again.
        auto_boot_allowed = false;
        info!("Boot mode: {:?}", mode);

        let selection = if mode == settings::BootMode::Instant {
            // The menu is never drawn: launch the default entry (index 0 — the
            // flash image when present, else the first SD app).  `boot_mode`
            // guarantees `boot_total > 0` here, so index 0 is a real boot target.
            info!("Auto-boot: INSTANT — launching the default entry");
            ui::Selection {
                index: 0,
                long_press: false,
            }
        } else {
            let countdown = match mode {
                settings::BootMode::Countdown(secs) => secs,
                // Wait: `run_selector` treats 0 as "no countdown, wait for a pick".
                _ => 0,
            };

            // In dev mode, race the menu selector against the background USB upload
            // listener: whichever resolves first wins.  The listener only ever
            // "returns" by loading and launching a received image (it never hands a
            // value back), so the menu branch is the only one that yields a selection
            // here.  Outside dev mode, just run the selector.
            //
            // Crucially, bring the USB device up *before* `run_selector` starts
            // drawing: USB bring-up reconfigures interrupts/clocks, and doing that
            // while an OLED frame DMA + PIC handshake is in flight can wedge the
            // display so the menu never redraws (the proven `usbmsc` path builds USB
            // before starting its OLED loop for the same reason).
            if cfg.dev_mode {
                use embassy_futures::select::{Either, select};
                let listener = devupload::prepare();
                match select(
                    ui::run_selector(&name_refs[..menu_total], 0, countdown),
                    listener.run(),
                )
                .await
                {
                    Either::First(sel) => sel,
                    Either::Second(never) => never,
                }
            } else {
                ui::run_selector(&name_refs[..menu_total], 0, countdown).await
            }
        };
        let ui::Selection {
            index: selected,
            long_press,
        } = selection;

        // The menu branch won (or dev mode is off): if a CDC listener was brought
        // up, tear it down so the port is free for a later DATA TRANSFER MSC init.
        if cfg.dev_mode {
            unsafe { rza1l_hal::usb::disconnect(0) };
        }

        // ---- Long-press on an SD `/APPS` ELF: store it to the flash slot ----
        // Only SD ELF entries are stored to flash; long-pressing the existing
        // FLASH entry or DATA TRANSFER does nothing special (falls through to
        // the normal short-press handling below).
        let is_sd_entry = selected < data_idx && !(has_flash && selected == 0);
        if long_press && is_sd_entry {
            let (root, selected_entry, label) = {
                let (_, root, entries) = sd_listing
                    .as_ref()
                    .expect("sd listing present for SD entry");
                let sd_idx = selected - flash_offset;
                (
                    *root,
                    entries.get(sd_idx).clone(),
                    entries.display_name(sd_idx),
                )
            };
            if ui::confirm_write_to_flash(label).await {
                write_app_to_flash(&mut vm, root, &selected_entry, label).await;
            }
            // Whether confirmed, cancelled, or failed, rebuild the menu so a
            // freshly-stored image shows up as the default FLASH entry.
            if let Some((volume, _, entries)) = sd_listing {
                let _ = vm.close_volume(volume);
                drop(entries);
            }
            drop(vm);
            continue;
        }

        // ---- SETTINGS screen (persists on exit, rebuilds the menu) ----
        // The screen edits a copy; we write flash once, only if something
        // actually changed, then `continue` so the next pass re-reads it (updating
        // the auto-boot mode and whether the USB listener runs).  Nothing is
        // launched.
        if selected == settings_idx {
            // Close FAT handles before touching the flash bus (the settings write
            // leaves memory-mapped read mode, like the app-slot store).
            if let Some((volume, _, entries)) = sd_listing {
                let _ = vm.close_volume(volume);
                drop(entries);
            }
            drop(vm);

            let mut new_cfg = cfg;
            ui::run_settings(&mut new_cfg).await;
            if new_cfg == cfg {
                continue;
            }
            info!("Settings: {:?} -> {:?}", cfg, new_cfg);

            let ok = unsafe { settings::write(&new_cfg).await };
            if ok {
                ui::show_message(b"SETTINGS", b"SAVED").await;
                embassy_time::Timer::after(embassy_time::Duration::from_millis(700)).await;
            } else {
                // The flash write didn't stick (the device stays responsive
                // thanks to the bounded SPIBSC waits). Show the JEDEC ID and
                // status register so the failure can be diagnosed: ID `01 02 20`
                // confirms manual-mode works; status `BP[2:0]` (bits 2-4) set
                // means the settings sector is write-protected.
                let id = rza1l_hal::spibsc::read_id();
                let sr = rza1l_hal::spibsc::read_status_reg();
                error!(
                    "Settings flash write failed: JEDEC={:02x} {:02x} {:02x}, SR={:#04x}",
                    id[0], id[1], id[2], sr
                );
                let mut line = *b"ID...... SR..";
                hex2(&mut line[2..4], id[0]);
                hex2(&mut line[4..6], id[1]);
                hex2(&mut line[6..8], id[2]);
                hex2(&mut line[11..13], sr);
                ui::show_message(b"FLASH WRITE FAIL", &line).await;
                embassy_time::Timer::after(embassy_time::Duration::from_secs(4)).await;
            }
            continue;
        }

        // ---- DATA TRANSFER mode (returns on BACK) ----
        if selected == data_idx {
            if let Some((volume, _, entries)) = sd_listing {
                let _ = vm.close_volume(volume);
                drop(entries);
            }
            drop(vm);
            usbmsc::run_data_transfer_mode().await;
            continue;
        }

        // ---- Flash boot path (never returns) ----
        if has_flash && selected == 0 {
            let img = flash_img.unwrap();
            info!(
                "Booting flash image: start={:#010x} end={:#010x} entry={:#010x}",
                img.code_start, img.code_end, img.entry
            );
            ui::show_message(b"BOOTING", b"FLASH FW").await;

            // Draw the boot droplet so the panel shows it through handoff.
            // Must run before interrupts/DMA are quiesced (see show_boot_logo).
            show_boot_logo().await;

            // Close FAT handles and quiesce, then copy from flash into SRAM via
            // the trampoline (single descriptor) like the SD ELF path.
            cortex_ar::interrupt::disable();
            if let Some((volume, _, entries)) = sd_listing {
                let _ = vm.close_volume(volume);
                drop(entries);
            }
            drop(vm);
            unsafe { quiesce_for_handoff() };

            let desc = img.desc();
            unsafe { launcher::launch_via_trampoline(&[desc], img.entry) }
        }

        // ---- SD ELF path ----
        let (volume, root, entries) =
            sd_listing.expect("sd listing must be present for an SD menu selection");
        let sd_idx = selected - flash_offset;
        let selected_entry = entries.get(sd_idx);
        let label = entries.display_name(sd_idx);
        info!("Loading: {:?}", selected_entry.name);
        ui::show_progress(label, 0).await;

        // Open the selected file from /APPS/.
        let file = match file_browser::open_app(&mut vm, root, selected_entry) {
            Ok(f) => f,
            Err(e) => {
                error!("Failed to open app: {:?}", e);
                ui::show_message(b"OPEN ERROR", b"BACK TO MENU").await;
                embassy_time::Timer::after(embassy_time::Duration::from_secs(2)).await;
                let _ = vm.close_volume(volume);
                drop(vm);
                drop(entries);
                continue;
            }
        };

        // Stream-load ELF segments.  SRAM-targeting segments are staged in SDRAM;
        // SDRAM-targeting segments are written to their final addresses directly.
        let load_result = match unsafe {
            elf::load_from_sd_with_progress(&mut vm, file, |done, total| async move {
                let raw = done.saturating_mul(100).checked_div(total).unwrap_or(100) as u8;
                ui::show_progress(label, raw).await;
            })
            .await
        } {
            Ok(r) => r,
            Err(e) => {
                let line2: &[u8] = match e {
                    elf::ElfError::BadMagic => b"BAD MAGIC",
                    elf::ElfError::WrongFormat => b"WRONG FORMAT",
                    elf::ElfError::BadLoadAddress => b"BAD LOAD ADDR",
                    elf::ElfError::Unstreamable => b"BAD LAYOUT",
                    _ => b"SEE LOG",
                };
                error!("ELF load error: {:?}", e);
                ui::show_message(b"LOAD ERROR", line2).await;
                embassy_time::Timer::after(embassy_time::Duration::from_secs(2)).await;
                let _ = vm.close_file(file);
                let _ = vm.close_volume(volume);
                drop(vm);
                drop(entries);
                continue;
            }
        };

        info!(
            "ELF loaded, entry = {:#010x}, {} SRAM segment(s) staged",
            load_result.entry, load_result.n_sram
        );

        // Draw the boot droplet so the panel shows it through handoff — even if
        // the launched app never uses the display. Must run before interrupts/DMA
        // are quiesced (see show_boot_logo).
        show_boot_logo().await;

        // Disable interrupts and close FAT handles before we hand off.
        cortex_ar::interrupt::disable();
        let _ = vm.close_volume(volume);
        drop(vm);
        drop(entries);

        // Quiesce every peripheral that keeps running on its own.  Masking CPU
        // interrupts is not enough: the SCIF1/PIC receive DMA is a *circular*
        // channel that keeps writing incoming bytes into its ring buffer
        // independently of the CPU, and after the trampoline overwrites SRAM with
        // the app that buffer is the app's memory — corrupting its TTB/stack/data
        // and crashing early init.  Stop all DMA, the OSTM timers, and the GIC so
        // the app boots into a quiet machine like a cold start.
        unsafe { quiesce_for_handoff() };

        if load_result.n_sram > 0 {
            // One or more segments target SRAM.  Use the trampoline in retention
            // RAM to move them after the bootloader's SRAM is no longer needed.
            unsafe {
                launcher::launch_via_trampoline(
                    &load_result.sram_descs[..load_result.n_sram],
                    load_result.entry,
                )
            }
        } else {
            // Pure SDRAM app — all segments already at their final addresses.
            unsafe { launcher::launch(load_result.entry) }
        }
    }
}
