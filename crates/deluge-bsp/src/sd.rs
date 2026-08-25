//! Deluge-BSP SD card driver.
//!
//! Exposes an async interface for reading and writing 512-byte sectors on the
//! Deluge's SD card (SDHI port 1).
//!
//! Dual-target: on the firmware (`target_os = "none"`) this drives the real
//! SDHI1 controller ([`device`]); on host/QEMU (`target_os != "none"`) there is
//! no SDHI hardware, so the same public API ([`init`], [`read_sectors`],
//! [`write_sectors`], [`is_ready`], [`is_inserted`], [`is_write_protected`],
//! [`total_sectors`]) is instead backed by a small file-backed disk image
//! ([`host`]), so callers (e.g. `deluge-bsp-rust`'s FatFS diskio shim) need no
//! `#[cfg]` of their own.
//!
//! ## SD protocol overview (device)
//!
//! [`init`] runs the full SD v2 initialization sequence:
//!   CMD0  → reset card to idle
//!   CMD8  → check host voltage (determines SDHC/SDXC support)
//!   ACMD41 (loop) → wait for card ready, learn high-capacity flag
//!   CMD2  → get CID (ignored here)
//!   CMD3  → get RCA
//!   CMD9  → get CSD (card capacity — requires RCA, card in Stand-by state)
//!   CMD7  → select card
//!   ACMD6 → switch to 4-bit bus
//!   CMD16 → set block length to 512 (needed for SDSC cards)
//!   CMD6  → query + switch to High-Speed mode (skipped on old cards)
//!   Switch clock from ~130 kHz (P1/512) to 33.3 MHz (P1/2, High-Speed) or
//!   16.7 MHz (P1/4) if the CMD6 switch is unsupported/refused
//!
//! Sector addressing:
//!   - SDHC/SDXC cards: block address (LBA directly)
//!   - SDSC cards: byte address (LBA × 512)
//!
//! ## Usage
//!
//! ```ignore
//! sd::init().await.expect("SD init failed");
//! let mut buf = [0u8; 512];
//! sd::read_sector(0, &mut buf).await.expect("read failed");
//! ```

// ---------------------------------------------------------------------------
// Error type (shared across device + host)
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SdError {
    /// Low-level SDHI hardware error. Device-only.
    #[cfg(target_os = "none")]
    Hardware(rza1l_hal::sdhi::SdhiError),
    /// Card not present.
    NoCard,
    /// Unsupported card type (e.g. MMC, old SD v1 without sane CMD8 response).
    UnsupportedCard,
    /// Protocol violation — unexpected response.
    Protocol,
    /// Driver not initialized; call [`init`] first.
    NotInitialized,
    /// Host file-backed I/O failure (open/seek/read/write on the backing
    /// image). Host-only.
    #[cfg(not(target_os = "none"))]
    HostIo(std::io::ErrorKind),
}

#[cfg(target_os = "none")]
impl From<rza1l_hal::sdhi::SdhiError> for SdError {
    fn from(e: rza1l_hal::sdhi::SdhiError) -> Self {
        SdError::Hardware(e)
    }
}

// ---------------------------------------------------------------------------
// CMD6 (SWITCH_FUNC) status block (shared: parsed on device, tested on host)
// ---------------------------------------------------------------------------

/// Decoded fields of the CMD6 (SWITCH_FUNC) 512-bit status block.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct SwitchStatus {
    /// Function group 1 (access mode) supports function 1 (High-Speed).
    pub hs_supported: bool,
    /// Function group 1 selected-function nibble
    /// (0x1 = High-Speed selected, 0xF = switch refused / function error).
    pub group1_selected: u8,
}

/// Parse the CMD6 switch-function status block.
///
/// The block arrives MSB-first: `buf[0]` holds status bits 511:504.
///   - Group-1 support mask = status bits 415:400 → bytes 12–13;
///     function 1 (High-Speed) = bit 401 = byte 13, bit 1.
///   - Group-1 selected function = status bits 379:376 → byte 16 low nibble.
pub(crate) fn parse_switch_status(buf: &[u8; 64]) -> SwitchStatus {
    SwitchStatus {
        hs_supported: buf[13] & 0x02 != 0,
        group1_selected: buf[16] & 0x0F,
    }
}

#[cfg(target_os = "none")]
pub use device::{
    init, invalidate, is_hc, is_hs, is_inserted, is_ready, is_write_protected, read_sectors,
    take_card_detect_events, total_sectors, write_sectors,
};
// The embedded-sdmmc adapters only exist when the `fat` feature pulls that crate in.
#[cfg(all(target_os = "none", feature = "fat"))]
pub use device::{DelugeBlockDevice, DelugeTimeSource, PartitionShim};
#[cfg(not(target_os = "none"))]
pub use host::{
    init, invalidate, is_inserted, is_ready, is_write_protected, read_sectors,
    take_card_detect_events, total_sectors, write_sectors,
};

// ---------------------------------------------------------------------------
// Device: real SDHI1 hardware driver
// ---------------------------------------------------------------------------

#[cfg(target_os = "none")]
mod device {
    use core::sync::atomic::{AtomicBool, AtomicU16, Ordering};

    use rza1l_hal::UNCACHED_MIRROR_OFFSET;
    use rza1l_hal::cache;
    use rza1l_hal::dmac;
    use rza1l_hal::sdhi::{self, SdhiError};

    use super::SdError;
    use crate::system::{SD_DMA_MAX_SECTORS, SD_DMA_RX_CH, SD_DMA_TX_CH};

    // The Deluge SD card is on SDHI port 1.
    const SD_PORT: u8 = 1;

    // ---------------------------------------------------------------------------
    // Well-known SD command register values (written to SD_CMD register).
    //
    // Most commands: lower 6 bits = command index.  The SDHI hardware infers the
    // response type from the command index internally.
    // CMD8 has bits [10:8] set (= 0x0408) to select the R7 response path.
    // ---------------------------------------------------------------------------

    const CMD0: u16 = 0; // GO_IDLE_STATE — no response
    const CMD2: u16 = 2; // ALL_SEND_CID — R2
    const CMD3: u16 = 3; // SEND_RELATIVE_ADDR — R6
    const CMD7: u16 = 7; // SELECT_CARD — R1b
    const CMD8: u16 = 0x0408; // SEND_IF_COND — R7 (special encoding)
    const CMD12: u16 = 12; // STOP_TRANSMISSION — R1b
    const CMD16: u16 = 16; // SET_BLOCKLEN — R1
    const CMD17: u16 = 17; // READ_SINGLE_BLOCK — R1 + data
    const CMD18: u16 = 18; // READ_MULTIPLE_BLOCK — R1 + data
    const CMD24: u16 = 24; // WRITE_BLOCK — R1 + data
    const CMD25: u16 = 25; // WRITE_MULTIPLE_BLOCK — R1 + data
    const CMD55: u16 = 55; // APP_CMD (prefix for ACMD) — R1

    /// CMD6 (SWITCH_FUNC) — extended mode, single-block read, R1 + 64-byte data.
    /// SD_CMD encoding 0x1C06 per TRM table 38.7 (MD[2:0]=100 extended/R1,
    /// MD3 = with-data, MD4 = read, MD5 = 0 single-block).  Normal mode cannot
    /// be used: the SDHI does not auto-decode CMD6 as a data command.
    const CMD6_DATA: u16 = 0x1C06;

    /// CMD6 mode-0 (query) argument: group 1 → function 1, groups 2–6 = 0xF
    /// (no change).  Answers "is High-Speed switchable?" without switching.
    const CMD6_ARG_QUERY_HS: u32 = 0x00FF_FF01;
    /// CMD6 mode-1 (switch) argument: switch group 1 to function 1 (High-Speed).
    const CMD6_ARG_SWITCH_HS: u32 = 0x80FF_FF01;

    /// ACMD6 (0x40 | 6): SET_BUS_WIDTH — R1
    const ACMD6: u16 = 0x40 | 6;
    /// ACMD41 (0x40 | 41): SD_SEND_OP_COND — R3
    const ACMD41: u16 = 0x40 | 41;

    // Multiple-block READ uses the SDHI's *extended* transfer mode (SD_CMD[15:8]):
    // bit 13 = multiple block, bit 12 = read, bit 11 = with-data, [10:8]=100 = R1,
    // [15:14]=01 = CMD12 not auto-issued. This is the Renesas driver's
    // `CMD18 | 0x7c00`.
    //
    // Single-block reads and *all* writes use *normal* mode — the plain command
    // index — exactly as the vendor HAL does (`_sd_send_mcmd(hndl, CMD17/24/25)`):
    // the SDHI derives the response and transfer type from the command index.
    //
    // IMPORTANT: high-capacity vs standard cards differ only in the command
    // *argument* (block- vs byte-address, handled by `lba_to_addr`), NOT in the
    // command encoding. The old `CMDxx | 0x7C00` "SDHC" constants were a bug:
    // 0x7C00 forces *multiple-block read* mode, so a single-block CMD17 made the
    // controller wait for a second block until the data-timeout (~1s), and the
    // write commands additionally got the read-direction bit set.
    const CMD18_MULTI: u16 = CMD18 | 0x7C00;

    // ---------------------------------------------------------------------------
    // Voltage / capacity constants
    // ---------------------------------------------------------------------------

    /// OCR voltage window: 3.2–3.4 V (matches Deluge hardware, SD_VOLT_3_3).
    const OCR_VDD_32_33: u32 = 0x0010_0000;
    /// OCR host capacity support: HCS bit (high-capacity) for SDHC/SDXC.
    const OCR_HCS: u32 = 0x4000_0000;
    /// OCR power-up status bit: set when card is ready.
    const OCR_BUSY: u32 = 0x8000_0000;

    /// CMD8 argument: VHS=1 (2.7–3.6 V) | check pattern 0xAA.
    const CMD8_ARG: u32 = 0x0000_01AA;

    /// ACMD41 argument: HCS + voltage window.
    const ACMD41_ARG: u32 = OCR_HCS | OCR_VDD_32_33;

    // ---------------------------------------------------------------------------
    // Global card state
    // ---------------------------------------------------------------------------

    /// RCA (Relative Card Address) — set during CMD3, used for CMD7 etc.
    static CARD_RCA: AtomicU16 = AtomicU16::new(0);
    /// `true` if the card is SDHC/SDXC (uses block addressing).
    static CARD_HC: AtomicBool = AtomicBool::new(false);
    /// `true` if the card accepted the CMD6 switch to High-Speed mode
    /// (SD_CLK = 33.3 MHz).  `false` = default speed (16.7 MHz).
    static CARD_HS: AtomicBool = AtomicBool::new(false);
    /// `true` once `init()` has completed successfully.
    static CARD_READY: AtomicBool = AtomicBool::new(false);

    // ---------------------------------------------------------------------------
    // DMA bounce buffer
    // ---------------------------------------------------------------------------
    //
    // DMA requires uncached memory.  Callers pass arbitrary (possibly cached)
    // buffers, so we maintain a statically-allocated bounce buffer and access it
    // through its uncached alias (physical address | 0x4000_0000).
    //
    // Access is serialised by the single-task async design: only one SD operation
    // runs at a time, so no locking is needed.

    #[repr(align(32))]
    struct SdDmaBuf([u8; SD_DMA_MAX_SECTORS * 512]);
    // Safety: access is serialised by the async executor (single SD task).
    static mut SD_DMA_BUF: SdDmaBuf = SdDmaBuf([0u8; SD_DMA_MAX_SECTORS * 512]);

    // ---------------------------------------------------------------------------
    // Internal helpers
    // ---------------------------------------------------------------------------

    /// Issue CMD55 (APP_CMD prefix) with the current RCA, then issue `acmd`.
    async unsafe fn send_acmd(acmd: u16, arg: u32) -> Result<(), SdError> {
        unsafe {
            let rca = CARD_RCA.load(Ordering::Relaxed);
            sdhi::set_arg(SD_PORT, (rca as u32) << 16);
            sdhi::send_cmd(SD_PORT, CMD55).await?;
            sdhi::set_arg(SD_PORT, arg);
            sdhi::send_cmd(SD_PORT, acmd).await?;
            Ok(())
        }
    }

    /// Attempt the CMD6 switch to High-Speed mode (SD spec ≥ 1.10 cards).
    ///
    /// Runs after the card is in Transfer state (post CMD7/ACMD6).  On success
    /// the *card* is in High-Speed mode and the caller may raise SD_CLK to
    /// P1/2 (33.3 MHz).  Every error is non-fatal to init: the caller falls
    /// back to the default-speed clock.  Pre-1.10 cards reject CMD6 as an
    /// illegal command → surfaces here as a response error → fallback.
    async unsafe fn try_high_speed() -> Result<(), SdError> {
        unsafe {
            let mut status = [0u8; 64];

            // Mode 0 (query): does the card support group-1 function 1?
            sdhi::read_status_block_sw(SD_PORT, CMD6_DATA, CMD6_ARG_QUERY_HS, &mut status).await?;
            let st = super::parse_switch_status(&status);
            if !st.hs_supported {
                return Err(SdError::UnsupportedCard);
            }

            // Mode 1 (switch): actually switch to High-Speed.
            sdhi::read_status_block_sw(SD_PORT, CMD6_DATA, CMD6_ARG_SWITCH_HS, &mut status).await?;
            let st = super::parse_switch_status(&status);
            if st.group1_selected != 0x1 {
                return Err(SdError::Protocol);
            }
            Ok(())
        }
    }

    // ---------------------------------------------------------------------------
    // Public API
    // ---------------------------------------------------------------------------

    /// Initialize the SD card (hardware + protocol).
    ///
    /// Must be called once, with the GIC initialized and SDHI IRQs registered.
    /// Safe to call again after a card-swap event.
    ///
    /// # Returns
    /// `Ok(())` on success; `Err(SdError)` on any failure.
    pub async fn init() -> Result<(), SdError> {
        log::debug!("sd: init port {}", SD_PORT);
        CARD_READY.store(false, Ordering::Release);
        // Reset the cached RCA to 0 *before* re-identifying.  During identification
        // the card sits in idle/ready state with RCA = 0, so the CMD55 that prefixes
        // ACMD41 must be addressed to RCA 0 (see `send_acmd`).  On a cold boot the
        // static is already 0, but on a re-init (e.g. returning from USB mass-storage
        // mode) it still holds the *previous* session's RCA from CMD3 — CMD55 would
        // then address a card that no longer answers to it, so every attempt returns
        // ResponseTimeout and the card looks dead.  Clearing it here makes re-init
        // behave exactly like the first boot.
        CARD_RCA.store(0, Ordering::Release);
        // Reset the High-Speed flag too: if this (re-)init fails, is_hs() must not
        // report a stale `true` from the previous session.
        CARD_HS.store(false, Ordering::Release);

        // ---- Bring up the SDHI controller + pins (once) ----
        unsafe {
            // ---- SD pin mux: P7_0–7 → SDHI1 (function 3) ----
            rza1l_hal::gpio::set_pin_mux(7, 0, 3); // SD_CD1  — card detect
            rza1l_hal::gpio::set_pin_mux(7, 1, 3); // SD_WP1  — write protect
            rza1l_hal::gpio::set_pin_mux(7, 2, 3); // SD_D11  — data bit 1
            rza1l_hal::gpio::set_pin_mux(7, 3, 3); // SD_D01  — data bit 0
            rza1l_hal::gpio::set_pin_mux(7, 4, 3); // SD_CLK1 — clock
            rza1l_hal::gpio::set_pin_mux(7, 5, 3); // SD_CMD1 — command
            rza1l_hal::gpio::set_pin_mux(7, 6, 3); // SD_D31  — data bit 3
            rza1l_hal::gpio::set_pin_mux(7, 7, 3); // SD_D21  — data bit 2

            sdhi::init(SD_PORT, crate::system::SD_OPTION);
            sdhi::register_irqs(SD_PORT);

            // Register the DMAC completion IRQ for the RX channel so that
            // read_blocks_dma can await DMAC TC after DATA_TRNS (M7 fix).
            dmac::register_completion_irq(SD_DMA_RX_CH);

            // Clean-invalidate the DMA bounce buffer's cacheable alias.  BSS
            // zeroing wrote dirty lines there; if those lines were later evicted
            // after a DMA fill via the uncached mirror, they would corrupt the
            // received data (M8 fix).
            let buf_start = core::ptr::addr_of!(SD_DMA_BUF) as usize;
            let buf_end = buf_start + core::mem::size_of::<SdDmaBuf>();
            cache::dma_clean_inv_range(buf_start, buf_end);
        }

        // ---- Wait for the card to wake up, then run the protocol ----
        //
        // On a *cold* power-up the card only starts its internal power-on once the
        // pins are muxed and the clock starts (above).  Critically, the SDHI
        // card-detect status also only reports "present" after `Ncycle` SD_CLK
        // cycles elapse with SD_CD held low (SD_OPTION[3:0]); until the card has
        // woken up it won't answer, so CMD55 returns ResponseTimeout (or the OCR
        // busy bit never clears → UnsupportedCard).  The stock Renesas driver simply
        // blind-waits ~1 s here.  Instead we retry the protocol over a ~1 s budget
        // and return the instant it succeeds: a *warm* card (already awake from a
        // previous session) is ready on the first try, while a *cold* card is given
        // the full second it needs — no manual reload required.
        const INIT_ATTEMPTS: u32 = 16;
        const RETRY_DELAY_MS: u64 = 75; // 16 × ~(protocol + 75 ms) ≈ 1.3 s budget

        // Minimum supply/clock settle (SD spec: ≥1 ms + 74 clocks) before CMD0.
        embassy_time::Timer::after_millis(15).await;

        let mut last_err = SdError::NotInitialized;
        for attempt in 1..=INIT_ATTEMPTS {
            match run_protocol().await {
                Ok(()) => {
                    CARD_READY.store(true, Ordering::Release);
                    // Consume the card-detect edges accumulated up to here: they
                    // describe the arrival of the card we have just identified, so
                    // reporting them onward would ask a caller to redo this work.
                    // Must come after the bring-up above, not before it —
                    // `sdhi::init` is what ungates the SDHI module clock, and the
                    // INFO1 read is only valid once it has. If the card was pulled
                    // mid-init this discards that removal edge too, which is what
                    // `deluge_block_poll_card_event`'s level check is there to
                    // catch.
                    unsafe { sdhi::take_card_detect_events(SD_PORT) };
                    log::debug!(
                        "sd: card ready on attempt {}/{} (HC={})",
                        attempt,
                        INIT_ATTEMPTS,
                        CARD_HC.load(Ordering::Relaxed)
                    );
                    return Ok(());
                }
                Err(e) => {
                    last_err = e;
                    log::debug!("sd: init attempt {}/{}: {:?}", attempt, INIT_ATTEMPTS, e);
                    if attempt < INIT_ATTEMPTS {
                        embassy_time::Timer::after_millis(RETRY_DELAY_MS).await;
                    }
                }
            }
        }
        log::warn!(
            "sd: init failed after {} attempts: {:?}",
            INIT_ATTEMPTS,
            last_err
        );
        Err(last_err)
    }

    /// Discard everything cached about the card, so no transfer is attempted until
    /// [`init`] has run again.
    ///
    /// Call this the moment a removal is detected. Every cached value here — the
    /// RCA assigned by CMD3, the SDHC/high-speed flags, the CSD-derived capacity —
    /// describes *that* card, and none of it carries over to whatever is inserted
    /// next. Left in place, the driver keeps addressing the old card's RCA: the new
    /// card never answers, so reads fail (and, because the SDHI command waits are
    /// interrupt-driven with no deadline of their own, a transfer that draws neither
    /// a completion nor an error interrupt does not fail but simply never returns).
    /// Clearing `CARD_READY` makes [`read_sectors`]/[`write_sectors`] refuse with
    /// [`SdError::NotInitialized`] instead, so the failure is immediate and visible.
    ///
    /// [`is_ready`] reads false afterwards, which is also what tells a caller
    /// polling for the card to come back that it has not come back *yet*.
    pub fn invalidate() {
        CARD_READY.store(false, Ordering::Release);
        CARD_RCA.store(0, Ordering::Release);
        CARD_HC.store(false, Ordering::Release);
        CARD_HS.store(false, Ordering::Release);
        // total_sectors() must not answer with the old card's capacity either.
        sdhi::set_card_blocks(SD_PORT, 0);
    }

    /// Consume the latched card insert/remove edges: `(removed, inserted)`.
    ///
    /// Edges, not levels — see [`sdhi::take_card_detect_events`] for why that
    /// distinction matters (a swap between two polls is invisible to a level read).
    /// Both `true` means the card was replaced.
    pub fn take_card_detect_events() -> (bool, bool) {
        // SAFETY: reads/clears SD_INFO1's card-detect latch for the port this
        // module owns; the clear is written so no other status flag is disturbed.
        unsafe { sdhi::take_card_detect_events(SD_PORT) }
    }

    /// Run the SD v2 card bring-up protocol (CMD0 … high-speed clock) once against
    /// an already-initialised controller.  [`init`] retries this until the card —
    /// which may still be waking up after a cold power-on — responds.
    async fn run_protocol() -> Result<(), SdError> {
        unsafe {
            // ---- CMD0: reset to IDLE ----
            // No response expected; ignore timeout.
            sdhi::set_arg(SD_PORT, 0);
            let _ = sdhi::send_cmd(SD_PORT, CMD0).await;

            embassy_time::Timer::after_millis(1).await;

            // ---- CMD8: check voltage — determines SD v2 / SDHC capability ----
            sdhi::set_arg(SD_PORT, CMD8_ARG);
            let cmd8_ok = sdhi::send_cmd(SD_PORT, CMD8).await.is_ok();

            // ---- ACMD41: initialize card ----
            // Loop until the card clears the busy bit in OCR (card-power-up done).
            let hcs_arg = if cmd8_ok { ACMD41_ARG } else { OCR_VDD_32_33 };
            let mut retries = 0u32;
            let ocr = loop {
                send_acmd(ACMD41, hcs_arg).await?;
                let ocr = sdhi::read_r1(SD_PORT); // R3 OCR comes via same regs

                // Bit 31 set → card no longer busy (initialization complete)
                if ocr & OCR_BUSY != 0 {
                    break ocr;
                }
                retries += 1;
                if retries > 1000 {
                    return Err(SdError::UnsupportedCard);
                }
                embassy_time::Timer::after_millis(1).await;
            };

            // Determine high-capacity flag
            let hc = cmd8_ok && (ocr & OCR_HCS != 0);
            CARD_HC.store(hc, Ordering::Release);
            log::debug!(
                "sd: cmd8_ok={} ocr={:#010x} hcs={} -> hc={}",
                cmd8_ok,
                ocr,
                ocr & OCR_HCS != 0,
                hc
            );

            // ---- CMD2: get CID (ignore content, just consume response) ----
            sdhi::set_arg(SD_PORT, 0);
            sdhi::send_cmd(SD_PORT, CMD2).await?;
            let _ = sdhi::read_r2(SD_PORT); // CID — discard

            // ---- CMD3: get RCA ----
            sdhi::set_arg(SD_PORT, 0);
            sdhi::send_cmd(SD_PORT, CMD3).await?;
            let r6 = sdhi::read_r1(SD_PORT);
            // R6 = [31:16] new RCA, [15:0] card status
            let rca = (r6 >> 16) as u16;
            CARD_RCA.store(rca, Ordering::Release);

            // ---- CMD9: get CSD (decode capacity for BlockDevice) ----
            // CMD9 requires the card to be in Stand-by state (post CMD3) with its RCA.
            // Per TRM/SD spec: argument = RCA in bits [31:16], lower 16 bits = 0.
            sdhi::set_arg(SD_PORT, (rca as u32) << 16);
            if sdhi::send_cmd(SD_PORT, 9u16).await.is_ok() {
                // The RZ/A1 SDHI returns R2 (CID/CSD) as the 120-bit content
                // CSD[127:8] right-justified — i.e. the array holds (full_CSD >> 8)
                // with the trailing CRC byte stripped.  Shift the four words left by
                // 8 bits to restore the canonical 128-bit CSD layout that
                // `decode_csd_capacity` (which uses spec bit positions) expects.
                let raw = sdhi::read_r2(SD_PORT);
                let csd = [
                    (raw[0] << 8) | (raw[1] >> 24),
                    (raw[1] << 8) | (raw[2] >> 24),
                    (raw[2] << 8) | (raw[3] >> 24),
                    raw[3] << 8,
                ];
                let total = decode_csd_capacity(csd, hc);
                log::debug!(
                    "sd: CSD={:08x} {:08x} {:08x} {:08x} struct={} -> {} sectors",
                    csd[0],
                    csd[1],
                    csd[2],
                    csd[3],
                    csd[0] >> 30,
                    total
                );
                sdhi::set_card_blocks(SD_PORT, total);
            }

            // ---- CMD7: select card (transition to Transfer state) ----
            sdhi::set_arg(SD_PORT, (rca as u32) << 16);
            sdhi::send_cmd(SD_PORT, CMD7).await?;

            // ---- ACMD6: set 4-bit bus ----
            // Argument 0x2 = 4-bit, 0x0 = 1-bit.
            send_acmd(ACMD6, 0x2).await?;

            // ---- CMD16: set block length to 512 bytes (SDSC cards) ----
            if !hc {
                sdhi::set_arg(SD_PORT, 512);
                sdhi::send_cmd(SD_PORT, CMD16).await?;
            }

            // ---- Clock: default-speed data clock BEFORE CMD6 ----
            // CMD6 is a *transfer-mode* command: it must not run at the 130 kHz
            // identification clock.  Some cards never deliver its 64-byte data
            // block there (observed on hardware: the controller then sits in
            // the data-phase wait until the SD_OPTION timeout — 2^27 SD_CLK
            // cycles ≈ 17 minutes at 130 kHz).  16.7 MHz is legal for every
            // card in Transfer state, and puts that timeout at ~8 s.
            sdhi::set_clock_fast(SD_PORT);

            // ---- Try CMD6 High-Speed (33.3 MHz), else stay at 16.7 MHz ----
            // The card is in Transfer state; CMD6 needs the data lines, so this
            // must come after ACMD6 (4-bit bus).  The SD spec allows the new
            // timing 8 clocks after the switch-status end bit — DATA_TRNS
            // (awaited inside read_status_block_sw) is past that point.
            match try_high_speed().await {
                Ok(()) => {
                    sdhi::set_clock_div(SD_PORT, sdhi::CLK_DIV_2);
                    CARD_HS.store(true, Ordering::Release);
                    log::info!("sd: High-Speed mode, SD_CLK = 33.3 MHz (P1/2)");
                }
                Err(e) => {
                    // Already at the default-speed clock; nothing to restore.
                    CARD_HS.store(false, Ordering::Release);
                    log::info!("sd: default speed, SD_CLK = 16.7 MHz (P1/4): {:?}", e);
                }
            }
        }

        Ok(())
    }

    /// Read a single 512-byte sector at logical block address `lba`.
    ///
    /// # Arguments
    /// * `lba`  — sector number (0-based).
    /// * `buf`  — destination buffer; must be exactly 512 bytes.
    pub async fn read_sector(lba: u32, buf: &mut [u8; 512]) -> Result<(), SdError> {
        if !CARD_READY.load(Ordering::Acquire) {
            return Err(SdError::NotInitialized);
        }

        let addr = lba_to_addr(lba);
        let cmd = CMD17;

        unsafe {
            let dma_ptr =
                (core::ptr::addr_of!(SD_DMA_BUF.0[0]) as usize + UNCACHED_MIRROR_OFFSET) as *mut u8;
            sdhi::set_block_count(SD_PORT, 1);
            sdhi::set_arg(SD_PORT, addr);
            sdhi::read_blocks_dma(SD_PORT, cmd, dma_ptr, 1, SD_DMA_RX_CH).await?;
            buf.copy_from_slice(core::slice::from_raw_parts(dma_ptr, 512));
        }
        Ok(())
    }

    /// Write a single 512-byte sector at logical block address `lba`.
    ///
    /// # Arguments
    /// * `lba`  — sector number (0-based).
    /// * `buf`  — source data; must be exactly 512 bytes.
    pub async fn write_sector(lba: u32, buf: &[u8; 512]) -> Result<(), SdError> {
        if !CARD_READY.load(Ordering::Acquire) {
            return Err(SdError::NotInitialized);
        }

        let addr = lba_to_addr(lba);
        let cmd = CMD24;

        unsafe {
            let dma_ptr =
                (core::ptr::addr_of!(SD_DMA_BUF.0[0]) as usize + UNCACHED_MIRROR_OFFSET) as *mut u8;
            core::ptr::copy_nonoverlapping(buf.as_ptr(), dma_ptr, 512);
            sdhi::set_block_count(SD_PORT, 1);
            sdhi::set_arg(SD_PORT, addr);
            sdhi::write_blocks_dma(SD_PORT, cmd, dma_ptr as *const u8, 1, SD_DMA_TX_CH).await?;
        }
        Ok(())
    }

    /// Read `count` consecutive sectors starting at `lba`.
    ///
    /// Uses CMD18 (READ_MULTIPLE_BLOCK) when `count > 1`, CMD17 otherwise.
    ///
    /// # Arguments
    /// * `lba`   — first sector (0-based).
    /// * `count` — number of sectors.
    /// * `buf`   — destination; must hold exactly `count * 512` bytes.
    pub async fn read_sectors(lba: u32, count: u32, buf: &mut [u8]) -> Result<(), SdError> {
        if !CARD_READY.load(Ordering::Acquire) {
            return Err(SdError::NotInitialized);
        }
        if buf.len() < (count as usize) * 512 {
            return Err(SdError::Protocol);
        }
        if count == 0 {
            return Ok(());
        }
        if count == 1 {
            let arr = buf[..512].as_mut_ptr() as *mut [u8; 512];
            return read_sector(lba, unsafe { &mut *arr }).await;
        }

        unsafe {
            let dma_ptr =
                (core::ptr::addr_of!(SD_DMA_BUF.0[0]) as usize + UNCACHED_MIRROR_OFFSET) as *mut u8;
            let mut remaining = count;
            let mut cur_lba = lba;
            let mut buf_offset = 0usize;
            while remaining > 0 {
                let chunk = remaining.min(SD_DMA_MAX_SECTORS as u32);
                let chunk_addr = lba_to_addr(cur_lba);
                let cmd = if chunk > 1 { CMD18_MULTI } else { CMD17 };
                sdhi::set_block_count(SD_PORT, chunk);
                sdhi::set_arg(SD_PORT, chunk_addr);
                let xfer = sdhi::read_blocks_dma(SD_PORT, cmd, dma_ptr, chunk, SD_DMA_RX_CH).await;
                // CMD18 (READ_MULTIPLE_BLOCK) runs in extended mode with auto-CMD12
                // disabled (CMD18 | 0x7C00, [15:14]=01), so the card keeps streaming
                // data until it receives STOP_TRANSMISSION.  The SD_STOP SEC bit only
                // bounds how many blocks the *controller* clocks in — it does not stop
                // the *card*.  Issue CMD12 manually (matching the vendor driver) —
                // and do it **even if the data phase failed**, or a bailed-out
                // multi-block read leaves the card streaming and the next command (or
                // the next sd::init) times out.  Propagate the transfer error first
                // since it's the more relevant failure, then any stop error.
                let stop = if chunk > 1 {
                    sdhi::stop_transfer(SD_PORT).await
                } else {
                    Ok(())
                };
                xfer?;
                stop?;
                let chunk_bytes = (chunk as usize) * 512;
                buf[buf_offset..buf_offset + chunk_bytes]
                    .copy_from_slice(core::slice::from_raw_parts(dma_ptr, chunk_bytes));
                remaining -= chunk;
                cur_lba += chunk;
                buf_offset += chunk_bytes;
            }
        }
        Ok(())
    }

    /// Write `count` consecutive sectors starting at `lba`.
    ///
    /// Uses CMD25 (WRITE_MULTIPLE_BLOCK) when `count > 1`, CMD24 otherwise.
    pub async fn write_sectors(lba: u32, count: u32, buf: &[u8]) -> Result<(), SdError> {
        if !CARD_READY.load(Ordering::Acquire) {
            return Err(SdError::NotInitialized);
        }
        if buf.len() < (count as usize) * 512 {
            return Err(SdError::Protocol);
        }
        if count == 0 {
            return Ok(());
        }
        if count == 1 {
            let arr = buf[..512].as_ptr() as *const [u8; 512];
            return write_sector(lba, unsafe { &*arr }).await;
        }

        unsafe {
            let dma_ptr =
                (core::ptr::addr_of!(SD_DMA_BUF.0[0]) as usize + UNCACHED_MIRROR_OFFSET) as *mut u8;
            let mut remaining = count;
            let mut cur_lba = lba;
            let mut buf_offset = 0usize;
            while remaining > 0 {
                let chunk = remaining.min(SD_DMA_MAX_SECTORS as u32);
                let chunk_addr = lba_to_addr(cur_lba);
                let cmd = if chunk > 1 { CMD25 } else { CMD24 };
                let chunk_bytes = (chunk as usize) * 512;
                core::ptr::copy_nonoverlapping(buf.as_ptr().add(buf_offset), dma_ptr, chunk_bytes);
                sdhi::set_block_count(SD_PORT, chunk);
                sdhi::set_arg(SD_PORT, chunk_addr);
                sdhi::write_blocks_dma(SD_PORT, cmd, dma_ptr as *const u8, chunk, SD_DMA_TX_CH)
                    .await?;
                remaining -= chunk;
                cur_lba += chunk;
                buf_offset += chunk_bytes;
            }
        }
        Ok(())
    }

    /// Returns `true` if `init()` completed successfully and the card is ready.
    pub fn is_ready() -> bool {
        CARD_READY.load(Ordering::Acquire)
    }

    /// Returns `true` if the card is a High Capacity (SDHC/SDXC) card.
    ///
    /// Only valid after [`init()`] has returned `Ok(())`.
    pub fn is_hc() -> bool {
        CARD_HC.load(Ordering::Relaxed)
    }

    /// Return `true` if the card is running in High-Speed mode (33.3 MHz SD_CLK).
    pub fn is_hs() -> bool {
        CARD_HS.load(Ordering::Acquire)
    }

    /// Returns `true` if a card is physically present (CD pin).
    pub fn is_inserted() -> bool {
        unsafe { sdhi::card_inserted(SD_PORT) }
    }

    /// Returns `true` if the card's physical write-protect (lock) tab is engaged.
    ///
    /// Reads the SDHI SD_WP signal.  Only meaningful once the controller is up
    /// (after [`init`]); returns `false` when no card is ready.
    pub fn is_write_protected() -> bool {
        if !CARD_READY.load(Ordering::Acquire) {
            return false;
        }
        unsafe { sdhi::card_write_protected(SD_PORT) }
    }

    /// Total number of 512-byte sectors on the card.
    ///
    /// Only valid after [`init()`] has returned `Ok(())`.  Returns 0 if the card
    /// is not ready.  Used to answer SCSI READ CAPACITY for USB mass storage.
    pub fn total_sectors() -> u32 {
        if !CARD_READY.load(Ordering::Acquire) {
            return 0;
        }
        sdhi::card_size_blocks(SD_PORT)
    }

    // ---------------------------------------------------------------------------
    // Internal: convert LBA to card address
    // ---------------------------------------------------------------------------

    fn lba_to_addr(lba: u32) -> u32 {
        if CARD_HC.load(Ordering::Relaxed) {
            lba // SDHC/SDXC: block addressing
        } else {
            // SDSC: byte addressing.  Saturate rather than panic on overflow — a
            // bogus high LBA (e.g. a host reading past a mis-decoded capacity)
            // would otherwise multiply-overflow u32.  A saturated address is out of
            // range for the card, which rejects it and surfaces a clean `SdError`
            // instead of crashing the firmware.
            lba.saturating_mul(512)
        }
    }

    // ---------------------------------------------------------------------------
    // CSD capacity decode
    // ---------------------------------------------------------------------------
    //
    // CSD v1 (SDSC): C_SIZE[73:62], C_SIZE_MULT[49:47], READ_BL_LEN[83:80].
    //   capacity = (C_SIZE + 1) × 2^(C_SIZE_MULT + 2) × 2^READ_BL_LEN  bytes.
    //   blocks (512B) = capacity / 512.
    //
    // CSD v2 (SDHC/SDXC): C_SIZE[69:48].
    //   capacity = (C_SIZE + 1) × 512 KiB  →  (C_SIZE + 1) × 1024  blocks.
    //
    // The SDHI R2 response is packed into [word0, word1, word2, word3] where
    // word0 = bits [127:96] of the 128-bit register.
    // CSD_STRUCTURE is bits [127:126] of CSD.

    fn decode_csd_capacity(csd: [u32; 4], hc: bool) -> u32 {
        if hc {
            // CSD v2: C_SIZE at bits [69:48] within the 128-bit CSD.
            // In our layout: word0=[127:96], word1=[95:64], word2=[63:32], word3=[31:0].
            // Bit 69 → word1 bit (69-64) = 5; bit 48 → word2 bit (48-32) = 16.
            // C_SIZE spans word1[5:0] and word2[31:16].
            let c_size_hi = csd[1] & 0x3F; // bits [69:64]
            let c_size_lo = (csd[2] >> 16) & 0xFFFF; // bits [63:48]
            let c_size = (c_size_hi << 16) | c_size_lo;
            (c_size + 1) * 1024 // blocks of 512 B
        } else {
            // CSD v1:
            // READ_BL_LEN at bits [83:80] → word1[19:16]
            // C_SIZE at bits [73:62] → word1[9:0] (bits 73:64) ++ word2[31:30] (bits 63:62)
            // C_SIZE_MULT at bits [49:47] → word2[17:15]
            let read_bl_len = (csd[1] >> 16) & 0xF;
            let c_size = ((csd[1] & 0x3FF) << 2) | ((csd[2] >> 30) & 0x3);
            let c_size_mult = (csd[2] >> 15) & 0x7;
            let block_len = 1u32 << read_bl_len;
            let mult = 1u32 << (c_size_mult + 2);
            let capacity_bytes = (c_size as u64 + 1) * mult as u64 * block_len as u64;
            (capacity_bytes / 512) as u32
        }
    }

    // ---------------------------------------------------------------------------
    // embedded-sdmmc BlockDevice + TimeSource
    // ---------------------------------------------------------------------------

    /// A synchronous `embedded_sdmmc::BlockDevice` backed by the SDHI hardware.
    ///
    /// Uses polling register reads (no Embassy executor needed).  [`init`] must
    /// have completed successfully before any method is called.
    ///
    /// Construct with `DelugeBlockDevice` (it is a ZST).
    #[cfg(feature = "fat")]
    pub struct DelugeBlockDevice;

    #[cfg(feature = "fat")]
    impl embedded_sdmmc::BlockDevice for DelugeBlockDevice {
        type Error = SdError;

        fn read(
            &self,
            blocks: &mut [embedded_sdmmc::Block],
            start_block_idx: embedded_sdmmc::BlockIdx,
        ) -> Result<(), SdError> {
            if !CARD_READY.load(Ordering::Acquire) {
                return Err(SdError::NotInitialized);
            }
            let count = blocks.len() as u32;
            if count == 0 {
                return Ok(());
            }
            let lba = start_block_idx.0;
            let addr = lba_to_addr(lba);
            // Single-block read = plain CMD17 (normal mode); multi-block read adds
            // the extended multiple-block bits. HC vs SC addressing is in `addr`.
            let cmd = if count > 1 { CMD18_MULTI } else { CMD17 };
            let ptr = blocks.as_mut_ptr() as *mut u8;
            // Interrupt-driven PIO transfer, mirroring the vendor HAL's
            // `_sd_software_trans`. embedded-sdmmc's `BlockDevice` is synchronous,
            // so run the async transfer to completion with block_on (fine for the
            // bootloader — nothing else runs). The real firmware uses the async DMA
            // path, [`read_sectors`], directly so the executor keeps running during
            // transfers.
            embassy_futures::block_on(async {
                unsafe {
                    sdhi::set_block_count(SD_PORT, count);
                    sdhi::set_arg(SD_PORT, addr);
                    sdhi::send_cmd(SD_PORT, cmd).await?;
                    let xfer = sdhi::read_blocks_sw(SD_PORT, ptr, count).await;
                    // CMD18 multi-block read leaves auto-CMD12 disabled (see
                    // `read_sectors`); stop the card explicitly — even if the data
                    // phase failed — so a bailed-out read never leaves the card
                    // streaming into the next command or the next sd::init.
                    let stop = if count > 1 {
                        sdhi::stop_transfer(SD_PORT).await
                    } else {
                        Ok(())
                    };
                    xfer?;
                    stop?;
                    Ok::<(), SdhiError>(())
                }
            })
            .map_err(SdError::from)
        }

        fn write(
            &self,
            blocks: &[embedded_sdmmc::Block],
            start_block_idx: embedded_sdmmc::BlockIdx,
        ) -> Result<(), SdError> {
            if !CARD_READY.load(Ordering::Acquire) {
                return Err(SdError::NotInitialized);
            }
            let count = blocks.len() as u32;
            if count == 0 {
                return Ok(());
            }
            let lba = start_block_idx.0;
            let addr = lba_to_addr(lba);
            // Single/multi-block write both use normal mode (plain command index).
            let cmd = if count > 1 { CMD25 } else { CMD24 };

            let ptr = blocks.as_ptr() as *const u8;
            // Interrupt-driven PIO transfer (see `read` above).
            embassy_futures::block_on(async {
                unsafe {
                    sdhi::set_block_count(SD_PORT, count);
                    sdhi::set_arg(SD_PORT, addr);
                    sdhi::send_cmd(SD_PORT, cmd).await?;
                    sdhi::write_blocks_sw(SD_PORT, ptr, count).await
                }
            })
            .map_err(SdError::from)
        }

        fn num_blocks(&self) -> Result<embedded_sdmmc::BlockCount, SdError> {
            if !CARD_READY.load(Ordering::Acquire) {
                return Err(SdError::NotInitialized);
            }
            let n = sdhi::card_size_blocks(SD_PORT);
            Ok(embedded_sdmmc::BlockCount(n))
        }
    }

    // ---------------------------------------------------------------------------
    // Superfloppy (no-MBR) compatibility shim
    // ---------------------------------------------------------------------------

    /// Internal mode for [`PartitionShim`], decided once at construction time.
    #[derive(Clone, Copy)]
    #[cfg(feature = "fat")]
    enum ShimMode {
        /// The card has a real MBR (or we couldn't probe it): forward every
        /// request to [`DelugeBlockDevice`] unchanged.
        Passthrough,
        /// The card is a "superfloppy" — a FAT volume boot record sits directly at
        /// LBA 0 with no partition table. We present a one-block-shifted virtual
        /// address space: virtual LBA 0 returns a synthesized MBR, and virtual LBA
        /// `n` (n >= 1) maps to physical LBA `n - 1`. `total` is the physical card
        /// size in blocks.
        Superfloppy { total: u32 },
    }

    /// A [`BlockDevice`](embedded_sdmmc::BlockDevice) wrapper that transparently
    /// supports both MBR-partitioned and "superfloppy" (no-partition-table) SD
    /// cards.
    ///
    /// [`embedded_sdmmc`] only understands MBR-partitioned cards: it reads LBA 0,
    /// requires the `0x55AA` signature, then validates byte 446 as a partition
    /// status byte. A superfloppy card has a FAT VBR at LBA 0 — which also ends in
    /// `0x55AA` — so the signature check passes but the "partition status" check
    /// reads FAT boot code and fails with `FormatError("Invalid partition status")`.
    ///
    /// This shim detects that case at construction (by probing LBA 0) and, for
    /// superfloppy cards, synthesizes a single-partition MBR pointing at the real
    /// VBR. MBR-partitioned cards are passed straight through with no shift.
    ///
    /// [`init`] must have completed successfully before this is constructed.
    #[cfg(feature = "fat")]
    pub struct PartitionShim {
        inner: DelugeBlockDevice,
        mode: ShimMode,
    }

    #[cfg(feature = "fat")]
    impl PartitionShim {
        /// Probe LBA 0 and pick a [`ShimMode`]. Any read error or ambiguous layout
        /// falls back to [`ShimMode::Passthrough`] (the previous behaviour).
        pub fn new() -> Self {
            let inner = DelugeBlockDevice;
            let mode = Self::detect(&inner);
            Self { inner, mode }
        }

        fn detect(inner: &DelugeBlockDevice) -> ShimMode {
            use embedded_sdmmc::{BlockDevice, BlockIdx};

            let mut block = [embedded_sdmmc::Block::new()];
            if let Err(e) = inner.read(&mut block, BlockIdx(0)) {
                log::warn!("sd: shim probe read failed: {:?} -> passthrough", e);
                return ShimMode::Passthrough;
            }
            let b = &block[0].contents;

            // No boot signature at all: not FAT and not an MBR — let the normal
            // path surface the error.
            if b[510] != 0x55 || b[511] != 0xAA {
                log::warn!("sd: shim no 0x55AA signature -> passthrough");
                return ShimMode::Passthrough;
            }

            // embedded-sdmmc accepts the card as MBR-partitioned when the
            // partition-0 status byte is 0x00 or 0x80.
            let looks_like_mbr = (b[446] & 0x7F) == 0x00;

            // A FAT VBR begins with a short/near jump (0xEB .. 0x90 or 0xE9) and
            // declares 512 bytes per logical sector in its BPB. To avoid a false
            // positive on an MBR whose boot code happens to start with a jump, also
            // require a couple of always-present FAT BPB invariants: a non-zero
            // reserved-sector count (offset 14) and 1 or 2 FATs (offset 16). These
            // hold for every FAT12/16/32 volume but are vanishingly unlikely to all
            // line up in MBR boot code.
            let jump_ok = b[0] == 0xEB || b[0] == 0xE9;
            let bytes_per_sector = u16::from_le_bytes([b[11], b[12]]);
            let reserved_sectors = u16::from_le_bytes([b[14], b[15]]);
            let num_fats = b[16];
            let looks_like_vbr = jump_ok
                && bytes_per_sector == 512
                && reserved_sectors != 0
                && (num_fats == 1 || num_fats == 2);

            // An unambiguous VBR wins even if byte 446 happens to read as a valid
            // MBR status byte (it falls inside the VBR's boot-code region and can be
            // anything): a real FAT VBR at LBA 0 is always a superfloppy.
            if looks_like_vbr {
                let total = inner.num_blocks().map(|c| c.0).unwrap_or(0);
                log::info!("sd: shim -> superfloppy (synthetic MBR, {} blocks)", total);
                ShimMode::Superfloppy { total }
            } else {
                log::info!(
                    "sd: shim -> passthrough (mbr={} vbr={})",
                    looks_like_mbr,
                    looks_like_vbr
                );
                ShimMode::Passthrough
            }
        }
    }

    #[cfg(feature = "fat")]
    impl Default for PartitionShim {
        fn default() -> Self {
            Self::new()
        }
    }

    /// Fill `buf` with a minimal MBR describing one FAT32 (LBA) partition that
    /// starts at LBA 1 and spans `total` blocks.
    fn synthesize_mbr(buf: &mut [u8; 512], total: u32) {
        buf.fill(0);
        // Partition entry 0 lives at offset 446 and is 16 bytes long.
        let p = &mut buf[446..462];
        p[0] = 0x00; // status: non-bootable (passes embedded-sdmmc's check)
        // p[1..4]  CHS of first sector  — ignored for LBA parsing, leave zero.
        p[4] = 0x0C; // partition type: FAT32 with LBA
        // p[5..8]  CHS of last sector   — ignored, leave zero.
        p[8..12].copy_from_slice(&1u32.to_le_bytes()); // LBA of first sector
        p[12..16].copy_from_slice(&total.to_le_bytes()); // number of sectors
        buf[510] = 0x55;
        buf[511] = 0xAA;
    }

    #[cfg(feature = "fat")]
    impl embedded_sdmmc::BlockDevice for PartitionShim {
        type Error = SdError;

        fn read(
            &self,
            blocks: &mut [embedded_sdmmc::Block],
            start_block_idx: embedded_sdmmc::BlockIdx,
        ) -> Result<(), SdError> {
            match self.mode {
                ShimMode::Passthrough => self.inner.read(blocks, start_block_idx),
                ShimMode::Superfloppy { total } => {
                    let start = start_block_idx.0;
                    if start == 0 {
                        if let Some((first, rest)) = blocks.split_first_mut() {
                            synthesize_mbr(&mut first.contents, total);
                            // Virtual blocks 1.. map to physical 0..
                            if !rest.is_empty() {
                                self.inner.read(rest, embedded_sdmmc::BlockIdx(0))?;
                            }
                        }
                        Ok(())
                    } else {
                        self.inner.read(blocks, embedded_sdmmc::BlockIdx(start - 1))
                    }
                }
            }
        }

        fn write(
            &self,
            blocks: &[embedded_sdmmc::Block],
            start_block_idx: embedded_sdmmc::BlockIdx,
        ) -> Result<(), SdError> {
            match self.mode {
                ShimMode::Passthrough => self.inner.write(blocks, start_block_idx),
                ShimMode::Superfloppy { .. } => {
                    let start = start_block_idx.0;
                    if start == 0 {
                        // The synthetic MBR is virtual-only; never write it back.
                        // Forward any trailing real blocks to physical LBA 0..
                        if let Some((_, rest)) = blocks.split_first()
                            && !rest.is_empty()
                        {
                            self.inner.write(rest, embedded_sdmmc::BlockIdx(0))?;
                        }
                        Ok(())
                    } else {
                        self.inner
                            .write(blocks, embedded_sdmmc::BlockIdx(start - 1))
                    }
                }
            }
        }

        fn num_blocks(&self) -> Result<embedded_sdmmc::BlockCount, SdError> {
            match self.mode {
                ShimMode::Passthrough => self.inner.num_blocks(),
                // One extra block for the synthetic MBR at virtual LBA 0.
                ShimMode::Superfloppy { total } => {
                    Ok(embedded_sdmmc::BlockCount(total.saturating_add(1)))
                }
            }
        }
    }

    /// An [`embedded_sdmmc::TimeSource`] that returns a fixed epoch-zero timestamp
    /// (1970-01-01 00:00:00) for all FAT file operations.
    ///
    /// File modification times will not be recorded accurately. No RTC peripheral
    /// is currently available on this BSP.
    #[cfg(feature = "fat")]
    pub struct DelugeTimeSource;

    #[cfg(feature = "fat")]
    impl embedded_sdmmc::TimeSource for DelugeTimeSource {
        fn get_timestamp(&self) -> embedded_sdmmc::Timestamp {
            embedded_sdmmc::Timestamp {
                year_since_1970: 0,
                zero_indexed_month: 0,
                zero_indexed_day: 0,
                hours: 0,
                minutes: 0,
                seconds: 0,
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Host: file-backed disk image standing in for the SD card
// ---------------------------------------------------------------------------

#[cfg(not(target_os = "none"))]
mod host {
    use std::fs::{File, OpenOptions};
    use std::io::{Read, Seek, SeekFrom, Write};
    use std::path::PathBuf;
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::sync::{Mutex, OnceLock};

    use super::SdError;

    const SECTOR_SIZE: usize = 512;

    /// Default image size: enough for host bring-up/round-trip checks, cheap to
    /// allocate (sparse on any filesystem that supports holes).
    const DEFAULT_SECTORS: u64 = 16 * 1024; // 8 MiB

    struct Disk {
        file: File,
        sectors: u32,
    }

    static DISK: OnceLock<Mutex<Disk>> = OnceLock::new();

    /// Mirrors the device's `CARD_READY`: `false` until [`init`] has opened (and,
    /// if needed, created/sized) the backing file.
    static READY: AtomicBool = AtomicBool::new(false);

    fn image_path() -> PathBuf {
        std::env::var_os("DELUGE_SD_IMAGE")
            .map(PathBuf::from)
            .unwrap_or_else(|| std::env::temp_dir().join("deluge-bsp-sd.img"))
    }

    fn disk() -> &'static Mutex<Disk> {
        DISK.get_or_init(|| {
            let path = image_path();
            let file = OpenOptions::new()
                .read(true)
                .write(true)
                .create(true)
                .truncate(false)
                .open(&path)
                .unwrap_or_else(|e| panic!("sd(host): failed to open backing file {path:?}: {e}"));
            let want_len = DEFAULT_SECTORS * SECTOR_SIZE as u64;
            let len = file
                .metadata()
                .unwrap_or_else(|e| panic!("sd(host): failed to stat backing file {path:?}: {e}"))
                .len();
            if len < want_len {
                file.set_len(want_len).unwrap_or_else(|e| {
                    panic!(
                        "sd(host): failed to size backing file {path:?} to {want_len} bytes: {e}"
                    )
                });
            }
            let sectors = (want_len.max(len) / SECTOR_SIZE as u64) as u32;
            log::info!(
                "sd(host): backing file {} ({sectors} sectors)",
                path.display()
            );
            Mutex::new(Disk { file, sectors })
        })
    }

    /// Host stand-in for [`init`](super::init): opens (and creates/sizes, if
    /// needed) the backing file. No real hardware to bring up — resolves without
    /// ever suspending.
    pub async fn init() -> Result<(), SdError> {
        let _ = disk();
        READY.store(true, Ordering::Release);
        Ok(())
    }

    /// Host stand-in for [`is_ready`](super::is_ready): `true` once [`init`] has
    /// run.
    pub fn is_ready() -> bool {
        READY.load(Ordering::Acquire)
    }

    /// Host stand-in for [`is_inserted`](super::is_inserted): the backing file
    /// stands in for the card for the whole process lifetime, so it always
    /// reads as present.
    pub fn is_inserted() -> bool {
        true
    }

    /// Host stand-in for [`is_write_protected`](super::is_write_protected): no
    /// write-protect concept for a plain host file.
    pub fn is_write_protected() -> bool {
        false
    }

    /// Host stand-in for [`invalidate`](super::invalidate): drops the ready flag,
    /// so [`is_ready`] reads false until [`init`] runs again (it re-opens the same
    /// backing file, so recovery works). Unreachable in practice — the only caller
    /// acts on a removal event, and [`take_card_detect_events`] never reports one
    /// here — but kept behaviourally faithful rather than a silent no-op.
    pub fn invalidate() {
        READY.store(false, Ordering::Release);
    }

    /// Host stand-in for
    /// [`take_card_detect_events`](super::take_card_detect_events): the backing
    /// file stands in for the card for the whole process lifetime, so it is never
    /// removed and never re-inserted — there is no edge to report.
    pub fn take_card_detect_events() -> (bool, bool) {
        (false, false)
    }

    /// Host stand-in for [`total_sectors`](super::total_sectors): the backing
    /// file's sector count (fixed at creation time).
    pub fn total_sectors() -> u32 {
        disk().lock().unwrap().sectors
    }

    /// Host stand-in for [`read_sectors`](super::read_sectors): seek + read
    /// `count` sectors from the backing file. Resolves without ever suspending.
    pub async fn read_sectors(lba: u32, count: u32, buf: &mut [u8]) -> Result<(), SdError> {
        if buf.len() < (count as usize) * SECTOR_SIZE {
            return Err(SdError::Protocol);
        }
        if count == 0 {
            return Ok(());
        }
        let mut d = disk().lock().unwrap();
        d.file
            .seek(SeekFrom::Start(lba as u64 * SECTOR_SIZE as u64))
            .map_err(|e| SdError::HostIo(e.kind()))?;
        d.file
            .read_exact(&mut buf[..(count as usize) * SECTOR_SIZE])
            .map_err(|e| SdError::HostIo(e.kind()))
    }

    /// Host stand-in for [`write_sectors`](super::write_sectors): seek + write
    /// `count` sectors to the backing file. Resolves without ever suspending.
    pub async fn write_sectors(lba: u32, count: u32, buf: &[u8]) -> Result<(), SdError> {
        if buf.len() < (count as usize) * SECTOR_SIZE {
            return Err(SdError::Protocol);
        }
        if count == 0 {
            return Ok(());
        }
        let mut d = disk().lock().unwrap();
        d.file
            .seek(SeekFrom::Start(lba as u64 * SECTOR_SIZE as u64))
            .map_err(|e| SdError::HostIo(e.kind()))?;
        d.file
            .write_all(&buf[..(count as usize) * SECTOR_SIZE])
            .map_err(|e| SdError::HostIo(e.kind()))
    }
}

#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::*;

    #[test]
    fn init_reports_ready() {
        embassy_futures::block_on(init()).expect("host init never fails");
        assert!(is_ready());
    }

    #[test]
    fn host_reports_always_inserted_and_not_write_protected() {
        assert!(is_inserted());
        assert!(!is_write_protected());
    }

    /// The host backing file is never removed, so there is never a card-detect
    /// edge to report — and, since a `(removed, _)` report is what drives
    /// `invalidate()`, that is also why nothing on host ever invalidates the
    /// card. (`invalidate()` itself is deliberately untested here: it clears the
    /// shared `READY` flag the other tests in this module rely on, and these run
    /// concurrently.)
    #[test]
    fn host_never_reports_a_card_detect_edge() {
        assert_eq!(take_card_detect_events(), (false, false));
        // Idempotent: repeated draining does not manufacture an edge either.
        assert_eq!(take_card_detect_events(), (false, false));
    }

    #[test]
    fn total_sectors_matches_default_image_size() {
        embassy_futures::block_on(init()).expect("host init never fails");
        assert_eq!(total_sectors(), 16 * 1024);
    }

    /// Round-trip a sector of non-trivial data through the host disk image.
    /// Uses a sector index (1000) not touched by the other tests in this module
    /// so they can run concurrently against the shared backing file.
    #[test]
    fn round_trip_sector() {
        embassy_futures::block_on(init()).expect("host init never fails");

        let sector: u32 = 1000;
        let mut pattern = [0u8; 512];
        for (i, b) in pattern.iter_mut().enumerate() {
            *b = (i % 256) as u8;
        }

        embassy_futures::block_on(write_sectors(sector, 1, &pattern)).expect("write_sectors");

        let mut readback = [0u8; 512];
        embassy_futures::block_on(read_sectors(sector, 1, &mut readback)).expect("read_sectors");

        assert_eq!(pattern, readback);
    }

    /// Multi-sector round trip, at an offset that doesn't overlap
    /// [`round_trip_sector`]'s range.
    #[test]
    fn round_trip_multiple_sectors() {
        embassy_futures::block_on(init()).expect("host init never fails");

        let sector: u32 = 2000;
        let count: u32 = 4;
        let mut pattern = vec![0u8; count as usize * 512];
        for (i, b) in pattern.iter_mut().enumerate() {
            *b = ((i * 7) % 256) as u8;
        }

        embassy_futures::block_on(write_sectors(sector, count, &pattern)).expect("write_sectors");

        let mut readback = vec![0u8; count as usize * 512];
        embassy_futures::block_on(read_sectors(sector, count, &mut readback))
            .expect("read_sectors");

        assert_eq!(pattern, readback);
    }

    #[test]
    fn read_sectors_rejects_undersized_buffer() {
        let mut too_small = [0u8; 511];
        let err = embassy_futures::block_on(read_sectors(0, 1, &mut too_small))
            .expect_err("buffer is one byte short of a sector");
        assert_eq!(err, SdError::Protocol);
    }
}

// ---------------------------------------------------------------------------
// Unit tests (host-side)
// ---------------------------------------------------------------------------

#[cfg(all(test, not(target_os = "none")))]
mod switch_status_tests {
    use super::*;

    #[test]
    fn hs_supported_and_selected() {
        let mut buf = [0u8; 64];
        buf[13] = 0x03; // group 1 supports functions 0 and 1
        buf[16] = 0x01; // group 1 switched to function 1 (High-Speed)
        let st = parse_switch_status(&buf);
        assert!(st.hs_supported);
        assert_eq!(st.group1_selected, 0x1);
    }

    #[test]
    fn hs_unsupported() {
        let mut buf = [0u8; 64];
        buf[13] = 0x01; // only function 0 (default speed)
        let st = parse_switch_status(&buf);
        assert!(!st.hs_supported);
        assert_eq!(st.group1_selected, 0x0);
    }

    #[test]
    fn switch_refused_is_0xf_and_nibbles_do_not_leak() {
        let mut buf = [0u8; 64];
        buf[13] = 0x03;
        buf[16] = 0x0F; // 0xF = function error / switch refused
        assert_eq!(parse_switch_status(&buf).group1_selected, 0xF);
        buf[16] = 0xF1; // upper nibble belongs to group 2 — must not leak
        assert_eq!(parse_switch_status(&buf).group1_selected, 0x1);
    }
}
