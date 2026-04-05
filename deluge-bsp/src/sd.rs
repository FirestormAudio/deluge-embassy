//! Deluge-BSP SD card driver.
//!
//! Exposes an async interface for reading and writing 512-byte sectors on the
//! Deluge's SD card (SDHI port 1).
//!
//! ## SD protocol overview
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
//!   Switch clock from ~130 kHz (P1/512) to ~16.7 MHz (P1/4)
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

use core::sync::atomic::{AtomicBool, AtomicU16, Ordering};

use rza1l_hal::UNCACHED_MIRROR_OFFSET;
use rza1l_hal::sdhi::{self, SdhiError};

// The Deluge SD card is on SDHI port 1.
const SD_PORT: u8 = 1;

use crate::system::{SD_DMA_MAX_SECTORS, SD_DMA_RX_CH, SD_DMA_TX_CH};

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

/// ACMD6 (0x40 | 6): SET_BUS_WIDTH — R1
const ACMD6: u16 = 0x40 | 6;
/// ACMD41 (0x40 | 41): SD_SEND_OP_COND — R3
const ACMD41: u16 = 0x40 | 41;

// CMD17/18 SDHC variant: upper bits encode high-capacity block addressing.
// Matches the Renesas driver: `CMD18 | 0x7c00u`.
const CMD17_SDHC: u16 = CMD17 | 0x7C00;
const CMD18_SDHC: u16 = CMD18 | 0x7C00;
const CMD24_SDHC: u16 = CMD24 | 0x7C00;
const CMD25_SDHC: u16 = CMD25 | 0x7C00;

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
// Error type
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SdError {
    /// Low-level SDHI hardware error.
    Hardware(SdhiError),
    /// Card not present.
    NoCard,
    /// Unsupported card type (e.g. MMC, old SD v1 without sane CMD8 response).
    UnsupportedCard,
    /// Protocol violation — unexpected response.
    Protocol,
    /// Driver not initialized; call [`init`] first.
    NotInitialized,
}

impl From<SdhiError> for SdError {
    fn from(e: SdhiError) -> Self {
        SdError::Hardware(e)
    }
}

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
    }

    // Short delay to allow power-up (hardware needs ~1 ms minimum,
    // awaiting a tiny future yields back to the executor which is enough).
    embassy_time::Timer::after_millis(5).await;

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
            let csd = sdhi::read_r2(SD_PORT);
            let total = decode_csd_capacity(csd, hc);
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

        // ---- Switch to high-speed clock ----
        sdhi::set_clock_fast(SD_PORT);
    }

    CARD_READY.store(true, Ordering::Release);
    log::debug!("sd: card ready, HC={}", CARD_HC.load(Ordering::Relaxed));
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
    let hc = CARD_HC.load(Ordering::Relaxed);
    let cmd = if hc { CMD17_SDHC } else { CMD17 };

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
    let hc = CARD_HC.load(Ordering::Relaxed);
    let cmd = if hc { CMD24_SDHC } else { CMD24 };

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

    let hc = CARD_HC.load(Ordering::Relaxed);

    unsafe {
        let dma_ptr =
            (core::ptr::addr_of!(SD_DMA_BUF.0[0]) as usize + UNCACHED_MIRROR_OFFSET) as *mut u8;
        let mut remaining = count;
        let mut cur_lba = lba;
        let mut buf_offset = 0usize;
        while remaining > 0 {
            let chunk = remaining.min(SD_DMA_MAX_SECTORS as u32);
            let chunk_addr = lba_to_addr(cur_lba);
            let cmd = if hc { CMD18_SDHC } else { CMD18 };
            sdhi::set_block_count(SD_PORT, chunk);
            sdhi::set_arg(SD_PORT, chunk_addr);
            sdhi::read_blocks_dma(SD_PORT, cmd, dma_ptr, chunk, SD_DMA_RX_CH).await?;
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

    let hc = CARD_HC.load(Ordering::Relaxed);

    unsafe {
        let dma_ptr =
            (core::ptr::addr_of!(SD_DMA_BUF.0[0]) as usize + UNCACHED_MIRROR_OFFSET) as *mut u8;
        let mut remaining = count;
        let mut cur_lba = lba;
        let mut buf_offset = 0usize;
        while remaining > 0 {
            let chunk = remaining.min(SD_DMA_MAX_SECTORS as u32);
            let chunk_addr = lba_to_addr(cur_lba);
            let cmd = if hc { CMD25_SDHC } else { CMD25 };
            let chunk_bytes = (chunk as usize) * 512;
            core::ptr::copy_nonoverlapping(buf.as_ptr().add(buf_offset), dma_ptr, chunk_bytes);
            sdhi::set_block_count(SD_PORT, chunk);
            sdhi::set_arg(SD_PORT, chunk_addr);
            sdhi::write_blocks_dma(SD_PORT, cmd, dma_ptr as *const u8, chunk, SD_DMA_TX_CH).await?;
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

/// Returns `true` if a card is physically present (CD pin).
pub fn is_inserted() -> bool {
    unsafe { sdhi::card_inserted(SD_PORT) }
}

// ---------------------------------------------------------------------------
// Internal: convert LBA to card address
// ---------------------------------------------------------------------------

fn lba_to_addr(lba: u32) -> u32 {
    if CARD_HC.load(Ordering::Relaxed) {
        lba // SDHC/SDXC: block addressing
    } else {
        lba * 512 // SDSC: byte addressing
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
        // READ_BL_LEN at bits [83:80] → word1 bits [19:16]
        // C_SIZE at bits [73:62] → word1 bits [9:8] + word2 bits [31:22]
        // C_SIZE_MULT at bits [49:47] → word2 bits [17:15]
        let read_bl_len = (csd[1] >> 16) & 0xF;
        let c_size_hi = csd[1] & 0x3; // bits [73:72]
        let c_size_lo = (csd[2] >> 22) & 0x3FF; // bits [71:62]
        let c_size = (c_size_hi << 10) | c_size_lo;
        let c_size_mult = (csd[2] >> 15) & 0x7;
        let block_len = 1u32 << read_bl_len; // bytes per block
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
pub struct DelugeBlockDevice;

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
        let hc = CARD_HC.load(Ordering::Relaxed);
        let cmd = if count > 1 {
            if hc { CMD18_SDHC } else { CMD18 }
        } else {
            if hc { CMD17_SDHC } else { CMD17 }
        };

        unsafe {
            sdhi::set_block_count(SD_PORT, count);
            sdhi::set_arg(SD_PORT, addr);
            sdhi::send_cmd_poll(SD_PORT, cmd)?;
            sdhi::read_blocks_poll(SD_PORT, blocks.as_mut_ptr() as *mut u8, count)?;
        }
        Ok(())
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
        let hc = CARD_HC.load(Ordering::Relaxed);
        let cmd = if count > 1 {
            if hc { CMD25_SDHC } else { CMD25 }
        } else {
            if hc { CMD24_SDHC } else { CMD24 }
        };

        unsafe {
            sdhi::set_block_count(SD_PORT, count);
            sdhi::set_arg(SD_PORT, addr);
            sdhi::send_cmd_poll(SD_PORT, cmd)?;
            sdhi::write_blocks_poll(SD_PORT, blocks.as_ptr() as *const u8, count)?;
        }
        Ok(())
    }

    fn num_blocks(&self) -> Result<embedded_sdmmc::BlockCount, SdError> {
        if !CARD_READY.load(Ordering::Acquire) {
            return Err(SdError::NotInitialized);
        }
        let n = sdhi::card_size_blocks(SD_PORT);
        Ok(embedded_sdmmc::BlockCount(n))
    }
}

/// A stub [`embedded_sdmmc::TimeSource`] returning a fixed epoch-zero timestamp.
///
/// Replace with a real RTC implementation when one is available.
pub struct DelugeTimeSource;

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
