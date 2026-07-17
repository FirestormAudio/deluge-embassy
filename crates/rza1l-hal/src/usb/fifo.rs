//! RUSB1 FIFO port operations.
//!
//! This module wraps the three hardware FIFO ports (CFIFO, D0FIFO, D1FIFO)
//! and provides the `sw_to_hw_fifo` / `hw_to_sw_fifo` byte-copy routines
//! that handle the RUSB1's MBW (memory bus width) switching rules.
//!
//! Routing policy (matches `dcd_rusb1.c`):
//! - Pipe 0 (DCP / control)         → CFIFO
//! - Pipes 1–2 (ISO)                → D0FIFO  (dedicated for audio)
//! - Pipes 3–15 (bulk / interrupt)  → D1FIFO
//!
//! ## MBW rules (TRM §28.3.8)
//! The MBW field in the FIFO SEL register must not change once a FIFO read
//! has begun.  We set MBW=32 when selecting CURPIPE (the common case for
//! 4-byte-aligned payloads) and only narrow it for sub-word transfers.
//!
//! ## RZA1 D1FIFO quirk
//! Writing D1FIFOSEL re-triggers the FIFO port switching state machine even
//! if only MBW changes.  Reading immediately after the MBW write returns
//! `0xFF`.  For payloads shorter than 4 bytes, we keep MBW=32 and unpack
//! the returned word instead of narrowing.

use super::regs::{
    FIFOCTR_BCLR, FIFOCTR_BVAL, FIFOCTR_DTLN_MASK, FIFOCTR_FRDY, FIFOSEL_CURPIPE_MASK,
    FIFOSEL_MBW_MASK, FIFOSEL_MBW_SHIFT, MBW_32, Rusb1Regs, rd, rd32, wr, wr32,
};

/// Hardware FIFO port: data register + SEL register + CTR register.
pub struct FifoPort {
    /// Pointer to the 32-bit data register (CFIFO / D0FIFO / D1FIFO).
    pub data: *mut u32,
    /// Pointer to the 16-bit select register (CFIFOSEL / D0FIFOSEL / D1FIFOSEL).
    pub sel: *mut u16,
    /// Pointer to the 16-bit control register (CFIFOCTR / D0FIFOCTR / D1FIFOCTR).
    pub ctr: *mut u16,
}

// Safety: FifoPort is a bundle of raw pointers to memory-mapped registers.
// All access is unsafe and protected by the caller.
unsafe impl Send for FifoPort {}

impl FifoPort {
    /// Construct the CFIFO port from a register block pointer.
    ///
    /// # Safety
    /// `regs` must be a valid pointer to the USB register block for the active port.
    pub unsafe fn cfifo(regs: *mut Rusb1Regs) -> Self {
        unsafe {
            Self {
                data: core::ptr::addr_of_mut!((*regs).cfifo),
                sel: core::ptr::addr_of_mut!((*regs).cfifosel),
                ctr: core::ptr::addr_of_mut!((*regs).cfifoctr),
            }
        }
    }

    /// Construct the D0FIFO port.
    ///
    /// # Safety
    /// Same as [`cfifo`].
    pub unsafe fn d0fifo(regs: *mut Rusb1Regs) -> Self {
        unsafe {
            Self {
                data: core::ptr::addr_of_mut!((*regs).d0fifo),
                sel: core::ptr::addr_of_mut!((*regs).d0fifosel),
                ctr: core::ptr::addr_of_mut!((*regs).d0fifoctr),
            }
        }
    }

    /// Construct the D1FIFO port.
    ///
    /// # Safety
    /// Same as [`cfifo`].
    pub unsafe fn d1fifo(regs: *mut Rusb1Regs) -> Self {
        unsafe {
            Self {
                data: core::ptr::addr_of_mut!((*regs).d1fifo),
                sel: core::ptr::addr_of_mut!((*regs).d1fifosel),
                ctr: core::ptr::addr_of_mut!((*regs).d1fifoctr),
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Pipe → FIFO port routing
// ---------------------------------------------------------------------------

/// Return the correct [`FifoPort`] for `pipe_num`.
///
/// Routing: pipe 0 → CFIFO, pipes 1-2 → D0FIFO, pipes 3+ → D1FIFO.
///
/// # Safety
/// `regs` must be valid.
pub unsafe fn fifo_for_pipe(regs: *mut Rusb1Regs, pipe_num: usize) -> FifoPort {
    unsafe {
        match pipe_num {
            0 => FifoPort::cfifo(regs),
            1 | 2 => FifoPort::d0fifo(regs),
            _ => FifoPort::d1fifo(regs),
        }
    }
}

// ---------------------------------------------------------------------------
// MBW helpers
// ---------------------------------------------------------------------------

/// Change the MBW field in a FIFO SEL register without touching other bits.
///
/// # Safety
/// `sel` must be a valid pointer to a FIFO select register.
unsafe fn set_mbw(sel: *mut u16, mbw: u16) {
    unsafe {
        let cur = rd(sel);
        wr(
            sel,
            (cur & !FIFOSEL_MBW_MASK) | ((mbw << FIFOSEL_MBW_SHIFT) & FIFOSEL_MBW_MASK),
        );
    }
}

// ---------------------------------------------------------------------------
// FIFO ready check
// ---------------------------------------------------------------------------

/// Returns `true` if the FIFO port is pointing at `pipe_num` AND is ready.
///
/// After writing CURPIPE in `fifo_select_pipe`, the hardware needs a few bus
/// cycles to settle.  This function busy-waits briefly for CURPIPE to match
/// and FRDY to assert.  Returns false if the FIFO is not ready within the
/// timeout — the caller should abort and retry on the next BRDY.
///
/// # Safety
/// `fifo` fields must be valid register pointers.
pub unsafe fn fifo_is_ready(fifo: &FifoPort, pipe_num: usize) -> bool {
    unsafe {
        // Confirm CURPIPE has latched to the requested pipe (TRM §28: after a
        // CURPIPE write, read it back to confirm it updated), then poll FRDY.
        // Correct selection — including the deselect-first dance the TRM mandates
        // when switching to a *receiving* pipe — is handled by the caller via
        // `fifo_select_recv` so the port can't surface the prior pipe's data.
        for _ in 0..64 {
            let sel = rd(fifo.sel);
            if (sel & FIFOSEL_CURPIPE_MASK) as usize != pipe_num {
                continue;
            }
            if (rd(fifo.ctr) & FIFOCTR_FRDY) != 0 {
                return true;
            }
        }
        false
    }
}

/// Select a **receiving-direction** (OUT) pipe on a DnFIFO port, following the
/// TRM §28 CURPIPE-change procedure: set CURPIPE to a different value (0 = no
/// pipe) first, confirm it latched, then select the target pipe.
///
/// This is required to avoid the shared-D1FIFO CDC IN/OUT cross-corruption: an
/// OUT drain that selects its pipe in a single write can observe the FRDY/data
/// of the *previously selected* IN pipe (e.g. crow's just-staged TX), reading
/// its bytes back onto the OUT path. Deselecting first flushes the port state
/// machine so the subsequent read reflects only the target pipe.
///
/// # Safety
/// `fifo` must be a valid DnFIFO port; `pipe_num` must be a DnFIFO pipe (≥ 1).
pub unsafe fn fifo_select_recv(fifo: &FifoPort, pipe_num: usize) {
    unsafe {
        // Step 1: deselect (CURPIPE = 0 → "no pipe" on D0/D1FIFO) and confirm.
        fifo_select_pipe(fifo, 0, false);
        for _ in 0..64 {
            if (rd(fifo.sel) & FIFOSEL_CURPIPE_MASK) == 0 {
                break;
            }
        }
        // Step 2: select the target receiving pipe.
        fifo_select_pipe(fifo, pipe_num, false);
    }
}

/// Return the number of valid data bytes waiting in the FIFO.
///
/// # Safety
/// `fifo` must be valid.
pub unsafe fn fifo_dtln(fifo: &FifoPort) -> u16 {
    unsafe { rd(fifo.ctr) & FIFOCTR_DTLN_MASK }
}

/// Issue BCLR (clear the FIFO buffer) on `fifo`.
///
/// # Safety
/// `fifo` must be valid.
pub unsafe fn fifo_bclr(fifo: &FifoPort) {
    unsafe {
        wr(fifo.ctr, FIFOCTR_BCLR);
    }
}

/// Issue BVAL (commit the write buffer) on `fifo`.
///
/// # Safety
/// `fifo` must be valid.
pub unsafe fn fifo_bval(fifo: &FifoPort) {
    unsafe {
        wr(fifo.ctr, FIFOCTR_BVAL);
    }
}

// ---------------------------------------------------------------------------
// Software → hardware FIFO (write path: IN endpoint or control IN)
// ---------------------------------------------------------------------------

/// Copy `len` bytes from `buf` into the hardware FIFO.
///
/// Uses 32-bit FIFO access throughout: the 4-byte body via native `u32` stores
/// and the 1-3 byte tail via 8-bit lane stores (MBW stays 32 — it is never
/// narrowed).  This mirrors the proven Linux `renesas_usbhs` PIO push, which is
/// the ground-truth reference for the RZ/A1L (the TinyUSB `dcd_rusb1.c` this was
/// originally modeled on was never validated on silicon and got the tail lane
/// wrong).
///
/// # Safety
/// - `fifo.data` / `fifo.sel` must be valid.
/// - The FIFO must have been selected via [`fifo_select_pipe`] (CURPIPE + MBW=32)
///   before calling.
pub unsafe fn sw_to_hw_fifo(fifo: &FifoPort, buf: *const u8, len: usize) {
    unsafe {
        // 32-bit FIFO access for the body.  With BIGEND=0 (the reset default,
        // never overridden) Table 28.7 maps byte N+0 → bits[7:0], i.e. little
        // endian matching the ARM, so a native `u32` write emits bytes in order
        // with no swap.  This quarters the MMIO write count vs byte access.
        //
        // Self-contained: we (re)assert MBW=32 here rather than relying on the
        // caller's pipe-select width, so every transmit caller (control, bulk,
        // host) gets 32-bit access.  Callers already select CURPIPE | MBW=32, so
        // this is a same-value write and does not re-trigger the port switch.
        set_mbw(fifo.sel, MBW_32);
        let mut p = buf;
        let mut rem = len;
        while rem >= 4 {
            let w = (p as *const u32).read_unaligned();
            wr32(fifo.data, w);
            p = p.add(4);
            rem -= 4;
        }

        // 1-3 byte tail.  Only non-multiple-of-4 transfers reach here (e.g. a
        // 13-byte CSW); block data is always 512-byte aligned.
        //
        // Do NOT narrow MBW: leave it at 32 and emit each remaining byte as an
        // 8-bit store to the correct byte lane.  Per TRM Table 28.9 (8-bit
        // access, BIGEND=0) the valid byte for the RZ/A1L sits on bits[31:24],
        // i.e. CPU byte-address `base + 3`.  Matching the proven Linux
        // `renesas_usbhs` PIO push (cfifo_byte_addr=0 → `addr + (3 - (i & 3))`),
        // byte `i` of the tail goes to lane `3 - (i & 3)`.  A prior version set
        // MBW=8 and did a 32-bit store with the byte in bits[7:0]; the hardware
        // then latched bits[31:24] (= 0), transmitting the tail as zero bytes.
        if rem > 0 {
            let base = fifo.data as *mut u8;
            let mut i = 0usize;
            while rem > 0 {
                core::ptr::write_volatile(base.add(3 - (i & 3)), *p);
                p = p.add(1);
                rem -= 1;
                i += 1;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Hardware → software FIFO (read path: OUT endpoint or control OUT)
// ---------------------------------------------------------------------------

/// Copy `len` bytes from the hardware FIFO into `buf`.
///
/// Uses MBW=8 (byte-at-a-time), matching the C reference driver
/// `pipe_read_packet` which accesses the FIFO as `volatile uint8_t*`.
/// This is the only safe approach: at MBW=32 the hardware delivers bytes
/// big-endian in the 32-bit word, which would require an explicit bswap.
///
/// # Safety
/// - `fifo.data` / `fifo.sel` must be valid.
/// - The FIFO must have been selected (CURPIPE written) before calling.
pub unsafe fn hw_to_sw_fifo(fifo: &FifoPort, buf: *mut u8, len: usize) {
    unsafe {
        if len == 0 {
            return;
        }

        // 32-bit reads.  MBW=32 was established by the preceding `fifo_select_pipe`
        // (whose `fifo_is_ready` poll absorbs the port-switch settle), so we do
        // NOT change MBW here — avoiding the documented "first read after an MBW
        // write returns 0xFF" hazard.  BIGEND=0 (Table 28.7) maps byte N+0 →
        // bits[7:0], i.e. little endian, so the word's bytes are already in order.
        // Quarters the MMIO read count vs the previous byte-at-a-time path.
        let mut p = buf;
        let mut rem = len;
        while rem >= 4 {
            let w = rd32(fifo.data);
            (p as *mut u32).write_unaligned(w);
            p = p.add(4);
            rem -= 4;
        }

        // 1-3 byte tail: read one more 32-bit word and copy its low `rem` bytes
        // (little-endian).  Only a non-multiple-of-4 packet reaches here — for MSC
        // that is just the 31-byte CBW, whose packet length equals the FIFO's
        // valid byte count, so the over-read is discarded by the caller's BCLR.
        if rem > 0 {
            let bytes = rd32(fifo.data).to_le_bytes();
            let mut i = 0;
            while i < rem {
                *p.add(i) = bytes[i];
                i += 1;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Select a pipe on a FIFO port
// ---------------------------------------------------------------------------

/// Write the CURPIPE + MBW fields of `fifo.sel` together.
///
/// Selects 32-bit FIFO access (MBW=32) for both directions.  `sw_to_hw_fifo` /
/// `hw_to_sw_fifo` move the data as 32-bit words (BIGEND=0 ⇒ little-endian,
/// matching the ARM), with a narrowed/extracted tail for non-word-aligned
/// lengths.  The `fifo_is_ready` poll that follows this call absorbs the FIFO
/// port-switch settle so the first access does not hit the "0xFF after MBW
/// write" hazard.
///
/// `isel`: set the ISEL bit for CFIFO IN direction (ignored for D0/D1FIFO).
///
/// # Safety
/// `fifo.sel` must be valid.
pub unsafe fn fifo_select_pipe(fifo: &FifoPort, pipe_num: usize, isel: bool) {
    unsafe {
        // ISEL selects the access direction for the *bidirectional* DCP buffer
        // only (CFIFO / pipe 0); see TRM §28.3 "The ISEL bit determines this only
        // for the DCP", and regs `FIFOSEL_ISEL` ("CFIFOSEL only").  For D0/D1FIFO
        // the direction is fixed by PIPECFG.DIR, and writing the ISEL bit there
        // flips the FIFO-port DIR, raising a *spurious* BRDY on every IN select
        // (TRM §28.4.2(2): "DIR bit changed 0→1").  That spurious BRDY marks the
        // single-packet IN transfer complete before the host has actually read
        // it, so `write()` returns early.  The reference driver never sets ISEL
        // on DnFIFO — only set it for the DCP.
        let isel_bit: u16 = if isel && pipe_num == 0 { 0x0020 } else { 0 };
        // 32-bit access for both directions; the copy routines handle any
        // non-word-aligned tail.
        let val: u16 = (pipe_num as u16) | (MBW_32 << FIFOSEL_MBW_SHIFT) | isel_bit;
        wr(fifo.sel, val);
    }
}

#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::*;

    #[test]
    fn sw_to_hw_roundtrip() {
        // Test the write path using a mock buffer (no actual hardware).
        // We verify that the unaligned reads in sw_to_hw_fifo don't panic.
        let src = [0x01u8, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07];
        let mut dest = 0u32;
        let fifo = FifoPort {
            data: &mut dest as *mut u32,
            sel: &mut 0u16 as *mut u16,
            ctr: &mut 0u16 as *mut u16,
        };
        // Just verify it doesn't panic for each length up to 7.
        for len in 0..=7 {
            unsafe {
                sw_to_hw_fifo(&fifo, src.as_ptr(), len);
            }
        }
    }

    #[test]
    fn sw_to_hw_tail_uses_bigend0_lanes() {
        // TRM Table 28.9 (8-bit access, BIGEND=0): the valid byte lane for the
        // RZ/A1L is bits[31:24] = CPU byte-address base+3, and tail byte `i`
        // lands on lane `3 - (i & 3)` (matching Linux renesas_usbhs,
        // cfifo_byte_addr=0).  Use a 4-byte scratch word as the mock FIFO data
        // register and write a pure 3-byte tail (no 4-byte body).
        let word_store = 0u32;
        let mut sel = (MBW_32 << FIFOSEL_MBW_SHIFT) as u16;
        let fifo = FifoPort {
            data: &word_store as *const u32 as *mut u32,
            sel: &mut sel as *mut u16,
            ctr: &mut 0u16 as *mut u16,
        };
        let src = [0xAAu8, 0xBB, 0xCC];
        unsafe { sw_to_hw_fifo(&fifo, src.as_ptr(), 3) };
        // Bytes of the mock register in memory order (little-endian): index k is
        // CPU byte-address base+k.  src[0]→lane3, src[1]→lane2, src[2]→lane1.
        let bytes = word_store.to_le_bytes();
        assert_eq!(bytes[3], 0xAA, "tail byte 0 must land on lane 3 (bits[31:24])");
        assert_eq!(bytes[2], 0xBB, "tail byte 1 must land on lane 2");
        assert_eq!(bytes[1], 0xCC, "tail byte 2 must land on lane 1");
        assert_eq!(bytes[0], 0x00, "lane 0 (bits[7:0]) is the prohibited lane, untouched");
    }

    #[test]
    fn hw_to_sw_sub_word() {
        // Verify sub-word unpack (keep MBW=32, read one word, extract bytes).
        let word: u32 = 0x04030201;
        let fifo = FifoPort {
            data: &word as *const u32 as *mut u32,
            sel: &mut 0u16 as *mut u16,
            ctr: &mut 0u16 as *mut u16,
        };
        let mut buf = [0u8; 3];
        unsafe {
            hw_to_sw_fifo(&fifo, buf.as_mut_ptr(), 3);
        }
        assert_eq!(buf, [0x01, 0x02, 0x03]);
    }
}
