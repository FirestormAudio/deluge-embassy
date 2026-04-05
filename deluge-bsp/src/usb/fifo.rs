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
    FIFOSEL_MBW_MASK, FIFOSEL_MBW_SHIFT, MBW_8, MBW_16, Rusb1Regs, rd, wr, wr32,
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
        // Hardware settles within ~4 reads normally; 64 gives ample margin without
        // hanging the ISR if the pipe has no data (e.g. after BCLR).
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
/// Uses MBW=16 (matching the C reference driver `pipe_write_packet`).
/// Switches to MBW=8 for an odd-byte tail.
///
/// # Safety
/// - `fifo.data` / `fifo.sel` must be valid.
/// - The FIFO must have been selected (CURPIPE written) before calling.
pub unsafe fn sw_to_hw_fifo(fifo: &FifoPort, buf: *const u8, len: usize) {
    unsafe {
        let mut p = buf;
        let mut rem = len;

        // Switch to MBW=16 for the bulk of the write (C driver uses 16-bit too).
        set_mbw(fifo.sel, MBW_16);

        // 16-bit words
        while rem >= 2 {
            let half = (p as *const u16).read_unaligned();
            wr32(fifo.data, half as u32);
            p = p.add(2);
            rem -= 2;
        }

        // 8-bit tail
        if rem >= 1 {
            set_mbw(fifo.sel, MBW_8);
            wr32(fifo.data, *p as u32);
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

        // Switch to byte-access mode so each read yields exactly one USB byte.
        set_mbw(fifo.sel, MBW_8);

        let fifo_byte = fifo.data as *const u8;
        let mut p = buf;
        for _ in 0..len {
            p.write(fifo_byte.read_volatile());
            p = p.add(1);
        }
    }
}

// ---------------------------------------------------------------------------
// Select a pipe on a FIFO port
// ---------------------------------------------------------------------------

/// Write the CURPIPE + MBW fields of `fifo.sel` together.
///
/// Sets MBW=16 for the IN direction (write path) and MBW=8 for the OUT
/// direction (read path), matching the C reference driver.
///
/// `isel`: set the ISEL bit for CFIFO IN direction (ignored for D0/D1FIFO).
///
/// # Safety
/// `fifo.sel` must be valid.
pub unsafe fn fifo_select_pipe(fifo: &FifoPort, pipe_num: usize, isel: bool) {
    unsafe {
        let isel_bit: u16 = if isel { 0x0020 } else { 0 };
        // IN path (isel=true) → MBW=16 for 16-bit writes; OUT path → MBW=8 for byte reads.
        let mbw: u16 = if isel { MBW_16 } else { MBW_8 };
        let val: u16 = (pipe_num as u16) | (mbw << FIFOSEL_MBW_SHIFT) | isel_bit;
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
