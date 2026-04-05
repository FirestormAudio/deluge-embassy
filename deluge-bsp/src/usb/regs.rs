//! RUSB1 peripheral register map for Renesas RZ/A1L.
//!
//! `Rusb1Regs` is a `#[repr(C)]` struct that mirrors `struct st_usb20` from
//! `src/RZA1/system/iodefines/usb20_iodefine.h`.  All register accesses go
//! through `core::ptr::read_volatile` / `write_volatile` to prevent the
//! compiler from eliding or reordering hardware-visible writes.
//!
//! ## Usage
//! ```ignore
//! let r = Rusb1Regs::for_port(0);   // borrow (read-only) or use raw ptr
//! unsafe { r.write_syscfg0(r.read_syscfg0() | SYSCFG_USBE); }
//! ```
//!
//! ## Bit constants
//! All bitmask constants are declared in this module as `pub const u16`.
//! Naming follows the C `usb_iobitmask.h` file with the `USB_` prefix removed
//! and snake_case used for the shift variants.

use core::ptr::{read_volatile, write_volatile};
use rza1l_hal::rusb1::{USB0_BASE, USB1_BASE};

// ---------------------------------------------------------------------------
// SYSCFG0
// ---------------------------------------------------------------------------
pub const SYSCFG_USBE: u16 = 0x0001;
pub const SYSCFG_UPLLE: u16 = 0x0002;
pub const SYSCFG_UCKSEL: u16 = 0x0004;
pub const SYSCFG_DPRPU: u16 = 0x0010;
pub const SYSCFG_DRPD: u16 = 0x0020;
pub const SYSCFG_DCFM: u16 = 0x0040;
pub const SYSCFG_HSE: u16 = 0x0080;

pub const SYSCFG_UCKSEL_SHIFT: u16 = 2;
pub const SYSCFG_DCFM_SHIFT: u16 = 6;

// ---------------------------------------------------------------------------
// SYSSTS0
// ---------------------------------------------------------------------------
pub const SYSSTS0_LNST: u16 = 0x0003;

// ---------------------------------------------------------------------------
// DVSTCTR0
// ---------------------------------------------------------------------------
pub const DVSTCTR0_RHST: u16 = 0x0007;
pub const DVSTCTR0_UACT: u16 = 0x0010;
pub const DVSTCTR0_RESUME: u16 = 0x0020;
pub const DVSTCTR0_USBRST: u16 = 0x0040;
pub const DVSTCTR0_WKUP: u16 = 0x0100;
pub const DVSTCTR0_VBUSEN: u16 = 0x0200; // Host mode: enable VBUS power (bit 9)

pub const DVSTCTR0_RHST_LS: u16 = 0b001;
pub const DVSTCTR0_RHST_FS: u16 = 0b010;
pub const DVSTCTR0_RHST_HS: u16 = 0b011;

// ---------------------------------------------------------------------------
// FIFO SEL / CTR  (shared shape between CFIFOSEL and DnFIFOSEL)
// ---------------------------------------------------------------------------
pub const FIFOSEL_CURPIPE_MASK: u16 = 0x000F;
pub const FIFOSEL_ISEL: u16 = 0x0020; // CFIFOSEL only
pub const FIFOSEL_BIGEND: u16 = 0x0100;
pub const FIFOSEL_MBW_MASK: u16 = 0x0C00;
pub const FIFOSEL_MBW_SHIFT: u32 = 10;
pub const FIFOSEL_REW: u16 = 0x4000;
pub const FIFOSEL_RCNT: u16 = 0x8000;

pub const FIFOCTR_DTLN_MASK: u16 = 0x0FFF;
pub const FIFOCTR_FRDY: u16 = 0x2000;
pub const FIFOCTR_BCLR: u16 = 0x4000;
pub const FIFOCTR_BVAL: u16 = 0x8000;

pub const MBW_8: u16 = 0;
pub const MBW_16: u16 = 1;
pub const MBW_32: u16 = 2;

// ---------------------------------------------------------------------------
// INTENB0
// ---------------------------------------------------------------------------
pub const INTENB0_BRDYE: u16 = 0x0100;
pub const INTENB0_NRDYE: u16 = 0x0200;
pub const INTENB0_BEMPE: u16 = 0x0400;
pub const INTENB0_CTRE: u16 = 0x0800;
pub const INTENB0_DVSE: u16 = 0x1000;
pub const INTENB0_SOFE: u16 = 0x2000;
pub const INTENB0_RSME: u16 = 0x4000;
pub const INTENB0_VBSE: u16 = 0x8000;

// ---------------------------------------------------------------------------
// INTENB1 (host-mode attach/detach/setup)
// ---------------------------------------------------------------------------
pub const INTENB1_SACKE: u16 = 0x0010; // Setup ACK enable
pub const INTENB1_SIGNE: u16 = 0x0020; // Setup IGNORE (NAK/error) enable
pub const INTENB1_ATTCHE: u16 = 0x0800;
pub const INTENB1_DTCHE: u16 = 0x1000;
pub const INTENB1_BCHGE: u16 = 0x4000;

// ---------------------------------------------------------------------------
// INTSTS0
// ---------------------------------------------------------------------------
pub const INTSTS0_CTSQ_MASK: u16 = 0x0007;
pub const INTSTS0_VALID: u16 = 0x0008;
pub const INTSTS0_DVSQ_MASK: u16 = 0x0070;
pub const INTSTS0_DVSQ_SHIFT: u32 = 4;
pub const INTSTS0_VBSTS: u16 = 0x0080;
pub const INTSTS0_BRDY: u16 = 0x0100;
pub const INTSTS0_NRDY: u16 = 0x0200;
pub const INTSTS0_BEMP: u16 = 0x0400;
pub const INTSTS0_CTRT: u16 = 0x0800;
pub const INTSTS0_DVST: u16 = 0x1000;
pub const INTSTS0_SOFR: u16 = 0x2000;
pub const INTSTS0_RESM: u16 = 0x4000;
pub const INTSTS0_VBINT: u16 = 0x8000;

// DVSQ sub-values
pub const DVSQ_POWERED: u16 = 0b000;
pub const DVSQ_DEFAULT: u16 = 0b001; // bus reset completed
pub const DVSQ_ADDRESS: u16 = 0b010;
pub const DVSQ_CONFIGURED: u16 = 0b011;
pub const DVSQ_SUSP0: u16 = 0b100;

// CTSQ sub-values
pub const CTSQ_IDLE: u16 = 0b000;
pub const CTSQ_READ_DATA: u16 = 0b001;
pub const CTSQ_READ_STATUS: u16 = 0b010;
pub const CTSQ_WRITE_DATA: u16 = 0b011;
pub const CTSQ_WRITE_STATUS: u16 = 0b100;
pub const CTSQ_WRITE_ZLP: u16 = 0b101;

// ---------------------------------------------------------------------------
// INTSTS1
// ---------------------------------------------------------------------------
pub const INTSTS1_SACK: u16 = 0x0010; // Setup ACK received
pub const INTSTS1_SIGN: u16 = 0x0020; // Setup IGNORE (NAK/error)
pub const INTSTS1_ATTCH: u16 = 0x0800;
pub const INTSTS1_DTCH: u16 = 0x1000;

// ---------------------------------------------------------------------------
// FRMNUM
// ---------------------------------------------------------------------------
pub const FRMNUM_FRNM_MASK: u16 = 0x07FF;
pub const FRMNUM_CRCE: u16 = 0x4000;
pub const FRMNUM_OVRN: u16 = 0x8000;

// ---------------------------------------------------------------------------
// DCPMAXP
// ---------------------------------------------------------------------------
pub const DCPMAXP_MXPS_MASK: u16 = 0x007F;
pub const DCPMAXP_DEVSEL_SHIFT: u32 = 12; // host: device address in DCPMAXP[15:12]

// ---------------------------------------------------------------------------
// DCPCFG (host mode)
// ---------------------------------------------------------------------------
pub const DCPCFG_DIR: u16 = 0x0010; // bit 4: data stage direction (1=host OUT)
pub const DCPCFG_SHTNAK: u16 = 0x0080; // NAK on short packet

// ---------------------------------------------------------------------------
// DCPCTR (DCP control — some bits overlap with PIPEnCTR)
// ---------------------------------------------------------------------------
pub const DCPCTR_SUREQ: u16 = 0x4000; // bit 14: issue SETUP token
pub const DCPCTR_SUREQCLR: u16 = 0x0800; // bit 11: cancel pending SETUP

// ---------------------------------------------------------------------------
// DCPCTR / PIPEnCTR
// ---------------------------------------------------------------------------
pub const PIPECTR_PID_MASK: u16 = 0x0003;
pub const PIPECTR_PID_NAK: u16 = 0b00;
pub const PIPECTR_PID_BUF: u16 = 0b01;
pub const PIPECTR_PID_STALL: u16 = 0b10; // 0b10 or 0b11
pub const PIPECTR_CCPL: u16 = 0x0004; // DCP only
pub const PIPECTR_PBUSY: u16 = 0x0020;
pub const PIPECTR_SQMON: u16 = 0x0040;
pub const PIPECTR_SQSET: u16 = 0x0080;
pub const PIPECTR_SQCLR: u16 = 0x0100;
pub const PIPECTR_ACLRM: u16 = 0x0200; // pipes 1-9,A-F
pub const PIPECTR_INBUFM: u16 = 0x4000; // pipes 1-5 only
pub const PIPECTR_BSTS: u16 = 0x8000;

// ---------------------------------------------------------------------------
// PIPECFG
// ---------------------------------------------------------------------------
pub const PIPECFG_EPNUM_MASK: u16 = 0x000F;
pub const PIPECFG_DIR: u16 = 0x0010; // 0=OUT, 1=IN
pub const PIPECFG_SHTNAK: u16 = 0x0080;
pub const PIPECFG_CNTMD: u16 = 0x0100;
pub const PIPECFG_DBLB: u16 = 0x0200;
pub const PIPECFG_BFRE: u16 = 0x0400;
pub const PIPECFG_TYPE_MASK: u16 = 0xC000;
pub const PIPECFG_TYPE_SHIFT: u32 = 14;
pub const PIPECFG_TYPE_BULK: u16 = 0b01 << 14;
pub const PIPECFG_TYPE_INTR: u16 = 0b10 << 14;
pub const PIPECFG_TYPE_ISO: u16 = 0b11 << 14;

// ---------------------------------------------------------------------------
// PIPEBUF
// ---------------------------------------------------------------------------
pub const PIPEBUF_BUFNMB_MASK: u16 = 0x00FF;
pub const PIPEBUF_BUFSIZE_MASK: u16 = 0x7C00;
pub const PIPEBUF_BUFSIZE_SHIFT: u32 = 10;

// ---------------------------------------------------------------------------
// PIPEMAXP
// ---------------------------------------------------------------------------
pub const PIPEMAXP_MXPS_MASK: u16 = 0x07FF;
pub const PIPEMAXP_DEVSEL_SHIFT: u32 = 12;

// ---------------------------------------------------------------------------
// PIPEPERI
// ---------------------------------------------------------------------------
// Layout (TRM §28.3.35):
//   [15:13] — reserved
//   [12]    IFIS — isochronous IN buffer flush (host mode: must be 0)
//   [11:3]  — reserved
//   [2:0]   IITV[2:0] — interval as 2^n frames (0..=7)
//
// IITV is only meaningful for interrupt/isochronous pipes (6-9 in host mode).
// For bulk pipes (3-5) and pipes 10-15, the TRM requires IITV=0b000.
//
// Conversion from USB bInterval (FS device, 1-255 frames) to IITV:
//   IITV = floor(log2(bInterval)).min(7)   (so bInterval=1 → 0, =2 → 1, =4 → 2, ...)
pub const PIPEPERI_IFIS: u16 = 1 << 12; // isochronous IN buffer flush enable
pub const PIPEPERI_IITV_MASK: u16 = 0x0007;

// ---------------------------------------------------------------------------
// DEVADD (host mode — per-device speed + hub configuration)
// ---------------------------------------------------------------------------
// Layout (TRM §28.3 DEVADDn):
//   [15]    — reserved
//   [14:11] UPPHUB[3:0] — USB address of the hub the device is behind
//                          (0000 = directly attached to this port)
//   [10:8]  HUBPORT[2:0] — hub port number
//                          (000 = directly attached to this port)
//   [7:6]   USBSPD[1:0]  — 00=unused, 01=LS, 10=FS, 11=HS
//   [5:0]   — reserved
//
// When UPPHUB/HUBPORT are non-zero the controller automatically issues
// split transactions (SSPLIT/CSPLIT) for the device — no firmware action
// beyond writing the register is needed.
/// USBSPD field in DEVADDn: bits [7:6] = 01 → LS, 10 → FS.
pub const DEVADD_USBSPD_SHIFT: u32 = 6;
pub const DEVADD_USBSPD_LS: u16 = 1 << 6; // Low-speed  (01)
pub const DEVADD_USBSPD_FS: u16 = 2 << 6; // Full-speed (10)
pub const DEVADD_USBSPD_HS: u16 = 3 << 6; // High-speed (11)
/// UPPHUB field in DEVADDn: bits [14:11] — hub USB address (0 = direct).
pub const DEVADD_UPPHUB_SHIFT: u32 = 11;
/// HUBPORT field in DEVADDn: bits [10:8] — hub port number (0 = direct).
pub const DEVADD_HUBPORT_SHIFT: u32 = 8;

// ---------------------------------------------------------------------------
// SUSPMODE
// ---------------------------------------------------------------------------
pub const SUSPMODE_SUSPM: u16 = 0x4000;

// ---------------------------------------------------------------------------
// Packet buffer constants  (§28 of the RZ/A1L TRM)
// ---------------------------------------------------------------------------
/// Total packet buffer size in bytes.
pub const PKT_BUF_SIZE_BYTES: usize = 8192;
/// Granularity of one buffer block in bytes.
pub const PKT_BUF_BLOCK_SIZE: usize = 64;
/// Total number of 64-byte blocks in the packet buffer.
pub const PKT_BUF_BLOCKS: usize = PKT_BUF_SIZE_BYTES / PKT_BUF_BLOCK_SIZE; // 128

// ---------------------------------------------------------------------------
// BUSWAIT — minimum wait cycles for >= 67 ns at 12 MHz P1 bus.
// At 12 MHz one cycle = 83.3 ns, so 1 wait cycle is sufficient.
// The Deluge firmware uses 0x0F (15 wait cycles) which is very conservative.
// ---------------------------------------------------------------------------
pub const BUSWAIT_VALUE: u16 = 0x000F;

// ---------------------------------------------------------------------------
// Register struct
// ---------------------------------------------------------------------------

/// Memory-mapped register block for one RUSB1 USB module.
///
/// The struct layout exactly matches `struct st_usb20` from
/// `usb20_iodefine.h`.  All fields declared `pub` for direct volatile
/// access in the driver modules; do **not** read or write them without
/// `read_volatile` / `write_volatile`.
///
/// Obtain a pointer via [`Rusb1Regs::ptr`].
#[repr(C)]
pub struct Rusb1Regs {
    pub syscfg0: u16,   // +0x00
    pub buswait: u16,   // +0x02
    pub syssts0: u16,   // +0x04
    _pad0: [u8; 2],     // +0x06
    pub dvstctr0: u16,  // +0x08
    _pad1: [u8; 2],     // +0x0A
    pub testmode: u16,  // +0x0C
    _pad2: [u8; 2],     // +0x0E
    pub d0fbcfg: u16,   // +0x10
    pub d1fbcfg: u16,   // +0x12
    pub cfifo: u32,     // +0x14 (32-bit data port)
    pub d0fifo: u32,    // +0x18
    pub d1fifo: u32,    // +0x1C
    pub cfifosel: u16,  // +0x20
    pub cfifoctr: u16,  // +0x22
    _pad3: [u8; 4],     // +0x24
    pub d0fifosel: u16, // +0x28
    pub d0fifoctr: u16, // +0x2A
    pub d1fifosel: u16, // +0x2C
    pub d1fifoctr: u16, // +0x2E
    pub intenb0: u16,   // +0x30
    pub intenb1: u16,   // +0x32
    _pad4: [u8; 2],     // +0x34
    pub brdyenb: u16,   // +0x36
    pub nrdyenb: u16,   // +0x38
    pub bempenb: u16,   // +0x3A
    pub sofcfg: u16,    // +0x3C
    _pad5: [u8; 2],     // +0x3E
    pub intsts0: u16,   // +0x40
    pub intsts1: u16,   // +0x42
    _pad6: [u8; 2],     // +0x44
    pub brdysts: u16,   // +0x46
    pub nrdysts: u16,   // +0x48
    pub bempsts: u16,   // +0x4A
    pub frmnum: u16,    // +0x4C
    pub ufrmnum: u16,   // +0x4E
    pub usbaddr: u16,   // +0x50
    _pad7: [u8; 2],     // +0x52
    pub usbreq: u16,    // +0x54
    pub usbval: u16,    // +0x56
    pub usbindx: u16,   // +0x58
    pub usbleng: u16,   // +0x5A
    pub dcpcfg: u16,    // +0x5C
    pub dcpmaxp: u16,   // +0x5E
    pub dcpctr: u16,    // +0x60
    _pad8: [u8; 2],     // +0x62
    pub pipesel: u16,   // +0x64
    _pad9: [u8; 2],     // +0x66
    pub pipecfg: u16,   // +0x68
    pub pipebuf: u16,   // +0x6A
    pub pipemaxp: u16,  // +0x6C
    pub pipeperi: u16,  // +0x6E
    // PIPEnCTR for pipes 1-9 (contiguous) then A-F
    pub pipe1ctr: u16, // +0x70
    pub pipe2ctr: u16, // +0x72
    pub pipe3ctr: u16, // +0x74
    pub pipe4ctr: u16, // +0x76
    pub pipe5ctr: u16, // +0x78
    pub pipe6ctr: u16, // +0x7A
    pub pipe7ctr: u16, // +0x7C
    pub pipe8ctr: u16, // +0x7E
    pub pipe9ctr: u16, // +0x80
    pub pipeactr: u16, // +0x82
    pub pipebctr: u16, // +0x84
    pub pipecctr: u16, // +0x86
    pub pipedctr: u16, // +0x88
    pub pipeectr: u16, // +0x8A
    pub pipefctr: u16, // +0x8C
    _pad10: [u8; 2],   // +0x8E
    // TRE/TRN pairs: pipes 1-5 are contiguous
    pub pipe1tre: u16, // +0x90
    pub pipe1trn: u16, // +0x92
    pub pipe2tre: u16, // +0x94
    pub pipe2trn: u16, // +0x96
    pub pipe3tre: u16, // +0x98
    pub pipe3trn: u16, // +0x9A
    pub pipe4tre: u16, // +0x9C
    pub pipe4trn: u16, // +0x9E
    pub pipe5tre: u16, // +0xA0
    pub pipe5trn: u16, // +0xA2
    // Pipes B-F (note: not contiguous with 1-5 in hardware)
    pub pipebtre: u16, // +0xA4
    pub pipebtrn: u16, // +0xA6
    pub pipectre: u16, // +0xA8
    pub pipectrn: u16, // +0xAA
    pub pipedtre: u16, // +0xAC
    pub pipedtrn: u16, // +0xAE
    pub pipeetre: u16, // +0xB0
    pub pipeetrn: u16, // +0xB2
    pub pipeftre: u16, // +0xB4
    pub pipeftrn: u16, // +0xB6
    // Pipe 9 and A TRE/TRN come *after* B-F in the hardware
    pub pipe9tre: u16, // +0xB8
    pub pipe9trn: u16, // +0xBA
    pub pipeatre: u16, // +0xBC
    pub pipeatrn: u16, // +0xBE
    _pad11: [u8; 16],  // +0xC0 – 0xCF
    // Device-address configuration registers (DEVADD0-A)
    pub devadd0: u16, // +0xD0
    pub devadd1: u16, // +0xD2
    pub devadd2: u16, // +0xD4
    pub devadd3: u16, // +0xD6
    pub devadd4: u16, // +0xD8
    pub devadd5: u16, // +0xDA
    pub devadd6: u16, // +0xDC
    pub devadd7: u16, // +0xDE
    pub devadd8: u16, // +0xE0
    pub devadd9: u16, // +0xE2
    pub devadda: u16, // +0xE4
    _pad12: [u8; 28], // +0xE6 – 0x101
    pub suspmode: u16, // +0x102
                      // Remainder (DMA burst buffers, not used in this driver) omitted.
}

impl Rusb1Regs {
    /// Return a raw mutable pointer to the register block for `port` (0 or 1).
    ///
    /// # Safety
    /// The caller must ensure only one `&mut` alias exists at a time, and that
    /// all accesses use `read_volatile` / `write_volatile`.
    #[inline]
    pub unsafe fn ptr(port: u8) -> *mut Self {
        let base = if port == 0 { USB0_BASE } else { USB1_BASE };
        base as *mut Self
    }
}

// ---------------------------------------------------------------------------
// Read / write helpers
// ---------------------------------------------------------------------------

/// Read a 16-bit USB register via `read_volatile`.
///
/// # Safety
/// `reg` must point to a valid, aligned, memory-mapped register.
#[inline]
pub unsafe fn rd(reg: *const u16) -> u16 {
    unsafe { read_volatile(reg) }
}

/// Write a 16-bit USB register via `write_volatile`.
///
/// # Safety
/// `reg` must point to a valid, aligned, memory-mapped register.
#[inline]
pub unsafe fn wr(reg: *mut u16, val: u16) {
    unsafe {
        write_volatile(reg, val);
    }
}

/// Read-modify-write: apply `mask` and `val` (both un-shifted).
///
/// # Safety
/// Same as [`wr`].
#[inline]
pub unsafe fn rmw(reg: *mut u16, mask: u16, val: u16) {
    unsafe {
        let cur = read_volatile(reg);
        write_volatile(reg, (cur & !mask) | (val & mask));
    }
}

/// Read a 32-bit FIFO data port.
///
/// # Safety
/// `reg` must be a valid 32-bit-aligned FIFO data register.
#[inline]
pub unsafe fn rd32(reg: *const u32) -> u32 {
    unsafe { read_volatile(reg) }
}

/// Write a 32-bit FIFO data port.
///
/// # Safety
/// `reg` must be a valid 32-bit-aligned FIFO data register.
#[inline]
pub unsafe fn wr32(reg: *mut u32, val: u32) {
    unsafe {
        write_volatile(reg, val);
    }
}

// ---------------------------------------------------------------------------
// Pipe control register helper
// ---------------------------------------------------------------------------

/// Return a pointer to the PIPEnCTR register for pipe `n`.
/// Pipe 0 → DCPCTR. Pipes 1-15 → PIPE1CTR … PIPEFCTR.
///
/// # Safety
/// `regs` must be a valid pointer to the USB register block.
pub unsafe fn pipectr_ptr(regs: *mut Rusb1Regs, n: usize) -> *mut u16 {
    unsafe {
        if n == 0 {
            core::ptr::addr_of_mut!((*regs).dcpctr)
        } else {
            // PIPE1CTR is at +0x70; pipes are contiguous u16 words.
            core::ptr::addr_of_mut!((*regs).pipe1ctr).add(n - 1)
        }
    }
}

/// Return a pointer to the `DEVADDn` register for device address `dev_addr` (0-10).
///
/// # Safety
/// `regs` must be valid.  `dev_addr` must be in the range 0-10.
pub unsafe fn devadd_ptr(regs: *mut Rusb1Regs, dev_addr: u8) -> *mut u16 {
    unsafe {
        debug_assert!((dev_addr as usize) <= 10);
        core::ptr::addr_of_mut!((*regs).devadd0).add(dev_addr as usize)
    }
}

// ---------------------------------------------------------------------------
// Compile-time layout assertions
// ---------------------------------------------------------------------------

#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::*;
    use core::mem::offset_of;

    #[test]
    fn reg_offsets() {
        assert_eq!(offset_of!(Rusb1Regs, syscfg0), 0x00);
        assert_eq!(offset_of!(Rusb1Regs, syssts0), 0x04);
        assert_eq!(offset_of!(Rusb1Regs, dvstctr0), 0x08);
        assert_eq!(offset_of!(Rusb1Regs, cfifo), 0x14);
        assert_eq!(offset_of!(Rusb1Regs, d0fifo), 0x18);
        assert_eq!(offset_of!(Rusb1Regs, d1fifo), 0x1C);
        assert_eq!(offset_of!(Rusb1Regs, cfifosel), 0x20);
        assert_eq!(offset_of!(Rusb1Regs, d0fifosel), 0x28);
        assert_eq!(offset_of!(Rusb1Regs, d1fifosel), 0x2C);
        assert_eq!(offset_of!(Rusb1Regs, intenb0), 0x30);
        assert_eq!(offset_of!(Rusb1Regs, brdyenb), 0x36);
        assert_eq!(offset_of!(Rusb1Regs, intsts0), 0x40);
        assert_eq!(offset_of!(Rusb1Regs, intsts1), 0x42);
        assert_eq!(offset_of!(Rusb1Regs, brdysts), 0x46);
        assert_eq!(offset_of!(Rusb1Regs, usbaddr), 0x50);
        assert_eq!(offset_of!(Rusb1Regs, usbreq), 0x54);
        assert_eq!(offset_of!(Rusb1Regs, dcpcfg), 0x5C);
        assert_eq!(offset_of!(Rusb1Regs, dcpmaxp), 0x5E);
        assert_eq!(offset_of!(Rusb1Regs, dcpctr), 0x60);
        assert_eq!(offset_of!(Rusb1Regs, pipesel), 0x64);
        assert_eq!(offset_of!(Rusb1Regs, pipecfg), 0x68);
        assert_eq!(offset_of!(Rusb1Regs, pipebuf), 0x6A);
        assert_eq!(offset_of!(Rusb1Regs, pipemaxp), 0x6C);
        assert_eq!(offset_of!(Rusb1Regs, pipe1ctr), 0x70);
        assert_eq!(offset_of!(Rusb1Regs, pipe9ctr), 0x80);
        assert_eq!(offset_of!(Rusb1Regs, pipefctr), 0x8C);
        assert_eq!(offset_of!(Rusb1Regs, pipe1tre), 0x90);
        assert_eq!(offset_of!(Rusb1Regs, pipebtre), 0xA4);
        assert_eq!(offset_of!(Rusb1Regs, pipe9tre), 0xB8);
        assert_eq!(offset_of!(Rusb1Regs, devadd0), 0xD0);
        assert_eq!(offset_of!(Rusb1Regs, suspmode), 0x102);
    }
}
