//! Generic Interrupt Controller (GIC) driver for the Cortex-A9 / RZ/A1L.
//!
//! The RZ/A1L uses the ARM GIC-400 (PL390) split into:
//!   - GIC Distributor (GICD) at 0xE820_1000  (the Renesas "INTC" peripheral)
//!   - GIC CPU Interface (GICC) at 0xE820_2000
//!
//! The chip has 587 interrupt sources total (IDs 0–586).
//!
//! ## Initialization sequence
//! Call [`init`] once before enabling global IRQ (`cpsie i`).
//! Register handlers via [`register`], enable individual sources via [`enable`].
//!
//! ## IRQ dispatch
//! The assembly IRQ handler in `startup.rs` calls [`dispatch`] with the raw
//! ICCIAR register value. This function looks up and calls the registered Rust
//! handler (with IRQs re-enabled for nesting), then returns so the assembly
//! can write ICCEOIR.

// ------- GIC Distributor register addresses (base 0xE820_1000) --------------
const GICD_BASE: usize = 0xE820_1000;

const GICD_CTLR: usize = GICD_BASE; // Distributor Control
const GICD_IGROUPR0: usize = GICD_BASE + 0x080; // Interrupt Security (ICDISR0)
const GICD_ISENABLER0: usize = GICD_BASE + 0x100; // Set-Enable (ICDISER0)
const GICD_ICENABLER0: usize = GICD_BASE + 0x180; // Clear-Enable (ICDICER0)
const GICD_IPRIORITYR0: usize = GICD_BASE + 0x400; // Priority (ICDIPR0)
const GICD_ITARGETSR0: usize = GICD_BASE + 0x800; // Target (ICDIPTR0)
const GICD_ICFGR0: usize = GICD_BASE + 0xC00; // Configuration (ICDICFR0)

// ------- GIC CPU Interface register addresses (base 0xE820_2000) -------------
const GICC_BASE: usize = 0xE820_2000;

pub(crate) const GICC_CTLR_ADDR: usize = GICC_BASE; // CPU Interface Control (ICCICR)
pub(crate) const GICC_PMR_ADDR: usize = GICC_BASE + 0x004; // Priority Mask (ICCPMR)
pub(crate) const GICC_BPR_ADDR: usize = GICC_BASE + 0x008; // Binary Point (ICCBPR)
pub(crate) const GICC_IAR_ADDR: usize = GICC_BASE + 0x00C; // Interrupt Acknowledge (ICCIAR)
pub(crate) const GICC_EOIR_ADDR: usize = GICC_BASE + 0x010; // End of Interrupt (ICCEOIR)
pub(crate) const GICC_HPPIR_ADDR: usize = GICC_BASE + 0x018; // Highest-Priority Pending (ICCHPIR)

pub(crate) const GICD_IPRIORITYR0_ADDR: usize = GICD_IPRIORITYR0;

// ---------------------------------------------------------------------------
// Priority field constants
// ---------------------------------------------------------------------------

/// GIC priority field is bits [7:3] of each IPRIORITYR byte; shift a 5-bit
/// priority value left by this amount to position it correctly.
const GIC_PRIORITY_SHIFT: u32 = 3;
/// Priority-word filled with the lowest active priority (31 << 3 = 0xF8)
/// repeated in every byte.  Written to all IPRIORITYR words during init to
/// start all interrupts at the lowest priority until explicitly lowered.
const GIC_PRIORITY_LOWEST_WORD: u32 = 0xF8F8_F8F8;
/// GICC_CTLR (ICCICR) bit 0: EnableS — enables forwarding of Group 0 (secure) interrupts.
/// BSP: INTC_ICCICR_EnableS = 0x01
const ICCICR_ENABLES: u32 = 0x1;
/// GICC_CTLR (ICCICR) bit 1: EnableNS — enables forwarding of Group 1 (non-secure) interrupts.
/// BSP: INTC_ICCICR_EnableNS = 0x02
const ICCICR_ENABLENS: u32 = 0x2;
/// ICCIAR bits [9:0]: ACKINTID — Interrupt Acknowledge ID field.
/// BSP: INTC_ICCIAR_ACKINTID = 0x3FF
const ICCIAR_ACKINTID: u32 = 0x3FF;
/// ICCIAR spurious interrupt ID lower bound (IDs ≥ 0x3FE are spurious).
/// 0x3FE = 1022 (spurious group IRQ), 0x3FF = 1023 (spurious FIQ).
const ICCIAR_SPURIOUS: u32 = 0x3FE;

// ---------------------------------------------------------------------------

/// Total number of interrupt sources on RZ/A1L.
pub const INT_ID_TOTAL: usize = 587;

/// Interrupt handler type. The handler is called with IRQ re-enabled
/// (nested interrupts supported), already in SYS mode.
pub type Handler = fn();

/// Wrapper making `UnsafeCell<Option<Handler>>` `Sync`.
///
/// Sound on this target because access is single-core: registration happens
/// before `cpsie i`, and dispatch only runs in the IRQ context (IRQ masked
/// for the caller).
struct HandlerCell(core::cell::UnsafeCell<Option<Handler>>);

// SAFETY: bare-metal single-core; no actual concurrent access.
unsafe impl Sync for HandlerCell {}

impl HandlerCell {
    const fn new() -> Self {
        HandlerCell(core::cell::UnsafeCell::new(None))
    }
    /// Write a new handler.  Caller must ensure no concurrent read.
    unsafe fn set(&self, h: Handler) {
        unsafe {
            *self.0.get() = Some(h);
        }
    }
    /// Read the current handler.  Caller must ensure no concurrent write.
    unsafe fn get(&self) -> Option<Handler> {
        unsafe { *self.0.get() }
    }
}

/// IRQ handler dispatch table — one slot per interrupt ID.
/// Initialized to `None` (unhandled = silently EOI'd).
///
/// # Safety contract
/// [`register`] must only be called before global IRQ is enabled.
/// [`gic_dispatch`] is only called from the IRQ handler with IRQ masked,
/// so no data race is possible on single-core.
static HANDLERS: [HandlerCell; INT_ID_TOTAL] = {
    #[allow(clippy::declare_interior_mutable_const)]
    const NONE: HandlerCell = HandlerCell::new();
    [NONE; INT_ID_TOTAL]
};

/// Initial ICDICFR edge/level configuration values (37 registers × 32 bits).
/// Taken verbatim from the Renesas `intc_icdicfrn_table[]` in `intc.c`.
/// Each 2-bit field: 0b01 = edge, 0b00 = level (for SPI sources bit[1] is the indicator).
static ICDICFR_INIT: [u32; 37] = [
    0xAAAAAAAA, /* ICDICFR0  :  15 to   0 */
    0x00000055, /* ICDICFR1  :  19 to  16 */
    0xFFFD5555, /* ICDICFR2  :  47 to  32 */
    0x555FFFFF, /* ICDICFR3  :  63 to  48 */
    0x55555555, /* ICDICFR4  :  79 to  64 */
    0x55555555, /* ICDICFR5  :  95 to  80 */
    0x55555555, /* ICDICFR6  : 111 to  96 */
    0x55555555, /* ICDICFR7  : 127 to 112 */
    0x5555F555, /* ICDICFR8  : 143 to 128 */
    0x55555555, /* ICDICFR9  : 159 to 144 */
    0x55555555, /* ICDICFR10 : 175 to 160 */
    0xF5555555, /* ICDICFR11 : 191 to 176 */
    0xF555F555, /* ICDICFR12 : 207 to 192 */
    0x5555F555, /* ICDICFR13 : 223 to 208 */
    0x55555555, /* ICDICFR14 : 239 to 224 */
    0x55555555, /* ICDICFR15 : 255 to 240 */
    0x55555555, /* ICDICFR16 : 271 to 256 */
    0xFD555555, /* ICDICFR17 : 287 to 272 */
    0x55555557, /* ICDICFR18 : 303 to 288 */
    0x55555555, /* ICDICFR19 : 319 to 304 */
    0xFFD55555, /* ICDICFR20 : 335 to 320 */
    0x5F55557F, /* ICDICFR21 : 351 to 336 */
    0xFD55555F, /* ICDICFR22 : 367 to 352 */
    0x55555557, /* ICDICFR23 : 383 to 368 */
    0x55555555, /* ICDICFR24 : 399 to 384 */
    0x55555555, /* ICDICFR25 : 415 to 400 */
    0x55555555, /* ICDICFR26 : 431 to 416 */
    0x55555555, /* ICDICFR27 : 447 to 432 */
    0x55555555, /* ICDICFR28 : 463 to 448 */
    0x55555555, /* ICDICFR29 : 479 to 464 */
    0x55555555, /* ICDICFR30 : 495 to 480 */
    0x55555555, /* ICDICFR31 : 511 to 496 */
    0x55555555, /* ICDICFR32 : 527 to 512 */
    0x55555555, /* ICDICFR33 : 543 to 528 */
    0x55555555, /* ICDICFR34 : 559 to 544 */
    0x55555555, /* ICDICFR35 : 575 to 560 */
    0x00155555, /* ICDICFR36 : 586 to 576 */
];

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/// Initialize the GIC distributor and CPU interface.
///
/// Initialise the Generic Interrupt Controller (GIC).
///
/// Mirrors `R_INTC_Init()` from `intc.c`. Must be called once, with global
/// IRQ disabled, before any [`register`] / [`enable`] calls.
///
/// ## Priority filter (PMR)
///
/// This function sets `GICC_PMR = 0xF8` (priority threshold = 31).  The GIC
/// forwards an interrupt to the CPU **only if its programmed priority is
/// strictly less than PMR**.  Because the hardware reset priority for every
/// interrupt is 31, an interrupt whose priority has *not* been explicitly
/// lowered via [`set_priority`] will **never fire** even after [`enable`] is
/// called.  Always call `set_priority(id, n)` with `n < 31` before `enable`.
///
/// # Safety
/// Writes to memory-mapped GIC registers. Must run exactly once before
/// `cpsie i`.
pub unsafe fn init() {
    unsafe {
        log::debug!("gic: init ({} IRQ sources)", INT_ID_TOTAL);
        let n_isr = (INT_ID_TOTAL / 32) + 1; // 19 — ICDISR count
        let _n_icfr = ICDICFR_INIT.len(); // 37 — ICDICFR count
        let n_ipr = (INT_ID_TOTAL / 4) + 1; // 147 — ICDIPR count
        let n_iptr = n_ipr; // 147 — ICDIPTR count
        let n_icer = n_isr; // 19  — ICDICER count

        // 1. Mark all interrupts as secure (Group 0), matching the C driver.
        let igroupr = GICD_IGROUPR0 as *mut u32;
        for i in 0..n_isr {
            igroupr.add(i).write_volatile(0);
        }

        // 2. Edge/level configuration — Renesas-specific table from intc.c.
        //    ICDICFR0 (index 0) is read-only per TRM (SGIs are always edge); skip it.
        let icfgr = GICD_ICFGR0 as *mut u32;
        for (i, &val) in ICDICFR_INIT.iter().enumerate().skip(1) {
            icfgr.add(i).write_volatile(val);
        }

        // 3. Set all interrupt priorities to the lowest active level (31).
        //    Priority field is [7:3], so 31 << 3 = 0xF8 per byte.
        let ipr = GICD_IPRIORITYR0 as *mut u32;
        for i in 0..n_ipr {
            ipr.add(i).write_volatile(GIC_PRIORITY_LOWEST_WORD);
        }

        // 4. Route all SPI interrupts (IDs 32+) to CPU 0 (target byte = 0x01).
        //    ICDIPTR0–ICDIPTR7 (IDs 0–31) are CPU-local and must not be touched.
        let iptr = GICD_ITARGETSR0 as *mut u32;
        for i in 8..n_iptr {
            iptr.add(i).write_volatile(0x0101_0101);
        }

        // 5. Disable all interrupts (ICDICER — write-1-to-clear).
        let icer = GICD_ICENABLER0 as *mut u32;
        for i in 0..n_icer {
            icer.add(i).write_volatile(0xFFFF_FFFF);
        }

        // 6. CPU interface: priority mask — accept all priorities 0–30.
        //    ICCPMR bit field [7:3] is valid; 31 << 3 = 0xF8.
        (GICC_PMR_ADDR as *mut u32).write_volatile(31u32 << GIC_PRIORITY_SHIFT);

        // 7. Binary point: group priority [7:3], sub-priority [2:0] unused.
        (GICC_BPR_ADDR as *mut u32).write_volatile(2);

        // 8. Enable CPU interface — FIQ+IRQ forwarding for secure interrupts.
        (GICC_CTLR_ADDR as *mut u32).write_volatile(ICCICR_ENABLES | ICCICR_ENABLENS);

        // 9. Enable GIC distributor.
        (GICD_CTLR as *mut u32).write_volatile(1);
        log::debug!("gic: distributor + CPU interface enabled");
    }
}

/// Register a Rust function as the handler for interrupt `id`.
///
/// Must only be called while global IRQ is disabled (before `cpsie i`).
///
/// # Safety
/// See struct-level safety note. Must be called before IRQ is enabled.
pub unsafe fn register(id: u16, handler: Handler) {
    unsafe {
        log::trace!("gic: register IRQ {}", id);
        if (id as usize) < INT_ID_TOTAL {
            HANDLERS[id as usize].set(handler);
        }
    }
}

/// Enable (unmask) interrupt `id` in the GIC distributor.
///
/// **Requires prior `set_priority` call:** `GICC_PMR` is set to priority 31
/// by [`init`].  An interrupt left at its reset priority of 31 will not
/// reach the CPU even after being enabled here.  Always call
/// [`set_priority`]`(id, n)` with `n < 31` before this function.
///
/// # Safety
/// Writes to memory-mapped peripheral registers.
pub unsafe fn enable(id: u16) {
    unsafe {
        if (id as usize) >= INT_ID_TOTAL {
            return;
        }
        let addr = (GICD_ISENABLER0 as *mut u32).add((id >> 5) as usize);
        addr.write_volatile(1u32 << (id & 31));
    }
}

/// Disable (mask) interrupt `id` in the GIC distributor.
///
/// # Safety
/// Writes to memory-mapped peripheral registers.
pub unsafe fn disable(id: u16) {
    unsafe {
        if (id as usize) >= INT_ID_TOTAL {
            return;
        }
        let addr = (GICD_ICENABLER0 as *mut u32).add((id >> 5) as usize);
        addr.write_volatile(1u32 << (id & 31));
    }
}

/// Set priority for interrupt `id` (0 = highest, 31 = lowest).
///
/// # Safety
/// Writes to memory-mapped peripheral registers.
pub unsafe fn set_priority(id: u16, priority: u8) {
    unsafe {
        if (id as usize) >= INT_ID_TOTAL || priority >= 32 {
            return;
        }
        let addr = (GICD_IPRIORITYR0 as *mut u32).add((id / 4) as usize);
        let shift = ((id % 4) * 8) as u32;
        let mut val = addr.read_volatile();
        val &= !(0xFFu32 << shift);
        val |= ((priority as u32) << GIC_PRIORITY_SHIFT) << shift; // bits [7:3] are the valid field
        addr.write_volatile(val);
    }
}

// ---------------------------------------------------------------------------
// RZ/A1L external-interrupt (IRQ0–7) helpers
// ---------------------------------------------------------------------------

/// Configure external interrupt `irq_num` (0–7) for **both-edge** detection.
///
/// Programs ICR1 at `0xFCFEF802`.  Per the TRM the mode bits must be cleared
/// to `0b00` (level-low) before writing the desired `0b11` (both-edges) value.
///
/// # Safety
/// Writes to MMIO.  Call before enabling the interrupt in the GIC.
pub unsafe fn set_irq_both_edges(irq_num: u8) {
    unsafe {
        const ICR1: usize = 0xFCFE_F802;
        let shift = (irq_num * 2) as u32;
        let mask = 0b11u16 << shift;
        let ptr = ICR1 as *mut u16;
        let cleared = ptr.read_volatile() & !mask; // clear to level-low (00) first
        ptr.write_volatile(cleared);
        ptr.write_volatile(cleared | (0b11u16 << shift)); // then both-edges (11)
    }
}

/// Clear the pending flag for external interrupt `irq_num` (0–7) in IRQRR.
///
/// Must be called inside the IRQ handler body before returning; otherwise the
/// GIC will re-assert the interrupt immediately after EOI.
///
/// # Safety
/// Writes to MMIO register at `0xFCFEF804`.
pub unsafe fn clear_irq_pending(irq_num: u8) {
    unsafe {
        const IRQRR: usize = 0xFCFE_F804;
        let ptr = IRQRR as *mut u16;
        ptr.write_volatile(ptr.read_volatile() & !(1u16 << irq_num));
    }
}

// ---------------------------------------------------------------------------
// Dispatch — called from assembly IRQ handler
// ---------------------------------------------------------------------------

/// Called from the IRQ handler assembly stub with the raw ICCIAR value.
///
/// Looks up and calls the registered handler for the interrupt. The handler
/// runs with IRQ re-enabled (Cortex-A9 nested interrupt support).
/// The assembly stub is responsible for writing ICCEOIR after this returns.
///
/// # Safety
/// Must only be called from the IRQ handler assembly, with the CPU in
/// SYS mode (as set up by the IRQ handler prologue).
#[cfg(target_os = "none")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn gic_dispatch(icciar: u32) {
    unsafe {
        let int_id = (icciar & ICCIAR_ACKINTID) as u16;

        // IDs 1022 (0x3FE) and 1023 (0x3FF) are spurious — no EOI needed for them
        // but we just return; the assembly will still write ICCEOIR with the value
        // we received, which is harmless for 0x3FE/0x3FF.
        if int_id >= ICCIAR_SPURIOUS as u16 {
            log::warn!("GIC: spurious IRQ {}", int_id);
            return;
        }

        if (int_id as usize) < INT_ID_TOTAL {
            if let Some(f) = HANDLERS[int_id as usize].get() {
                // Re-enable IRQ to allow higher-priority interrupts to preempt.
                cortex_ar::interrupt::enable();
                f();
                cortex_ar::interrupt::disable();
            } else {
                log::warn!("GIC: id={} no handler — EOI only", int_id);
            }
        }
    }
}

#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::{
        GICC_BASE, GICC_CTLR_ADDR, GICC_EOIR_ADDR, GICC_IAR_ADDR, GICC_PMR_ADDR, GICD_BASE,
        GICD_CTLR, GICD_ICFGR0, GICD_IPRIORITYR0, GICD_ISENABLER0, ICDICFR_INIT, INT_ID_TOTAL,
    };

    #[test]
    fn int_id_total() {
        // RZ/A1L has 587 interrupt sources (IDs 0–586).
        assert_eq!(INT_ID_TOTAL, 587);
    }

    #[test]
    fn gicd_base_address() {
        assert_eq!(GICD_BASE, 0xE820_1000);
    }

    #[test]
    fn gicc_base_address() {
        assert_eq!(GICC_BASE, 0xE820_2000);
    }

    /// Spot-check key CPU-interface register addresses against the HW manual.
    #[test]
    fn gicc_register_addresses() {
        assert_eq!(GICC_CTLR_ADDR, 0xE820_2000); // ICCICR
        assert_eq!(GICC_PMR_ADDR, 0xE820_2004); // ICCPMR
        assert_eq!(GICC_IAR_ADDR, 0xE820_200C); // ICCIAR
        assert_eq!(GICC_EOIR_ADDR, 0xE820_2010); // ICCEOIR
    }

    /// ICDICFR0 starts at GICD+0xC00; there must be exactly 37 entries to
    /// cover all 587 interrupt sources at 2 bits each (37×16 = 592 ≥ 587).
    #[test]
    fn icdicfr_table_length() {
        assert_eq!(ICDICFR_INIT.len(), 37);
    }

    #[test]
    fn icdicfr_base_address() {
        assert_eq!(GICD_ICFGR0, GICD_BASE + 0xC00);
    }

    /// First and last ICDICFR entries from the Renesas intc.c table.
    #[test]
    fn icdicfr_spot_values() {
        assert_eq!(ICDICFR_INIT[0], 0xAAAAAAAA); // ICDICFR0 : IDs 15–0
        assert_eq!(ICDICFR_INIT[1], 0x00000055); // ICDICFR1 : IDs 19–16
        assert_eq!(ICDICFR_INIT[36], 0x00155555); // ICDICFR36: IDs 586–576
    }

    #[test]
    fn gicd_isenabler_base_address() {
        assert_eq!(GICD_ISENABLER0, GICD_BASE + 0x100);
    }

    #[test]
    fn gicd_ipriorityr_base_address() {
        assert_eq!(GICD_IPRIORITYR0, GICD_BASE + 0x400);
    }
}
