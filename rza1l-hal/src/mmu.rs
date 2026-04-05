//! MMU initialisation and control for RZ/A1L (Cortex-A9).
//!
//! Builds a flat (VA = PA) 1 MB-section Level-1 Translation Table that covers
//! the full 4 GB address space and matches the attribute zones from the
//! Renesas C BSP (`ttb_init.S`), then enables the MMU.
//!
//! ## Address-space map
//!
//! | Area | Size   | PA range                          | Cache policy       |
//! |------|--------|-----------------------------------|--------------------|
//! |  0   | 128 MB | 0x00000000–0x07FFFFFF (CS0/CS1)   | Cached (NOR)       |
//! |  1   | 128 MB | 0x08000000–0x0FFFFFFF (CS2/CS3)   | Cached (SDRAM)     |
//! |  2   | 128 MB | 0x10000000–0x17FFFFFF (CS4/CS5)   | Strongly-ordered   |
//! |  3   | 128 MB | 0x18000000–0x1FFFFFFF (SPI)       | Cached (serial fl) |
//! |  4   |  10 MB | 0x20000000–0x209FFFFF (OCRAM)     | Cached (SRAM)      |
//! |  5   | 502 MB | 0x20A00000–0x3FFFFFFF (I/O area 1)| Strongly-ordered   |
//! |  6   | 128 MB | 0x40000000–0x47FFFFFF (CS0/CS1 m) | Non-cached         |
//! |  7   | 128 MB | 0x48000000–0x4FFFFFFF (CS2/CS3 m) | Non-cached         |
//! |  8   | 128 MB | 0x50000000–0x57FFFFFF (CS4/CS5 m) | Strongly-ordered   |
//! |  9   | 128 MB | 0x58000000–0x5FFFFFFF (SPI m)     | Non-cached         |
//! | 10   |  10 MB | 0x60000000–0x609FFFFF (OCRAM m)   | Non-cached         |
//! | 11   |2550 MB | 0x60A00000–0xFFFFFFFF (I/O area 2)| Strongly-ordered   |

use core::arch::asm;

// ---------------------------------------------------------------------------
// 1 MB section descriptor attribute bits [19:0]
// (bits [1:0] = 0b10 → section entry; AP=b11 full access; Domain=15)
// ---------------------------------------------------------------------------

/// Strongly-ordered memory (device I/O). TEX=000, C=0, B=0.
pub const PARA_STRONGLY_ORDERED: u32 = 0x0DE2;
/// Normal non-cached.  TEX=001, C=0, B=0 → inner/outer non-cacheable normal memory.
/// (The bit encoding: AP[2:0]=b011=full access, Domain=15, nG=1, S=1, TEX=001, C=0, B=0.)
pub const PARA_NORMAL_NOT_CACHE: u32 = 0x1DE2;
/// Normal write-back, write-allocate (fully cached). TEX=001, C=1, B=1.
pub const PARA_NORMAL_CACHE: u32 = 0x1DEE;

// ---------------------------------------------------------------------------
// Area table — (size_in_mb, attribute) pairs, low address → high address.
// ---------------------------------------------------------------------------
const AREAS: &[(u32, u32)] = &[
    (128, PARA_NORMAL_CACHE),      // area  0  CS0/CS1 NOR flash
    (128, PARA_NORMAL_CACHE),      // area  1  CS2/CS3 SDRAM
    (128, PARA_STRONGLY_ORDERED),  // area  2  CS4/CS5
    (128, PARA_NORMAL_CACHE),      // area  3  SPI / SPI2 serial flash
    (10, PARA_NORMAL_CACHE),       // area  4  internal SRAM (3 MB phys + guard)
    (502, PARA_STRONGLY_ORDERED),  // area  5  I/O area 1
    (128, PARA_NORMAL_NOT_CACHE),  // area  6  CS0/CS1 mirror
    (128, PARA_NORMAL_NOT_CACHE),  // area  7  CS2/CS3 mirror (SDRAM mirror)
    (128, PARA_STRONGLY_ORDERED),  // area  8  CS4/CS5 mirror
    (128, PARA_NORMAL_NOT_CACHE),  // area  9  SPI mirror
    (10, PARA_NORMAL_NOT_CACHE), // area 10  SRAM mirror (0x60020000 is first valid byte; 0x60000000-0x6001FFFF reserved)
    (2550, PARA_STRONGLY_ORDERED), // area 11  I/O area 2
];

// ---------------------------------------------------------------------------
// Translation table storage — must be 16 KB aligned (== its own size)
// ---------------------------------------------------------------------------

#[repr(C, align(16384))]
struct Ttb([u32; 4096]);

/// The Level-1 Translation Table.  Zero-initialised (BSS), filled by
/// [`init_and_enable`] before the MMU is turned on.
static mut TTB: Ttb = Ttb([0; 4096]);

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/// Build the flat VA=PA Translation Table, configure CP15 registers, and
/// enable the MMU.
///
/// Leaves caches off; call [`crate::cache::l1_enable`] and
/// [`crate::cache::l2_init`] afterwards.
///
/// # Safety
/// * Must be called exactly once after BSS zeroing and before caches are
///   enabled.
/// * IRQ and FIQ must be disabled.
/// * Modifies CP15 registers: SCTLR, TTBCR, TTBR0, DACR, TLBIALL.
pub unsafe fn init_and_enable() {
    unsafe {
        // SAFETY: TTB is static mut; we call this exactly once at startup before
        // any other core accesses it.
        let table_ptr = core::ptr::addr_of_mut!(TTB) as *mut u32;
        log::debug!(
            "mmu: building TTB ({} entries) at {:#010x}",
            4096usize,
            table_ptr as usize
        );
        // Descriptor[n] = (n_mb << 20) | attr  →  flat VA=PA mapping.
        let mut idx: u32 = 0;
        for &(size_mb, attr) in AREAS {
            for _ in 0..size_mb {
                let descriptor = (idx << 20) | attr;
                table_ptr.add(idx as usize).write_volatile(descriptor);
                idx += 1;
            }
        }
        // idx should equal 4096 here (all entries filled).
        log::trace!("mmu: TTB filled ({} entries)", idx);

        // TTBCR = 0: N=0, use TTBR0 for all translations.
        asm!("mcr p15, 0, {0}, c2, c0, 2", in(reg) 0u32, options(nomem, nostack));
        isb();

        // TTBR0: table base + RGN=0b01 (outer WB cached) + IRGN=0b01 (inner WB WA).
        // Bits[4:3] = RGN[1:0] = 0b01 (outer write-back, write-allocate).
        // Bit[6] = IRGN[0] (with IRGN[1] in bit[0] = 0) gives IRGN = 0b01 (inner WB-WA).
        let ttbr0 = (table_ptr as u32) | 0x08 | 0x40;
        asm!("mcr p15, 0, {0}, c2, c0, 0", in(reg) ttbr0, options(nomem, nostack));
        isb();
        log::trace!("mmu: TTBR0 = {:#010x}", ttbr0);

        // DACR: all 16 domains → Client access (b01 per pair) = 0x55555555.
        asm!("mcr p15, 0, {0}, c3, c0, 0", in(reg) 0x5555_5555u32, options(nomem, nostack));
        isb();

        // Invalidate TLBs before enabling the MMU.
        asm!("mcr p15, 0, {0}, c8, c7, 0", in(reg) 0u32, options(nomem, nostack)); // TLBIALL
        dsb();
        isb();

        // Enable MMU: set SCTLR.M (bit 0).
        // Caches (bits 2 and 12) are kept off here; enable them via cache module.
        let mut sctlr: u32;
        asm!("mrc p15, 0, {0}, c1, c0, 0", out(reg) sctlr, options(nomem, nostack));
        sctlr |= 1; // M bit
        asm!("mcr p15, 0, {0}, c1, c0, 0", in(reg) sctlr, options(nomem, nostack));
        isb();
        log::debug!("mmu: enabled (SCTLR.M set)");
    }
}

#[inline(always)]
unsafe fn isb() {
    unsafe {
        asm!("isb", options(nomem, nostack));
    }
}

#[inline(always)]
unsafe fn dsb() {
    unsafe {
        asm!("dsb", options(nomem, nostack));
    }
}

// ---------------------------------------------------------------------------
// Runtime MMU operations
// ---------------------------------------------------------------------------

/// Invalidate the entire TLB (TLBIALL + DSB + ISB).
///
/// Call after modifying any entry in the Translation Table, or after
/// [`update_section`], if you need to flush more than one VA at once.
///
/// # Safety
/// Must be called with IRQ and FIQ disabled to avoid a TLB miss on a
/// half-invalidated table.
pub unsafe fn invalidate_tlb_all() {
    unsafe {
        asm!("mcr p15, 0, {0}, c8, c7, 0", in(reg) 0u32, options(nomem, nostack)); // TLBIALL
        dsb();
        isb();
    }
}

/// Update the 1 MB section descriptor for the region containing physical
/// address `pa` and flush the corresponding TLB entry.
///
/// `attr` uses the same bit encoding as the `PARA_*` constants in this
/// module; bits `[19:0]` carry the attribute flags and bits `[31:20]` are
/// replaced with the aligned section base address derived from `pa`.
///
/// After this call, the new attributes are immediately visible to the CPU.
///
/// # Examples
///
/// ```no_run
/// // Remap the internal SRAM region as strongly-ordered (e.g. for DMA setup):
/// unsafe { rza1::mmu::update_section(0x2000_0000, rza1::mmu::PARA_STRONGLY_ORDERED) };
/// ```
///
/// # Safety
/// * [`init_and_enable`] must have been called first.
/// * Changing the cacheability of a region while the cache holds dirty lines
///   from it is **undefined behaviour** — flush the relevant lines with
///   [`crate::cache::l1_d_clean_inv_all`] (and disable L1/L2 for the region
///   if it will become strongly-ordered) before calling this.
/// * IRQ and FIQ must be disabled across this call.
pub unsafe fn update_section(pa: u32, attr: u32) {
    unsafe {
        let idx = (pa >> 20) as usize;
        let descriptor = (pa & 0xFFF0_0000) | (attr & 0x000F_FFFF);
        // SAFETY: TTB is a flat 4096-entry table; idx is always in [0, 4095].
        let entry_ptr = (core::ptr::addr_of_mut!(TTB) as *mut u32).add(idx);
        entry_ptr.write_volatile(descriptor);
        dsb();
        // TLBIMVA: flush the TLB entry for this VA (= PA in the flat map).
        // ASID field [7:0] = 0 (we use global mappings, nG=0 in all descriptors).
        asm!("mcr p15, 0, {0}, c8, c7, 1", in(reg) pa, options(nomem, nostack)); // TLBIMVA
        dsb();
        isb();
    }
}
