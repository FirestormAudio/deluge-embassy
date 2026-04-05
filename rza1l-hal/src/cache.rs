//! L1 and L2 cache initialisation for RZ/A1L (Cortex-A9 + PL310).
//!
//! Ports `R_CACHE_L1Init()` from `l1_cache_init.S` and `L2CacheInit()` from
//! `cache.c`.
//!
//! ## Ordering requirement
//! 1. [`mmu::init_and_enable`](crate::mmu::init_and_enable) must be called
//!    first so that the PL310 registers at `0x3FFF_F000` are accessible.
//! 2. Call [`l1_enable`] next — enables L1 I-cache, D-cache, branch
//!    prediction, and D-side prefetch.
//! 3. Call [`l2_init`] last — must come after [`l1_enable`] to avoid the L1
//!    flushing dirty data into a not-yet-valid L2.
//!
//! ## L2C (PL310) base address: 0x3FFF_F000
//! Register offsets follow the ARM PL310 TRM (r3p2):
//!
//! | Register          | Offset  |
//! |-------------------|---------|
//! | REG1_CONTROL      | 0x100   |
//! | REG2_INT_CLEAR    | 0x220   |
//! | REG7_INV_WAY      | 0x77C   |
//! | REG9_D_LOCKDOWN0  | 0x900   |
//! | REG9_I_LOCKDOWN0  | 0x904   |

use core::arch::asm;

// PL310 register addresses
const L2C_BASE: usize = 0x3FFF_F000;
const L2C_REG1_CONTROL: usize = L2C_BASE + 0x100;
const L2C_REG2_INT_CLR: usize = L2C_BASE + 0x220;
const L2C_REG7_INV_WAY: usize = L2C_BASE + 0x77C;
const L2C_REG9_D_LOCK0: usize = L2C_BASE + 0x900;
const L2C_REG9_I_LOCK0: usize = L2C_BASE + 0x904;

const L2C_8WAY: u32 = 0x0000_00FF; // bitmask for all 8 ways (REG7_INV_WAY)

// ---------------------------------------------------------------------------
// CP15 SCTLR / ACTLR bit constants (Cortex-A9 TRM §4.3.5, §4.3.6)
// ---------------------------------------------------------------------------

/// SCTLR bit 2: C — L1 data-cache enable.
const SCTLR_DCACHE: u32 = 1 << 2;
/// SCTLR bit 11: Z — branch prediction enable.
const SCTLR_ZBRANCH: u32 = 1 << 11;
/// SCTLR bit 12: I — L1 instruction-cache enable.
const SCTLR_ICACHE: u32 = 1 << 12;
/// ACTLR bit 2: enable L1 D-side prefetch (Cortex-A9 TRM §4.3.6).
const ACTLR_DPREFETCH: u32 = 1 << 2;

// ---------------------------------------------------------------------------
// PL310 L2C interrupt and enable constants
// ---------------------------------------------------------------------------

/// L2C_REG2_INT_CLEAR: write 1 to each of bits [8:0] to clear all 9 interrupt sources.
const L2C_INT_CLR_ALL: u32 = 0x0000_01FF;
/// L2C_REG1_CONTROL bit 0: enable L2 cache.
const L2C_ENABLE: u32 = 0x0000_0001;

// ---------------------------------------------------------------------------
// L1 cache
// ---------------------------------------------------------------------------

/// Enable Cortex-A9 L1 I-cache, D-cache, branch prediction, and D-side
/// prefetch.
///
/// Port of `R_CACHE_L1Init()` from `l1_cache_init.S`.
///
/// # Safety
/// Modifies CP15 `SCTLR` and `ACTLR`.  Must be called with the MMU already
/// enabled so that the cache attribute bits in the TTB are honoured.
pub unsafe fn l1_enable() {
    unsafe {
        log::debug!("cache: enabling L1 I+D, branch prediction, D-prefetch");
        // Read SCTLR and set I-cache (bit 12), D-cache (bit 2), Z/branch (bit 11).
        let mut sctlr: u32;
        asm!("mrc p15, 0, {0}, c1, c0, 0", out(reg) sctlr, options(nomem, nostack));
        sctlr |= SCTLR_ICACHE | SCTLR_ZBRANCH | SCTLR_DCACHE;
        // DSB: drain the write buffer so all prior stores reach physical OCRAM
        // before the cache starts intercepting loads (the ISB that follows
        // activates D-cache for the very next instruction).
        asm!("dsb", options(nostack));
        asm!("mcr p15, 0, {0}, c1, c0, 0", in(reg) sctlr, options(nomem, nostack));
        asm!("isb", options(nomem, nostack));
        log::trace!("cache: SCTLR = {:#010x}", sctlr);

        // Read ACTLR and set D-side prefetch (bit 2).
        let mut actlr: u32;
        asm!("mrc p15, 0, {0}, c1, c0, 1", out(reg) actlr, options(nomem, nostack));
        actlr |= ACTLR_DPREFETCH;
        asm!("mcr p15, 0, {0}, c1, c0, 1", in(reg) actlr, options(nomem, nostack));
        asm!("isb", options(nomem, nostack));
        log::debug!("cache: L1 enabled");
    }
}

// ---------------------------------------------------------------------------
// L2 cache (PL310)
// ---------------------------------------------------------------------------

/// Initialise and enable the PL310 L2 cache.
///
/// Port of `L2CacheInit()` from `cache.c`:
/// 1. Disable L2 cache.
/// 2. Flush all 8 ways (invalidate by way).
/// 3. Clear interrupt status.
/// 4. Unlock all D-cache and I-cache ways.
/// 5. Enable L2 cache.
///
/// All 8 D-cache ways are left unlocked.  DMA coherency is maintained by
/// always accessing DMA buffers through the non-cacheable OCRAM mirror alias
/// (mapped as `PARA_NORMAL_NOT_CACHE` in the MMU TTB), so the L2 never
/// allocates lines for DMA traffic.
///
/// # Safety
/// Writes to memory-mapped PL310 registers.  Must be called after
/// [`l1_enable`].
pub unsafe fn l2_init() {
    unsafe {
        log::debug!("cache: initialising L2 (PL310)");
        // 1. Disable L2 cache.
        wr32(L2C_REG1_CONTROL, 0x0000_0000);

        // 2. Flush all 8 ways — poll until hardware clears all bits.
        wr32(L2C_REG7_INV_WAY, L2C_8WAY);
        log::trace!("cache: L2 invalidating all 8 ways...");
        while rd32(L2C_REG7_INV_WAY) & L2C_8WAY != 0 {}
        log::trace!("cache: L2 all ways invalidated");

        // 3. Clear all interrupt sources in one write (bits [8:0]).
        wr32(L2C_REG2_INT_CLR, L2C_INT_CLR_ALL);

        // 4. Unlock both D-cache and I-cache ways (all 8 ways active).
        //    DMA coherency is handled at the MMU level: all DMA buffers are
        //    accessed exclusively through the OCRAM non-cacheable mirror alias
        //    (0x60000000–0x609FFFFF, PARA_NORMAL_NOT_CACHE), which prevents L2
        //    from allocating lines for DMA accesses regardless of lockdown.
        //    Only one AXI master (the Cortex-A9) is connected on RZ/A1L so only
        //    LOCKDOWN0 needs to be written (masters 1–7 are unused).
        wr32(L2C_REG9_D_LOCK0, 0x0000_0000);
        wr32(L2C_REG9_I_LOCK0, 0x0000_0000);

        // 5. Enable L2 cache.
        wr32(L2C_REG1_CONTROL, L2C_ENABLE);
    }
}

// ---------------------------------------------------------------------------
// L1 set/way operations (full-cache maintenance)
// ---------------------------------------------------------------------------
//
// Port of `L1_D_CacheOperationAsm` from `l1_cache_operation.s`.
//
// Iterates all cache levels reported by CLIDR that contain a D-cache or
// unified cache and issues one of:
//   OP=0  DCISW  — Invalidate data cache line by set/way
//   OP=1  DCCSW  — Clean data cache line by set/way
//   OP=2  DCCISW — Clean and Invalidate data cache line by set/way
//
// The const generic OP is resolved at compile time so only one instruction
// variant ends up in each monomorphised version.

unsafe fn l1_d_op_by_setway<const OP: u8>() {
    unsafe {
        let clidr: u32;
        asm!("mrc p15, 1, {0}, c0, c0, 1", out(reg) clidr, options(nomem, nostack));

        // CLIDR[26:24] = LoC (level of coherency).
        // The inner loop uses ctr_level = level << 1.
        let loc: u32 = (clidr >> 23) & 0xE; // = LoC * 2
        if loc == 0 {
            asm!("dsb", options(nostack));
            return;
        }

        let mut ctr_level: u32 = 0;
        while ctr_level < loc {
            // CLIDR has 3-bit cache-type fields packed at level*3.
            let shift = ctr_level + (ctr_level >> 1); // = level * 3
            let ctype = (clidr >> shift) & 0x7;

            if ctype >= 2 {
                // D-cache or unified cache present — select via CSSELR.
                asm!("mcr p15, 2, {0}, c0, c0, 0", in(reg) ctr_level, options(nomem, nostack));
                asm!("isb", options(nomem, nostack));

                // Read CCSIDR to obtain set/way geometry.
                let ccsidr: u32;
                asm!("mrc p15, 1, {0}, c0, c0, 0", out(reg) ccsidr, options(nomem, nostack));

                // Bit-field extraction (matches vendor assembly):
                //   line_size = log2(line_size_bytes)  = CCSIDR[2:0] + 4
                //   max_way   = CCSIDR[12:3]  (number of ways minus 1)
                //   max_set   = CCSIDR[27:13] (number of sets minus 1)
                //   way_shift = CLZ(max_way)  (bit position of way field in set/way reg)
                let line_size: u32 = (ccsidr & 0x7) + 4;
                let max_way: u32 = (ccsidr >> 3) & 0x3FF;
                let max_set: u32 = (ccsidr >> 13) & 0x7FFF;
                let way_shift: u32 = max_way.leading_zeros();

                let mut way = max_way;
                loop {
                    let mut set = max_set;
                    loop {
                        let sw: u32 = (way << way_shift) | (set << line_size) | ctr_level;
                        if OP == 0 {
                            // DCISW: invalidate by set/way
                            asm!("mcr p15, 0, {0}, c7, c6, 2",
                                 in(reg) sw, options(nomem, nostack));
                        } else if OP == 1 {
                            // DCCSW: clean by set/way
                            asm!("mcr p15, 0, {0}, c7, c10, 2",
                                 in(reg) sw, options(nomem, nostack));
                        } else {
                            // DCCISW: clean and invalidate by set/way
                            asm!("mcr p15, 0, {0}, c7, c14, 2",
                                 in(reg) sw, options(nomem, nostack));
                        }
                        if set == 0 {
                            break;
                        }
                        set -= 1;
                    }
                    if way == 0 {
                        break;
                    }
                    way -= 1;
                }
            }
            ctr_level += 2;
        }
        asm!("dsb", options(nostack));
    }
}

/// Invalidate the entire L1 D-cache by iterating all sets and ways (DCISW).
///
/// Port of `L1_D_CacheOperationAsm(0)` from `l1_cache_operation.s`.
///
/// **Warning:** Dirty lines are **discarded without writeback**.  Only safe
/// at startup (before caches are enabled) or in carefully sequenced shutdown
/// code.  For normal maintenance use [`l1_d_clean_inv_all`].
///
/// # Safety
/// Modifies CP15 registers CSSELR and issues DCISW.
pub unsafe fn l1_d_invalidate_all() {
    unsafe { l1_d_op_by_setway::<0>() }
}

/// Clean (write-back) the entire L1 D-cache by iterating all sets and ways
/// (DCCSW).
///
/// Port of `L1_D_CacheOperationAsm(1)` from `l1_cache_operation.s`.
///
/// # Safety
/// Modifies CP15 CSSELR and issues DCCSW for every line.
pub unsafe fn l1_d_clean_all() {
    unsafe { l1_d_op_by_setway::<1>() }
}

/// Clean and invalidate the entire L1 D-cache by iterating all sets and ways
/// (DCCISW).
///
/// Port of `L1_D_CacheOperationAsm(2)` from `l1_cache_operation.s`.
///
/// # Safety
/// Modifies CP15 CSSELR and issues DCCISW for every line.
pub unsafe fn l1_d_clean_inv_all() {
    unsafe { l1_d_op_by_setway::<2>() }
}

/// Invalidate all L1 I-cache lines (ICIALLU).
///
/// Port of `L1_I_CacheFlushAllAsm` from `l1_cache_operation.s`.
///
/// # Safety
/// Issues ICIALLU + DSB + ISB.
pub unsafe fn l1_i_invalidate_all() {
    unsafe {
        asm!("mcr p15, 0, {0}, c7, c5, 0", in(reg) 0u32, options(nomem, nostack)); // ICIALLU
        asm!("dsb", options(nostack));
        asm!("isb", options(nomem, nostack));
    }
}

// ---------------------------------------------------------------------------
// DMA cache maintenance (range operations by virtual address)
// ---------------------------------------------------------------------------
//
// Port of `v7_dma_inv_range` and `v7_dma_flush_range` from
// `deluge/drivers/cache/invalidate.S` (Linux-derived ARMv7 DMA helpers).
//
// On the RZA1L the VA=PA flat mapping is always in effect, so "virtual
// address" and "physical address" are identical.
//
// CP15 instructions used:
//   DCIMVAC  (c7, c6, 1)  — invalidate D-cache line by MVA to PoC
//   DCCMVAC  (c7, c10, 1) — clean D-cache line by MVA to PoC
//   DCCIMVAC (c7, c14, 1) — clean and invalidate D-cache line by MVA to PoC
//
// Cortex-A9 L1 D-cache line size is fixed at 32 bytes.

/// Cortex-A9 L1 D-cache line size in bytes (fixed for this SoC).
const L1_LINE_BYTES: usize = 32;

/// Invalidate D-cache lines that cover the address range `[start, end)`.
///
/// Translates to one DCIMVAC per cache line.  Partial lines at the start and
/// end of the range are **clean-and-invalidated** (DCCIMVAC) rather than
/// plain-invalidated so that dirty data adjacent to the requested region is
/// not silently discarded — this matches the behaviour of the Linux
/// `v7_dma_inv_range` helper.
///
/// **Use case:** before a DMA *read* (device → memory).  Ensures the CPU
/// will not serve a stale value from its cache after the DMA engine has
/// written new data to DRAM.
///
/// # Safety
/// Issuing DCIMVAC on a dirty line that spans data outside `[start, end)`
/// would discard that data without writeback; the boundary DCCIMVAC prevents
/// this.  Still, the caller must ensure no other thread is concurrently
/// writing to the same cache lines.
pub unsafe fn dma_inv_range(start: usize, end: usize) {
    unsafe {
        const MASK: usize = L1_LINE_BYTES - 1;

        let mut first = start & !MASK; // round start DOWN to line boundary
        let last = end & !MASK; // round end DOWN to line boundary

        // If start is unaligned, clean+inv the first partial line so that any
        // dirty data before *start* within the line is preserved.
        if start & MASK != 0 {
            asm!("mcr p15, 0, {0}, c7, c14, 1",
                 in(reg) first as u32, options(nomem, nostack)); // DCCIMVAC
        }

        // If end is unaligned, clean+inv the last partial line so that dirty
        // data after *end* within the line is preserved.  This line is NOT
        // re-invalidated by the main loop (the loop stops at < last).
        if end & MASK != 0 {
            asm!("mcr p15, 0, {0}, c7, c14, 1",
                 in(reg) last as u32, options(nomem, nostack)); // DCCIMVAC
        }

        // Invalidate all full lines in [first, last).
        while first < last {
            asm!("mcr p15, 0, {0}, c7, c6, 1",
                 in(reg) first as u32, options(nomem, nostack)); // DCIMVAC
            first += L1_LINE_BYTES;
        }

        asm!("dsb", options(nostack));
    }
}

/// Clean (write-back) D-cache lines that cover the address range
/// `[start, end)` (DCCMVAC per line).
///
/// **Use case:** before a DMA *write* (memory → device).  Ensures that dirty
/// data written by the CPU is flushed to DRAM before the DMA engine reads it.
///
/// # Safety
/// Issues DCCMVAC per cache line + DSB.  The range must not be in flight from
/// another DMA simultaneously.
pub unsafe fn dma_clean_range(start: usize, end: usize) {
    unsafe {
        let mut addr = start & !(L1_LINE_BYTES - 1);
        while addr < end {
            asm!("mcr p15, 0, {0}, c7, c10, 1",
                 in(reg) addr as u32, options(nomem, nostack)); // DCCMVAC
            addr += L1_LINE_BYTES;
        }
        asm!("dsb", options(nostack));
    }
}

/// Clean and invalidate D-cache lines that cover the address range
/// `[start, end)` (DCCIMVAC per line).
///
/// Port of `v7_dma_flush_range` from `deluge/drivers/cache/invalidate.S`.
///
/// **Use case:** bidirectional DMA buffers, or before giving a buffer to a
/// device that will both read and write it.
///
/// # Safety
/// Issues DCCIMVAC per cache line + DSB.
pub unsafe fn dma_clean_inv_range(start: usize, end: usize) {
    unsafe {
        let mut addr = start & !(L1_LINE_BYTES - 1);
        while addr < end {
            asm!("mcr p15, 0, {0}, c7, c14, 1",
                 in(reg) addr as u32, options(nomem, nostack)); // DCCIMVAC
            addr += L1_LINE_BYTES;
        }
        asm!("dsb", options(nostack));
    }
}

// ---------------------------------------------------------------------------
// L2 cache (PL310) additional operations
// ---------------------------------------------------------------------------

const L2C_REG7_CACHE_SYNC: usize = L2C_BASE + 0x730;

/// Disable the PL310 L2 cache.
///
/// Port of `L2CacheDisable()` from `cache.c`.
///
/// **Warning:** Ensure L1 D-cache is cleaned and invalidated
/// ([`l1_d_clean_inv_all`]) before disabling L2 to prevent loss of dirty data
/// that L2 may be holding.
///
/// # Safety
/// Writes to `REG1_CONTROL`.
pub unsafe fn l2_disable() {
    unsafe {
        wr32(L2C_REG1_CONTROL, 0x0000_0000);
    }
}

/// Invalidate all 8 ways of the PL310 L2 cache (poll until complete).
///
/// Port of `L2CacheFlushAll()` from `cache.c`.
///
/// # Safety
/// Writes to `REG7_INV_WAY` and polls it.  L2 must be disabled first to avoid
/// cache coherency hazards with the CPU.
pub unsafe fn l2_invalidate_all() {
    unsafe {
        wr32(L2C_REG7_INV_WAY, L2C_8WAY);
        while rd32(L2C_REG7_INV_WAY) & L2C_8WAY != 0 {}
        // Issue a cache sync to ensure the operation is complete before
        // subsequent accesses.
        wr32(L2C_REG7_CACHE_SYNC, 0);
    }
}

#[inline(always)]
unsafe fn wr32(addr: usize, val: u32) {
    unsafe {
        core::ptr::write_volatile(addr as *mut u32, val);
    }
}

#[inline(always)]
unsafe fn rd32(addr: usize) -> u32 {
    unsafe { core::ptr::read_volatile(addr as *const u32) }
}
