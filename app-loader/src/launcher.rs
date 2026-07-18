//! Cache flush and branch-to-application launcher.
//!
//! Before transferring control to a freshly loaded ELF image the bootloader
//! must:
//!   1. Write back and invalidate the entire L1 D-cache so the loaded bytes
//!      reach physical RAM.
//!   2. Write back all L2 (PL310) dirty lines.
//!   3. Invalidate the entire L1 I-cache so the CPU fetches from the newly
//!      written physical RAM rather than stale cache lines.
//!   4. Issue a DSB + ISB to serialise the pipeline.
//!   5. Branch to the application entry point.
//!
//! ## Trampoline
//!
//! Apps normally target SRAM (`0x20020000+`), the same region occupied by the
//! running bootloader.  Writing PT_LOAD segments there directly would corrupt
//! the bootloader in flight.  The solution is a two-phase approach:
//!
//! 1. [`crate::elf::load_from_sd`] stages SRAM-targeting segments in SDRAM
//!    (`0x0FD20000+`) and returns per-segment [`SramSegDesc`] descriptors.
//!
//! 2. [`launch_via_trampoline`] copies a small PIC trampoline blob +
//!    the descriptor table into the data-retention RAM region
//!    (`0x20000000–0x2001FFFF`), which the first-stage bootloader never
//!    writes or executes from.  After flushing all caches it jumps into
//!    that trampoline, which moves each staged segment from SDRAM to its
//!    final SRAM address and then branches to `e_entry`.
//!
//! Because the trampoline runs from retention RAM it is immune to the SRAM
//! overwrites it performs.  It also never uses the stack (the current stack
//! frame in SRAM would be overwritten mid-copy).

use core::arch::{asm, global_asm};

use rza1l_hal::cache;

use crate::elf::SramSegDesc;

// ---------------------------------------------------------------------------
// Retention-RAM layout
// ---------------------------------------------------------------------------

/// Base address of the data-retention RAM window.  The trampoline code is
/// copied here at run time.
const RETRAM_CODE: usize = 0x2000_0000;

/// Offset within retention RAM where the `SramSegDesc` table is placed.
/// Chosen to give the trampoline code plenty of room (256 bytes).
const RETRAM_TABLE_OFFSET: usize = 0x100;

// ---------------------------------------------------------------------------
// Trampoline blob (global_asm)
// ---------------------------------------------------------------------------
//
// This is a position-independent ARM32 routine.  It is linked into the
// bootloader's SRAM image, but we *copy* it into retention RAM at
// `RETRAM_CODE` before jumping there.
//
// Calling convention (matching `extern "C"` / AAPCS):
//   R0 = *const SramSegDesc  (table in retention RAM)
//   R1 = count: u32
//   R2 = entry: u32
//
// The function never returns and never touches the stack, because SP still
// points into SRAM which is being overwritten.
global_asm!(
    r#"
    .section .text._trampoline, "ax"
    .code 32
    .global _trampoline_start
    .global _trampoline_end
_trampoline_start:
    @ r0 = descriptor table ptr, r1 = count, r2 = entry point
    @
    @ Disable the MMU and L1 caches before copying.  The RZ/A1L's caches do not
    @ reliably honour cache-maintenance operations (DCCMVAC/DCCMVAU and the
    @ ARM-recommended L1 flush logic) -- see the Deluge firmware's chainload.S,
    @ which copies a new image into RAM the same way.  A cached copy therefore
    @ leaves the upper part of a large image stranded in cache and never written
    @ to physical RAM.  With caches off every store goes straight to memory.
    @ The page table is a flat VA=PA map, so turning the MMU off does not move
    @ this code (it runs from retention RAM at its physical address) and the
    @ copy addresses stay valid.
    cpsid   if                       @ mask IRQ/FIQ for the handoff
    mrc     p15, 0, r9, c1, c0, 0    @ read SCTLR
    bic     r9, r9, #(1 << 12)       @ I:  disable L1 instruction cache
    bic     r9, r9, #(1 <<  2)       @ C:  disable L1 data cache
    bic     r9, r9, #(1 <<  0)       @ M:  disable MMU
    mcr     p15, 0, r9, c1, c0, 0    @ write SCTLR
    dsb
    isb
.Lseg_loop:
    cmp     r1, #0
    beq     .Ldone
    ldr     r3, [r0, #0]      @ src  (SDRAM staging address)
    ldr     r4, [r0, #4]      @ dst  (final SRAM address)
    ldr     r5, [r0, #8]      @ filesz
    ldr     r6, [r0, #12]     @ zero_extra
.Lcopy_loop:
    cmp     r5, #0
    beq     .Lzero_loop
    ldrb    r7, [r3], #1
    strb    r7, [r4], #1
    sub     r5, r5, #1
    b       .Lcopy_loop
.Lzero_loop:
    cmp     r6, #0
    beq     .Lnext_seg
    mov     r7, #0
    strb    r7, [r4], #1
    sub     r6, r6, #1
    b       .Lzero_loop
.Lnext_seg:
    add     r0, r0,  #16      @ advance to next descriptor
    sub     r1, r1,  #1
    b       .Lseg_loop
.Ldone:
    @ Stores went straight to physical RAM (caches off).  Invalidate the I-cache
    @ and branch predictor and barrier before branching so the app entry fetches
    @ the freshly written code, then jump.  The app re-enables its own MMU/cache.
    mov     r7, #0
    mcr     p15, 0, r7, c7, c5, 0    @ ICIALLU: invalidate entire I-cache
    mcr     p15, 0, r7, c7, c5, 6    @ BPIALL:  invalidate branch predictor
    dsb
    isb
    bx      r2                @ jump to app entry point
_trampoline_end:
    "#
);

unsafe extern "C" {
    /// First byte of the trampoline blob in the linked image.
    static _trampoline_start: u8;
    /// First byte past the end of the trampoline blob.
    static _trampoline_end: u8;
}

// ---------------------------------------------------------------------------
// Cache-flush helpers (shared between both launch paths)
// ---------------------------------------------------------------------------

#[inline]
unsafe fn flush_all_caches() {
    unsafe {
        cache::l1_d_clean_inv_all();

        const L2C_BASE: usize = 0x3FFF_F000;
        const L2C_CTRL: *mut u32 = (L2C_BASE + 0x100) as *mut u32;
        const L2C_CLEAN_INV_WAY: *mut u32 = (L2C_BASE + 0x7FC) as *mut u32;
        const L2C_CACHE_SYNC: *mut u32 = (L2C_BASE + 0x730) as *mut u32;
        const L2C_8WAY: u32 = 0xFF;

        L2C_CLEAN_INV_WAY.write_volatile(L2C_8WAY);
        while L2C_CLEAN_INV_WAY.read_volatile() & L2C_8WAY != 0 {}
        // Drain the PL310 eviction write-buffer before disabling L2.
        L2C_CACHE_SYNC.write_volatile(0);
        L2C_CTRL.write_volatile(0);

        cache::l1_i_invalidate_all();

        asm!("dsb", options(nostack));
        asm!("isb", options(nomem, nostack));
    }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/// Flush all caches and branch directly to `entry`.
///
/// Use this when the app has no SRAM-targeting segments (i.e.
/// [`crate::elf::LoadResult::n_sram`] is zero).
///
/// # Safety
/// `entry` must be the address of a valid ARM-state function.
pub unsafe fn launch(entry: u32) -> ! {
    unsafe {
        flush_all_caches();
        let f: unsafe extern "C" fn() -> ! = core::mem::transmute(entry as usize);
        f()
    }
}

/// Stage-then-launch: copy the trampoline + descriptors to retention RAM,
/// flush all caches, then branch into the trampoline.
///
/// The trampoline copies each `SramSegDesc` from SDRAM staging to its final
/// SRAM address, then branches to `entry`.
///
/// # Safety
/// - `descs[..n_sram]` must have been filled by [`crate::elf::load_from_sd`].
/// - Nothing in the retention RAM region (`0x20000000–0x2001FFFF`) must be
///   live; the first-stage bootloader guarantees this.
/// - `entry` must be the address of a valid ARM-state function.
pub unsafe fn launch_via_trampoline(descs: &[SramSegDesc], entry: u32) -> ! {
    unsafe {
        let code_start = core::ptr::addr_of!(_trampoline_start);
        let code_end = core::ptr::addr_of!(_trampoline_end);
        let code_len = code_end as usize - code_start as usize;

        // The descriptor table sits at RETRAM_TABLE_OFFSET; the code must fit
        // below it or the copy below would clobber the table we're about to write.
        debug_assert!(
            code_len <= RETRAM_TABLE_OFFSET,
            "trampoline code overruns the descriptor table offset"
        );

        // Copy trampoline code into retention RAM.
        core::ptr::copy_nonoverlapping(code_start, RETRAM_CODE as *mut u8, code_len);

        // Copy descriptor table right after the code (at RETRAM_TABLE_OFFSET).
        let table_dst = (RETRAM_CODE + RETRAM_TABLE_OFFSET) as *mut SramSegDesc;
        core::ptr::copy_nonoverlapping(descs.as_ptr(), table_dst, descs.len());

        flush_all_caches();

        // Jump into the trampoline in retention RAM.
        // R0 = table, R1 = count, R2 = entry  (AAPCS).
        let trampoline: unsafe extern "C" fn(*const SramSegDesc, u32, u32) -> ! =
            core::mem::transmute(RETRAM_CODE);
        trampoline(table_dst as *const SramSegDesc, descs.len() as u32, entry)
    }
}
