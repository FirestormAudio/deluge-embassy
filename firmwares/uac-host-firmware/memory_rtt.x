/* Board-specific memory layout for the Synthstrom Deluge (RZ/A1L) — RTT enabled.
 * This file is INCLUDE'd by rza1l_rtt.x (from the rza1l-hal crate) when the
 * firmware is built with --features rtt (the default).
 * Edit here to change RAM/SDRAM sizes, RTT placement, or stack sizes. */

MEMORY {
    /* 2.875 MB on-chip SRAM (pages 0–4, non-retention banks).
       0x20000000–0x2001FFFF is data-retention RAM; the bootloader refuses to
       copy a firmware image whose CODE_START falls below 0x20020000, so the
       firmware image must start at or above that address. */
    RAM (rwx) : ORIGIN = 0x20020000, LENGTH = 0x002E0000

    /* 64 KB reserved at the top of SRAM for the RTT ring buffer.
       Placed just below the stacks (stacks occupy the top 160 KB).
       Cached alias: 0x202B0000–0x202BFFFF
       The uncached mirror is the same physical page + 0x40000000. */
    RTT_RAM   (rw) : ORIGIN = 0x202B0000, LENGTH = 0x00010000  /* 64 KB cached   */
    NCACHE_RTT_RAM (rw): ORIGIN = 0x602B0000, LENGTH = 0x00010000  /* 64 KB uncached */

    /* 64 MB external SDRAM (CS3) */
    SDRAM (rwx) : ORIGIN = 0x0C000000, LENGTH = 0x04000000
}

/* Stack sizes */
PROGRAM_STACK_SIZE = 0x8000;   /* 32 KB — application / SYS mode */
IRQ_STACK_SIZE     = 0x2000;   /*  8 KB */
FIQ_STACK_SIZE     = 0x2000;   /*  8 KB */
SVC_STACK_SIZE     = 0x2000;   /*  8 KB */
ABT_STACK_SIZE     = 0x2000;   /*  8 KB */

/* Top byte address of on-chip SRAM (used to anchor stack sections) */
INTERNAL_RAM_END = 0x20300000;
