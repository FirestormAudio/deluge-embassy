use core::arch::global_asm;

global_asm!(
    r#"
    /*
     * Vector table — must be section .vector_table so the linker places it
     * at 0x20020000 (first bytes of the binary loaded into SRAM).
     *
     * IMPORTANT: ORIGIN(RAM) must be >= 0x20020000. The bootloader (spibsc_init2.c)
     * reads CODE_START from flash+0x20 and skips the entire copy if the destination
     * is below 0x20020000. 0x20000000–0x2001FFFF is data-retention RAM that the
     * bootloader explicitly excludes from the valid firmware destination range.
     *
     * Layout after the 8 vector entries (offset 0x20 onward):
     *   code_start   .word _start            — bootloader: copy-from address
     *   code_end     .word end               — bootloader: end of image
     *   code_execute .word _start            — bootloader: jump target
     *                .asciz "..."            — bootloader validation signature
     *
     * The literal pool for the LDR pc,=label instructions is placed by the
     * assembler via .ltorg, after the metadata so it does not land between
     * the vector entries and the bootloader words.
     */
    .section .vector_table, "ax"
    .code 32
    .global _start
_start:
    ldr pc, =_reset_handler       /* 0x00  Reset                   */
    ldr pc, =_undef_handler       /* 0x04  Undefined instruction  */
    ldr pc, =_svc_handler         /* 0x08  Supervisor call        */
    ldr pc, =_prefetch_handler    /* 0x0C  Prefetch abort         */
    ldr pc, =_abort_handler       /* 0x10  Data abort             */
    ldr pc, =_reserved_handler    /* 0x14  Reserved               */
    ldr pc, =_irq_handler         /* 0x18  IRQ                    */
    ldr pc, =_fiq_handler         /* 0x1C  FIQ                    */

    /* Bootloader metadata — fixed offsets from _start */
    .word _start                  /* 0x20  code_start             */
    .word end                     /* 0x24  code_end               */
    .word _start                  /* 0x28  code_execute           */
    .asciz ".BootLoad_ValidProgramTest."   /* 0x2C  signature      */
    .align 4

    /* Force literal pool here so it follows the metadata, not before it */
    .ltorg

    /*
     * Reset handler — minimal startup for first-milestone bare-metal boot.
     *
     * Steps:
     *   1. Multi-core guard (only core 0 proceeds)
     *   2. Disable I-cache, D-cache, MMU (clean slate — skip full MMU init
     *      for now; we run entirely from internal SRAM)
     *   3. Enable NEON/VFP coprocessor access
     *   4. Set up per-mode stacks
     *   5. Zero BSS
     *   6. Branch to rust_main
     */
    .section .text._reset_handler, "ax"
    .code 32
    .global _reset_handler
_reset_handler:
    /* Mask IRQ, FIQ and async aborts immediately — old firmware may have
     * left CPSR with interrupts enabled. With VBAR=0 any stray exception
     * would branch into SPI flash (the old bootloader at 0x00000000). */
    cpsid aif

    /* Configure the heartbeat LED pin as a plain GPIO immediately so any
     * early exception can signal a visible fault pattern without relying on
     * Rust, the allocator, RTT, or later board init. */
    bl _boot_led_setup
    bl _boot_led_write_low

    /*
     * 1a. Disable MMU, D-cache, I-cache — leave HIVEC alone for now.
     *
     * Do NOT clear HIVEC here. HIVEC must stay set (ROM vectors) until VBAR
     * is written, so any exception in this window still reaches the ROM
     * handlers rather than branching to address 0.
     */
    mrc p15, 0, r0, c1, c0, 0    /* Read SCTLR                   */
    bic r0, r0, #(1 << 12)       /* I-cache off                  */
    bic r0, r0, #(1 << 2)        /* D-cache off                  */
    bic r0, r0, #(1 << 0)        /* MMU off                      */
    mcr p15, 0, r0, c1, c0, 0
    isb
    dsb

    /*
     * 1b. Invalidate TLBs, I-cache, and D-cache.
     *
     * ARM mandates these invalidations before re-enabling caches/MMU because
     * the cache and TLB state is UNDEFINED after reset (or after a soft
     * reset from previous firmware).  Stale "valid" tags would cause the
     * CPU to return garbage on the first cached load.
     *
     * Mirrors R_CACHE_L1Init / reset_handler.S from the Renesas C BSP.
     */
    /* Invalidate entire Unified/Data TLB */
    mov  r0, #0
    mcr  p15, 0, r0, c8, c7, 0   /* TLBIALL                      */

    /* Invalidate I-cache (also flushes branch target cache) */
    mov  r0, #0
    mcr  p15, 0, r0, c7, c5, 0   /* ICIALLU                      */
    isb

    /*
     * Invalidate all D-cache levels by iterating every Set and Way.
     *
     * Faithfully ported from reset_handler.S (both Deluge and Azure RTOS
     * BSPs are identical here — Renesas reference implementation).
     *
     * Register usage (mirrors original):
     *   r0  = CLIDR
     *   r1  = CCSIDR
     *   r2  = line-length shift
     *   r3  = number of cache levels << 1 (from CLIDR LoC field)
     *   r4  = max way index (CCSIDR[12:3])
     *   r5  = CLZ(max_way) — way bit-position in DCISW word
     *   r7  = current set counter (outer loop, counts down to 0)
     *   r9  = current way counter (inner loop, counts down to 0)
     *   r10 = current cache level << 1
     *   r11 = DCISW argument
     *
     * Loop structure (outer = set, inner = way) matches the original exactly.
     * r7 must NOT be inside the way loop or it resets on every way iteration.
     */
    mrc  p15, 1, r0, c0, c0, 1   /* Read CLIDR                   */
    ands r3, r0, #0x07000000      /* Extract LoC field            */
    mov  r3, r3, lsr #23          /* Total cache levels << 1      */
    beq  .Ldcache_inval_done
    mov  r10, #0                  /* Start at level 0             */
.Ldcache_level_loop:
    add  r2, r10, r10, lsr #1     /* r2 = 3 × level               */
    mov  r1, r0, lsr r2           /* Shift CLIDR to this level    */
    and  r1, r1, #7               /* Isolate 3-bit cache type     */
    cmp  r1, #2
    blt  .Ldcache_skip            /* No D-cache at this level     */
    mcr  p15, 2, r10, c0, c0, 0  /* CSSELR = this level          */
    isb                           /* Sync before reading CCSIDR   */
    mrc  p15, 1, r1, c0, c0, 0   /* Read CCSIDR                  */
    and  r2, r1, #7               /* Line-length field            */
    add  r2, r2, #4               /* +4 → log2(bytes per line)    */
    ldr  r4, =0x3FF
    ands r4, r4, r1, lsr #3       /* Max way  = CCSIDR[12:3]      */
    clz  r5, r4                   /* Way bit-position in DCISW    */
    ldr  r7, =0x7FFF
    ands r7, r7, r1, lsr #13      /* Max set  = CCSIDR[27:13]     */
.Ldcache_set_loop:                /* Outer loop: iterate sets     */
    mov  r9, r4                   /* Reset way counter each set   */
.Ldcache_way_loop:                /* Inner loop: iterate ways     */
    orr  r11, r10, r9, lsl r5    /* Encode level | way           */
    orr  r11, r11, r7, lsl r2    /* Encode set                   */
    mcr  p15, 0, r11, c7, c6, 2  /* DCISW — invalidate by S/W    */
    subs r9, r9, #1
    bge  .Ldcache_way_loop        /* Next way                     */
    subs r7, r7, #1
    bge  .Ldcache_set_loop        /* Next set                     */
.Ldcache_skip:
    add  r10, r10, #2
    cmp  r3, r10
    bgt  .Ldcache_level_loop      /* Next cache level             */
.Ldcache_inval_done:
    dsb
    isb

    /* 1d. Set VBAR to our vector table BEFORE clearing HIVEC. */
    ldr r0, =_start
    mcr p15, 0, r0, c12, c0, 0   /* Write VBAR                   */
    isb

    /* 1e. Now safe to clear HIVEC — exceptions will use VBAR = _start.
     *     Also clear TE (bit 30, Thumb Exceptions) so exception vectors
     *     execute in ARM state.  Old firmware (e.g. Deluge) runs in Thumb
     *     mode and sets TE=1; our vector table uses ARM `ldr pc, [pc, #n]`
     *     entries which decode as undefined Thumb-2 when TE=1. */
    mrc p15, 0, r0, c1, c0, 0
    bic r0, r0, #(1 << 13)       /* Clear HIVEC (use low vectors) */
    bic r0, r0, #(1 << 30)       /* Clear TE (ARM exception entry) */
    mcr p15, 0, r0, c1, c0, 0
    isb

    /* 2. Multi-core guard: put secondary cores to sleep.
     * CPU0 must NOT execute WFI here or cold boot can stall forever waiting
     * for an interrupt before the firmware has enabled its own handlers. */
    mrc p15, 0, r0, c0, c0, 5    /* Read MPIDR                   */
    ands r0, r0, #0x3             /* Aff0 = CPU ID in bits[1:0] (Cortex-A9 MPCore) */
    beq .Lprimary_core            /* CPU0 continues immediately   */
.Lsecondary_core_sleep:
    wfine                         /* Secondary core: wait for int */
    b .Lsecondary_core_sleep      /* Never continue on non-zero cores */
.Lprimary_core:

    /*
     * 3. Enable CPU writes to data-retention RAM (0x20000000–0x2001FFFF).
     *
     * CPG.SYSCR3 at 0xFCFE0408 — bit[3:0] enable the four retention RAM
     * banks. Reset default is 0x00 (all banks write-protected). Without
     * this, any CPU store to retention RAM causes a Data Abort.
     *
     * Safe to access now: MMU is disabled, so 0xFCFE0408 is a direct
     * physical access to the CPG peripheral (Strongly-Ordered).
     *
     * Dummy read after write is required by the hardware spec.
     */
    ldr r1, =0xFCFE0408           /* CPG.SYSCR3 address           */
    mov r0, #0x0F                 /* Enable all 4 banks           */
    strb r0, [r1]
    ldrb r0, [r1]                 /* Dummy read                   */

    /*
     * Enable NEON/VFP coprocessor access.
     *
     * CPACR[23:22]=CP11 and CPACR[21:20]=CP10 are set to 0b11 (full access
     * in privileged and unprivileged modes).  Then FPEXC.EN (bit 30) is set
     * to power on the VFP/NEON hardware.
     *
     * This does NOT require STB/CPG clock init — VFP is CPU-internal.
     * It must be done here (before Rust) because stb::init() in rust_main
     * may itself emit VFP instructions (the toolchain is free to use NEON
     * for memset etc. once features are re-enabled).
     */
    mrc p15, 0, r0, c1, c0, 2    /* Read CPACR                   */
    orr r0, r0, #(0xF << 20)     /* Enable CP10 and CP11         */
    mcr p15, 0, r0, c1, c0, 2    /* Write CPACR                  */
    isb
    mov r0, #0x40000000           /* FPEXC.EN = 1                 */
    vmsr FPEXC, r0

    /* 3. Set up per-mode stacks and clear stale banked registers.
     *
     * For each exception mode: set SP, clear LR (= 0, causes PABT at known
     * address rather than jumping to stale previous-firmware code), and set
     * SPSR = SYS-mode ARM CPSR (0x1DF) so any spurious exception return
     * restores SYS mode rather than whatever the previous firmware left.
     *
     * This guards against the case where a FIQ fires before our own interrupt
     * initialisation is complete: `subs pc, lr, #4` in the FIQ handler uses
     * both LR_fiq and SPSR_fiq.  If those contain stale values the return
     * branches into random memory and/or restores a wrong CPSR mode.
     *
     * Safe SPSR value: SYS mode (bits[4:0]=11111), ARM state (T=0),
     * IRQ disabled (I=1), async-abort disabled (A=1) = 0x000001DF.
     * (NMFI means F cannot be set; write is attempted anyway.)
     */
    ldr r0, =0x000001DF           /* safe SPSR value (SYS ARM I+A disabled) */
    mov r1, #0                    /* safe LR  value (PABT at 0 if misused)  */

    msr cpsr_c, #0xD2             /* IRQ mode, I+F masked         */
    ldr sp, =irq_stack_end
    mov lr, r1
    msr spsr_fsxc, r0

    msr cpsr_c, #0xD1             /* FIQ mode                     */
    ldr sp, =fiq_stack_end
    mov lr, r1
    msr spsr_fsxc, r0

    msr cpsr_c, #0xD3             /* SVC mode                     */
    ldr sp, =svc_stack_end
    mov lr, r1
    msr spsr_fsxc, r0

    msr cpsr_c, #0xD7             /* ABT mode                     */
    ldr sp, =abt_stack_end
    mov lr, r1
    msr spsr_fsxc, r0

    /* Note: UNDEF mode is intentionally NOT entered here.
     * Our UNDEF handler is a bare infinite loop with no stack.
     * probe-rs resets SPSR_und and LR_und to safe values before restart.
     * Entering UNDEF mode briefly to init it would create a window where
     * a nested FIQ could return to UNDEF mode unexpectedly. */

    msr cpsr_c, #0xDF             /* SYS mode (application)       */
    ldr sp, =program_stack_end

    /* 5. Zero BSS */
    ldr r0, =__bss_start__
    ldr r1, =__bss_end__
    mov r2, #0
.Lbss_loop:
    cmp r0, r1
    strlt r2, [r0], #4
    blt .Lbss_loop

    /* 6. Branch to Rust entry point (never returns) */
    bl main
    b .                           /* Safety: infinite loop        */

_boot_led_setup:
    /* P6_7 = heartbeat LED. Match gpio::set_as_output(6, 7):
     *   PMC6  @ 0xFCFE3418, bit7 = 0 (GPIO)
     *   PM6   @ 0xFCFE3318, bit7 = 0 (output)
     *   PIPC6 @ 0xFCFE7218, bit7 = 0 (software control) */
    ldr r0, =0xFCFE3418
    ldrh r1, [r0]
    bic r1, r1, #0x80
    strh r1, [r0]

    ldr r0, =0xFCFE3318
    ldrh r1, [r0]
    bic r1, r1, #0x80
    strh r1, [r0]

    ldr r0, =0xFCFE7218
    ldrh r1, [r0]
    bic r1, r1, #0x80
    strh r1, [r0]
    bx lr

_boot_led_write_high:
    ldr r0, =0xFCFE3018
    ldrh r1, [r0]
    orr r1, r1, #0x80
    strh r1, [r0]
    bx lr

_boot_led_write_low:
    ldr r0, =0xFCFE3018
    ldrh r1, [r0]
    bic r1, r1, #0x80
    strh r1, [r0]
    bx lr

_boot_delay:
    subs r0, r0, #1
    bne _boot_delay
    bx lr

_boot_fault_loop:
    /* r4 = number of short blinks in the repeating fault pattern */
    bl _boot_led_setup
.Lboot_fault_outer:
    mov r5, r4
.Lboot_fault_pulse:
    bl _boot_led_write_high
    ldr r0, =0x00800000
    bl _boot_delay
    bl _boot_led_write_low
    ldr r0, =0x00800000
    bl _boot_delay
    subs r5, r5, #1
    bne .Lboot_fault_pulse
    ldr r0, =0x02000000
    bl _boot_delay
    b .Lboot_fault_outer

    /* Default exception handlers — halt the core */
    .global _undef_handler
    .global _svc_handler
    .global _prefetch_handler
    .global _abort_handler
    .global _reserved_handler
    .global _fiq_handler
_undef_handler:
    mov r4, #1
    b _boot_fault_loop
_svc_handler:
    mov r4, #2
    b _boot_fault_loop
_prefetch_handler:
    mov r4, #3
    b _boot_fault_loop
_abort_handler:
    b    _abort_rtt              /* branch to cfg-selected implementation */
_reserved_handler:
    mov r4, #5
    b _boot_fault_loop
    /* FIQ handler: return from interrupt.
     * On FIQ entry LR_fiq = interrupted PC + 4; subtract 4 to re-run
     * the interrupted instruction.  SUBS restores CPSR from SPSR_fiq.
     * Uses only the FIQ-banked r8–r12/lr so no save/restore needed. */
_fiq_handler:       subs pc, lr, #4
"#
);

// RTT-enabled data-abort body: writes "DABT" to RTT channel 0 then halts.
#[cfg(feature = "rtt")]
global_asm!(
    r#"
_abort_rtt:
    mov  r4, #4
    /* rtt-target 0.6 RttControlBlock layout (32-bit ARM, usize = 4 bytes):
     *   _SEGGER_RTT + 0x1C  acUp[0].buffer  (*mut u8)
     *   _SEGGER_RTT + 0x24  acUp[0].write   (AtomicUsize / WrOff)
     */
    ldr  r0, =_SEGGER_RTT
    ldr  r1, [r0, #0x1C]          /* r1 = acUp[0].buffer          */
    ldr  r2, [r0, #0x24]          /* r2 = WrOff                   */
    ldr  r3, =0x54424144          /* little-endian "DABT"          */
    str  r3, [r1, r2]             /* buffer[WrOff] = "DABT"        */
    add  r2, r2, #4
    str  r2, [r0, #0x24]          /* WrOff += 4                   */
    b    _boot_fault_loop         /* Blink 4x repeating pattern   */
"#
);

// No-RTT data-abort body: halt immediately.
#[cfg(not(feature = "rtt"))]
global_asm!(
    r#"
_abort_rtt:
    mov  r4, #4
    b    _boot_fault_loop
"#
);

// ---------------------------------------------------------------------------
// IRQ handler — full GIC dispatch with ARM errata workarounds
// ---------------------------------------------------------------------------
//
// This is a faithful port of `irqfiq_handler.S` from the Renesas BSP,
// adapted to call the Rust `gic_dispatch(icciar)` function.
//
// ARM errata handled:
//   801120  — dummy ICCHPIR read before ICCIAR ensures correct data
//   733075  — spurious ID 0 or >=1022: read-modify-write ICDIPR0, re-read ICCIAR
//
// Register usage on entry (IRQ mode, IRQ/FIQ disabled):
//   LR_irq = interrupted PC + 4
//   SPSR_irq = CPSR of interrupted mode
//
// Stack layout built by this handler (IRQ stack → SYS stack):
//   IRQ stack: [SPSR] [LR_irq - 4]      (srsdb / manual push/pop)
//   SYS stack: r0-r3, r12               (caller-saved AAPCS)
//              alignment pad (0 or 4)
//              r0(=icciar), r1(=pad), r2, r3, r4, lr_sys   (around bl gic_dispatch)

global_asm!(
    r#"
    .equ GICC_IAR_ADDR,   0xE820200C
    .equ GICC_EOIR_ADDR,  0xE8202010
    .equ GICC_HPPIR_ADDR, 0xE8202018
    .equ GICD_IPR0_ADDR,  0xE8201400
    .equ SYS_MODE, 0x1F
    .equ IRQ_MODE, 0x12

    .section .text._irq_handler, "ax"
    .code 32
    .global _irq_handler
    .type _irq_handler, %function
_irq_handler:
    /* Adjust LR: on IRQ entry LR_irq = interrupted PC + 4; subtract 4
     * so MOVS PC,LR returns to the correct instruction. */
    sub     lr, lr, #4

    /* Push return address and SPSR onto the IRQ-mode stack. */
    push    {{lr}}
    mrs     lr, spsr
    push    {{lr}}

    /* Switch to SYS mode (same stack as application; IRQ/FIQ still masked). */
    cps     #SYS_MODE

    /* Save caller-saved AAPCS registers. */
    push    {{r0-r3, r12}}

    /* ---- ARM Errata 801120: dummy HPPIR read before ICCIAR ---- */
    ldr     r2, =GICC_HPPIR_ADDR
    ldr     r2, [r2]

    /* Read Interrupt Acknowledge Register -> r3 = raw icciar */
    ldr     r2, =GICC_IAR_ADDR
    ldr     r3, [r2]

    /* Extract interrupt ID (bits [9:0]) for errata-733075 check. */
    ubfx    r0, r3, #0, #10

    /* ---- ARM Errata 733075: SGI ID 0 or spurious IDs >= 1022 ---- */
    cmp     r0, #0
    beq     .Lirq_errata_733075
    ldr     r1, =1022
    cmp     r0, r1
    bge     .Lirq_errata_733075
    b       .Lirq_post_errata

.Lirq_errata_733075:
    /* Read-modify-write ICDIPR0 to resolve the errata, then re-read ICCIAR. */
    ldr     r2, =GICD_IPR0_ADDR
    ldr     r0, [r2]
    str     r0, [r2]
    dsb
    ldr     r2, =GICC_HPPIR_ADDR
    ldr     r2, [r2]
    ldr     r2, =GICC_IAR_ADDR
    ldr     r3, [r2]

.Lirq_post_errata:
    /* r0 = full icciar (used for both gic_dispatch arg and ICCEOIR write). */
    mov     r0, r3

    /* Align stack to 8 bytes (AAPCS requirement for BL). */
    mov     r1, sp
    and     r1, r1, #4
    sub     sp, sp, r1

    /* Push: r0(icciar), r1(pad amount), r2, r3, r4(padding reg), lr_sys. */
    push    {{r0-r4, lr}}

    /* Call Rust dispatch: gic_dispatch(icciar) -- r0 already set. */
    bl      gic_dispatch

    /* Restore registers; r0 restored to icciar for the EOI write below. */
    pop     {{r0-r4, lr}}

    /* Undo stack alignment. */
    add     sp, sp, r1

    /* Disable IRQ, memory + instruction barriers, then write EOI.
     * TRM §7.8.3: do NOT write ICCEOIR for spurious IDs 1022/1023.
     * Extract int_id bits [9:0] from r0; skip EOI if id >= 1022. */
    cpsid   i
    dsb
    isb
    ubfx    r3, r0, #0, #10
    ldr     r2, =1022
    cmp     r3, r2
    bge     .Lirq_skip_eoi
    ldr     r2, =GICC_EOIR_ADDR
    str     r0, [r2]
.Lirq_skip_eoi:

    /* Restore caller-saved registers. */
    pop     {{r0-r3, r12}}

    /* Return to interrupted mode: switch to IRQ mode, restore SPSR + LR. */
    cps     #IRQ_MODE
    pop     {{lr}}
    msr     spsr_cxsf, lr
    pop     {{lr}}
    movs    pc, lr          /* return from IRQ exception */

    .size _irq_handler, . - _irq_handler
"#
);
