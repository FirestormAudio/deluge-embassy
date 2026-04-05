OUTPUT_FORMAT("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")
OUTPUT_ARCH(arm)
ENTRY(_start)

/*
 * Linker script for RZ/A1L with SEGGER RTT enabled.
 *
 * The binary crate (or BSP) must provide memory.x on the linker search path.
 * It must define (at minimum):
 *
 *   MEMORY {
 *       RAM            (rwx) : ORIGIN = ..., LENGTH = ...
 *       RTT_RAM        (rw)  : ORIGIN = ..., LENGTH = ...  -- cached alias for RTT page
 *       NCACHE_RTT_RAM (rw)  : ORIGIN = ..., LENGTH = ...  -- uncached alias (same physical page + 0x40000000)
 *       SDRAM          (rwx) : ORIGIN = ..., LENGTH = ...  -- optional
 *   }
 *
 *   PROGRAM_STACK_SIZE = ...;
 *   IRQ_STACK_SIZE     = ...;
 *   FIQ_STACK_SIZE     = ...;
 *   SVC_STACK_SIZE     = ...;
 *   ABT_STACK_SIZE     = ...;
 *
 *   INTERNAL_RAM_END   = ...;   -- top byte address of on-chip SRAM
 *
 * For RTT-free builds use rza1l.x instead (no RTT_RAM/NCACHE_RTT_RAM needed).
 */
INCLUDE memory.x

TTB_SIZE = 0x8000;

SECTIONS {
    /* _SEGGER_RTT control block in the uncached SRAM mirror so the debug
       probe can always see RTT writes, even after D-cache is enabled.
       RTT_RAM (cached) and NCACHE_RTT_RAM (uncached) map to the same physical
       page; reserving both prevents BSS or stacks from being placed there. */
    .rtt_cached_reserve (NOLOAD) : ALIGN(4) { . += 0x10000; } > RTT_RAM
    .rtt_buffer (NOLOAD) : ALIGN(4) {
        KEEP(*(.rtt_buffer .rtt_buffer.*))
        . = ALIGN(4);
    } > NCACHE_RTT_RAM

    /* Match the vendor Deluge layout more closely: reserve low SRAM for the
       MMU translation table and BSS, and place the first loadable code above
       that reserved region. */
    .ttb_mmu1 ORIGIN(RAM) (NOLOAD) : ALIGN(0x4000) {
        ttb_mmu1_base = .;
        . += TTB_SIZE;
        . = ALIGN(4);
        ttb_mmu1_end = .;
    } > RAM

    .bss (NOLOAD) : ALIGN(4) {
        __bss_start__ = .;
        *(.bss .bss.*)
        *(COMMON)
        . = ALIGN(4);
        __bss_end__ = .;
    } > RAM

    /*
     * Vector table + bootloader metadata.
     * Must be the first loadable bytes of the binary.
     * The bootloader reads fixed offsets past the 8 vector entries:
     *   +0x20: code_start   (word: address of _start)
     *   +0x24: code_end     (word: address of end)
     *   +0x28: code_execute (word: execution entry = _start)
     *   +0x2C: ".BootLoad_ValidProgramTest." signature
     */
    .vector_table : ALIGN(0x20) {
        KEEP(*(.vector_table))
    } > RAM

    /* Main code */
    .text : ALIGN(4) {
        *(.text .text.*)
        . = ALIGN(4);
    } > RAM

    .rodata : ALIGN(4) {
        *(.rodata .rodata.*)
        . = ALIGN(4);
    } > RAM

    .data : ALIGN(8) {
        *(.data .data.*)
        . = ALIGN(8);
    } > RAM

    /*
     * Round up to next 64 KB boundary so the bootloader knows how much to
     * copy from SPI flash into SRAM (matches original linker script).
     */
    . = ALIGN(0x10000);
    end = .;
    _end = .;

    /* SRAM heap: free space between the image and the RTT/stack reservation */
    __sram_heap_start = end;
    __sram_heap_end   = ORIGIN(RTT_RAM);

    /*
     * Exception-mode stacks — NOLOAD, placed just below the program stack.
     * Stacks grow downward; _end symbols are the initial SP values.
     */
    .irq_stack (INTERNAL_RAM_END - PROGRAM_STACK_SIZE - ABT_STACK_SIZE - SVC_STACK_SIZE - FIQ_STACK_SIZE - IRQ_STACK_SIZE) (NOLOAD) : {
        irq_stack_start = .;
        . += IRQ_STACK_SIZE;
        irq_stack_end = .;
        fiq_stack_start = .;
        . += FIQ_STACK_SIZE;
        fiq_stack_end = .;
        svc_stack_start = .;
        . += SVC_STACK_SIZE;
        svc_stack_end = .;
        abt_stack_start = .;
        . += ABT_STACK_SIZE;
        abt_stack_end = .;
    } > RAM

    /* Program (SYS mode) stack at the very top of SRAM */
    .program_stack (INTERNAL_RAM_END - PROGRAM_STACK_SIZE) (NOLOAD) : {
        program_stack_start = .;
        . += PROGRAM_STACK_SIZE;
        program_stack_end = .;
    } > RAM

    /DISCARD/ : {
        *(.ARM.exidx*)
        *(.ARM.extab*)
    }
}
