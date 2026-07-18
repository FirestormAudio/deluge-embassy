//! Pure ELF32 helpers for the streaming SD app-loader: header validation,
//! uncached-mirror resolution, and `PT_LOAD` target classification + SRAM
//! staging-address math.
//!
//! The loader itself streams segments off the SD card and writes physical RAM
//! (hardware), but every *decision* it makes about where a segment may go — and
//! the staging arithmetic that keeps an SRAM-targeting segment from clobbering
//! the running bootloader — is pure and lives here so it can be unit-tested.

// --- ELF32 constants -------------------------------------------------------

/// ELF magic bytes.
pub const ELF_MAGIC: [u8; 4] = [0x7F, b'E', b'L', b'F'];
/// ELF class: 32-bit.
pub const ELFCLASS32: u8 = 1;
/// ELF data encoding: little-endian.
pub const ELFDATA2LSB: u8 = 1;
/// ELF type: executable.
pub const ET_EXEC: u16 = 2;
/// ELF machine: ARM.
pub const EM_ARM: u16 = 0x28;
/// Program-header type: loadable segment.
pub const PT_LOAD: u32 = 1;

// --- Deluge load-region geometry -------------------------------------------

/// Uncached mirror alias offset (`rza1l_hal::UNCACHED_MIRROR_OFFSET`).
pub const UNCACHED_MIRROR_OFFSET: u32 = 0x4000_0000;

/// SDRAM region usable by app images: `0x0C000000..0x0FD20000`. The top
/// 2.875 MB (`0x0FD20000..0x10000000`) is the SRAM staging window (see
/// [`SDRAM_STAGE_BASE`]) and is off-limits to app `PT_LOAD` segments.
pub const SDRAM_LO: u32 = 0x0C00_0000;

/// Upper-SRAM region apps may target: `0x20020000..0x20300000`.
pub const SRAM_LOAD_ORIGIN: u32 = 0x2002_0000;
/// Exclusive end of the permitted SRAM load region.
pub const SRAM_HI: u32 = 0x2030_0000;

/// Exclusive top of the 64 MB SDRAM.
const SDRAM_TOP: u32 = 0x1000_0000;

/// Base of the SDRAM staging window for SRAM-targeting segments, pinned to the
/// top of SDRAM and exactly as large as the on-chip SRAM app region it shadows.
/// A segment for SRAM address `p` is parked at
/// `SDRAM_STAGE_BASE + (p - SRAM_LOAD_ORIGIN)`.
pub const SDRAM_STAGE_BASE: u32 = SDRAM_TOP - (SRAM_HI - SRAM_LOAD_ORIGIN);
/// Exclusive end of the directly-writable SDRAM app region. Equals
/// [`SDRAM_STAGE_BASE`]: everything above is staging, not app-usable.
pub const SDRAM_HI: u32 = SDRAM_STAGE_BASE;

/// Maximum program headers the loader processes.
pub const MAX_PHDRS: usize = 8;

/// Where a `PT_LOAD` segment is allowed to land.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum LoadTarget {
    /// SDRAM (`0x0C000000..0x0FD20000`): written directly to its final address.
    Sdram,
    /// Upper SRAM (`0x20020000..0x20300000`): staged in SDRAM, relocated later.
    Sram,
}

/// Descriptor for one staged SRAM segment, handed to the relocation trampoline.
///
/// `repr(C)` so the trampoline can read fields with plain `ldr` at fixed
/// offsets; the field order/size is part of that ABI — do not reorder.
#[derive(Clone, Copy, Default, PartialEq, Eq, Debug)]
#[repr(C)]
pub struct SegDesc {
    /// Source address in the SDRAM staging window.
    pub src: u32,
    /// Final SRAM destination address.
    pub dst: u32,
    /// Bytes to copy from `src` to `dst`.
    pub filesz: u32,
    /// Extra bytes to zero after the copy (`p_memsz - p_filesz`).
    pub zero_extra: u32,
}

/// Read a little-endian `u16` at `off`.
#[inline]
pub fn le16(buf: &[u8], off: usize) -> u16 {
    u16::from_le_bytes([buf[off], buf[off + 1]])
}

/// Read a little-endian `u32` at `off`.
#[inline]
pub fn le32(buf: &[u8], off: usize) -> u32 {
    u32::from_le_bytes([buf[off], buf[off + 1], buf[off + 2], buf[off + 3]])
}

/// Validate the 52-byte ELF32 header as a little-endian ARM executable.
///
/// Returns `Ok(())` if the loader should accept the image; the streaming loader
/// runs this on the first header bytes before reading any program header.
pub fn validate_header(buf: &[u8]) -> Result<(), HeaderError> {
    if buf.len() < 52 {
        return Err(HeaderError::TooShort);
    }
    if buf[0..4] != ELF_MAGIC {
        return Err(HeaderError::BadMagic);
    }
    if buf[4] != ELFCLASS32 || buf[5] != ELFDATA2LSB {
        return Err(HeaderError::WrongFormat);
    }
    if le16(buf, 16) != ET_EXEC || le16(buf, 18) != EM_ARM {
        return Err(HeaderError::WrongFormat);
    }
    Ok(())
}

/// Reason [`validate_header`] rejected an image.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum HeaderError {
    /// Fewer than 52 bytes — not a complete ELF32 header.
    TooShort,
    /// Magic bytes are not `\x7FELF`.
    BadMagic,
    /// Not a 32-bit little-endian ARM executable.
    WrongFormat,
}

/// Resolve an uncached **mirror-alias** address to the physical address it
/// shadows. OCRAM/SDRAM are aliased at `physical + 0x4000_0000`; classification
/// and staging math must use the underlying physical address.
pub fn mirror_to_phys(addr: u32) -> u32 {
    if (0x6000_0000..0x60A0_0000).contains(&addr) || (0x4C00_0000..0x5000_0000).contains(&addr) {
        addr - UNCACHED_MIRROR_OFFSET
    } else {
        addr
    }
}

/// Classify the physical target range `addr..addr+len`, or `None` if it is
/// outside every region a `PT_LOAD` segment may occupy.
///
/// `addr` must already be [`mirror_to_phys`]-resolved.
pub fn classify_load_range(addr: u32, len: u32) -> Option<LoadTarget> {
    if len == 0 {
        // Empty segments are harmless; no staging needed.
        return Some(LoadTarget::Sdram);
    }
    let end = addr.checked_add(len)?;
    if addr >= SDRAM_LO && end <= SDRAM_HI {
        return Some(LoadTarget::Sdram);
    }
    if addr >= SRAM_LOAD_ORIGIN && end <= SRAM_HI {
        return Some(LoadTarget::Sram);
    }
    None
}

/// Staging address for an SRAM-targeting segment whose final address is `dst`.
///
/// `dst` must be in `[SRAM_LOAD_ORIGIN, SRAM_HI)` (i.e. classified as
/// [`LoadTarget::Sram`]); the result is inside the SDRAM staging window.
pub fn sram_stage_addr(dst: u32) -> u32 {
    SDRAM_STAGE_BASE + (dst - SRAM_LOAD_ORIGIN)
}

// --- Per-segment placement (shared by the SD and USB loaders) --------------

/// What a loader should do with one `PT_LOAD` segment, derived purely from its
/// physical address and size.  Both the streaming SD loader and the slice-based
/// USB dev-upload loader funnel every segment through [`place_segment`] so the
/// "where does this land?" decision (and its address math) has a single,
/// host-tested definition and the two paths can never drift apart.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum SegmentPlacement {
    /// Data-retention RAM (`0x20000000..0x2001FFFF`): skip it. That region is
    /// reserved for the relocation trampoline; apps self-zero any BSS there.
    Skip,
    /// Copy the segment's file bytes to `write_addr` now. For an SDRAM target
    /// `write_addr` is the final address (`sram == false`); for an SRAM target it
    /// is the SDRAM staging address (`sram == true`) and the trampoline relocates
    /// it to its final SRAM address later.
    Write {
        /// Address to copy the segment's file bytes to immediately.
        write_addr: u32,
        /// `true` if this is an SRAM-targeting segment parked in staging.
        sram: bool,
    },
}

/// Decide where a `PT_LOAD` segment with the given **program-header** physical
/// address and memory size goes.  `p_paddr` is the raw header value (it may be an
/// uncached mirror alias); `p_memsz` is `p_memsz` from the header.
///
/// Returns [`BadLoadAddress`] if the resolved range is outside every region a
/// segment may occupy (the caller maps that to its own "bad load address" error).
pub fn place_segment(p_paddr: u32, p_memsz: u32) -> Result<SegmentPlacement, BadLoadAddress> {
    let phys = mirror_to_phys(p_paddr);
    // Retention RAM is reserved for the trampoline; never written by the loader.
    if (0x2000_0000..SRAM_LOAD_ORIGIN).contains(&phys) {
        return Ok(SegmentPlacement::Skip);
    }
    match classify_load_range(phys, p_memsz) {
        // Write SDRAM segments through the header address so a segment that asked
        // for the non-cacheable mirror still lands there.
        Some(LoadTarget::Sdram) => Ok(SegmentPlacement::Write {
            write_addr: p_paddr,
            sram: false,
        }),
        Some(LoadTarget::Sram) => Ok(SegmentPlacement::Write {
            write_addr: sram_stage_addr(phys),
            sram: true,
        }),
        None => Err(BadLoadAddress),
    }
}

/// [`place_segment`] was handed a physical address/size that resolves to a region
/// no `PT_LOAD` segment may occupy. Callers map this to their own error.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct BadLoadAddress;

// --- Slice-sourced load plan (USB dev-upload path) -------------------------

/// One copy operation in a [`LoadPlan`]: copy `filesz` bytes from offset
/// `src_off` in the ELF image to `write_addr`, then zero `zero_extra` bytes after
/// them.  For an SRAM segment (`sram`), `write_addr` is the staging address and
/// `final_dst` is where the trampoline must relocate it (with the same
/// `zero_extra`); for an SDRAM segment `final_dst == write_addr`.
#[derive(Clone, Copy, Default, PartialEq, Eq, Debug)]
pub struct LoadOp {
    /// Byte offset of the segment's file content within the ELF image.
    pub src_off: u32,
    /// Address the loader copies the file bytes to now.
    pub write_addr: u32,
    /// The segment's final runtime address (`p_paddr`).
    pub final_dst: u32,
    /// Number of file bytes to copy.
    pub filesz: u32,
    /// Bytes to zero immediately after the copy (`p_memsz - p_filesz`).
    pub zero_extra: u32,
    /// `true` if this segment is staged in SDRAM for later SRAM relocation.
    pub sram: bool,
}

/// A fully-parsed, validated plan for loading an ELF image held entirely in a
/// byte slice (the USB dev-upload path).  Produced by [`parse_load_plan`]; the
/// device then executes each [`LoadOp`] with raw memory writes.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct LoadPlan {
    /// Application entry point (`e_entry`).
    pub entry: u32,
    /// Copy operations, in program-header order.
    pub ops: [LoadOp; MAX_PHDRS],
    /// Number of valid entries in `ops`.
    pub n_ops: usize,
    /// Number of `ops` that are SRAM-staged (`sram == true`).
    pub n_sram: usize,
}

/// Why [`parse_load_plan`] rejected an ELF image slice.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum PlanError {
    /// Header magic is not `\x7FELF`.
    BadMagic,
    /// Not a 32-bit LE ARM executable, or a malformed/oversized header table.
    WrongFormat,
    /// A `PT_LOAD` segment targets a region no segment may occupy.
    BadLoadAddress,
    /// A program header or a segment's file range runs past the end of the slice.
    Truncated,
}

impl From<HeaderError> for PlanError {
    fn from(e: HeaderError) -> Self {
        match e {
            HeaderError::TooShort => PlanError::Truncated,
            HeaderError::BadMagic => PlanError::BadMagic,
            HeaderError::WrongFormat => PlanError::WrongFormat,
        }
    }
}

/// Parse and validate an in-memory ELF32 image, producing the [`LoadPlan`] the
/// USB dev-upload loader executes.  Pure (no hardware): it validates the header,
/// bounds-checks the program-header table and every segment's file range against
/// `elf.len()`, classifies each `PT_LOAD` via [`place_segment`], and records a
/// [`LoadOp`] for each segment that must be written.
///
/// This is the host-testable core of the device's `load_from_slice`.
pub fn parse_load_plan(elf: &[u8]) -> Result<LoadPlan, PlanError> {
    validate_header(elf)?;

    let e_entry = le32(elf, 24);
    let e_phoff = le32(elf, 28) as usize;
    let e_phentsize = le16(elf, 42) as usize;
    let e_phnum = le16(elf, 44) as usize;

    if e_phentsize != 32 || e_phnum > MAX_PHDRS {
        return Err(PlanError::WrongFormat);
    }
    // The whole program-header table must lie within the image.
    let ph_table_end = e_phoff
        .checked_add(e_phnum.checked_mul(32).ok_or(PlanError::WrongFormat)?)
        .ok_or(PlanError::WrongFormat)?;
    if ph_table_end > elf.len() {
        return Err(PlanError::Truncated);
    }

    let mut ops = [LoadOp::default(); MAX_PHDRS];
    let mut n_ops = 0usize;
    let mut n_sram = 0usize;

    for i in 0..e_phnum {
        let ph = &elf[e_phoff + i * 32..][..32];
        if le32(ph, 0) != PT_LOAD {
            continue;
        }
        let p_offset = le32(ph, 4);
        let p_paddr = le32(ph, 12);
        let p_filesz = le32(ph, 16);
        let p_memsz = le32(ph, 20);

        // A well-formed PT_LOAD always has filesz <= memsz.
        if p_filesz > p_memsz {
            return Err(PlanError::WrongFormat);
        }
        // The segment's file bytes must lie within the image.
        let seg_end = (p_offset as u64) + (p_filesz as u64);
        if seg_end > elf.len() as u64 {
            return Err(PlanError::Truncated);
        }

        match place_segment(p_paddr, p_memsz).map_err(|_| PlanError::BadLoadAddress)? {
            SegmentPlacement::Skip => continue,
            SegmentPlacement::Write { write_addr, sram } => {
                ops[n_ops] = LoadOp {
                    src_off: p_offset,
                    write_addr,
                    final_dst: p_paddr,
                    filesz: p_filesz,
                    zero_extra: p_memsz - p_filesz,
                    sram,
                };
                n_ops += 1;
                if sram {
                    n_sram += 1;
                }
            }
        }
    }

    Ok(LoadPlan {
        entry: e_entry,
        ops,
        n_ops,
        n_sram,
    })
}

// --- FSB metadata --------------------------------------------------------

/// Image offsets of the first-stage-bootloader (FSB) metadata words, emitted by
/// `rza1l-hal/src/startup.rs` just past the eight-entry vector table.
pub const FSB_CODE_START: usize = 0x20;
/// Offset of the `code_end` word (one past the last image byte).
pub const FSB_CODE_END: usize = 0x24;
/// Offset of the `code_execute` (entry point) word.
pub const FSB_CODE_EXECUTE: usize = 0x28;
/// Offset of the validity signature.
pub const FSB_SIGNATURE_OFF: usize = 0x2C;
/// Signature string a bootable image carries at [`FSB_SIGNATURE_OFF`].
pub const FSB_SIGNATURE: &[u8] = b".BootLoad_ValidProgramTest.";

/// Validated FSB metadata read from a flat firmware image.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct FsbMeta {
    /// Load address of image byte 0.
    pub code_start: u32,
    /// One past the last image byte (linker `end`, 64 KB-rounded).
    pub code_end: u32,
    /// Entry point.
    pub entry: u32,
}

/// Why [`validate_fsb_metadata`] rejected a flattened image.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum FsbError {
    /// Image is shorter than the metadata block at `+0x2C`.
    TooSmall,
    /// The `.BootLoad_ValidProgramTest.` signature is absent at `+0x2C`.
    BadSignature,
    /// `code_start` disagrees with the image's lowest load address.
    CodeStartMismatch,
    /// `code_end <= code_start`.
    BadCodeEnd,
    /// Entry point is outside `code_start..code_end`.
    EntryOutOfRange,
    /// Flat image extends past the span the FSB will copy (`code_end`).
    ImageTooLong,
    /// Flat image is larger than the destination flash app slot, so it cannot be
    /// programmed without overrunning the slot.  Raised by the slot-store path,
    /// which knows the slot length; [`validate_fsb_metadata`] never returns it.
    TooLargeForSlot,
    /// A post-program readback of the flash slot did not match the source image,
    /// so the stored image is corrupt.  Raised by the slot-store path; carries the
    /// byte offset of the first mismatch.  `validate_fsb_metadata` never returns it.
    VerifyFailed(u32),
}

/// Validate the FSB metadata embedded in a flattened (`objcopy -O binary`-style)
/// firmware image, the same checks the on-flash boot path applies before it will
/// copy and jump.  Catching a bad image here means it is refused *before* the
/// flash slot is erased, instead of silently failing to boot.
///
/// `image_base` is the lowest load address (LMA) of the flattened image — byte 0
/// of `image` — which must equal the metadata's `code_start`.
pub fn validate_fsb_metadata(image: &[u8], image_base: u32) -> Result<FsbMeta, FsbError> {
    if image.len() < FSB_SIGNATURE_OFF + FSB_SIGNATURE.len() {
        return Err(FsbError::TooSmall);
    }
    if &image[FSB_SIGNATURE_OFF..FSB_SIGNATURE_OFF + FSB_SIGNATURE.len()] != FSB_SIGNATURE {
        return Err(FsbError::BadSignature);
    }

    let code_start = le32(image, FSB_CODE_START);
    let code_end = le32(image, FSB_CODE_END);
    let entry = le32(image, FSB_CODE_EXECUTE);

    // Byte 0 of the image *is* code_start, so the metadata must agree with the
    // actual lowest LMA the linker emitted.
    if code_start != image_base {
        return Err(FsbError::CodeStartMismatch);
    }
    if code_end <= code_start {
        return Err(FsbError::BadCodeEnd);
    }
    if entry < code_start || entry >= code_end {
        return Err(FsbError::EntryOutOfRange);
    }
    // The flat image must fit inside the span the FSB copies. It is normally
    // *shorter* (code_end is 64 KB-rounded); only a longer image is a real bug.
    if image.len() > (code_end - code_start) as usize {
        return Err(FsbError::ImageTooLong);
    }

    Ok(FsbMeta {
        code_start,
        code_end,
        entry,
    })
}

/// Locate the bootable image base within a flattened, possibly multi-segment
/// image by anchoring on the FSB signature.
///
/// The flash-boot path keys off the `.BootLoad_ValidProgramTest.` signature at
/// [`FSB_SIGNATURE_OFF`] from image byte 0, so byte 0 must be the vector table.
/// The flattener bases the staged image at the *lowest* `PT_LOAD` load address,
/// which for SDK firmware (`rza1l-hal`'s startup) already *is* the vector table —
/// so this returns `Some(0)`.
///
/// Some firmware lays out differently: the DelugeFirmware linker places a
/// leading `NOLOAD` region (MMU translation table + mode stacks) at a lower
/// address than the vector table *inside the same `PT_LOAD`*, so the segment's
/// `p_paddr` is below `_start`.  Byte 0 of the flat image is then that
/// zero-filled region, and the metadata/signature the boot path expects at
/// `+0x20`/`+0x2C` land deep in the image.  That leading region is not part of
/// the bootable image the FSB copies (the firmware's own startup rebuilds the
/// TTB and stacks), so the flattener drops it: this returns the offset of the
/// vector table (`signature offset − FSB_SIGNATURE_OFF`), matching the
/// section-based layout `objcopy -O binary` produces.
///
/// Returns `None` if the signature is absent (a genuinely unsigned image) or
/// only appears within the first metadata block's worth of bytes (too early to
/// be a real `+0x2C` signature) — callers leave the image unshifted so
/// [`validate_fsb_metadata`] reports the precise rejection.
pub fn find_fsb_base(image: &[u8]) -> Option<usize> {
    image
        .windows(FSB_SIGNATURE.len())
        .position(|w| w == FSB_SIGNATURE)
        .and_then(|sig_off| sig_off.checked_sub(FSB_SIGNATURE_OFF))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn elf_header(class: u8, data: u8, etype: u16, machine: u16) -> [u8; 52] {
        let mut h = [0u8; 52];
        h[0..4].copy_from_slice(&ELF_MAGIC);
        h[4] = class;
        h[5] = data;
        h[16..18].copy_from_slice(&etype.to_le_bytes());
        h[18..20].copy_from_slice(&machine.to_le_bytes());
        h
    }

    #[test]
    fn header_accepts_arm_le_exec() {
        let h = elf_header(ELFCLASS32, ELFDATA2LSB, ET_EXEC, EM_ARM);
        assert_eq!(validate_header(&h), Ok(()));
    }

    #[test]
    fn header_rejects_bad_inputs() {
        assert_eq!(validate_header(&[0u8; 10]), Err(HeaderError::TooShort));

        let mut h = elf_header(ELFCLASS32, ELFDATA2LSB, ET_EXEC, EM_ARM);
        h[1] = b'Z';
        assert_eq!(validate_header(&h), Err(HeaderError::BadMagic));

        assert_eq!(
            validate_header(&elf_header(2, ELFDATA2LSB, ET_EXEC, EM_ARM)),
            Err(HeaderError::WrongFormat),
            "ELFCLASS64"
        );
        assert_eq!(
            validate_header(&elf_header(ELFCLASS32, 2, ET_EXEC, EM_ARM)),
            Err(HeaderError::WrongFormat),
            "big-endian"
        );
        assert_eq!(
            validate_header(&elf_header(ELFCLASS32, ELFDATA2LSB, 1, EM_ARM)),
            Err(HeaderError::WrongFormat),
            "ET_REL"
        );
        assert_eq!(
            validate_header(&elf_header(ELFCLASS32, ELFDATA2LSB, ET_EXEC, 0x3E)),
            Err(HeaderError::WrongFormat),
            "x86-64"
        );
    }

    #[test]
    fn mirror_resolves_aliases() {
        // OCRAM mirror 0x6000_0000 → 0x2000_0000.
        assert_eq!(mirror_to_phys(0x6000_0000), 0x2000_0000);
        assert_eq!(mirror_to_phys(0x6009_FFFF), 0x2009_FFFF);
        // SDRAM mirror 0x4C00_0000 → 0x0C00_0000.
        assert_eq!(mirror_to_phys(0x4C00_0000), 0x0C00_0000);
        // Non-mirror addresses pass through unchanged.
        assert_eq!(mirror_to_phys(0x2002_0000), 0x2002_0000);
        assert_eq!(mirror_to_phys(0x0C00_0000), 0x0C00_0000);
        // Just outside the mirror windows: unchanged.
        assert_eq!(mirror_to_phys(0x60A0_0000), 0x60A0_0000);
        assert_eq!(mirror_to_phys(0x5000_0000), 0x5000_0000);
    }

    #[test]
    fn classify_sdram_and_sram() {
        assert_eq!(
            classify_load_range(SDRAM_LO, 0x1000),
            Some(LoadTarget::Sdram)
        );
        assert_eq!(
            classify_load_range(SDRAM_HI - 1, 1),
            Some(LoadTarget::Sdram)
        );
        assert_eq!(
            classify_load_range(SRAM_LOAD_ORIGIN, 0x1000),
            Some(LoadTarget::Sram)
        );
        assert_eq!(classify_load_range(SRAM_HI - 4, 4), Some(LoadTarget::Sram));
    }

    #[test]
    fn classify_empty_segment_is_sdram() {
        // Even at an otherwise-illegal address, a zero-length segment is fine.
        assert_eq!(classify_load_range(0x0000_0000, 0), Some(LoadTarget::Sdram));
    }

    #[test]
    fn classify_rejects_out_of_region_and_overlaps() {
        // Below SDRAM.
        assert_eq!(classify_load_range(0x0800_0000, 0x10), None);
        // SDRAM segment spilling into the staging window is rejected.
        assert_eq!(classify_load_range(SDRAM_HI - 4, 0x100), None);
        // SRAM segment past the top of the region.
        assert_eq!(classify_load_range(SRAM_HI - 4, 0x100), None);
        // Gap between SDRAM and SRAM (e.g. low OCRAM the bootloader uses).
        assert_eq!(classify_load_range(0x2000_0000, 0x10), None);
        // Address+len overflow must not panic.
        assert_eq!(classify_load_range(0xFFFF_FF00, 0x200), None);
    }

    #[test]
    fn staging_address_math() {
        // Byte 0 of the SRAM region maps to the staging base.
        assert_eq!(sram_stage_addr(SRAM_LOAD_ORIGIN), SDRAM_STAGE_BASE);
        // Offset within SRAM is preserved into the staging window.
        assert_eq!(
            sram_stage_addr(SRAM_LOAD_ORIGIN + 0x1234),
            SDRAM_STAGE_BASE + 0x1234
        );
        // The whole SRAM region stays inside SDRAM (staging window ≤ ~2.875 MB).
        let top = sram_stage_addr(SRAM_HI - 1);
        assert!(top < 0x1000_0000, "staging must stay within 64 MB SDRAM");
    }

    #[test]
    fn sdram_ceiling_is_staging_base_at_top_of_sdram() {
        // The app-segment ceiling equals the staging base, and the staging window
        // (= the on-chip SRAM app-region size) sits flush against the top of SDRAM.
        assert_eq!(SDRAM_HI, 0x0FD2_0000);
        assert_eq!(SDRAM_HI, SDRAM_STAGE_BASE);
        assert_eq!(SDRAM_STAGE_BASE + (SRAM_HI - SRAM_LOAD_ORIGIN), 0x1000_0000);
    }

    /// Build a minimal flat image carrying valid FSB metadata.
    fn fsb_image(code_start: u32, code_end: u32, entry: u32, len: usize) -> Vec<u8> {
        let mut img = vec![0u8; len.max(FSB_SIGNATURE_OFF + FSB_SIGNATURE.len())];
        img[FSB_CODE_START..FSB_CODE_START + 4].copy_from_slice(&code_start.to_le_bytes());
        img[FSB_CODE_END..FSB_CODE_END + 4].copy_from_slice(&code_end.to_le_bytes());
        img[FSB_CODE_EXECUTE..FSB_CODE_EXECUTE + 4].copy_from_slice(&entry.to_le_bytes());
        img[FSB_SIGNATURE_OFF..FSB_SIGNATURE_OFF + FSB_SIGNATURE.len()]
            .copy_from_slice(FSB_SIGNATURE);
        img
    }

    #[test]
    fn fsb_accepts_well_formed_image() {
        let img = fsb_image(0x2002_0000, 0x2003_0000, 0x2002_0100, 0x800);
        assert_eq!(
            validate_fsb_metadata(&img, 0x2002_0000),
            Ok(FsbMeta {
                code_start: 0x2002_0000,
                code_end: 0x2003_0000,
                entry: 0x2002_0100,
            })
        );
    }

    #[test]
    fn fsb_rejects_bad_images() {
        // Too short to hold the metadata block.
        assert_eq!(
            validate_fsb_metadata(&[0u8; 16], 0),
            Err(FsbError::TooSmall)
        );

        // Missing signature.
        let mut img = fsb_image(0x2002_0000, 0x2003_0000, 0x2002_0100, 0x800);
        img[FSB_SIGNATURE_OFF] = 0;
        assert_eq!(
            validate_fsb_metadata(&img, 0x2002_0000),
            Err(FsbError::BadSignature)
        );

        // code_start disagrees with the image's lowest load address.
        let img = fsb_image(0x2002_0000, 0x2003_0000, 0x2002_0100, 0x800);
        assert_eq!(
            validate_fsb_metadata(&img, 0x2002_1000),
            Err(FsbError::CodeStartMismatch)
        );

        // code_end <= code_start.
        let img = fsb_image(0x2002_0000, 0x2002_0000, 0x2002_0000, 0x800);
        assert_eq!(
            validate_fsb_metadata(&img, 0x2002_0000),
            Err(FsbError::BadCodeEnd)
        );

        // Entry outside code_start..code_end.
        let img = fsb_image(0x2002_0000, 0x2003_0000, 0x2003_0000, 0x800);
        assert_eq!(
            validate_fsb_metadata(&img, 0x2002_0000),
            Err(FsbError::EntryOutOfRange)
        );

        // Flat image longer than code_end - code_start.
        let img = fsb_image(0x2002_0000, 0x2002_0100, 0x2002_0000, 0x800);
        assert_eq!(
            validate_fsb_metadata(&img, 0x2002_0000),
            Err(FsbError::ImageTooLong)
        );
    }

    #[test]
    fn find_fsb_base_zero_when_vectors_lead() {
        // SDK layout: signature already at +0x2C, so the base is byte 0.
        let img = fsb_image(0x2002_0000, 0x2003_0000, 0x2002_0100, 0x800);
        assert_eq!(find_fsb_base(&img), Some(0));
    }

    #[test]
    fn find_fsb_base_skips_leading_noload_region() {
        // DelugeFirmware layout: a zero-filled leading region (MMU TTB / stacks)
        // precedes the vector table, so the signature sits deeper in the flat
        // image. The returned base re-anchors byte 0 on the vector table.
        const LEAD: usize = 0x4_1300;
        let body = fsb_image(0x2006_1300, 0x2021_dfde, 0x2006_1300, 0x800);
        let mut img = vec![0u8; LEAD];
        img.extend_from_slice(&body);
        assert_eq!(find_fsb_base(&img), Some(LEAD));
        // Re-basing there yields a body whose metadata validates.
        let base = find_fsb_base(&img).unwrap();
        assert_eq!(
            validate_fsb_metadata(&img[base..], 0x2006_1300),
            Ok(FsbMeta {
                code_start: 0x2006_1300,
                code_end: 0x2021_dfde,
                entry: 0x2006_1300,
            })
        );
    }

    #[test]
    fn find_fsb_base_none_when_unsigned() {
        // No signature anywhere: caller leaves the image unshifted and lets
        // validate_fsb_metadata report BadSignature.
        assert_eq!(find_fsb_base(&[0u8; 0x400]), None);
    }

    #[test]
    fn seg_desc_is_repr_c_four_u32() {
        // The trampoline reads this by fixed offset; lock the layout.
        assert_eq!(core::mem::size_of::<SegDesc>(), 16);
        assert_eq!(core::mem::align_of::<SegDesc>(), 4);
    }

    // --- place_segment ------------------------------------------------------

    #[test]
    fn place_sdram_writes_through_header_address() {
        // SDRAM target: written directly to its (possibly mirror) header address.
        assert_eq!(
            place_segment(SDRAM_LO + 0x1000, 0x200),
            Ok(SegmentPlacement::Write {
                write_addr: SDRAM_LO + 0x1000,
                sram: false,
            })
        );
        // Non-cacheable SDRAM mirror keeps writing through the mirror address but
        // classifies on the resolved physical address.
        assert_eq!(
            place_segment(0x4C00_1000, 0x200),
            Ok(SegmentPlacement::Write {
                write_addr: 0x4C00_1000,
                sram: false,
            })
        );
    }

    #[test]
    fn place_sram_targets_staging() {
        assert_eq!(
            place_segment(SRAM_LOAD_ORIGIN + 0x1234, 0x40),
            Ok(SegmentPlacement::Write {
                write_addr: SDRAM_STAGE_BASE + 0x1234,
                sram: true,
            })
        );
    }

    #[test]
    fn place_retention_ram_is_skipped() {
        assert_eq!(place_segment(0x2000_0000, 0x10), Ok(SegmentPlacement::Skip));
        assert_eq!(
            place_segment(0x2001_FFFF, 1),
            Ok(SegmentPlacement::Skip),
            "last retention byte"
        );
    }

    #[test]
    fn place_rejects_out_of_region() {
        assert_eq!(place_segment(0x0800_0000, 0x10), Err(BadLoadAddress));
        assert_eq!(place_segment(SRAM_HI - 4, 0x100), Err(BadLoadAddress));
    }

    // --- parse_load_plan ----------------------------------------------------

    /// Build a minimal ELF32 image with the given `PT_LOAD` segments.
    /// Each segment is `(p_offset, p_paddr, p_filesz, p_memsz)`; the file content
    /// for each is whatever already sits at `p_offset` in the produced buffer
    /// (zero), which is fine for the plan tests (they don't inspect content).
    fn elf_with_segments(entry: u32, segs: &[(u32, u32, u32, u32)], total_len: usize) -> Vec<u8> {
        let phoff = 52u32;
        let phnum = segs.len() as u16;
        let mut buf = vec![0u8; total_len.max((phoff + phnum as u32 * 32) as usize)];
        buf[0..4].copy_from_slice(&ELF_MAGIC);
        buf[4] = ELFCLASS32;
        buf[5] = ELFDATA2LSB;
        buf[16..18].copy_from_slice(&ET_EXEC.to_le_bytes());
        buf[18..20].copy_from_slice(&EM_ARM.to_le_bytes());
        buf[24..28].copy_from_slice(&entry.to_le_bytes()); // e_entry
        buf[28..32].copy_from_slice(&phoff.to_le_bytes()); // e_phoff
        buf[42..44].copy_from_slice(&32u16.to_le_bytes()); // e_phentsize
        buf[44..46].copy_from_slice(&phnum.to_le_bytes()); // e_phnum
        for (i, &(off, paddr, filesz, memsz)) in segs.iter().enumerate() {
            let ph = (phoff as usize) + i * 32;
            buf[ph..ph + 4].copy_from_slice(&PT_LOAD.to_le_bytes());
            buf[ph + 4..ph + 8].copy_from_slice(&off.to_le_bytes());
            buf[ph + 12..ph + 16].copy_from_slice(&paddr.to_le_bytes());
            buf[ph + 16..ph + 20].copy_from_slice(&filesz.to_le_bytes());
            buf[ph + 20..ph + 24].copy_from_slice(&memsz.to_le_bytes());
        }
        buf
    }

    #[test]
    fn plan_mixes_sram_sdram_and_skips_retention() {
        // SRAM segment (staged), an SDRAM segment (direct), and a retention-RAM
        // segment that must be skipped (no op emitted).
        let img = elf_with_segments(
            SRAM_LOAD_ORIGIN + 0x40,
            &[
                (0x200, SRAM_LOAD_ORIGIN, 0x100, 0x180), // SRAM: 0x80 BSS tail
                (0x400, SDRAM_LO, 0x80, 0x80),           // SDRAM: no BSS
                (0x000, 0x2000_0000, 0x10, 0x10),        // retention: skipped
            ],
            0x600,
        );
        let plan = parse_load_plan(&img).unwrap();
        assert_eq!(plan.entry, SRAM_LOAD_ORIGIN + 0x40);
        assert_eq!(plan.n_ops, 2, "retention segment skipped");
        assert_eq!(plan.n_sram, 1);

        assert_eq!(
            plan.ops[0],
            LoadOp {
                src_off: 0x200,
                write_addr: SDRAM_STAGE_BASE, // staged for SRAM_LOAD_ORIGIN
                final_dst: SRAM_LOAD_ORIGIN,
                filesz: 0x100,
                zero_extra: 0x80,
                sram: true,
            }
        );
        assert_eq!(
            plan.ops[1],
            LoadOp {
                src_off: 0x400,
                write_addr: SDRAM_LO,
                final_dst: SDRAM_LO,
                filesz: 0x80,
                zero_extra: 0,
                sram: false,
            }
        );
    }

    #[test]
    fn plan_rejects_bad_images() {
        // Bad magic.
        let mut img = elf_with_segments(
            SRAM_LOAD_ORIGIN,
            &[(0x80, SRAM_LOAD_ORIGIN, 0x10, 0x10)],
            0x100,
        );
        img[1] = b'Z';
        assert_eq!(parse_load_plan(&img), Err(PlanError::BadMagic));

        // filesz > memsz.
        let img = elf_with_segments(
            SRAM_LOAD_ORIGIN,
            &[(0x80, SRAM_LOAD_ORIGIN, 0x20, 0x10)],
            0x100,
        );
        assert_eq!(parse_load_plan(&img), Err(PlanError::WrongFormat));

        // Segment file range past the end of the slice.
        let img = elf_with_segments(
            SRAM_LOAD_ORIGIN,
            &[(0x80, SRAM_LOAD_ORIGIN, 0x100, 0x100)],
            0x100,
        );
        assert_eq!(parse_load_plan(&img), Err(PlanError::Truncated));

        // Bad load address.
        let img = elf_with_segments(SRAM_LOAD_ORIGIN, &[(0x80, 0x0800_0000, 0x10, 0x10)], 0x100);
        assert_eq!(parse_load_plan(&img), Err(PlanError::BadLoadAddress));

        // Too many program headers.
        let many: Vec<(u32, u32, u32, u32)> = (0..=MAX_PHDRS)
            .map(|_| (0x80, SRAM_LOAD_ORIGIN, 0, 0))
            .collect();
        let img = elf_with_segments(SRAM_LOAD_ORIGIN, &many, 0x400);
        assert_eq!(parse_load_plan(&img), Err(PlanError::WrongFormat));
    }
}
