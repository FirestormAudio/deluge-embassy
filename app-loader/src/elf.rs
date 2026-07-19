//! Minimal streaming ELF32-LE loader for ARM firmware images.
//!
//! ## Load strategy
//!
//! An app image that targets upper SRAM (`0x20020000+`) would corrupt the
//! running bootloader if its `PT_LOAD` segments were written directly: the
//! bootloader's data, heap and stacks live there.  To avoid this, SRAM-targeting
//! segments are staged in SDRAM during the ELF read phase, then relocated to
//! their final SRAM addresses by a small trampoline running from data-retention
//! RAM (`0x20000000–0x2001FFFF`).  That region is untouched by both the
//! first-stage bootloader and the apps, so the trampoline is safe to execute
//! while SRAM is being overwritten.
//!
//! SDRAM-targeting segments (`0x0C000000–0x0EFFFFFF`) are written directly;
//! they cannot overwrite the bootloader.

use deluge_bsp::fat::{self, FatError, RawFile};
// The ELF wire-format constants and the pure load-range / staging math live in
// the host-testable `deluge-image` crate so there is a single host-tested
// implementation (see `crates/deluge-image/src/elf.rs`).
use deluge_bsp::flash;
use deluge_image::elf::{
    ELF_MAGIC, ELFCLASS32, ELFDATA2LSB, EM_ARM, ET_EXEC, LoadTarget, MAX_PHDRS, PT_LOAD, PlanError,
    SegmentPlacement, classify_load_range, find_fsb_base, le16, le32, mirror_to_phys,
    place_segment, sram_stage_addr,
};
use embassy_time::{Duration, Timer};

/// Chunk size for streamed segment copies (one FAT sector).
const CHUNK: usize = 512;

/// Descriptor for one SRAM-targeting `PT_LOAD` segment (the trampoline ABI).
/// Defined in [`deluge_image::elf`] as `SegDesc`; re-exported here under the
/// loader's historical name so [`crate::launcher`] / [`crate::flashboot`] are
/// unaffected.
pub use deluge_image::elf::SegDesc as SramSegDesc;

/// Result returned by a successful [`load_from_sd`] call.
pub struct LoadResult {
    /// Application entry point (`e_entry`).
    pub entry: u32,
    /// Descriptors for SRAM segments parked in SDRAM staging.
    pub sram_descs: [SramSegDesc; MAX_PHDRS],
    /// Number of valid entries in `sram_descs`.
    pub n_sram: usize,
}

/// Errors from the ELF loader.
#[derive(Debug, Clone)]
pub enum ElfError {
    /// ELF magic bytes are not `\x7FELF`.
    BadMagic,
    /// Not a 32-bit little-endian ARM executable ELF.
    WrongFormat,
    /// A `PT_LOAD` segment's physical address is outside the permitted regions.
    BadLoadAddress,
    /// The uploaded ELF cannot be stream-loaded: its `PT_LOAD` segments are not
    /// in non-decreasing, non-overlapping file order, so the loader would have
    /// to seek backward to place them.
    Unstreamable,
    /// SD card / FAT read or seek error.
    Io(#[allow(dead_code)] FatError),
    /// `read` returned 0 before the segment was fully copied.
    UnexpectedEof,
    /// A `PT_LOAD` segment targets a region other than upper SRAM, so the image
    /// cannot be stored as a single-descriptor flash-boot image.
    NotFlashable,
    /// The flattened image is larger than the flash app slot.
    TooLarge,
}

impl From<FatError> for ElfError {
    fn from(e: FatError) -> Self {
        ElfError::Io(e)
    }
}

impl From<PlanError> for ElfError {
    fn from(e: PlanError) -> Self {
        match e {
            PlanError::BadMagic => ElfError::BadMagic,
            PlanError::WrongFormat => ElfError::WrongFormat,
            PlanError::BadLoadAddress => ElfError::BadLoadAddress,
            // A truncated image / short program-header table is an I/O-shaped
            // failure the slice loader maps to its EOF variant.
            PlanError::Truncated => ElfError::UnexpectedEof,
            PlanError::Unordered => ElfError::Unstreamable,
        }
    }
}

/// Read exactly `buf.len()` bytes from `file` into `buf`.
fn read_exact(
    vm: &mut fat::DelugeVolumeManager,
    file: RawFile,
    buf: &mut [u8],
) -> Result<(), ElfError> {
    let mut pos = 0;
    while pos < buf.len() {
        let n = vm.read(file, &mut buf[pos..])?;
        if n == 0 {
            return Err(ElfError::UnexpectedEof);
        }
        pos += n;
    }
    Ok(())
}

/// Stream-load an ELF32 file from the SD card.
///
/// Parses program headers and processes all `PT_LOAD` segments.
///
/// - **SDRAM targets** (`0x0C000000–0x0EFFFFFF`): written to their final
///   addresses immediately (they cannot overwrite the bootloader).
/// - **SRAM targets** (`0x20020000–0x202FFFFF`): written to the SDRAM
///   staging area; the caller must invoke
///   [`crate::launcher::launch_via_trampoline`] to move them before
///   branching to the app.
///
/// Returns a [`LoadResult`] containing the entry point and any SRAM segment
/// descriptors that still need relocating.  The file cursor is moved
/// arbitrarily; the caller should close the file after this returns.
///
/// # Safety
/// Writes to physical RAM derived from ELF program headers.  Each
/// destination is validated before any write, but the caller must ensure no
/// live data occupies the SDRAM staging window (`0x0FD20000–0x0FFFFFFF`).
pub async unsafe fn load_from_sd_with_progress<F, Fut>(
    vm: &mut fat::DelugeVolumeManager,
    file: RawFile,
    mut on_progress: F,
) -> Result<LoadResult, ElfError>
where
    F: FnMut(u32, u32) -> Fut,
    Fut: core::future::Future<Output = ()>,
{
    // 1. Read and validate the 52-byte ELF header.
    let mut hdr = [0u8; 52];
    read_exact(vm, file, &mut hdr)?;

    if hdr[0..4] != ELF_MAGIC {
        return Err(ElfError::BadMagic);
    }
    if hdr[4] != ELFCLASS32 || hdr[5] != ELFDATA2LSB {
        return Err(ElfError::WrongFormat);
    }
    if le16(&hdr, 16) != ET_EXEC {
        return Err(ElfError::WrongFormat);
    }
    if le16(&hdr, 18) != EM_ARM {
        return Err(ElfError::WrongFormat);
    }

    let e_entry = le32(&hdr, 24);
    let e_phoff = le32(&hdr, 28);
    let e_phentsize = le16(&hdr, 42) as usize;
    let e_phnum = le16(&hdr, 44) as usize;

    if e_phentsize != 32 {
        return Err(ElfError::WrongFormat);
    }
    if e_phnum > MAX_PHDRS {
        return Err(ElfError::WrongFormat);
    }
    let phnum = e_phnum;

    // 2. Seek to and read all program headers into a stack buffer.
    //    MAX_PHDRS × 32 bytes = 256 bytes stack.
    vm.file_seek_from_start(file, e_phoff)?;
    let mut phdr_buf = [0u8; MAX_PHDRS * 32];
    read_exact(vm, file, &mut phdr_buf[..phnum * e_phentsize])?;

    // 3. Copy each PT_LOAD segment to its write destination.
    let mut chunk_buf = [0u8; CHUNK];

    let mut sram_descs = [SramSegDesc::default(); MAX_PHDRS];
    let mut n_sram: usize = 0;

    // Pre-compute total bytes that will be streamed from SD for true load
    // progress.  Use the shared `place_segment` decision so the set of segments
    // counted here can never drift from the set the copy loop below actually
    // streams: retention-RAM segments are `Skip`ped (not copied), everything else
    // is counted.  A placement error is left for the copy loop to surface; the
    // progress denominator it would have contributed to is irrelevant then.
    let mut total_bytes: u32 = 0;
    for i in 0..phnum {
        let ph = &phdr_buf[i * e_phentsize..][..32];
        if le32(ph, 0) != PT_LOAD {
            continue;
        }
        let p_paddr = le32(ph, 12);
        let p_memsz = le32(ph, 20);
        if let Ok(SegmentPlacement::Skip) = place_segment(p_paddr, p_memsz) {
            continue;
        }
        total_bytes = total_bytes.saturating_add(le32(ph, 16));
    }
    let mut copied_bytes: u32 = 0;
    let mut last_percent: u8 = if total_bytes == 0 { 100 } else { 0 };
    on_progress(copied_bytes, total_bytes).await;

    for i in 0..phnum {
        let ph = &phdr_buf[i * e_phentsize..][..32];

        if le32(ph, 0) != PT_LOAD {
            continue;
        }

        let p_offset = le32(ph, 4);
        let p_paddr = le32(ph, 12);
        let p_filesz = le32(ph, 16) as usize;
        let p_memsz = le32(ph, 20) as usize;

        // A well-formed PT_LOAD always has filesz <= memsz.
        if p_filesz > p_memsz {
            return Err(ElfError::WrongFormat);
        }

        // Classify and place the segment via the shared host-tested decision
        // (the same one `StreamRouter` and the USB dev-upload path use), so the
        // FAT loader and the streaming loaders can never drift on where a
        // segment may land:
        //   * data-retention RAM (0x20000000-0x2001FFFF) is skipped — reserved
        //     for the trampoline; the app's own startup zeroes any BSS there;
        //   * SDRAM targets are written through `p_paddr` (so a segment that
        //     asked for the uncached mirror still lands there);
        //   * SRAM targets are staged in SDRAM, keyed by the physical offset.
        let (write_addr, is_sram) =
            match place_segment(p_paddr, p_memsz as u32).map_err(|_| ElfError::BadLoadAddress)? {
                SegmentPlacement::Skip => continue,
                SegmentPlacement::Write { write_addr, sram } => (write_addr, sram),
            };

        vm.file_seek_from_start(file, p_offset)?;

        let mut remaining = p_filesz;
        let mut dst = write_addr as *mut u8;

        unsafe {
            while remaining > 0 {
                let want = remaining.min(CHUNK);
                let n = vm.read(file, &mut chunk_buf[..want])?;
                if n == 0 {
                    return Err(ElfError::UnexpectedEof);
                }
                core::ptr::copy_nonoverlapping(chunk_buf.as_ptr(), dst, n);
                dst = dst.add(n);
                remaining -= n;

                copied_bytes = copied_bytes.saturating_add(n as u32);
                let percent = copied_bytes
                    .saturating_mul(100)
                    .checked_div(total_bytes)
                    .unwrap_or(100) as u8;
                if percent != last_percent {
                    on_progress(copied_bytes, total_bytes).await;
                    last_percent = percent;
                    // Let other tasks keep running while large images stream.
                    Timer::after(Duration::from_millis(0)).await;
                }
            }

            // Zero the BSS-like gap between filesz and memsz.
            if p_memsz > p_filesz {
                core::ptr::write_bytes(dst, 0, p_memsz - p_filesz);
            }
        }

        // Record SRAM segments so the caller can drive the trampoline.
        if is_sram {
            sram_descs[n_sram] = SramSegDesc {
                src: write_addr,
                dst: p_paddr,
                filesz: p_filesz as u32,
                zero_extra: (p_memsz - p_filesz) as u32,
            };
            n_sram += 1;
        }
    }

    if last_percent < 100 {
        on_progress(total_bytes, total_bytes).await;
    }

    Ok(LoadResult {
        entry: e_entry,
        sram_descs,
        n_sram,
    })
}

/// A flat firmware image staged in the SDRAM staging window, ready to be
/// programmed into the flash app slot.
pub struct FlashStage {
    /// Pointer to image byte 0 (the vector table) inside the SDRAM staging
    /// window — re-based past any leading NOLOAD region by [`find_fsb_base`].
    pub ptr: *const u8,
    /// Flattened image length in bytes, from the vector table to the highest
    /// SRAM `PT_LOAD` LMA end.
    pub len: usize,
    /// Load address (LMA) of image byte 0 — the vector table base, which equals
    /// the FSB `code_start` metadata word.
    pub code_start: u32,
}

/// Stream an ELF32 file from the SD card and flatten its `PT_LOAD` segments into
/// a contiguous raw image in the SDRAM staging window — the same `.bin` layout
/// `objcopy -O binary` produces, which the on-flash boot path
/// ([`crate::flashboot`]) expects.
///
/// Only firmware that is **fully SRAM-linked** can be stored to flash: the
/// flash-boot path reconstructs the image with a single trampoline descriptor
/// (`slot -> code_start`), so a segment outside upper SRAM (e.g. an SDRAM
/// segment) cannot be carried and is rejected with [`ElfError::NotFlashable`].
///
/// Inter-segment gaps are zero-filled to match the host flattener. The returned
/// [`FlashStage`] points into the staging window; the caller programs it into
/// the slot (see [`crate::flashboot::store_image_to_slot`]).
///
/// # Safety
/// Writes the SDRAM staging window (`0x0FD20000+`); the caller must ensure no
/// live data occupies it (true during the boot menu, like the SD ELF loader).
pub async unsafe fn flatten_to_flash_staging<F, Fut>(
    vm: &mut fat::DelugeVolumeManager,
    file: RawFile,
    mut on_progress: F,
) -> Result<FlashStage, ElfError>
where
    F: FnMut(u32, u32) -> Fut,
    Fut: core::future::Future<Output = ()>,
{
    // 1. Read and validate the 52-byte ELF header (same checks as the loader).
    let mut hdr = [0u8; 52];
    read_exact(vm, file, &mut hdr)?;
    if hdr[0..4] != ELF_MAGIC {
        return Err(ElfError::BadMagic);
    }
    if hdr[4] != ELFCLASS32 || hdr[5] != ELFDATA2LSB {
        return Err(ElfError::WrongFormat);
    }
    if le16(&hdr, 16) != ET_EXEC || le16(&hdr, 18) != EM_ARM {
        return Err(ElfError::WrongFormat);
    }

    let e_phoff = le32(&hdr, 28);
    let e_phentsize = le16(&hdr, 42) as usize;
    let e_phnum = le16(&hdr, 44) as usize;
    if e_phentsize != 32 || e_phnum > MAX_PHDRS {
        return Err(ElfError::WrongFormat);
    }

    // 2. Read all program headers.
    vm.file_seek_from_start(file, e_phoff)?;
    let mut phdr_buf = [0u8; MAX_PHDRS * 32];
    read_exact(vm, file, &mut phdr_buf[..e_phnum * 32])?;

    // 3. First pass: classify every loadable segment, require SRAM, and compute
    //    the flattened image bounds [lo, hi) plus the total bytes to stream.
    let mut lo = u32::MAX;
    let mut hi = 0u32;
    let mut total_bytes = 0u32;
    let mut have_content = false;
    for i in 0..e_phnum {
        let ph = &phdr_buf[i * 32..][..32];
        if le32(ph, 0) != PT_LOAD {
            continue;
        }
        let p_filesz = le32(ph, 16);
        if p_filesz == 0 {
            continue; // .bss-only segment: nothing to flash.
        }
        let p_phys = mirror_to_phys(le32(ph, 12));
        if classify_load_range(p_phys, p_filesz) != Some(LoadTarget::Sram) {
            return Err(ElfError::NotFlashable);
        }
        lo = lo.min(p_phys);
        hi = hi.max(p_phys + p_filesz);
        total_bytes = total_bytes.saturating_add(p_filesz);
        have_content = true;
    }
    if !have_content {
        return Err(ElfError::WrongFormat);
    }

    let image_len = (hi - lo) as usize;
    if image_len as u32 > flash::SLOT_LEN {
        return Err(ElfError::TooLarge);
    }

    // Image byte 0 lands at the staging address of the lowest LMA.
    let stage_base = sram_stage_addr(lo);
    // Zero the staging image so inter-segment gaps read as 0x00, matching
    // `objcopy -O binary` (the SDRAM window may hold stale bytes).
    unsafe { core::ptr::write_bytes(stage_base as *mut u8, 0, image_len) };

    // 4. Second pass: stream each segment's file bytes into staging.
    let mut chunk_buf = [0u8; CHUNK];
    let mut copied: u32 = 0;
    let mut last_percent: u8 = if total_bytes == 0 { 100 } else { 0 };
    on_progress(0, total_bytes).await;

    for i in 0..e_phnum {
        let ph = &phdr_buf[i * 32..][..32];
        if le32(ph, 0) != PT_LOAD {
            continue;
        }
        let p_offset = le32(ph, 4);
        let p_filesz = le32(ph, 16) as usize;
        if p_filesz == 0 {
            continue;
        }
        let p_phys = mirror_to_phys(le32(ph, 12));
        let mut dst = sram_stage_addr(p_phys) as *mut u8;

        vm.file_seek_from_start(file, p_offset)?;
        let mut remaining = p_filesz;
        unsafe {
            while remaining > 0 {
                let want = remaining.min(CHUNK);
                let n = vm.read(file, &mut chunk_buf[..want])?;
                if n == 0 {
                    return Err(ElfError::UnexpectedEof);
                }
                core::ptr::copy_nonoverlapping(chunk_buf.as_ptr(), dst, n);
                dst = dst.add(n);
                remaining -= n;

                copied = copied.saturating_add(n as u32);
                let percent = copied
                    .saturating_mul(100)
                    .checked_div(total_bytes)
                    .unwrap_or(100) as u8;
                if percent != last_percent {
                    on_progress(copied, total_bytes).await;
                    last_percent = percent;
                    Timer::after(Duration::from_millis(0)).await;
                }
            }
        }
    }

    if last_percent < 100 {
        on_progress(total_bytes, total_bytes).await;
    }

    // Re-base the flat image on the FSB vector table.
    //
    // The image is currently based at `lo`, the lowest PT_LOAD load address.
    // For SDK firmware (`rza1l-hal`'s startup) the vector table *is* the lowest
    // loadable address, so `base_off == 0` and nothing moves. A DelugeFirmware-
    // style image instead places a leading NOLOAD region (MMU TTB + mode stacks)
    // at a lower address than the vector table inside the same PT_LOAD, so byte 0
    // here is that zero-filled region and the metadata/signature the boot path
    // reads at `+0x20`/`+0x2C` would land deep in the image. Anchor on the
    // signature and drop the leading region so byte 0 is the vector table again —
    // the same layout `objcopy -O binary` (section-based) produces, and what the
    // on-flash boot path (`flashboot`) and `validate_fsb_metadata` expect. If the
    // image carries no signature, leave it unshifted so the store path's
    // validation reports the precise `BadSignature` rejection.
    let staged = unsafe { core::slice::from_raw_parts(stage_base as *const u8, image_len) };
    let base_off = find_fsb_base(staged).unwrap_or(0);

    Ok(FlashStage {
        ptr: (stage_base + base_off as u32) as *const u8,
        len: image_len - base_off,
        code_start: lo + base_off as u32,
    })
}
