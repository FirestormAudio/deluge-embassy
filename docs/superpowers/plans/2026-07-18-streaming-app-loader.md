# Streaming USB app-loader + raised SDRAM ceiling — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let a `platform`-profile launcher image boot from `/APPS/` by raising the app-loader's SDRAM ceiling, funded by rewriting the USB dev-upload path to stream segments straight to memory (deleting its 12 MB scratch buffer).

**Architecture:** The device app-loader classifies every `PT_LOAD` segment through one pure, host-tested function (`place_segment`) against constants in `deluge-image`. Move the SRAM staging window to the top of SDRAM and raise `SDRAM_HI` into the space freed by deleting the USB scratch. The USB receive path changes from *buffer-whole-ELF-then-copy* to a sequential router (`StreamRouter`) that writes each segment to its destination as bytes arrive, checksums incrementally, and verifies before handoff. The host packer's `memmap.py` ceiling is corrected to match.

**Tech Stack:** Rust (`no_std` `deluge-image`, `app-loader` firmware, `cargo-deluge` host tool), Python (`deluge_mkimage`), `cargo test` (host), `pytest` (mkimage), JTAG/J-Link (on-device).

## Global Constraints

- **Two repos.** `deluge-sdk` (this repo, branch `feat/streaming-app-loader`): `crates/deluge-image`, `app-loader`, `tools/cargo-deluge`. `deluge-linux-sdk` (`~/GitHub/deluge-linux-sdk`): `tools/deluge_mkimage/memmap.py`.
- **The device ceiling and host ceiling MUST stay equal:** `deluge-image` `SDRAM_HI` == `memmap.py` `SDRAM_END` == `0x0FD20000`.
- **Ceiling is derived, not hand-picked:** `SDRAM_STAGE_BASE = 0x10000000 - (SRAM_HI - SRAM_LOAD_ORIGIN)`; `SDRAM_HI = SDRAM_STAGE_BASE`. Result `0x0FD20000`.
- **Wire protocol is unchanged:** `magic b"DLUP" | version u8 | flags u8 | len u32 | crc32 u32 | <len ELF bytes>`, CRC over the whole payload.
- **No `deluge-linux` / U-Boot / base-bundle change.** Rootfs load address `0x0E800000` and manifest `max_size = 0x1000000` are untouched.
- **Streaming assumption:** uploaded ELFs have the program-header table in the first `HEADER_BUF` (308) bytes and `PT_LOAD` segments in non-decreasing, non-overlapping `p_offset`. Violations are rejected (`BAD LAYOUT`), never mis-loaded.
- **⚠️ Requires re-flashing the app-loader firmware** (Task 9) for any device-visible effect. `llvm-objcopy --strip-all` output (what `cargo deluge run` sends) satisfies the streaming assumption.
- DRY, YAGNI, TDD, frequent commits. Host-testable logic lives in `deluge-image` with unit tests; device wiring is verified by build + on-device.

---

### Task 1: Raise the SDRAM ceiling constants

**Files:**
- Modify: `crates/deluge-image/src/elf.rs:28-45` (constants block) and its doc comment on `SDRAM_LO`
- Test: `crates/deluge-image/src/elf.rs` (`#[cfg(test)] mod tests`)

**Interfaces:**
- Produces: `pub const SDRAM_HI: u32 = 0x0FD2_0000`, `pub const SDRAM_STAGE_BASE: u32 = 0x0FD2_0000` (values change; names/types unchanged). All existing consumers (`classify_load_range`, `place_segment`, `sram_stage_addr`) are unchanged.

- [ ] **Step 1: Write the failing test**

Add to the `mod tests` block in `crates/deluge-image/src/elf.rs`:

```rust
#[test]
fn sdram_ceiling_is_staging_base_at_top_of_sdram() {
    // The app-segment ceiling equals the staging base, and the staging window
    // (= the on-chip SRAM app-region size) sits flush against the top of SDRAM.
    assert_eq!(SDRAM_HI, 0x0FD2_0000);
    assert_eq!(SDRAM_HI, SDRAM_STAGE_BASE);
    assert_eq!(SDRAM_STAGE_BASE + (SRAM_HI - SRAM_LOAD_ORIGIN), 0x1000_0000);
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cargo test -p deluge-image sdram_ceiling_is_staging_base_at_top_of_sdram`
Expected: FAIL — `assertion \`left == right\` failed: left: 0xf000000, right: 0xfd20000`.

- [ ] **Step 3: Replace the constants block**

In `crates/deluge-image/src/elf.rs`, replace the region-geometry constants (currently `SDRAM_LO`, `SDRAM_HI`, `SRAM_LOAD_ORIGIN`, `SRAM_HI`, `SDRAM_STAGE_BASE`) with:

```rust
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
```

- [ ] **Step 4: Run the full crate test suite to verify it passes**

Run: `cargo test -p deluge-image`
Expected: PASS (all tests, including the pre-existing `staging_address_math` and `classify_*` tests, which are written relative to these constants).

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-image/src/elf.rs
git commit -m "feat(deluge-image): raise SDRAM app ceiling to 0x0FD20000 (staging at top)"
```

---

### Task 2: Incremental CRC-32

**Files:**
- Modify: `crates/deluge-image/src/crc.rs` (add `Crc32`, re-express `crc32`)
- Modify: `crates/deluge-image/src/lib.rs:20` (export `Crc32`)
- Test: `crates/deluge-image/src/crc.rs` (`mod tests`)

**Interfaces:**
- Produces: `pub struct Crc32`; `Crc32::new() -> Crc32`; `Crc32::update(&mut self, data: &[u8])`; `Crc32::finalize(&self) -> u32`. `crc32(data)` is unchanged in signature and value.

- [ ] **Step 1: Write the failing test**

Add to `mod tests` in `crates/deluge-image/src/crc.rs`:

```rust
#[test]
fn streaming_matches_one_shot() {
    use super::Crc32;
    let data = b"The quick brown fox jumps over the lazy dog";
    let mut c = Crc32::new();
    c.update(&data[..10]);
    c.update(&data[10..20]);
    c.update(&data[20..]);
    assert_eq!(c.finalize(), crc32(data));
    assert_eq!(c.finalize(), 0x414F_A339);
    assert_eq!(Crc32::new().finalize(), 0x0000_0000);
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cargo test -p deluge-image streaming_matches_one_shot`
Expected: FAIL to compile — `cannot find type \`Crc32\``.

- [ ] **Step 3: Add `Crc32` and re-express `crc32`**

In `crates/deluge-image/src/crc.rs`, replace the `crc32` function with:

```rust
/// Incremental CRC-32 (IEEE) accumulator, so a stream can be checksummed without
/// buffering it: `Crc32::new().update(a).update(b).finalize()` equals
/// `crc32(a followed by b)`.
#[derive(Clone, Copy)]
pub struct Crc32(u32);

impl Crc32 {
    /// Start a fresh CRC accumulation.
    pub fn new() -> Self {
        Crc32(0xFFFF_FFFF)
    }

    /// Fold `data` into the running CRC.
    pub fn update(&mut self, data: &[u8]) {
        let mut crc = self.0;
        for &byte in data {
            crc ^= byte as u32;
            let mut bit = 0;
            while bit < 8 {
                // Branch-free: `mask` is all-ones when the low bit is set.
                let mask = (crc & 1).wrapping_neg();
                crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
                bit += 1;
            }
        }
        self.0 = crc;
    }

    /// Finish and return the CRC-32 value (does not consume the accumulator).
    pub fn finalize(&self) -> u32 {
        !self.0
    }
}

impl Default for Crc32 {
    fn default() -> Self {
        Self::new()
    }
}

/// Compute the CRC-32 (IEEE) of `data`.
pub fn crc32(data: &[u8]) -> u32 {
    let mut c = Crc32::new();
    c.update(data);
    c.finalize()
}
```

- [ ] **Step 4: Export `Crc32`**

In `crates/deluge-image/src/lib.rs`, change the CRC re-export line:

```rust
pub use crc::{Crc32, crc32};
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `cargo test -p deluge-image crc`
Expected: PASS (`known_vectors`, `single_byte_difference_changes_crc`, `streaming_matches_one_shot`).

- [ ] **Step 6: Commit**

```bash
git add crates/deluge-image/src/crc.rs crates/deluge-image/src/lib.rs
git commit -m "feat(deluge-image): add incremental Crc32 accumulator"
```

---

### Task 3: Add `ElfError::Unstreamable` + `BAD LAYOUT` OLED string

**Files:**
- Modify: `app-loader/src/elf.rs:49-70` (`ElfError` enum)
- Modify: `app-loader/src/devupload.rs:198-203` (OLED match)
- Modify: `app-loader/src/main.rs:639-642` (SD-path OLED match, defensive)

**Interfaces:**
- Produces: `ElfError::Unstreamable` variant. Consumed by Task 4's `From<PlanError>` arm and Task 5's receive loop.

- [ ] **Step 1: Add the enum variant**

In `app-loader/src/elf.rs`, inside `pub enum ElfError`, add after the `BadLoadAddress` variant:

```rust
    /// The uploaded ELF cannot be stream-loaded: its `PT_LOAD` segments are not
    /// in non-decreasing, non-overlapping file order, or its program-header
    /// table is not within the streamed header prefix.
    Unstreamable,
```

- [ ] **Step 2: Add the OLED arm on the USB path**

In `app-loader/src/devupload.rs`, in the `match e` block that maps `elf::ElfError` to a second OLED line (currently ending `elf::ElfError::BadLoadAddress => b"BAD LOAD ADDR"`), add:

```rust
                    elf::ElfError::Unstreamable => b"BAD LAYOUT",
```

- [ ] **Step 3: Add the OLED arm on the SD path (defensive)**

In `app-loader/src/main.rs`, in the SD ELF-load `match e` block (the one with `elf::ElfError::BadLoadAddress => b"BAD LOAD ADDR"` near line 641), add the same arm above the `_ => b"SEE LOG"` fallback:

```rust
                    elf::ElfError::Unstreamable => b"BAD LAYOUT",
```

- [ ] **Step 4: Build the app-loader to verify it compiles**

Run: `cargo build -p app-loader --release`
Expected: `Finished` (a new unused variant is not an error).

- [ ] **Step 5: Commit**

```bash
git add app-loader/src/elf.rs app-loader/src/devupload.rs app-loader/src/main.rs
git commit -m "feat(app-loader): add Unstreamable error + BAD LAYOUT OLED string"
```

---

### Task 4: `StreamRouter` in deluge-image

**Files:**
- Modify: `crates/deluge-image/src/elf.rs` (add `RoutedSeg`, `RouteStep`, `StreamRouter`; add `PlanError::Unordered`)
- Modify: `app-loader/src/elf.rs:80-92` (`From<PlanError>` match — keep the workspace exhaustive)
- Test: `crates/deluge-image/src/elf.rs` (`mod tests`)

**Interfaces:**
- Consumes: `validate_header`, `le16`, `le32`, `place_segment`, `SegmentPlacement`, `PT_LOAD`, `MAX_PHDRS` (existing).
- Produces:
  - `pub struct RoutedSeg { pub file_start: u32, pub file_end: u32, pub write_addr: u32, pub final_dst: u32, pub memsz: u32, pub sram: bool }`
  - `pub struct RouteStep { pub dst: Option<u32>, pub run: u32 }`
  - `StreamRouter::new(buf: &[u8]) -> Result<StreamRouter, PlanError>`
  - `StreamRouter::entry(&self) -> u32`, `header_end(&self) -> u32`, `segments(&self) -> &[RoutedSeg]`, `route_at(&self, off: u32) -> RouteStep`
  - `PlanError::Unordered`

- [ ] **Step 1: Write the failing tests**

Add to `mod tests` in `crates/deluge-image/src/elf.rs`:

```rust
/// Build an ELF32 header + program-header table (no segment bodies) for router
/// tests. Each phdr tuple is `(p_type, p_offset, p_paddr, p_filesz, p_memsz)`.
fn elf_front(entry: u32, phdrs: &[(u32, u32, u32, u32, u32)]) -> Vec<u8> {
    let e_phoff = 52u32;
    let mut buf = vec![0u8; 52 + phdrs.len() * 32];
    buf[0..4].copy_from_slice(&[0x7F, b'E', b'L', b'F']);
    buf[4] = 1; // ELFCLASS32
    buf[5] = 1; // ELFDATA2LSB
    buf[16..18].copy_from_slice(&2u16.to_le_bytes()); // ET_EXEC
    buf[18..20].copy_from_slice(&0x28u16.to_le_bytes()); // EM_ARM
    buf[24..28].copy_from_slice(&entry.to_le_bytes());
    buf[28..32].copy_from_slice(&e_phoff.to_le_bytes());
    buf[42..44].copy_from_slice(&32u16.to_le_bytes()); // e_phentsize
    buf[44..46].copy_from_slice(&(phdrs.len() as u16).to_le_bytes());
    for (i, &(t, off, paddr, filesz, memsz)) in phdrs.iter().enumerate() {
        let p = 52 + i * 32;
        buf[p..p + 4].copy_from_slice(&t.to_le_bytes());
        buf[p + 4..p + 8].copy_from_slice(&off.to_le_bytes());
        buf[p + 12..p + 16].copy_from_slice(&paddr.to_le_bytes());
        buf[p + 16..p + 20].copy_from_slice(&filesz.to_le_bytes());
        buf[p + 20..p + 24].copy_from_slice(&memsz.to_le_bytes());
    }
    buf
}

#[test]
fn router_routes_sdram_and_stages_sram() {
    // One SDRAM segment (p_offset 0, covering the header) and one SRAM segment.
    let front = elf_front(
        SDRAM_LO,
        &[
            (PT_LOAD, 0, SDRAM_LO, 0x200, 0x200),
            (PT_LOAD, 0x200, SRAM_LOAD_ORIGIN, 0x40, 0x80),
        ],
    );
    let r = StreamRouter::new(&front).unwrap();
    assert_eq!(r.entry(), SDRAM_LO);
    assert_eq!(r.header_end(), 52 + 2 * 32);

    // A byte at file offset 0 lands at the SDRAM segment's paddr.
    assert_eq!(r.route_at(0), RouteStep { dst: Some(SDRAM_LO), run: 0x200 });
    // Offset 0x100 is 0x100 into that segment.
    assert_eq!(r.route_at(0x100), RouteStep { dst: Some(SDRAM_LO + 0x100), run: 0x100 });
    // The SRAM segment routes to the staging window, not its final SRAM address.
    assert_eq!(
        r.route_at(0x200),
        RouteStep { dst: Some(sram_stage_addr(SRAM_LOAD_ORIGIN)), run: 0x40 }
    );
    // Its recorded final destination is the SRAM address for the trampoline.
    let sram = r.segments().iter().find(|s| s.sram).unwrap();
    assert_eq!(sram.final_dst, SRAM_LOAD_ORIGIN);
    assert_eq!(sram.memsz, 0x80);
    // Past the last file byte: discard to the end.
    assert_eq!(r.route_at(0x240), RouteStep { dst: None, run: u32::MAX });
}

#[test]
fn router_discards_gaps_between_segments() {
    let front = elf_front(
        SDRAM_LO,
        &[
            (PT_LOAD, 0x100, SDRAM_LO, 0x40, 0x40),
            (PT_LOAD, 0x200, SDRAM_LO + 0x1000, 0x40, 0x40),
        ],
    );
    let r = StreamRouter::new(&front).unwrap();
    // Header/gap before the first segment is discarded up to its start.
    assert_eq!(r.route_at(0), RouteStep { dst: None, run: 0x100 });
    // Gap between the two segments (0x140..0x200) is discarded.
    assert_eq!(r.route_at(0x140), RouteStep { dst: None, run: 0x0C0 });
}

#[test]
fn router_rejects_backward_segments() {
    // Second segment starts before the first one's file bytes end.
    let front = elf_front(
        SDRAM_LO,
        &[
            (PT_LOAD, 0x200, SDRAM_LO, 0x80, 0x80),
            (PT_LOAD, 0x100, SDRAM_LO + 0x1000, 0x40, 0x40),
        ],
    );
    assert_eq!(StreamRouter::new(&front), Err(PlanError::Unordered));
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cargo test -p deluge-image router_`
Expected: FAIL to compile — `cannot find type \`StreamRouter\`` / `RouteStep` / `no variant Unordered`.

- [ ] **Step 3: Add `PlanError::Unordered`**

In `crates/deluge-image/src/elf.rs`, add to `pub enum PlanError`:

```rust
    /// `PT_LOAD` segments are not in non-decreasing, non-overlapping file order,
    /// so the streaming loader cannot place them without seeking backward.
    Unordered,
```

- [ ] **Step 4: Add the `StreamRouter` and its types**

Append to `crates/deluge-image/src/elf.rs` (after the `place_segment` / `BadLoadAddress` block):

```rust
// --- Streaming load router (USB dev-upload path) ----------------------------

/// One `PT_LOAD` segment resolved for streaming: where its file bytes sit in the
/// upload and where they are written.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct RoutedSeg {
    /// File offset of the segment's first byte (`p_offset`).
    pub file_start: u32,
    /// File offset one past the segment's last file byte (`p_offset + p_filesz`).
    pub file_end: u32,
    /// Address the loader copies the file bytes to now: the final SDRAM address,
    /// or the SDRAM staging address for an SRAM-targeting segment.
    pub write_addr: u32,
    /// Final runtime address (`p_paddr`) — the SRAM destination for a staged
    /// segment; equals `write_addr` for an SDRAM segment.
    pub final_dst: u32,
    /// In-memory size (`p_memsz`); `memsz - (file_end - file_start)` bytes are
    /// zeroed after the copy.
    pub memsz: u32,
    /// `true` if staged in SDRAM for later SRAM relocation.
    pub sram: bool,
}

/// How the streaming loader treats the upload byte at a given file offset: copy
/// it (and the following `run` bytes, up to the next segment boundary) to `dst`,
/// or discard them when `dst` is `None`.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct RouteStep {
    /// `Some(addr)` to copy the run to `addr`; `None` to discard it.
    pub dst: Option<u32>,
    /// Bytes until the next routing boundary. `u32::MAX` past the last segment.
    pub run: u32,
}

/// A validated, streamable plan for a USB-uploaded ELF: the ordered `PT_LOAD`
/// segments plus the entry point, built from the image's front matter (ELF header
/// + program-header table). The device routes the byte stream through
/// [`StreamRouter::route_at`] without ever seeking backward — the host-testable
/// core of the streaming dev-upload loader.
pub struct StreamRouter {
    entry: u32,
    header_end: u32,
    segs: [RoutedSeg; MAX_PHDRS],
    n_segs: usize,
}

impl StreamRouter {
    /// Parse and validate the front matter (`buf` must hold at least the 52-byte
    /// ELF header and the whole program-header table). Produces the ordered,
    /// non-overlapping route table, or a [`PlanError`].
    pub fn new(buf: &[u8]) -> Result<StreamRouter, PlanError> {
        validate_header(buf)?;

        let entry = le32(buf, 24);
        let e_phoff = le32(buf, 28);
        let e_phentsize = le16(buf, 42) as usize;
        let e_phnum = le16(buf, 44) as usize;

        if e_phentsize != 32 || e_phnum > MAX_PHDRS || e_phoff < 52 {
            return Err(PlanError::WrongFormat);
        }
        let header_end = e_phoff
            .checked_add(
                (e_phnum as u32)
                    .checked_mul(32)
                    .ok_or(PlanError::WrongFormat)?,
            )
            .ok_or(PlanError::WrongFormat)?;
        if header_end as usize > buf.len() {
            return Err(PlanError::Truncated);
        }

        let mut segs = [RoutedSeg {
            file_start: 0,
            file_end: 0,
            write_addr: 0,
            final_dst: 0,
            memsz: 0,
            sram: false,
        }; MAX_PHDRS];
        let mut n_segs = 0usize;
        let mut prev_file_end = 0u32;

        for i in 0..e_phnum {
            let ph = &buf[e_phoff as usize + i * 32..][..32];
            if le32(ph, 0) != PT_LOAD {
                continue;
            }
            let p_offset = le32(ph, 4);
            let p_paddr = le32(ph, 12);
            let p_filesz = le32(ph, 16);
            let p_memsz = le32(ph, 20);

            if p_filesz > p_memsz {
                return Err(PlanError::WrongFormat);
            }
            let file_end = p_offset.checked_add(p_filesz).ok_or(PlanError::WrongFormat)?;

            // Streaming cannot seek backward: segments must arrive in
            // non-decreasing, non-overlapping file order.
            if p_offset < prev_file_end {
                return Err(PlanError::Unordered);
            }
            prev_file_end = file_end;

            match place_segment(p_paddr, p_memsz).map_err(|_| PlanError::BadLoadAddress)? {
                SegmentPlacement::Skip => continue,
                SegmentPlacement::Write { write_addr, sram } => {
                    segs[n_segs] = RoutedSeg {
                        file_start: p_offset,
                        file_end,
                        write_addr,
                        final_dst: p_paddr,
                        memsz: p_memsz,
                        sram,
                    };
                    n_segs += 1;
                }
            }
        }

        Ok(StreamRouter {
            entry,
            header_end,
            segs,
            n_segs,
        })
    }

    /// Application entry point (`e_entry`).
    pub fn entry(&self) -> u32 {
        self.entry
    }

    /// Bytes of front matter (ELF header + program-header table) the caller must
    /// buffer to build this router.
    pub fn header_end(&self) -> u32 {
        self.header_end
    }

    /// The resolved segments, in file order.
    pub fn segments(&self) -> &[RoutedSeg] {
        &self.segs[..self.n_segs]
    }

    /// Route the upload byte at file offset `off`: the address it (and the next
    /// `run` bytes, up to a segment boundary) copies to, or discard if `dst` is
    /// `None`. The caller advances `off` by `min(run, chunk_remaining)`.
    pub fn route_at(&self, off: u32) -> RouteStep {
        for seg in &self.segs[..self.n_segs] {
            if off < seg.file_start {
                return RouteStep {
                    dst: None,
                    run: seg.file_start - off,
                };
            }
            if off < seg.file_end {
                return RouteStep {
                    dst: Some(seg.write_addr + (off - seg.file_start)),
                    run: seg.file_end - off,
                };
            }
        }
        RouteStep {
            dst: None,
            run: u32::MAX,
        }
    }
}
```

- [ ] **Step 5: Keep the app-loader `From<PlanError>` exhaustive**

In `app-loader/src/elf.rs`, in `impl From<PlanError> for ElfError`, add an arm (before or after the `Truncated` arm):

```rust
            PlanError::Unordered => ElfError::Unstreamable,
```

- [ ] **Step 6: Run the router tests and build the workspace**

Run: `cargo test -p deluge-image router_`
Expected: PASS (all three router tests).

Run: `cargo build -p app-loader --release`
Expected: `Finished` (the `From` match is exhaustive again).

- [ ] **Step 7: Commit**

```bash
git add crates/deluge-image/src/elf.rs app-loader/src/elf.rs
git commit -m "feat(deluge-image): add StreamRouter for the USB dev-upload path"
```

---

### Task 5: Rewrite the USB receive path to stream

**Files:**
- Modify: `app-loader/src/devupload.rs` (imports, remove `SCRATCH_*`, rewrite `receive`, add `route_bytes` + `PacketReader::next_chunk`, remove `read_to_ptr`)
- Modify: `app-loader/src/elf.rs` (remove `load_from_slice`; drop `parse_load_plan` from the `use` list)

**Interfaces:**
- Consumes: `deluge_image::Crc32`; `deluge_image::elf::{StreamRouter, RouteStep, le16, le32, MAX_PHDRS, SDRAM_HI, SDRAM_LO}`; `elf::{LoadResult, SramSegDesc}`; existing `PacketReader::{read_exact, sync_to_magic, byte, refill}`; `handoff`.
- Produces: a streaming `receive` that never allocates a whole-image buffer.

- [ ] **Step 1: Update imports and constants in `devupload.rs`**

At the top of `app-loader/src/devupload.rs`, add to the `use` items:

```rust
use deluge_image::Crc32;
use deluge_image::elf::{MAX_PHDRS, SDRAM_HI, SDRAM_LO, StreamRouter};
```

Also fix the stale module doc comment at the top of `devupload.rs` (the lines describing "streamed into a high-SDRAM scratch window ... handed to `load_from_slice`"): replace that sentence with one describing the streaming behavior, e.g. *"The image is streamed straight to each `PT_LOAD` segment's load / SRAM-staging address as it arrives, checksummed incrementally, and launched once the CRC verifies."*

Remove the `SCRATCH_ADDR` and `SCRATCH_LEN` constants and their comment block. Add:

```rust
// ── Streaming loader window ───────────────────────────────────────────────────
//
// The upload is routed straight to each segment's load / SRAM-staging address as
// it arrives — there is no whole-image scratch buffer. Only the front matter
// (ELF header + program-header table) is buffered, and the app-segment ceiling
// bounds a sane maximum upload length.
/// Bytes of front matter buffered to build the [`StreamRouter`]: ELF header plus
/// the largest program-header table.
const HEADER_BUF: usize = 52 + MAX_PHDRS * 32;
/// Largest upload accepted (the SDRAM app-region span). Per-segment placement is
/// still validated; this only rejects absurd lengths early.
const MAX_UPLOAD: u32 = SDRAM_HI - SDRAM_LO;
```

- [ ] **Step 2: Add the `route_bytes` helper**

Add near the top-level functions in `app-loader/src/devupload.rs`:

```rust
/// Copy a contiguous run of upload bytes starting at file offset `off` to their
/// segment destinations, discarding bytes that fall in gaps.
///
/// # Safety
/// Writes physical RAM at addresses the [`StreamRouter`] validated via
/// `place_segment`; the caller must ensure no live data occupies those regions
/// (true during the boot menu, like the SD loader).
unsafe fn route_bytes(router: &StreamRouter, mut off: u32, mut data: &[u8]) {
    while !data.is_empty() {
        let step = router.route_at(off);
        let take = (step.run as usize).min(data.len());
        if let Some(dst) = step.dst {
            unsafe { core::ptr::copy_nonoverlapping(data.as_ptr(), dst as *mut u8, take) };
        }
        off += take as u32;
        data = &data[take..];
    }
}
```

- [ ] **Step 3: Replace `PacketReader::read_to_ptr` with `next_chunk`**

In `app-loader/src/devupload.rs`, delete the `read_to_ptr` method and add:

```rust
    /// Borrow the next run of received bytes (up to `remaining`), refilling from
    /// USB as needed. Never returns empty until the caller has taken `remaining`
    /// bytes across calls.
    async fn next_chunk(&mut self, remaining: usize) -> &[u8] {
        if self.pos >= self.fill {
            self.refill().await;
        }
        let take = (self.fill - self.pos).min(remaining);
        let chunk = &self.buf[self.pos..self.pos + take];
        self.pos += take;
        chunk
    }
```

- [ ] **Step 4: Rewrite `receive`**

In `app-loader/src/devupload.rs`, replace the body of the `loop` in `async fn receive` — from after `reader.read_exact(&mut tail).await;` parses `version/len/expect_crc` through the `elf::load_from_slice` match — with the streaming version. The full loop becomes:

```rust
    let mut reader = PacketReader::new(rx);
    loop {
        // Resync to the frame magic, then read the fixed header tail.
        reader.sync_to_magic().await;
        let mut tail = [0u8; HEADER_TAIL];
        reader.read_exact(&mut tail).await;
        let version = tail[0];
        let _flags = tail[1];
        let len = u32::from_le_bytes([tail[2], tail[3], tail[4], tail[5]]);
        let expect_crc = u32::from_le_bytes([tail[6], tail[7], tail[8], tail[9]]);

        if version != VERSION || len == 0 || len > MAX_UPLOAD {
            warn!("devupload: bad header (version={version}, len={len}); resyncing");
            continue;
        }

        ui::UPLOAD_ACTIVE.store(true, Ordering::Release);
        info!("devupload: streaming {len} byte image");

        let mut crc = Crc32::new();

        // 1. Buffer + CRC the front matter (ELF header + program headers).
        let mut prefix = [0u8; HEADER_BUF];
        reader.read_exact(&mut prefix[..52]).await;
        let e_phoff = deluge_image::elf::le32(&prefix, 28);
        let e_phnum = deluge_image::elf::le16(&prefix, 44) as usize;
        if e_phoff < 52 || e_phnum > MAX_PHDRS {
            warn!("devupload: bad phdr table (e_phoff={e_phoff}, e_phnum={e_phnum})");
            ui::show_message(b"UPLOAD ERROR", b"BAD LAYOUT").await;
            Timer::after(Duration::from_secs(2)).await;
            ui::UPLOAD_ACTIVE.store(false, Ordering::Release);
            continue;
        }
        let header_end = e_phoff as usize + e_phnum * 32;
        if header_end > HEADER_BUF || header_end as u32 > len {
            warn!("devupload: phdr table outside header window (header_end={header_end})");
            ui::show_message(b"UPLOAD ERROR", b"BAD LAYOUT").await;
            Timer::after(Duration::from_secs(2)).await;
            ui::UPLOAD_ACTIVE.store(false, Ordering::Release);
            continue;
        }
        if header_end > 52 {
            reader.read_exact(&mut prefix[52..header_end]).await;
        }
        crc.update(&prefix[..header_end]);

        // 2. Build + validate the route plan.
        let router = match StreamRouter::new(&prefix[..header_end]) {
            Ok(r) => r,
            Err(e) => {
                let line2: &[u8] = match elf::ElfError::from(e) {
                    elf::ElfError::BadMagic => b"BAD MAGIC",
                    elf::ElfError::WrongFormat => b"WRONG FORMAT",
                    elf::ElfError::BadLoadAddress => b"BAD LOAD ADDR",
                    elf::ElfError::Unstreamable => b"BAD LAYOUT",
                    _ => b"SEE LOG",
                };
                warn!("devupload: image rejected: {e:?}");
                ui::show_message(b"UPLOAD ERROR", line2).await;
                Timer::after(Duration::from_secs(2)).await;
                ui::UPLOAD_ACTIVE.store(false, Ordering::Release);
                continue;
            }
        };

        // 3. Replay the buffered front matter through the router (handles a
        //    first segment whose p_offset is 0), then stream the remainder.
        unsafe { route_bytes(&router, 0, &prefix[..header_end]) };
        let total = len as usize;
        let mut off = header_end as u32;
        let mut remaining = total - header_end;
        let mut last_pct = u8::MAX;
        while remaining > 0 {
            let chunk = reader.next_chunk(remaining).await;
            crc.update(chunk);
            unsafe { route_bytes(&router, off, chunk) };
            off += chunk.len() as u32;
            remaining -= chunk.len();
            let pct = (((total - remaining) as u64) * 100 / total as u64) as u8;
            if pct != last_pct {
                ui::show_progress(b"RECEIVING", pct).await;
                last_pct = pct;
            }
        }

        // 4. Verify integrity before anything irreversible.
        if crc.finalize() != expect_crc {
            warn!("devupload: CRC mismatch");
            ui::show_message(b"UPLOAD ERROR", b"BAD CRC").await;
            Timer::after(Duration::from_secs(2)).await;
            ui::UPLOAD_ACTIVE.store(false, Ordering::Release);
            continue;
        }

        // 5. Zero-extend BSS tails and collect SRAM descriptors for handoff.
        let mut sram_descs = [SramSegDesc::default(); MAX_PHDRS];
        let mut n_sram = 0usize;
        for seg in router.segments() {
            let filesz = seg.file_end - seg.file_start;
            let zero_extra = seg.memsz - filesz;
            if zero_extra > 0 {
                unsafe {
                    core::ptr::write_bytes(
                        (seg.write_addr + filesz) as *mut u8,
                        0,
                        zero_extra as usize,
                    )
                };
            }
            if seg.sram {
                sram_descs[n_sram] = SramSegDesc {
                    src: seg.write_addr,
                    dst: seg.final_dst,
                    filesz,
                    zero_extra,
                };
                n_sram += 1;
            }
        }

        let result = elf::LoadResult {
            entry: router.entry(),
            sram_descs,
            n_sram,
        };
        info!("devupload: image loaded, entry={:#010x}", result.entry);
        handoff(result).await
    }
```

Ensure `use crate::elf::SramSegDesc;` is available — it is re-exported from `crate::elf`; reference it as `elf::SramSegDesc` if not already imported by name. (The `use crate::{elf, launcher, ui};` at the top makes `elf::SramSegDesc`, `elf::LoadResult`, `elf::ElfError` reachable.) Adjust the two bare `SramSegDesc` uses above to `elf::SramSegDesc` if the compiler reports it unresolved.

- [ ] **Step 5: Remove `load_from_slice` and its import in `app-loader/src/elf.rs`**

In `app-loader/src/elf.rs`, delete the entire `pub unsafe fn load_from_slice(...)` function and its doc comment. In the `use deluge_image::elf::{ ... }` list, remove `parse_load_plan` (leave `PlanError`, `SegmentPlacement`, `place_segment`, `classify_load_range`, `mirror_to_phys`, `le16`, `le32`, `find_fsb_base`, `sram_stage_addr` as still used).

- [ ] **Step 6: Build the app-loader firmware**

Run: `cargo build -p app-loader --release`
Expected: `Finished`. If the compiler flags an unresolved `SramSegDesc`, prefix the two uses with `elf::`. If it warns `parse_load_plan` unused elsewhere, it is removed in Task 6.

- [ ] **Step 7: Commit**

```bash
git add app-loader/src/devupload.rs app-loader/src/elf.rs
git commit -m "feat(app-loader): stream USB uploads straight to load addresses (drop 12 MB scratch)"
```

---

### Task 6: Remove the dead slice-load plan from deluge-image

**Files:**
- Modify: `crates/deluge-image/src/elf.rs` (remove `parse_load_plan`, `LoadPlan`, `LoadOp`, and their tests)
- Test: existing `deluge-image` suite

**Interfaces:**
- Removes: `parse_load_plan`, `LoadPlan`, `LoadOp` (no remaining callers after Task 5). `PlanError` stays (used by `StreamRouter`). `place_segment`, `validate_header`, `classify_load_range` stay.

- [ ] **Step 1: Delete the slice-plan code**

In `crates/deluge-image/src/elf.rs`, delete: the `LoadOp` struct, the `LoadPlan` struct, and the `pub fn parse_load_plan(...)` function with their doc comments. Keep `PlanError` (and its `From<HeaderError>` impl).

- [ ] **Step 2: Delete the slice-plan tests**

In the `mod tests` block, delete the tests that reference `parse_load_plan` / `LoadOp` (e.g. `parse_load_plan(...).unwrap()`, `assert_eq!(parse_load_plan(&img), Err(...))`). Keep all `validate_header`, `classify_*`, `staging_address_math`, `place_segment`, `router_*`, and the Task 1 ceiling test.

Also update the crate doc comment in `crates/deluge-image/src/lib.rs` (it currently says the `elf` module provides "the slice-sourced load plan the USB dev-upload path uses"): change that clause to describe the streaming router, e.g. *"and the [`elf::StreamRouter`] that routes a USB-uploaded ELF's segments straight to memory as they arrive."*

- [ ] **Step 3: Build and test the whole workspace**

Run: `cargo test -p deluge-image`
Expected: PASS, no `unused`/`dead_code` warnings for the removed items.

Run: `cargo build -p app-loader --release`
Expected: `Finished`.

- [ ] **Step 4: Commit**

```bash
git add crates/deluge-image/src/elf.rs
git commit -m "refactor(deluge-image): remove slice load-plan superseded by StreamRouter"
```

---

### Task 7: Update the host uploader's size cap

**Files:**
- Modify: `tools/cargo-deluge/src/run.rs` (the `MAX_UPLOAD_BYTES` const + comment, near line 82)

**Interfaces:**
- Consumes: nothing new. Produces: an updated local pre-check message.

- [ ] **Step 1: Replace the constant and its comment**

In `tools/cargo-deluge/src/run.rs`, replace the `MAX_UPLOAD_BYTES` definition and its doc comment with:

```rust
/// The device streams uploads straight to each segment's load address — there is
/// no whole-image scratch window anymore — so the only real bound is that the
/// segments fit in the SDRAM app region (`SDRAM_LO..SDRAM_HI`, `0x0C000000..
/// 0x0FD20000`). Catch an absurd image locally with a useful message instead of
/// streaming megabytes only for the device to reject a segment.
const MAX_UPLOAD_BYTES: usize = 0x0FD2_0000 - 0x0C00_0000; // SDRAM app-region span
```

- [ ] **Step 2: Update the oversized-image error text**

In the same file, the `if bytes.len() > MAX_UPLOAD_BYTES` error message mentions "the loader's {MAX_UPLOAD_BYTES}-byte upload window". Change it to:

```rust
        return Err(format!(
            "image is {} bytes, larger than the SDRAM app region ({MAX_UPLOAD_BYTES} bytes) \
             — build with --release, or trim the app.",
            bytes.len(),
        ));
```

- [ ] **Step 3: Build the host tool**

Run: `cargo build -p cargo-deluge`
Expected: `Finished`.

- [ ] **Step 4: Commit**

```bash
git add tools/cargo-deluge/src/run.rs
git commit -m "chore(cargo-deluge): size the upload cap to the SDRAM app region"
```

---

### Task 8: Correct the host packer ceiling (deluge-linux-sdk)

**Files:**
- Modify: `~/GitHub/deluge-linux-sdk/tools/deluge_mkimage/memmap.py:15-16`
- Test: `~/GitHub/deluge-linux-sdk/tests/mkimage/test_elf.py`

**Interfaces:**
- Produces: `memmap.SDRAM_END = 0x0FD20000` (matches device `SDRAM_HI`).

> All commands in this task run in `~/GitHub/deluge-linux-sdk`. Commit to that repo (branch first if on `main`).

- [ ] **Step 1: Write the failing test**

Add to `~/GitHub/deluge-linux-sdk/tests/mkimage/test_elf.py`:

```python
def test_sdram_end_matches_device_ceiling():
    # Must equal the app-loader's SDRAM_HI in
    # deluge-sdk/crates/deluge-image/src/elf.rs; the top 2.875 MB is the SRAM
    # staging window, off-limits to app segments.
    assert memmap.SDRAM_END == 0x0FD20000


def test_rootfs_max_size_over_ceiling_is_rejected(tmp_path):
    # A rootfs at 0x0E800000 may be at most 0x0FD20000 - 0x0E800000 = 0x1520000;
    # one byte more must be rejected by memmap.load.
    import json
    doc = {
        "segments": [
            {"file": "rootfs.uimg", "load": "0x0E800000",
             "max_size": "0x1520001", "generated": True},
        ]
    }
    p = tmp_path / "map.json"
    p.write_text(json.dumps(doc))
    with pytest.raises(ValueError):
        memmap.load(p)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python3 -m pytest tests/mkimage/test_elf.py -q -k "sdram_end or over_ceiling"`
Expected: FAIL — `assert 0x10000000 == 0x0FD20000` (the `over_ceiling` test also fails: the oversized `max_size` is currently accepted).

- [ ] **Step 3: Fix the constant**

In `~/GitHub/deluge-linux-sdk/tools/deluge_mkimage/memmap.py`, replace lines 15-16:

```python
SDRAM_BASE = 0x0C000000
# App-segment ceiling — must equal the app-loader's SDRAM_HI
# (deluge-sdk/crates/deluge-image/src/elf.rs). The top 2.875 MB
# (0x0FD20000..0x10000000) is the loader's SRAM staging window, not app-usable.
SDRAM_END = 0x0FD20000
```

- [ ] **Step 4: Run the full mkimage suite**

Run: `python3 -m pytest tests/mkimage/ -q`
Expected: PASS. (If any pre-existing test pinned a segment near `0x0F000000..0x0FD20000` it may need its expected-address updated; the manifest's `rootfs.uimg` at `0x0E800000`/`max_size 0x1000000` stays valid.)

- [ ] **Step 5: Verify the real launcher image now validates end-to-end**

Run:
```bash
DELUGE_BASE=~/GitHub/deluge-linux/out/deluge-base-0.2.0 \
python3 tools/deluge-mkimage \
  ~/GitHub/deluge-linux-sdk/target/armv7-unknown-linux-musleabihf/release/launcher \
  --profile platform -o /tmp/LAUNCHER.ELF
```
Expected: `ALL CHECKS PASSED` including `rootfs.uimg end 0x0f0ac350 in SDRAM` (now under the `0x0FD20000` ceiling).

- [ ] **Step 6: Commit**

```bash
git add tools/deluge_mkimage/memmap.py tests/mkimage/test_elf.py
git commit -m "fix(mkimage): match device SDRAM ceiling (0x0FD20000), reject over-ceiling rootfs"
```

---

### Task 9: On-device verification (manual, JTAG)

**Files:** none (hardware validation).

**Interfaces:** consumes the built `app-loader` firmware and the packed launcher image.

> This is the only step that proves the fix on hardware. It cannot be automated here; do it deliberately and record the result.

- [ ] **Step 1: Flash the rebuilt app-loader firmware**

Build the app-loader (`cargo build -p app-loader --release`) and flash it over J-Link (see the JTAG workflow notes). The resident loader must be the new build — without this the device still enforces the old `0x0F000000` ceiling.

- [ ] **Step 2: Boot the platform launcher from SD**

Copy `/tmp/LAUNCHER.ELF` (from Task 8 Step 5) to the card's `/APPS/`, pick it from the boot menu.
Expected: it loads and boots (droplet → launcher UI). **No `BAD LOAD ADDR`.**

- [ ] **Step 3: Exercise the streaming USB path**

With DEV MODE on, from `deluge-sdk`:
```bash
cargo deluge run -p <a small example, e.g. snake>
```
Expected: OLED shows `RECEIVING` progress, then the example launches from RAM. Try one example with an SRAM-targeting segment to cover the staging path.

- [ ] **Step 4: Record the outcome**

Note the result (pass/fail, any OLED error) in the PR description. If `BAD LAYOUT` appears on a `cargo deluge run`, the stripped ELF's segments are out of file order — capture its `readelf -l` and open a follow-up for host-side canonicalization (explicitly out of scope here).

---

## Notes for the implementer

- **Run host tests from `deluge-sdk` root** for Rust (`cargo test -p deluge-image`) and from `deluge-linux-sdk` root for Python (`python3 -m pytest tests/mkimage/`).
- **Building `app-loader`** targets the RZ/A1L firmware profile; use the project's normal firmware build (`cargo build -p app-loader --release`). It is a compile-only gate — the behavioral proof is Task 9.
- If a step's code does not compile due to a name you cannot resolve, prefer the fully-qualified `deluge_image::elf::…` / `crate::elf::…` path over guessing an import; the interfaces above list the exact items each task adds.
