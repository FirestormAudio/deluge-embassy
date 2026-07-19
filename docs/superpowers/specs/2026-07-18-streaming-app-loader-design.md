# Streaming USB app-loader + raised SDRAM ceiling — design & spec

Rewrite the USB dev-upload path from *buffer-the-whole-ELF-then-load* into a
**sequential router** that streams each `PT_LOAD` segment straight to its load
(or SRAM-staging) address. That deletes the 12 MB SDRAM scratch window, which
frees the top of SDRAM so the app-segment ceiling can rise from `0x0F000000` to
`0x0FD20000` — growing the rootfs budget from **8 MB to 21.1 MB** and removing
the dev-upload size cap. The host packer's `memmap.py` is corrected to the same
ceiling so oversized images fail at build time instead of on the device.

> **Status:** design proposal — scoped, awaiting sign-off before planning.
> **Branch:** `feat/streaming-app-loader` (deluge-sdk).
> **Spans two repos:** `deluge-sdk` (deluge-image, app-loader, cargo-deluge) and
> `deluge-linux-sdk` (`tools/deluge_mkimage/memmap.py`). No `deluge-linux` /
> U-Boot / base-bundle change.

---

## 1. Motivation

Packaging the `launcher` app into a `platform`-profile image and booting it from
the SD card's `/APPS/` fails on the OLED with **`BAD LOAD ADDR`**. Root cause,
confirmed by rebuilding the image and reading the packed segment header:

- `rootfs.uimg` loads at `0x0E800000`; the combined rootfs (platform.cpio 7.29 MB
  + the ~1.4 MB launcher app, uImage-wrapped) is **9,093,968 B**, so the segment
  ends at **`0x0F0AC350`**.
- The device app-loader's usable SDRAM ceiling is **`SDRAM_HI = 0x0F000000`**;
  the top of SDRAM is carved into the 2.875 MB SRAM staging window and a 12 MB
  USB dev-upload scratch buffer.
- `0x0F0AC350 > 0x0F000000` → `classify_load_range` returns `None` →
  `place_segment` → `BadLoadAddress` → `BAD LOAD ADDR`.
- The host packer waved it through only because
  `tools/deluge_mkimage/memmap.py` uses `SDRAM_END = 0x10000000` (the full 64 MB)
  instead of the device's real `0x0F000000`. `base`-profile images squeak under
  8 MB; `platform` does not.

The 12 MB scratch is what pins the ceiling low. It exists purely to hold a whole
uploaded ELF for CRC + copy on the **USB** path; the **SD** path (production)
never touches it, and streaming makes it unnecessary on USB too.

## 2. Goals & non-goals

**Goals**
- Boot the `platform`-profile launcher image from `/APPS/` by raising the
  app-segment ceiling to `0x0FD20000` (21.1 MB rootfs budget).
- Replace the USB `buffer-then-load` path with a streaming router that writes
  segments directly to their destinations, deleting the 12 MB scratch and the
  arbitrary 12 MB dev-upload cap.
- Correct `memmap.py` so the host packer enforces the *same* ceiling the device
  does — turning a silent device-side brick into a clear build-time error.
- Keep the streaming address/routing math **pure and host-tested**, exactly as
  `place_segment` / `parse_load_plan` are today.

**Non-goals**
- Changing the **SD** load path (`load_from_sd_with_progress` already streams).
- Changing the **wire protocol** (`magic|version|flags|len|crc32|<elf>`) or the
  whole-file CRC.
- Any `deluge-linux` / U-Boot / base-bundle change. The rootfs load address
  (`0x0E800000`, duplicated in U-Boot's bootcmd) is untouched, and the manifest's
  `rootfs.uimg max_size = 0x1000000` (16 MB) stays valid under the new ceiling.
- Host-side ELF canonicalization (reordering segments for streamability). The
  device validates and rejects out-of-order ELFs; canonicalization is a future
  add only if a real toolchain case appears.

## 3. Memory layout

Derive the two moved constants from the SRAM app-region size, pinned to the top
of SDRAM, so they cannot drift:

```rust
const SDRAM_TOP:        u32 = 0x1000_0000;
const SRAM_REGION_SIZE: u32 = SRAM_HI - SRAM_LOAD_ORIGIN;       // 0x2E0000
pub const SDRAM_STAGE_BASE: u32 = SDRAM_TOP - SRAM_REGION_SIZE; // 0x0FD20000
pub const SDRAM_HI:         u32 = SDRAM_STAGE_BASE;             // 0x0FD20000
```

New 64 MB SDRAM map:

```
0x0C000000 ┬ app region  (u-boot, dtb, zImage, rootfs — all PT_LOAD segments)
0x0E800000 │   rootfs.uimg   budget: 0x0FD20000 - 0x0E800000 = 0x1520000 = 21.1 MB
0x0FD20000 ┼ SDRAM_HI = SDRAM_STAGE_BASE      ← new ceiling
0x0FD20000 │ SRAM staging window (2.875 MB = the on-chip SRAM app-region size)
0x10000000 ┴ end of SDRAM
```

All placement logic already flows through these constants, so
`classify_load_range`, `place_segment`, and `sram_stage_addr` need **no logic
change** — only the constant values move. Doc comments are corrected (the current
"top 1 MB reserved" text is already wrong; the window is 2.875 MB). A test pins
`SDRAM_HI == SDRAM_STAGE_BASE` and `SDRAM_STAGE_BASE + SRAM_REGION_SIZE ==
SDRAM_TOP`; the existing `staging_address_math` check (`sram_stage_addr(SRAM_HI-1)
= 0x0FFFFFFF < 0x10000000`) still holds.

**Why staging goes to the very top, not the app region:** the staging window and
a large rootfs are never both live in one image (staging is only used by apps
with on-chip-SRAM segments; the platform image has none). One uniform ceiling at
`0x0FD20000` is safe for both kinds of image: an app's SDRAM segments end at or
below `SDRAM_HI`, and its SRAM segments stage in `[0x0FD20000, 0x10000000)` —
disjoint by construction.

## 4. Streaming USB loader

The wire frame is unchanged: `magic | version | flags | len u32 | crc32 u32 |
<stripped ELF>`, CRC over the whole payload. Only the device's handling changes,
from "store `len` bytes in scratch → CRC → copy PT_LOAD ranges out" to a
sequential router:

1. **Front matter.** Buffer only the ELF header (52 B) + program-header table
   (≤ `MAX_PHDRS`×32 = 256 B) on the stack. Require `e_phoff + e_phnum*32 ≤
   HEADER_MAX` (~1 KB) so the table arrives up front; else reject. Validate
   exactly as the SD path (magic, `ELFCLASS32`, `ELFDATA2LSB`, `ET_EXEC`,
   `EM_ARM`, `e_phentsize == 32`, `e_phnum ≤ MAX_PHDRS`).
2. **Route table.** For each `PT_LOAD`, run the shared `place_segment()` and
   record `{ file_start = p_offset, file_end = p_offset + p_filesz, dst,
   memsz, sram }` (`Skip` placements — retention RAM — are dropped). **Reject** if
   segments are not in non-decreasing `p_offset`, overlap in the file, or start
   before the current stream position — a stream cannot seek backward. New
   `ElfError::Unstreamable` → OLED `b"BAD LAYOUT"`, logged with the offending
   offsets.
3. **Route the body.** Track the absolute file offset. For each incoming chunk,
   split it at segment boundaries: copy bytes inside a segment to its `dst`
   (advancing `dst`), discard bytes in gaps, and zero-fill `memsz − filesz` when
   a segment's file bytes complete. **CRC every payload byte** (kept or
   discarded) incrementally, so integrity coverage is identical to today.
4. **Verify, then hand off.** At end-of-payload, compare the running CRC to the
   header's. **Mismatch → abort with no handoff** and resume listening; segments
   already written to SDRAM/staging are inert until the jump, so a rejected
   upload cannot run. Match → build `LoadResult { entry, sram_descs, n_sram }`
   and hand off exactly as today (`launch_via_trampoline` when `n_sram > 0`, else
   `launch`).

**Pure, host-tested core (`deluge-image`).** A `StreamRouter`:
- `StreamRouter::new(header_bytes) -> Result<StreamRouter, PlanError>` — parse +
  validate + order-check, producing the ordered route table.
- `route(file_off, len) -> impl Iterator<Item = RouteSpan>` where a `RouteSpan`
  is either `Copy { dst_off, len }` or `Discard { len }` — pure arithmetic,
  fully unit-testable across chunk boundaries.

The device receive loop owns only the `unsafe` memcpy, BSS zeroing, CRC update,
and OLED progress. This mirrors how `place_segment` / `parse_load_plan` isolate
pure math from hardware today.

**Incremental CRC.** Add a streaming API to `crates/deluge-image/src/crc.rs`
(`Crc32 { new, update, finalize }` or `crc32_update(crc, &[u8]) -> u32`) and
re-express the existing one-shot `crc32()` on top of it, so the host framing CRC
and the pinned test vectors stay byte-identical.

**Removed:** `app-loader/src/devupload.rs` `SCRATCH_ADDR` / `SCRATCH_LEN` and the
buffer/CRC/`load_from_slice` block; `deluge-image` `load_from_slice`,
`parse_load_plan`, `LoadPlan`, `LoadOp`, `PlanError` variants no longer used, and
their tests (each has exactly one caller — the USB receive loop — verified).
`validate_header` and `place_segment` are retained and shared.

## 5. Host + validation changes

- **`tools/deluge_mkimage/memmap.py`** (deluge-linux-sdk): `SDRAM_END =
  0x0FD20000`, matching the device ceiling, with a comment that the top 2.875 MB
  is the loader's SRAM staging window (not app-usable). This makes the packer's
  `elf.check()` and `memmap.load()` reject anything that would `BAD LOAD ADDR`,
  and lets the current launcher image validate. Re-run `test_elf.py`.
- **`tools/cargo-deluge/src/run.rs`** (deluge-sdk): drop the artificial
  `MAX_UPLOAD_BYTES = 12 MiB` scratch cap (replace with a sanity bound, e.g. the
  SDRAM span) and update the "upload scratch window" comment — there is no scratch
  anymore. Keep the `llvm-objcopy --strip-all` step: the host still sends a normal
  stripped ELF, whose front-loaded, ascending-`p_offset` layout satisfies the
  router's ordering requirement.
- **No manifest / U-Boot change.** `rootfs.uimg max_size = 0x1000000` (end
  `0x0F800000`) is valid under `0x0FD20000`; the `0x0E800000` load address is
  unchanged.

## 6. Testing & rollout

**Host unit tests**
- `deluge-image`: `StreamRouter` ordering accept/reject, byte routing across
  chunk boundaries, gap discard, BSS zero-extend, SRAM→staging routing; `crc32`
  streaming == one-shot; updated `elf.rs` constant/layout tests.
- `deluge-linux-sdk`: `test_elf.py` for the new `SDRAM_END`.

**On-device (JTAG/J-Link)**
- Boot the `platform` launcher image from `/APPS/` — the original bug, fixed by
  the raised ceiling.
- `cargo deluge run` a small example to exercise the streaming receive path
  end-to-end (including an SRAM-segment app to cover staging).

**⚠️ Re-flash required.** This changes the resident app-loader firmware; the
device must be reflashed for it to take effect. The `memmap.py` fix alone (no
reflash) only makes oversized images *fail at build time* — correct, but booting
the launcher needs the firmware change.

## 7. Risks & assumptions

- **Streamability assumption.** Uploaded ELFs must have the phdr table up front
  and `PT_LOAD` segments in ascending `p_offset`. Held by normal toolchain /
  `llvm-objcopy --strip-all` output. Violations are rejected with `BAD LAYOUT`,
  never mis-loaded. Host canonicalization deferred.
- **Verify-after-write safety.** Streaming writes segments before the CRC is
  confirmed. Safe because the only irreversible step is the jump to `entry`,
  gated on CRC match; a bad upload leaves inert bytes in SDRAM and resumes
  listening.
- **Two-repo lockstep.** The device `SDRAM_HI` and host `SDRAM_END` must stay
  equal. Both are set to `0x0FD20000` here; a comment in each points at the
  other.
