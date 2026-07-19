# Stage 2c — unify the two ELF packers (thin-caller dedup) — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove the duplicated ELF pack/check logic in `deluge-linux`'s `tools/mk-app-elf.py` + `tools/check-app-elf.py` by making them thin delegators to the single implementation in `tools/deluge_mkimage/elf.py`, with **no behavior or interface change**.

**Architecture:** `deluge_mkimage/elf.py` already holds the canonical `pack(out, segments, dir, rootfs, entry)` and `check(elf, segments)`. `mk-app-elf.py`'s `pack`/`_blobs`/`_check_overlap` are byte-identical duplicates; `check-app-elf.py`'s `check` is identical except it takes a map path (and loads it). Both become thin CLIs: `mk-app-elf.py` re-exports `elf.pack`; `check-app-elf.py` keeps its `check(elf, map)` signature as a one-line wrapper over `elf.check(elf, memmap.load(map))`. The tools' CLIs and the module names the tests use (`mk_app_elf.pack`, `check_app_elf.check`) are preserved, so `make appelf`, `release.yml`/`build.yml`, and the pytest suite are all unchanged.

**Tech Stack:** Python 3, pytest, the deluge-linux Makefile.

## Global Constraints

- **Single source of truth:** `pack`, `_blobs`, `_check_overlap`, and `check` exist **only** in `tools/deluge_mkimage/elf.py` after this change. Delete the copies in `mk-app-elf.py`/`check-app-elf.py`.
- **No interface change:** keep the CLIs exactly — `mk-app-elf.py --map MAP --dir DIR [--rootfs FILE] [--entry HEX] OUT`, `check-app-elf.py --map MAP ELF` (exit 0 iff pass). Keep the module-level names the tests import: `mk_app_elf.pack(out, segments, dir, rootfs_path=…)` and `check_app_elf.check(elf_path, map_path)`.
- **No behavior change:** `elf.pack`/`elf.check` are byte-identical to the deleted code (same output strings, same SDRAM checks), so CI output/exit codes are unchanged.
- `mk-app-elf.py`/`check-app-elf.py` run as scripts from `tools/` (their dir is on `sys.path[0]`), so `from deluge_mkimage import elf` and `import memmap` resolve. This is the deferred **Stage 2c** from spec §10b.
- Repo touched: `deluge-linux` only.

---

## Task 1: Reduce `mk-app-elf.py` + `check-app-elf.py` to thin delegators

**Files:**
- Modify: `~/GitHub/deluge-linux/tools/mk-app-elf.py` (delete duplicated pack; re-export from `deluge_mkimage.elf`)
- Modify: `~/GitHub/deluge-linux/tools/check-app-elf.py` (delete duplicated check body; wrap `elf.check`)

- [ ] **Step 1: Rewrite `tools/mk-app-elf.py`**

Replace the whole file with:

```python
#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0
"""Pack raw binaries as PT_LOAD segments into an ARM ET_EXEC ELF32 for the
Deluge app-loader.

A thin CLI over deluge_mkimage.elf — the single ELF-packing implementation
shared with deluge-mkimage. Load addresses come from memory-map.json (never the
command line); the map's `generated` segment (the rootfs) is supplied with
--rootfs and wrapped in a uImage ramdisk header.

Usage: mk-app-elf.py --map MAP.json --dir DIR [--rootfs FILE] OUT.ELF
"""
import argparse

import memmap
from deluge_mkimage.elf import pack  # the single implementation (re-exported)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", required=True)
    ap.add_argument("--dir", required=True)
    ap.add_argument("--rootfs")
    ap.add_argument("--entry", type=lambda v: int(v, 16))
    ap.add_argument("out")
    args = ap.parse_args()
    pack(args.out, memmap.load(args.map), args.dir, args.rootfs, args.entry)


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Rewrite `tools/check-app-elf.py`**

Replace the whole file with:

```python
#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0
"""Validate an ELF against the Deluge app-loader's acceptance checks and against
memory-map.json. Mirrors deluge-sdk/crates/deluge-image/src/elf.rs.

A thin CLI over deluge_mkimage.elf.check — the single validator shared with
deluge-mkimage. Exit 0 iff all pass.

Usage: check-app-elf.py --map MAP.json IMAGE.ELF
"""
import argparse
import sys

import memmap
from deluge_mkimage import elf


def check(elf_path, map_path) -> bool:
    """Validate `elf_path` against the segments in `map_path`."""
    return elf.check(elf_path, memmap.load(map_path))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", required=True)
    ap.add_argument("elf")
    args = ap.parse_args()
    sys.exit(0 if check(args.elf, args.map) else 1)


if __name__ == "__main__":
    main()
```

- [ ] **Step 3: Confirm the duplication is gone**

```bash
cd ~/GitHub/deluge-linux
grep -nE "def _blobs|def _check_overlap|def pack|def _fail|def _ok|struct.pack" tools/mk-app-elf.py tools/check-app-elf.py || echo "  no duplicated pack/check bodies remain (good)"
wc -l tools/mk-app-elf.py tools/check-app-elf.py
```
Expected: `no duplicated pack/check bodies remain (good)`, and both files are now short (~30 lines each).

- [ ] **Step 4: Full pytest suite passes (tests use `mk_app_elf.pack` + `check_app_elf.check`)**

```bash
cd ~/GitHub/deluge-linux
python3 -m pytest tests/ -q 2>&1 | tail -5
```
Expected: all pass (same count as before, 62). `tests/test_mk_app_elf.py` calls `mk_app_elf.pack(...)` (now the re-exported single impl) and `check_app_elf.check(out, map)` (now the wrapper) — both resolve and behave identically. If an import fails, re-check Step 1/2's `from deluge_mkimage…` lines.

- [ ] **Step 5: Commit**

```bash
cd ~/GitHub/deluge-linux
git add tools/mk-app-elf.py tools/check-app-elf.py
git commit -q -m "tools: unify ELF packers — mk-app-elf/check-app-elf delegate to deluge_mkimage.elf

The pack/_blobs/_check_overlap/check logic was duplicated between the appelf
scripts and deluge_mkimage.elf. Keep the single implementation in
deluge_mkimage.elf; reduce mk-app-elf.py + check-app-elf.py to thin CLIs over it.
Interfaces (CLI flags, mk_app_elf.pack / check_app_elf.check) and output are
unchanged; make appelf / release.yml / the pytest suite are untouched.
Closes the deferred Stage 2c (spec 10b)."
echo done
```
Expected: `done`.

---

## Task 2: End-to-end — `make appelf` still packs a valid LINUX.ELF

**Files:** none (build + verify the real appliance-packing path CI uses)

- [ ] **Step 1: Confirm `out/` has the boot artifacts + rootfs (from the earlier bundle build)**

```bash
cd ~/GitHub/deluge-linux
ls out/u-boot.bin out/r7s72102-deluge.dtb out/zImage out/rootfs.cpio 2>&1
```
Expected: all four listed. (If missing, run `make base` first — the `stage3` bundle build already produced them.)

- [ ] **Step 2: Pack + check via the unified tools (exactly what `make appelf` + CI run)**

```bash
cd ~/GitHub/deluge-linux
make appelf 2>&1 | tail -8
```
Expected: `mk-app-elf.py` writes `out/LINUX.ELF` (`wrote out/LINUX.ELF: N segment(s) …`), then `check-app-elf.py` prints `ALL CHECKS PASSED`, then `Output: out/LINUX.ELF`. This exercises the re-exported `elf.pack` + the delegated `elf.check` through the unchanged Makefile target.

- [ ] **Step 3: No commit (verification only)**

If Steps 1-2 are green, Stage 2c is complete. A failure routes to Task 1 (a re-export/wrapper mistake).

---

## Self-Review

**Spec coverage (spec §10b — deferred Stage 2c):** the two ELF packers unified onto the single `deluge_mkimage.elf` implementation ✓ (Task 1); duplication (`pack`/`_blobs`/`_check_overlap`/`check`) removed ✓ (Task 1 Step 3); `make appelf`/`release.yml` interfaces preserved ✓ (thin CLIs, verified Task 2). Took the spec's "thin caller" option (not a full fold), which achieves one implementation with no interface churn.

**Placeholder scan:** none — both replacement files are complete.

**Type/name consistency:** `elf.pack(out, segments, dir, rootfs, entry)` and `elf.check(elf, segments)` signatures match the delegators' calls; `mk_app_elf.pack` (re-export) and `check_app_elf.check(elf, map)` (wrapper) keep the exact names/shapes `tests/test_mk_app_elf.py` imports and calls.

**Sequencing/greenness:** Task 1 is self-contained + pytest-verified; Task 2 confirms the CI/Makefile path end-to-end. No behavior change, so `deluge-linux` stays green throughout; `release.yml`/`build.yml` need no edits.
