# Stage 2b — relocate `deluge-mkimage` into deluge-linux (share modules) — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Move the `deluge_mkimage` app-image packer from `deluge-linux-sdk` into `deluge-linux` and ship it inside the release bundle, reconciling it with `deluge-linux`'s existing `mk-app-elf.py` tooling by sharing a single `memmap.py` + `uimage.py` (fixing a stale SDRAM ceiling in the process).

**Architecture:** `deluge-linux` already has `tools/{memmap.py, uimage.py, mk-app-elf.py, check-app-elf.py}` (its `make appelf` / `LINUX.ELF` packer). The incoming `deluge_mkimage` package carries its own `memmap.py`/`uimage.py`; instead of duplicating, `deluge_mkimage` is brought in **without** those two modules and refactored to import the flat `tools/` copies. The single `tools/memmap.py` is corrected to the real app-segment ceiling `0x0FD20000`. `mk-bundle.sh` then copies the packer + its shared modules into the bundle so SDK consumers run `$DELUGE_BASE/tools/deluge-mkimage`.

**Tech Stack:** Python 3, pytest, bash (Buildroot bundle assembler), GitHub Actions.

## Global Constraints

- **Approach #1 (share), unify later.** One `tools/memmap.py` + one `tools/uimage.py`, used by both `mk-app-elf.py` (flat `import memmap`) and `deluge_mkimage` (also flat). The full unification of the two packers is **deferred Stage 2c** (spec §10b) — do **not** attempt it here.
- **SDRAM ceiling:** `tools/memmap.py` must have `SDRAM_END = 0x0FD20000` (matches the app-loader `SDRAM_HI` in `deluge-sdk/crates/deluge-image/src/elf.rs`; the top 2.875 MB `0x0FD20000..0x10000000` is the SRAM staging window, not app-usable). `deluge-linux`'s copy currently has the stale `0x10000000` — fix it.
- `deluge_mkimage`'s **only** imports of the shared modules are `cli.py:18` (`from . import appelf, bundle, cpio, elf, memmap`) and `elf.py:17` (`from . import memmap, uimage`). Those become flat `import memmap` / `import memmap, uimage`; every other `from . import …` (package-internal `appelf`/`bundle`/`cpio`/`elf`) stays package-relative.
- The bundle must ship `tools/{deluge-mkimage, deluge_mkimage/, memmap.py, uimage.py}` together (the shim adds `<bundle>/tools` to `sys.path`; the packer then resolves `import memmap`/`import uimage` from there).
- Repos touched: `~/GitHub/deluge-linux` only (Tasks 1-4). Source of the moved files: `~/GitHub/deluge-linux-sdk` (read-only). `deluge-linux-sdk` is **not** modified here (its retirement is Stage 4). History: the moved files are copied fresh; their history remains in `deluge-linux-sdk`'s git (archived at Stage 4).

---

## Task 1: Fix the stale SDRAM ceiling in `deluge-linux`'s memmap

**Files:**
- Modify: `~/GitHub/deluge-linux/tools/memmap.py`

- [ ] **Step 1: Show the stale constant**

Run: `grep -n "SDRAM_END" ~/GitHub/deluge-linux/tools/memmap.py`
Expected: `SDRAM_END = 0x10000000  # exclusive: 64 MB` (the stale value).

- [ ] **Step 2: Correct it to the real ceiling**

In `~/GitHub/deluge-linux/tools/memmap.py`, replace the line
```python
SDRAM_END = 0x10000000  # exclusive: 64 MB
```
with
```python
# App-segment ceiling — must equal the app-loader's SDRAM_HI
# (deluge-sdk/crates/deluge-image/src/elf.rs). The top 2.875 MB
# (0x0FD20000..0x10000000) is the loader's SRAM staging window, not app-usable.
SDRAM_END = 0x0FD20000
```

- [ ] **Step 3: Verify deluge-linux's existing tool tests still pass**

```bash
cd ~/GitHub/deluge-linux
python3 -m pytest tests/ -v 2>&1 | tail -20
```
Expected: all existing tests pass (`tests/test_memmap.py`, `tests/test_mk_app_elf.py`, `tests/test_uimage.py`). The lowered ceiling does not break them (none pins segments into `[0x0FD20000, 0x10000000)`; the real `memory-map.json` rootfs at `0x0E800000` stays valid). If a test fails because it asserted the old ceiling, fix that test's expected value to `0x0FD20000` and note it.

- [ ] **Step 4: Commit**

```bash
cd ~/GitHub/deluge-linux
git add tools/memmap.py
git commit -q -m "tools/memmap: correct SDRAM ceiling to 0x0FD20000 (app-loader SDRAM_HI)

The packer's app-segment ceiling was a stale 0x10000000 (full 64 MB); the top
2.875 MB is the app-loader's SRAM staging window and is not app-usable. Match
the corrected ceiling the SDK's memmap + deluge-image already use, so mk-app-elf
no longer waves through an image that overruns staging."
echo done
```
Expected: `done`.

---

## Task 2: Bring `deluge_mkimage` into deluge-linux, sharing memmap/uimage

**Files:**
- Create: `~/GitHub/deluge-linux/tools/deluge-mkimage` (shim, copied)
- Create: `~/GitHub/deluge-linux/tools/deluge_mkimage/{__init__,cli,appelf,bundle,cpio,elf}.py` (copied, **without** `memmap.py`/`uimage.py`)
- Create: `~/GitHub/deluge-linux/bundle.lock` (copied)
- Create: `~/GitHub/deluge-linux/tests/mkimage/{conftest,test_appelf,test_bundle,test_cli,test_cpio,test_elf}.py` (copied)
- Modify (after copy): `~/GitHub/deluge-linux/tools/deluge_mkimage/cli.py`, `.../elf.py` (flat imports)

**Interfaces:**
- Consumes: the shared `~/GitHub/deluge-linux/tools/{memmap,uimage}.py` (Task 1's corrected `memmap.py`).
- Produces: a runnable `tools/deluge-mkimage` in deluge-linux + a `tests/mkimage/` suite.

- [ ] **Step 1: Copy the package (excluding its memmap.py/uimage.py and caches)**

```bash
SRC=~/GitHub/deluge-linux-sdk; DST=~/GitHub/deluge-linux
mkdir -p "$DST/tools/deluge_mkimage" "$DST/tests/mkimage"
for m in __init__ cli appelf bundle cpio elf; do cp "$SRC/tools/deluge_mkimage/$m.py" "$DST/tools/deluge_mkimage/$m.py"; done
cp "$SRC/tools/deluge-mkimage" "$DST/tools/deluge-mkimage" && chmod +x "$DST/tools/deluge-mkimage"
cp "$SRC/bundle.lock" "$DST/bundle.lock"
for t in conftest test_appelf test_bundle test_cli test_cpio test_elf; do cp "$SRC/tests/mkimage/$t.py" "$DST/tests/mkimage/$t.py"; done
echo "--- copied; confirm memmap/uimage NOT in the package ---"
ls "$DST/tools/deluge_mkimage/" | grep -E "memmap|uimage" && echo "LEAK (bad)" || echo "shared modules not duplicated (good)"
```
Expected: `shared modules not duplicated (good)`.

- [ ] **Step 2: Refactor `cli.py` to import `memmap` flat**

In `~/GitHub/deluge-linux/tools/deluge_mkimage/cli.py`, change line 18 from
```python
from . import appelf, bundle, cpio, elf, memmap
```
to
```python
from . import appelf, bundle, cpio, elf
import memmap
```

- [ ] **Step 3: Refactor `elf.py` to import `memmap` + `uimage` flat**

In `~/GitHub/deluge-linux/tools/deluge_mkimage/elf.py`, change line 17 from
```python
from . import memmap, uimage
```
to
```python
import memmap
import uimage
```

- [ ] **Step 4: Run the full deluge-linux pytest suite (existing + new mkimage)**

```bash
cd ~/GitHub/deluge-linux
python3 -m pytest tests/ -v 2>&1 | tail -30
```
Expected: every test passes — the existing `tests/test_{memmap,mk_app_elf,uimage}.py` **and** the new `tests/mkimage/test_{appelf,bundle,cli,cpio,elf}.py`. In particular `tests/mkimage/test_elf.py` asserts `memmap.SDRAM_END == 0x0FD20000` — which now holds against the single shared `tools/memmap.py`. (`test_cpio.py` shells out to the system `cpio`; it is present on this machine.)

- [ ] **Step 5: Commit**

```bash
cd ~/GitHub/deluge-linux
git add tools/deluge-mkimage tools/deluge_mkimage/ tests/mkimage/ bundle.lock
git commit -q -m "tools: bring in deluge_mkimage packer, sharing tools/memmap+uimage

Moved from deluge-linux-sdk. The app-aware image packer (bake an app into the
rootfs + pack the appliance ELF, or --bare validate+copy) now lives with the
platform. It shares the single tools/memmap.py + tools/uimage.py with
mk-app-elf.py (flat imports) rather than carrying duplicates. Full unification
of the two packers is deferred (spec Stage 2c)."
echo done
```
Expected: `done`.

---

## Task 3: Ship the packer in the release bundle

**Files:**
- Modify: `~/GitHub/deluge-linux/scripts/mk-bundle.sh`

**Interfaces:**
- Produces: `<bundle>/tools/{deluge-mkimage, deluge_mkimage/, memmap.py, uimage.py}`, so a consumer runs `$DELUGE_BASE/tools/deluge-mkimage`.

- [ ] **Step 1: Add a bundle `tools/` payload step**

In `~/GitHub/deluge-linux/scripts/mk-bundle.sh`, immediately after the manifest copy line (`cp "$TOP/board/synthstrom/deluge/memory-map.json" "$STAGE/manifest.json"`), insert:

```bash
# The app-image packer, shipped so SDKs pack images without cloning this repo:
# the shim + package + the shared memmap/uimage modules it imports flat.
mkdir -p "$STAGE/tools"
cp "$TOP/tools/deluge-mkimage" "$STAGE/tools/"
cp -r "$TOP/tools/deluge_mkimage" "$STAGE/tools/deluge_mkimage"
cp "$TOP/tools/memmap.py" "$TOP/tools/uimage.py" "$STAGE/tools/"
rm -rf "$STAGE/tools/deluge_mkimage/__pycache__"
```

- [ ] **Step 2: Simulate the bundle `tools/` layout and smoke-test the packer imports**

A full `make bundle` is heavy (both profiles + SDK); prove the *layout* is runnable by staging just the `tools/` payload the script now copies, then invoking the shim:

```bash
cd ~/GitHub/deluge-linux
rm -rf /tmp/fakebundle && mkdir -p /tmp/fakebundle/tools
cp tools/deluge-mkimage /tmp/fakebundle/tools/
cp -r tools/deluge_mkimage /tmp/fakebundle/tools/deluge_mkimage
cp tools/memmap.py tools/uimage.py /tmp/fakebundle/tools/
rm -rf /tmp/fakebundle/tools/deluge_mkimage/__pycache__
python3 /tmp/fakebundle/tools/deluge-mkimage 2>&1 | head -5; echo "exit=$?"
```
Expected: the shim runs from the bundle layout — it reaches argument handling and errors on the **missing `app` argument** (a `usage:`/argparse error), **not** an `ImportError`/`ModuleNotFoundError`. This proves the shipped `tools/` layout resolves `deluge_mkimage.cli` + flat `memmap`/`uimage`.

- [ ] **Step 3: Commit**

```bash
cd ~/GitHub/deluge-linux
git add scripts/mk-bundle.sh
git commit -q -m "mk-bundle: ship deluge-mkimage (+ shared memmap/uimage) in the bundle

SDKs pack app images by invoking \$DELUGE_BASE/tools/deluge-mkimage; ship the
shim, the deluge_mkimage package, and the flat memmap/uimage modules it imports."
echo done
```
Expected: `done`.

---

## Task 4: CI — run the mkimage tests with `cpio` available

**Files:**
- Modify: `~/GitHub/deluge-linux/.github/workflows/build.yml`

- [ ] **Step 1: Give the `tools` job the `cpio` binary**

In `~/GitHub/deluge-linux/.github/workflows/build.yml`, the `tools` job runs `python3 -m pytest tests/ -v`. `tests/mkimage/test_cpio.py` shells out to the system `cpio`, which the `tools` job does not install. Add an install step before `pip install pytest`:

```yaml
      - run: sudo apt-get update && sudo apt-get install -y cpio
```

so the `tools` job steps read:
```yaml
    steps:
      - uses: actions/checkout@v4
      - run: sudo apt-get update && sudo apt-get install -y cpio
      - run: pip install pytest
      - run: python3 -m pytest tests/ -v
```

- [ ] **Step 2: Commit**

```bash
cd ~/GitHub/deluge-linux
git add .github/workflows/build.yml
git commit -q -m "ci: install cpio in the tools job for the mkimage cpio tests"
echo done
```
Expected: `done`.

---

## Self-Review

**Spec coverage (reorg spec §6 "→ deluge-linux: mkimage" + §10b share-decision):** packer moved to `deluge-linux` ✓ (Task 2); shipped in the bundle ✓ (Task 3); shares one `memmap.py`/`uimage.py`, no duplicate modules ✓ (Task 2 Step 1 assertion); stale ceiling fixed ✓ (Task 1); CI runs the suite ✓ (Task 4). Unification of the two packers explicitly **deferred to Stage 2c** (Global Constraints + spec §10b) — not attempted here.

**Placeholder scan:** none — every edit is a concrete before/after; the only path constant (`/tmp/fakebundle`) is a throwaway smoke fixture.

**Type/name consistency:** `SDRAM_END = 0x0FD20000` is the single value across `tools/memmap.py` (Task 1), the `test_elf.py` assertion it must satisfy (Task 2 Step 4), and the Global Constraints. The bundle payload list `tools/{deluge-mkimage, deluge_mkimage/, memmap.py, uimage.py}` is identical in Task 3 Step 1 (real copy), Step 2 (smoke copy), and the Global Constraints.

**Sequencing/greenness:** Task 1 (ceiling) before Task 2 (whose `test_elf.py` pins the new value). Task 2 before Task 3 (bundle copies the moved files). Each commit leaves `deluge-linux` green: Task 1 fixes a constant + passes existing tests; Task 2 adds an opt-in packer + tests; Task 3/4 are additive. `deluge-linux-sdk` is untouched throughout.
