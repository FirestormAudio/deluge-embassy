# deluge-ndk standup (reorg Phase 0 · Stage 1) — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Extract the C library (`libdeluge`) + its CMake app-authoring surface + C examples + C tests out of `deluge-linux-sdk` into a new standalone repo `deluge-ndk`, with git history preserved, that builds host + cross and passes its C test suite — touching no other repo.

**Architecture:** A `git filter-repo` history-preserving extraction of the C-only subtree into a fresh repo, then prune the moved-elsewhere bits (Rust crates, `xtask`, `mkimage`, Rust examples already fall away because they are not in the keep-list; the `mkimage` *pytest* wiring and the checked-in generated toolchain file are removed by hand), a C-only CI workflow, and a new README/LICENSE. `deluge-linux-sdk` is left **untouched and still building** — nothing is deleted from it in this stage.

**Tech Stack:** C11, CMake ≥3.21, CTest, ALSA (`libasound`), `git-filter-repo`, GitHub Actions, `gh` CLI.

## Global Constraints

- The **linkable artifact name is unchanged**: `libdeluge.a`, `-ldeluge`, `deluge.pc`, headers under `include/deluge/`. Only the *repo/product* is renamed to `deluge-ndk`.
- The C library is a **static archive**, sources are an **explicit 15-file list** (not a glob), built `-Wall -Wextra -Werror -std=c11`, linking `ALSA + Threads + m`.
- `CMakeLists.txt` must stay **FetchContent-safe**: when not top-level it defines only the `deluge` target + `deluge_add_app()` (external C/C++ apps consume it via FetchContent, and Stage 2's Buildroot package fetches it).
- **`deluge-linux-sdk` is not modified in this stage.** Its retirement happens in Stage 4.
- New repo owner/URL: **`github.com/FirestormAudio/deluge-ndk`** (matches `FirestormAudio/deluge-sdk` and `FirestormAudio/deluge-linux`).
- Working directory for the new repo: **`~/GitHub/deluge-ndk`**.

---

## Phase 0 staging overview (context — only Stage 1 is detailed here)

This plan is **Stage 1 of 4**. Later stages get their own plans once their prerequisites (a *pushed* `deluge-ndk`, then a *published* bundle) exist — writing them now would require placeholder revs/URLs/paths this skill forbids.

| stage | deliverable | publish gate before next |
| --- | --- | --- |
| **1 (this plan)** | `deluge-ndk` repo stood up, builds host+cross, C tests green | **push `deluge-ndk`** → gives Stage 2 a real `_SITE` URL + rev |
| 2 | `libdeluge` Buildroot **staging** package in `deluge-linux`; `mkimage` moved there; bundle ships the `libdeluge` sysroot + `mkimage` | **cut a bundle release** → gives Stage 3 a real sysroot path |
| 3 | Rust crates (`deluge-sys`; `deluge-linux`→`deluge-hal-linux`; `deluge-linux-ui`) + Rust examples move into `deluge-sdk`; `cargo deluge linux` subcommand; `DELUGE_SDK_ROOT` repointed at the bundle sysroot | deluge-sdk green on the new bundle |
| 4 | retire `deluge-linux-sdk` (archive) | — |

---

## Task 1: Extract the C subtree into `~/GitHub/deluge-ndk` with history

**Files:**
- Create: `~/GitHub/deluge-ndk/` (fresh git repo, filtered from `deluge-linux-sdk`)

**Interfaces:**
- Produces: a repo whose tree contains exactly `CMakeLists.txt`, `cmake/`, `deluge.pc.in`, `include/`, `src/`, `tests/` (minus `tests/mkimage/`), the 10 C example `.c` files + `examples/app/`, `.github/workflows/ci.yml`, `README.md`, `.gitignore` — each with its original history.

- [ ] **Step 1: Confirm `git-filter-repo` is installed**

Run: `git filter-repo --version`
Expected: a version string (e.g. `git-filter-repo 2.38.0`). If "command not found", install: `pipx install git-filter-repo` (or `sudo pacman -S git-filter-repo`), then re-run.

- [ ] **Step 2: Fresh mirror-clone the source repo to a scratch working copy**

```bash
rm -rf ~/GitHub/deluge-ndk /tmp/deluge-ndk-extract
git clone --no-local ~/GitHub/deluge-linux-sdk /tmp/deluge-ndk-extract
cd /tmp/deluge-ndk-extract
```

Expected: a clean clone; `git log --oneline | head -1` shows `f2837fe` (or newer) at HEAD. **`--no-local` is required** — a plain local clone hardlinks objects and `git filter-repo` (Step 3) refuses to rewrite it ("does not look like a fresh clone"). `--no-local` forces a freshly-packed clone filter-repo accepts.

- [ ] **Step 3: Keep-pass — retain only the C-side paths (allowlist)**

```bash
cd /tmp/deluge-ndk-extract
git filter-repo \
  --path CMakeLists.txt \
  --path cmake/ \
  --path deluge.pc.in \
  --path include/ \
  --path src/ \
  --path tests/ \
  --path examples/app/ \
  --path examples/sine_out.c \
  --path examples/midi_echo.c \
  --path examples/event_dump.c \
  --path examples/display_demo.c \
  --path examples/selftest.c \
  --path examples/cv_demo.c \
  --path examples/usb_role.c \
  --path examples/usb_data.c \
  --path examples/usb_audio_measure.c \
  --path examples/usb_midi_dump.c \
  --path .github/workflows/ci.yml \
  --path README.md \
  --path .gitignore
```

Expected: filter-repo runs to completion ("Parsed N commits … Completely finished"). The Rust crates, `xtask`, `tools/`, `Cargo.*`, `.cargo/`, `bundle.lock`, and the Rust example dirs are **absent** because they were not listed.

- [ ] **Step 4: Drop-pass — remove the mkimage pytest tree (moves to deluge-linux in Stage 2)**

```bash
cd /tmp/deluge-ndk-extract
git filter-repo --force --invert-paths --path tests/mkimage/
```

Expected: completes; `test -d tests/mkimage && echo PRESENT || echo GONE` prints `GONE`.

- [ ] **Step 5: Verify the extracted tree is exactly the C surface**

Run:
```bash
cd /tmp/deluge-ndk-extract
git ls-files | grep -E '^(crates|xtask|tools|Cargo|\.cargo)' || echo "NONE (good)"
ls src/*.c | wc -l          # expect 15
ls include/deluge/*.h | wc -l   # expect 12
```
Expected: first line prints `NONE (good)`; then `15`; then `12`.

- [ ] **Step 6: Move the scratch copy into place**

```bash
mv /tmp/deluge-ndk-extract ~/GitHub/deluge-ndk
cd ~/GitHub/deluge-ndk
git remote remove origin 2>/dev/null || true
```

Expected: `~/GitHub/deluge-ndk` exists and `git remote -v` prints nothing (origin cleared; it points to the new repo in Task 7).

---

## Task 2: Prune generated/stale artifacts

**Files:**
- Delete: `~/GitHub/deluge-ndk/cmake/DelugeToolchain.cmake` (checked-in generated file with a hardcoded local path)
- Modify: `~/GitHub/deluge-ndk/.gitignore`

- [ ] **Step 1: Remove the generated, hardcoded toolchain file (keep the `.in` template)**

```bash
cd ~/GitHub/deluge-ndk
git rm --quiet cmake/DelugeToolchain.cmake
test -f cmake/DelugeToolchain.cmake.in && echo "template kept (good)"
```
Expected: prints `template kept (good)`. (`CMakeLists.txt` regenerates the concrete file from the `.in` at configure time when `DELUGE_BASE` is set.)

- [ ] **Step 2: Ignore the generated toolchain file and build dirs**

Append to `~/GitHub/deluge-ndk/.gitignore` (create if the filter didn't carry one):

```gitignore
/build/
/build-cross/
/stage/
cmake/DelugeToolchain.cmake
```

- [ ] **Step 3: Commit the prune**

```bash
cd ~/GitHub/deluge-ndk
git add .gitignore
git commit -q -m "chore(ndk): drop checked-in generated toolchain file; ignore build dirs"
echo done
```
Expected: `done`.

---

## Task 3: De-wire the mkimage pytest from the CTest suite (C tests only)

**Files:**
- Modify: `~/GitHub/deluge-ndk/tests/CMakeLists.txt` (remove the trailing `Python3` + `mkimage` `add_test` block)

- [ ] **Step 1: Show the block to remove**

Run: `tail -8 ~/GitHub/deluge-ndk/tests/CMakeLists.txt`
Expected output (this is what gets deleted):
```
# The image packer is Python; run its suite under ctest so `ctest` stays the one
# command that tests this repo.
find_package(Python3 COMPONENTS Interpreter REQUIRED)
add_test(NAME mkimage
    COMMAND ${Python3_EXECUTABLE} -m pytest ${CMAKE_CURRENT_SOURCE_DIR}/mkimage -v)
```

- [ ] **Step 2: Delete exactly those 5 lines (comment + find_package + add_test)**

Edit `tests/CMakeLists.txt` and remove the block shown above (the two comment lines, the `find_package(Python3 …)` line, and the two-line `add_test(NAME mkimage …)`). Leave every C `add_test(NAME …)` above it intact.

- [ ] **Step 3: Verify no dangling mkimage/Python references remain**

Run: `grep -nE 'mkimage|Python3' ~/GitHub/deluge-ndk/tests/CMakeLists.txt || echo "clean"`
Expected: `clean`.

- [ ] **Step 4: Commit**

```bash
cd ~/GitHub/deluge-ndk
git add tests/CMakeLists.txt
git commit -q -m "test(ndk): drop mkimage pytest from ctest (packer moves to deluge-linux)"
echo done
```
Expected: `done`.

---

## Task 4: New-repo identity — README, LICENSE, top-level CMake sanity

**Files:**
- Create: `~/GitHub/deluge-ndk/README.md` (overwrite the inherited SDK README)
- Create: `~/GitHub/deluge-ndk/LICENSE-MIT`, `~/GitHub/deluge-ndk/LICENSE-APACHE`

**Interfaces:**
- Consumes: `CMakeLists.txt`'s existing `PROJECT_IS_TOP_LEVEL` gating (unchanged) — verified here, not modified.

- [ ] **Step 1: Confirm `CMakeLists.txt` is already standalone-correct (no edit expected)**

Run: `grep -nE 'PROJECT_IS_TOP_LEVEL|CMAKE_SOURCE_DIR STREQUAL|include\(cmake/DelugeApp' ~/GitHub/deluge-ndk/CMakeLists.txt`
Expected: shows the top-level guard and the **unconditional** `include(cmake/DelugeApp.cmake)`. This confirms the FetchContent-safe boundary survives the move; **no change needed**.

- [ ] **Step 2: Write the new README**

Create `~/GitHub/deluge-ndk/README.md`:

```markdown
# deluge-ndk

The **Deluge Native Development Kit** — the C/C++ SDK for writing apps for
[Synthstrom Deluge] running [deluge-linux]. Provides `libdeluge` (device
discovery, audio I/O, MIDI, input, LEDs, CV/gate, display, USB) plus the CMake
integration (`deluge_add_app()`, `find_package(Deluge)`) that turns your app
into a bootable image.

Apps link `libdeluge` statically and are packed into a Deluge image by
`deluge-mkimage` (shipped in the deluge-linux release **bundle**).

- **C / C++** app authors: this repo.
- **Rust** app authors: use [deluge-sdk] (the async, batteries-included SDK).

## Build

```sh
export DELUGE_BASE=~/GitHub/deluge-linux/out/deluge-base-0.2.0   # an unpacked bundle
cmake -B build -DCMAKE_TOOLCHAIN_FILE=$DELUGE_BASE/DelugeToolchain.cmake
cmake --build build
```

Host build + tests (no bundle needed):

```sh
cmake -B build && cmake --build build && ctest --test-dir build
```

## Consuming from your own app (FetchContent)

```cmake
include(FetchContent)
FetchContent_Declare(deluge
  GIT_REPOSITORY https://github.com/FirestormAudio/deluge-ndk.git
  GIT_TAG v0.1.0)
FetchContent_MakeAvailable(deluge)
deluge_add_app(myapp SOURCES main.c)
```

## License

`MIT OR Apache-2.0`. See `LICENSE-MIT` and `LICENSE-APACHE`.

[Synthstrom Deluge]: https://synthstrom.com/product/deluge/
[deluge-linux]: https://github.com/FirestormAudio/deluge-linux
[deluge-sdk]: https://github.com/FirestormAudio/deluge-sdk
```

- [ ] **Step 3: Add the license files (default `MIT OR Apache-2.0`, matching deluge-sdk)**

> **Maintainer note:** the spec flagged this as a conscious call (LGPL vs permissive). Default here is `MIT OR Apache-2.0` to match `deluge-sdk`; confirm before Task 7 push. If a copyleft posture is wanted instead, swap these files and the README line.

```bash
cd ~/GitHub/deluge-ndk
cp ~/GitHub/deluge-sdk/LICENSE-MIT LICENSE-MIT
cp ~/GitHub/deluge-sdk/LICENSE-APACHE LICENSE-APACHE
ls LICENSE-*
```
Expected: `LICENSE-APACHE  LICENSE-MIT`.

- [ ] **Step 4: Commit identity**

```bash
cd ~/GitHub/deluge-ndk
git add README.md LICENSE-MIT LICENSE-APACHE
git commit -q -m "docs(ndk): repo README + MIT/Apache dual license"
echo done
```
Expected: `done`.

---

## Task 5: C-only CI workflow

**Files:**
- Modify: `~/GitHub/deluge-ndk/.github/workflows/ci.yml` (drop the pytest, host-`stage`, and Rust-build steps — those belong to other repos now)

- [ ] **Step 1: Replace `ci.yml` with the C-only pipeline**

Overwrite `~/GitHub/deluge-ndk/.github/workflows/ci.yml`:

```yaml
name: ci
on: [push, pull_request]
jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - run: sudo apt-get update && sudo apt-get install -y libasound2-dev libgpiod-dev
      - run: cmake -S . -B build && cmake --build build
      - run: ctest --test-dir build --output-on-failure
      - run: cmake --install build --prefix "$PWD/stage"
      - run: test -f stage/lib/libdeluge.a && test -f stage/lib/pkgconfig/deluge.pc
```

(Removed vs the inherited file: `cpio` apt dep and `python3 -m pytest tests/mkimage` — the packer left; and the `dtolnay/rust-toolchain` + `cargo build … crates/deluge-linux` steps — Rust left. Kept the install + a smoke assertion that the archive and `.pc` install.)

- [ ] **Step 2: Commit CI**

```bash
cd ~/GitHub/deluge-ndk
git add .github/workflows/ci.yml
git commit -q -m "ci(ndk): C-only build + ctest + install smoke"
echo done
```
Expected: `done`.

---

## Task 6: Verify host build + tests, and a cross build

**Files:** none (verification only)

- [ ] **Step 1: Host configure + build**

```bash
cd ~/GitHub/deluge-ndk
rm -rf build
cmake -S . -B build && cmake --build build
```
Expected: configures (finds ALSA + Threads), builds `libdeluge.a` and the C example/test executables with **no `-Werror` failures**.

- [ ] **Step 2: Run the C test suite**

Run: `ctest --test-dir build --output-on-failure`
Expected: all C tests pass (`core`, `discover`, `audio_convert`, `open`, `input_decode`, `pad_decode`, `leds`, `cv`, `usb`, `usb_uevent`, `usb_bind`) — `100% tests passed`. **No `mkimage` test listed** (removed in Task 3).

- [ ] **Step 3: Install smoke — archive + headers + pc land**

```bash
cd ~/GitHub/deluge-ndk
cmake --install build --prefix "$PWD/stage"
ls stage/lib/libdeluge.a stage/lib/pkgconfig/deluge.pc stage/include/deluge/deluge.h
```
Expected: all three paths listed (this is exactly the `DELUGE_SDK_ROOT` shape `deluge-sys` will later consume from the bundle sysroot).

- [ ] **Step 4: Cross build against an unpacked bundle**

```bash
cd ~/GitHub/deluge-ndk
export DELUGE_BASE=~/GitHub/deluge-linux/out/deluge-base-0.2.0
rm -rf build-cross
cmake -S . -B build-cross -DCMAKE_TOOLCHAIN_FILE=$DELUGE_BASE/DelugeToolchain.cmake
cmake --build build-cross --target deluge
file build-cross/libdeluge.a 2>/dev/null; ls build-cross/libdeluge.a
```
Expected: configures with the arm musl toolchain and builds `build-cross/libdeluge.a` (ARM archive). If `deluge-base-0.2.0` is absent, substitute the newest `~/GitHub/deluge-linux/out/deluge-base-*` directory.

- [ ] **Step 5: Commit nothing (verification only) — record success**

No commit. If any step failed, fix in the relevant task above before proceeding. Do **not** proceed to Task 7 until Steps 1-4 are green.

---

## Task 7: Publish `deluge-ndk` (publish checkpoint — maintainer-gated)

**Files:** none (remote creation + push)

> **This is an outward-facing action.** It creates a repository and pushes history. Do not run it without explicit maintainer go-ahead. Default visibility is **private**; flip to public deliberately later. Its output (the pushed URL + a rev) is the input Stage 2's Buildroot package needs.

- [ ] **Step 1: Create the remote (private) and push**

```bash
cd ~/GitHub/deluge-ndk
gh repo create FirestormAudio/deluge-ndk --private --source=. --remote=origin --description "Deluge Native Development Kit — C/C++ SDK (libdeluge)"
git push -u origin HEAD
```
Expected: repo created at `github.com/FirestormAudio/deluge-ndk`; branch pushed; GitHub Actions `ci` starts.

- [ ] **Step 2: Record the pinned handle for Stage 2**

Run: `git -C ~/GitHub/deluge-ndk rev-parse HEAD`
Expected: a commit SHA. **Note it** — Stage 2's Buildroot `libdeluge.mk` uses `LIBDELUGE_SITE = https://github.com/FirestormAudio/deluge-ndk.git`, `LIBDELUGE_SITE_METHOD = git`, and `LIBDELUGE_VERSION = <this SHA or a tag>`.

- [ ] **Step 3: Confirm remote CI is green**

Run: `gh run list --repo FirestormAudio/deluge-ndk --limit 1`
Expected: the latest `ci` run concludes `success`.

---

## Self-Review

**Spec coverage (Stage 1 slice of the reorg spec §6 "→ deluge-ndk"):** root `CMakeLists.txt` ✓(Task 1/4), `cmake/` all 4 ✓(Task 1; generated one pruned Task 2), `deluge.pc.in` ✓, `include/deluge/*` 12 ✓(verified Task 1.5), `src/*` 15 ✓, C examples + `examples/app/` ✓, C tests + fixtures ✓(Task 6.2), `deluge_add_app`/`FindDeluge` path ✓(Task 4.1). Sysroot **not** built here ✓ (shipped by the bundle in Stage 2). mkimage/pytest correctly **excluded** ✓(Task 1.4, Task 3). Rust correctly **excluded** ✓(Task 1.5).

**Placeholder scan:** none — every step has concrete commands, the license default is a real choice (MIT/Apache) with a flagged confirm, and the one deferred value (the pinned rev/URL for Stage 2) is *produced* by Task 7.2, not assumed.

**Type/name consistency:** artifact stays `libdeluge`/`deluge.pc`/`-ldeluge` throughout; repo/URL `FirestormAudio/deluge-ndk` consistent across README, CI, Task 7; `DELUGE_BASE`/`DELUGE_SDK_ROOT` used with the same meanings as the existing toolchain + `deluge-sys` build.rs.

**Left green:** `deluge-linux-sdk` is not touched in this stage; it continues to build exactly as before. Only Stages 3-4 remove things from it.
