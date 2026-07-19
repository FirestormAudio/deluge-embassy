# Stage 2a — libdeluge Buildroot package + sysroot-in-bundle (local path) — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `deluge-linux` build `libdeluge` (from the local `~/GitHub/deluge-ndk` checkout) as a Buildroot **staging** package so it lands in the SDK sysroot the base bundle publishes — giving the Rust/C/C++ SDKs a `DELUGE_SDK_ROOT` to link against.

**Architecture:** `libdeluge` becomes a Buildroot **cmake-package** (the first in this tree), staging-only (like `deluge-alsa-static`), sourced from the local `deluge-ndk` path via `_SITE_METHOD = local`. Because `BR2_TARGET_SDK=y` ships the whole staging sysroot inside `make sdk`, the package's staging install flows into the bundle's `toolchain/…/sysroot` automatically — no bundle-assembler change. A small CMake-option addition to `deluge-ndk` lets the package build **just the library** (not the examples/tests).

**Tech Stack:** Buildroot (`BR2_EXTERNAL`, cmake-package infra), CMake, C, ALSA.

## Global Constraints

- `deluge-ndk` **stays local and unpublished** at `~/GitHub/deluge-ndk`. The package uses `LIBDELUGE_SITE_METHOD = local`. **At eventual publish, swap** to `LIBDELUGE_SITE = https://github.com/FirestormAudio/deluge-ndk.git`, `LIBDELUGE_SITE_METHOD = git`, `LIBDELUGE_VERSION = <tag/sha>`.
- `libdeluge` is **staging-only**: `LIBDELUGE_INSTALL_STAGING = YES`, `LIBDELUGE_INSTALL_TARGET = NO`. Apps static-link it; the rootfs never needs it.
- Artifact/metadata names **unchanged**: `libdeluge.a`, `deluge.pc`, headers under `include/deluge/`.
- The sysroot ships in the bundle **via `make sdk` automatically** — do **not** edit `scripts/mk-bundle.sh` for it.
- License: `MIT OR Apache-2.0` (matches the `deluge-ndk` repo); `LICENSE_FILES = LICENSE-MIT LICENSE-APACHE`.
- Buildroot external prefix is `DELUGE`; `external.mk` auto-includes `package/*/*.mk` (no edit there). New packages must be `source`d in the top-level `Config.in` and enabled in `configs/deluge_defconfig`.
- Repos touched: `~/GitHub/deluge-ndk` (Task 1, one commit) and `~/GitHub/deluge-linux` (Tasks 2-3, commits; Tasks 4-5 verify). `deluge-linux-sdk` is **not** touched (mkimage relocation is Stage 2b).

---

## Task 1: `deluge-ndk` — gate examples/tests behind CMake options

**Files:**
- Modify: `~/GitHub/deluge-ndk/CMakeLists.txt`

**Interfaces:**
- Produces: two cache options `DELUGE_SDK_EXAMPLES` (default `ON`) and `DELUGE_SDK_TESTS` (default `ON`). The Buildroot package (Task 2) passes both `OFF` so only the `deluge` library + its install rules (`libdeluge.a`, headers, `deluge.pc`) build.

- [ ] **Step 1: Add the two options after `project()`**

In `~/GitHub/deluge-ndk/CMakeLists.txt`, immediately after line 27 (`add_compile_options(-Wall -Wextra -Werror)`), insert:

```cmake

# SDK-dev knobs. Default ON so a plain in-tree build (and CI) still builds the
# worked examples + the ctest suite. A packaging build (e.g. the deluge-linux
# Buildroot package) sets both OFF to build only the `deluge` library + its
# install rules (libdeluge.a, headers, deluge.pc).
option(DELUGE_SDK_EXAMPLES "Build the in-tree C example apps" ON)
option(DELUGE_SDK_TESTS "Build the ctest suite" ON)
```

- [ ] **Step 2: Guard the example executables**

In the `if(PROJECT_IS_TOP_LEVEL)` block, wrap the example section. Change the region that currently spans the 10 `add_executable(...)`/`target_link_libraries(...)` pairs **and** the `if(CMAKE_CROSSCOMPILING) add_subdirectory(examples/app) endif()` (currently lines 59-94) so it is enclosed in `if(DELUGE_SDK_EXAMPLES) … endif()`. Concretely, insert `if(DELUGE_SDK_EXAMPLES)` on the line before `add_executable(sine_out examples/sine_out.c)` and `endif()` on the line after the `endif()` that closes the `CMAKE_CROSSCOMPILING` block. The `list(APPEND CMAKE_MODULE_PATH …)` (line 57) stays where it is (above the guard) — it is only consumed by `examples/app`, so it is harmless when examples are off.

- [ ] **Step 3: Guard the tests**

Wrap the tail (currently lines 96-97):

```cmake
    if(DELUGE_SDK_TESTS)
        enable_testing()
        add_subdirectory(tests)
    endif()
```

- [ ] **Step 4: Verify default build unchanged (examples + tests still build)**

```bash
cd ~/GitHub/deluge-ndk
rm -rf build && cmake -S . -B build >/dev/null && cmake --build build >/dev/null 2>&1
ctest --test-dir build 2>&1 | tail -2
ls build/sine_out build/deluge-selftest >/dev/null && echo "examples built"
```
Expected: `ctest` reports `100% tests passed, 0 tests failed out of 11`; then `examples built`.

- [ ] **Step 5: Verify library-only build (options OFF) installs lib + headers + pc, no examples/tests**

```bash
cd ~/GitHub/deluge-ndk
rm -rf build-lib && cmake -S . -B build-lib -DDELUGE_SDK_EXAMPLES=OFF -DDELUGE_SDK_TESTS=OFF >/dev/null
cmake --build build-lib >/dev/null 2>&1
ls build-lib/sine_out 2>/dev/null && echo "LEAK: example built" || echo "no examples (good)"
ctest --test-dir build-lib 2>&1 | grep -q "No tests were found" && echo "no tests (good)" || echo "tests present"
rm -rf stage-lib && cmake --install build-lib --prefix "$PWD/stage-lib" >/dev/null
ls stage-lib/lib/libdeluge.a stage-lib/lib/pkgconfig/deluge.pc stage-lib/include/deluge/deluge.h
```
Expected: `no examples (good)`, `no tests (good)`, then the three install paths listed. (This mirrors exactly what Buildroot's staging install must produce.)

- [ ] **Step 6: Commit (in the `deluge-ndk` repo)**

```bash
cd ~/GitHub/deluge-ndk
rm -rf build build-lib stage-lib stage build-cross cmake/DelugeToolchain.cmake
git add CMakeLists.txt
git commit -q -m "build: DELUGE_SDK_EXAMPLES / DELUGE_SDK_TESTS options (default ON)

A packaging build (the deluge-linux Buildroot package) sets both OFF to build
only the deluge library + its install rules; in-tree/CI builds keep examples +
tests by default."
echo done
```
Expected: `done`.

---

## Task 2: `deluge-linux` — create the `libdeluge` Buildroot package

**Files:**
- Create: `~/GitHub/deluge-linux/package/libdeluge/libdeluge.mk`
- Create: `~/GitHub/deluge-linux/package/libdeluge/Config.in`

**Interfaces:**
- Consumes: the `DELUGE_SDK_EXAMPLES`/`DELUGE_SDK_TESTS` options from Task 1.
- Produces: a `BR2_PACKAGE_LIBDELUGE` package that staging-installs `libdeluge.a` + `include/deluge/` + `deluge.pc`.

- [ ] **Step 1: Write `package/libdeluge/libdeluge.mk`**

```makefile
################################################################################
#
# libdeluge
#
# The Deluge userspace C HAL (the deluge-ndk repo): a static archive
# (libdeluge.a) + public headers + deluge.pc, installed into the STAGING sysroot
# ONLY. Apps statically link it and run from the SD card, so nothing lands in
# the target rootfs. It ships in the SDK sysroot the base bundle publishes
# (BR2_TARGET_SDK), which is how the Rust / C / C++ SDKs build against it.
#
# LOCAL DEVELOPMENT: sourced from a local deluge-ndk checkout. When deluge-ndk
# is published, switch to:
#   LIBDELUGE_VERSION = <git tag or sha>
#   LIBDELUGE_SITE = https://github.com/FirestormAudio/deluge-ndk.git
#   LIBDELUGE_SITE_METHOD = git
#
################################################################################

LIBDELUGE_VERSION = 0.1.0
LIBDELUGE_SITE ?= /home/kate/GitHub/deluge-ndk
LIBDELUGE_SITE_METHOD = local
LIBDELUGE_LICENSE = MIT OR Apache-2.0
LIBDELUGE_LICENSE_FILES = LICENSE-MIT LICENSE-APACHE

# Compiles against ALSA headers (pkg_check_modules(ALSA REQUIRED alsa)).
LIBDELUGE_DEPENDENCIES = alsa-lib

# Build only the library: no in-tree examples, no ctest suite.
LIBDELUGE_CONF_OPTS = -DDELUGE_SDK_EXAMPLES=OFF -DDELUGE_SDK_TESTS=OFF

# Staging only — the SDK links libdeluge.a into apps; the rootfs never needs it.
LIBDELUGE_INSTALL_STAGING = YES
LIBDELUGE_INSTALL_TARGET = NO

$(eval $(cmake-package))
```

- [ ] **Step 2: Write `package/libdeluge/Config.in`**

```
config BR2_PACKAGE_LIBDELUGE
	bool "libdeluge"
	depends on BR2_PACKAGE_ALSA_LIB
	help
	  The Deluge userspace C HAL (deluge-ndk): libdeluge.a + public headers
	  + deluge.pc, installed into the staging sysroot ONLY (never the target
	  rootfs). Apps statically link it; it ships in the SDK sysroot the base
	  bundle publishes, which is how the Rust / C / C++ SDKs build against it.
```

- [ ] **Step 3: Commit**

```bash
cd ~/GitHub/deluge-linux
git add package/libdeluge/libdeluge.mk package/libdeluge/Config.in
git commit -q -m "package/libdeluge: staging cmake-package for the deluge-ndk C HAL

First cmake-package in the tree. Staging-only; sourced from a local deluge-ndk
checkout for now (git site at publish). Ships in the SDK sysroot via make sdk."
echo done
```
Expected: `done`.

---

## Task 3: `deluge-linux` — wire the package into Config.in + defconfig

**Files:**
- Modify: `~/GitHub/deluge-linux/Config.in`
- Modify: `~/GitHub/deluge-linux/configs/deluge_defconfig`

- [ ] **Step 1: Source the package's Config.in**

In `~/GitHub/deluge-linux/Config.in`, add the `libdeluge` source line inside the `menu "Deluge packages"` block, so it reads:

```
menu "Deluge packages"
	source "$BR2_EXTERNAL_DELUGE_PATH/package/deluge-alsa-static/Config.in"
	source "$BR2_EXTERNAL_DELUGE_PATH/package/deluge-splash/Config.in"
	source "$BR2_EXTERNAL_DELUGE_PATH/package/libdeluge/Config.in"
endmenu
```

- [ ] **Step 2: Enable the package in the base defconfig**

In `~/GitHub/deluge-linux/configs/deluge_defconfig`, add `BR2_PACKAGE_LIBDELUGE=y` immediately after the existing `BR2_PACKAGE_DELUGE_SPLASH=y` line:

```
BR2_PACKAGE_DELUGE_ALSA_STATIC=y
BR2_PACKAGE_DELUGE_SPLASH=y
BR2_PACKAGE_LIBDELUGE=y
```

- [ ] **Step 3: Confirm the defconfig still resolves (no typo/dependency break)**

```bash
cd ~/GitHub/deluge-linux
make -C .buildroot BR2_EXTERNAL="$PWD" deluge_defconfig >/dev/null 2>&1 && \
  grep -q '^BR2_PACKAGE_LIBDELUGE=y' .buildroot/.config && echo "enabled in resolved .config" || echo "NOT enabled — check dependency"
```
Expected: `enabled in resolved .config`. (If not, `BR2_PACKAGE_ALSA_LIB` must be `y` — it is, per the defconfig — so a failure means a Config.in typo.)

- [ ] **Step 4: Commit**

```bash
cd ~/GitHub/deluge-linux
git add Config.in configs/deluge_defconfig
git commit -q -m "buildroot: enable libdeluge (Config.in source + defconfig)"
echo done
```
Expected: `done`.

---

## Task 4: Build the package and verify the staging install

**Files:** none (build + verification)

- [ ] **Step 1: Clean the local deluge-ndk build artifacts (so the local-method copy is clean)**

```bash
rm -rf ~/GitHub/deluge-ndk/build ~/GitHub/deluge-ndk/build-cross ~/GitHub/deluge-ndk/stage \
       ~/GitHub/deluge-ndk/stage-lib ~/GitHub/deluge-ndk/build-lib ~/GitHub/deluge-ndk/cmake/DelugeToolchain.cmake
echo cleaned
```
Expected: `cleaned`. (`_SITE_METHOD = local` copies the whole checkout; keep it lean.)

- [ ] **Step 2: Build the base profile (incremental — builds the new libdeluge package)**

```bash
cd ~/GitHub/deluge-linux
make base 2>&1 | tail -15
```
Expected: Buildroot configures (defconfig + base fragment), builds `libdeluge` (a `>>> libdeluge 0.1.0 …` build log), and finishes without error. Kernel/toolchain are already built, so this is incremental. If `libdeluge` fails to configure/compile, capture the `>>> libdeluge` error block.

- [ ] **Step 3: Verify libdeluge is in the staging sysroot**

```bash
S=~/GitHub/deluge-linux/.buildroot/output/staging/usr
ls "$S/lib/libdeluge.a" "$S/include/deluge/deluge.h" "$S/lib/pkgconfig/deluge.pc"
echo "--- deluge.pc ---" && cat "$S/lib/pkgconfig/deluge.pc"
```
Expected: all three paths listed; `deluge.pc` shows `Name: deluge`, `Libs: -L${libdir} -ldeluge`, `Requires: alsa`.

- [ ] **Step 4: Verify the archive is ARM (not a stray host build)**

```bash
S=~/GitHub/deluge-linux/.buildroot/output/staging/usr
ar t "$S/lib/libdeluge.a" | head -3
cd /tmp && ar x "$S/lib/libdeluge.a" core.c.o 2>/dev/null && file core.c.o && rm -f core.c.o
```
Expected: object members listed (e.g. `core.c.o`); `file` reports `ELF 32-bit LSB relocatable, ARM, EABI5`.

---

## Task 5: Verify the sysroot ships in the SDK tarball (bundle proof)

**Files:** none (build + verification)

This proves the bundle's `toolchain/…/sysroot` will carry `libdeluge` — i.e. `DELUGE_SDK_ROOT = <bundle>/toolchain/arm-buildroot-linux-musleabihf/sysroot/usr` is valid for Stage 3. It uses `make sdk` (the same step `mk-bundle.sh` runs), which is lighter than a full `make bundle` (no second-profile rootfs).

- [ ] **Step 1: Build the SDK toolchain tarball**

```bash
cd ~/GitHub/deluge-linux
make sdk 2>&1 | tail -6
ls .buildroot/output/images/*sdk-buildroot.tar.gz
```
Expected: the SDK tarball path is listed.

- [ ] **Step 2: Confirm libdeluge is inside the SDK tarball's sysroot**

```bash
cd ~/GitHub/deluge-linux
TB=$(ls .buildroot/output/images/*sdk-buildroot.tar.gz | head -1)
tar tzf "$TB" | grep -E 'arm-buildroot-linux-musleabihf/sysroot/usr/(lib/libdeluge\.a|include/deluge/deluge\.h|lib/pkgconfig/deluge\.pc)$'
```
Expected: all three paths appear in the listing — the sysroot shipped in the SDK tarball (hence in the bundle's `toolchain/` after `mk-bundle.sh` extracts it). A full `make bundle` would place these at `out/deluge-base-<ver>/toolchain/arm-buildroot-linux-musleabihf/sysroot/usr/…`; that heavier build is left for release time.

---

## Self-Review

**Spec coverage (reorg spec §5 "libdeluge sysroot in bundle" + §6 "→ deluge-linux" package half):** libdeluge as a Buildroot staging package ✓ (Task 2); wired into Config.in + defconfig ✓ (Task 3); sysroot lands in the bundle via `make sdk` ✓ (Task 5, verified) with **no `mk-bundle.sh` change** (Global Constraint honored). Local-path sourcing with a documented publish-time swap ✓ (Task 2 header + Global Constraints). The `deluge-ndk` CMake-option change needed to build only the library ✓ (Task 1). **mkimage relocation is explicitly Stage 2b, not here.**

**Placeholder scan:** none — every `.mk`/`Config.in`/defconfig edit is complete; the one machine-specific value (`LIBDELUGE_SITE = /home/kate/GitHub/deluge-ndk`) is deliberate (local, unpublished) with the exact publish-time replacement stated.

**Type/name consistency:** `DELUGE_SDK_EXAMPLES`/`DELUGE_SDK_TESTS` option names identical in Task 1 (declaration + guards) and Task 2 (`LIBDELUGE_CONF_OPTS`). `BR2_PACKAGE_LIBDELUGE` identical across Config.in, defconfig, and the verify grep. `libdeluge.a`/`deluge.pc`/`include/deluge/deluge.h` consistent from Task 1 install through Tasks 4-5 verification.

**Sequencing/greenness:** Task 1 (deluge-ndk) must land before Task 4 (the local copy needs the options). Tasks 2-3 (package + wiring) before Task 4 (build). Each repo stays buildable: Task 1 keeps default builds unchanged (options ON); deluge-linux only gains an opt-in package.
