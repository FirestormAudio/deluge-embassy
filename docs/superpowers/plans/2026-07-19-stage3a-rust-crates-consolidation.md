# Stage 3a — consolidate the Rust libdeluge crates into deluge-sdk — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Move the three Rust libdeluge crates (`deluge-sys`, `deluge-linux`, `deluge-linux-ui`) from `deluge-linux-sdk` into the `deluge-sdk` workspace — renaming `deluge-linux`→`deluge-hal-linux`, collapsing the `../../../deluge-sdk/...` cross-repo path-deps to intra-workspace paths — and verify they build against a `libdeluge` sysroot.

**Architecture:** The three crates are `std` and build for Linux targets (host or `armv7-unknown-linux-musleabihf`), **not** the device `no_std` triple — so, exactly like `deluge-sim-link`, they go in the workspace `exclude` list (built explicitly / as path-deps, never as default device members). `deluge-linux-ui`'s two cross-repo path-deps (`deluge-ui-toolkit`, `deluge-grid-toolkit`) become in-workspace `../` paths. `deluge-sys`'s `build.rs` still finds `libdeluge` via `DELUGE_SDK_ROOT`; verification points it at a **host** `libdeluge` built from `deluge-ndk` (the cross/bundle build is Stage 3b's `cargo deluge linux`).

**Tech Stack:** Rust (Cargo, edition 2021), bindgen, CMake (host `libdeluge`).

## Global Constraints

- The crates are **excluded** from the device workspace (they are `std`/Linux, not `armv7a-none-eabihf`). Add them to `deluge-sdk/Cargo.toml`'s `exclude`, mirroring `crates/deluge-sim-link`.
- **Rename `deluge-linux` → `deluge-hal-linux`** (package name, crate dir, and every `deluge_linux` code reference → `deluge_hal_linux`) to clear the collision with the `deluge-linux` platform *repo*. Do **not** rename `deluge-linux-ui` or `deluge-sys`. Word-boundary only: never touch `deluge_linux_ui`/`deluge-linux-ui`.
- Collapse `deluge-linux-ui`'s deps: `../../../deluge-sdk/crates/deluge-ui-toolkit` → `../deluge-ui-toolkit`, `../../../deluge-sdk/crates/deluge-grid-toolkit` → `../deluge-grid-toolkit`, and `deluge-linux` → `deluge-hal-linux` (`../deluge-hal-linux`).
- `DELUGE_SDK_ROOT` is the sysroot prefix `deluge-sys/build.rs` reads (`{root}/include`, `{root}/lib`). For 3a verification it points at a **host** `libdeluge` install (from `deluge-ndk`'s CMake host build).
- **Plain copy** (no git-history rewrite): the crates' history remains in `deluge-linux-sdk` (archived at Stage 4), consistent with Stage 2b. `deluge-linux-sdk` is **not modified** here; the Rust **examples** move in Stage 3b.
- Repo touched: `deluge-sdk` only (crates added + workspace edit). Device builds must remain unaffected (the excluded crates never build under the device target).

---

## Task 1: Copy + rename the three crates into deluge-sdk

**Files:**
- Create: `deluge-sdk/crates/deluge-sys/{Cargo.toml, build.rs, wrapper.h, src/lib.rs}` (copied)
- Create: `deluge-sdk/crates/deluge-hal-linux/{Cargo.toml, src/lib.rs}` (copied from `deluge-linux`, renamed)
- Create: `deluge-sdk/crates/deluge-linux-ui/{Cargo.toml, src/*.rs}` (copied)

**Interfaces:**
- Produces crate `deluge-hal-linux` (lib crate `deluge_hal_linux`), unchanged `deluge-sys`, and `deluge-linux-ui` depending on both plus the in-workspace toolkits.

- [ ] **Step 1: Copy the crate trees (excluding build artifacts)**

```bash
SRC=~/GitHub/deluge-linux-sdk/crates; DST=~/GitHub/deluge-sdk/crates
cp -r "$SRC/deluge-sys" "$DST/deluge-sys"
cp -r "$SRC/deluge-linux" "$DST/deluge-hal-linux"
cp -r "$SRC/deluge-linux-ui" "$DST/deluge-linux-ui"
rm -rf "$DST/deluge-sys/target" "$DST/deluge-hal-linux/target" "$DST/deluge-linux-ui/target"
ls "$DST/deluge-sys/build.rs" "$DST/deluge-hal-linux/src/lib.rs" "$DST/deluge-linux-ui/src/lib.rs"
```
Expected: the three paths listed. (If `deluge-sys` has no `wrapper.h`/other files, that's fine — copy carried whatever exists.)

- [ ] **Step 2: Rename the `deluge-hal-linux` package**

Edit `deluge-sdk/crates/deluge-hal-linux/Cargo.toml`: change `name = "deluge-linux"` → `name = "deluge-hal-linux"`. Its `deluge-sys` dep path stays `{ path = "../deluge-sys" }` (both are now siblings under `deluge-sdk/crates/`).

- [ ] **Step 3: Rewire `deluge-linux-ui`'s dependencies to in-workspace paths**

Edit `deluge-sdk/crates/deluge-linux-ui/Cargo.toml`'s `[dependencies]` to:

```toml
[dependencies]
deluge-hal-linux = { path = "../deluge-hal-linux" }
embedded-graphics = "0.8"
deluge-ui-toolkit = { path = "../deluge-ui-toolkit" }
deluge-grid-toolkit = { path = "../deluge-grid-toolkit" }
```

- [ ] **Step 4: Rename `deluge_linux` code references in the moved crates**

Replace every `deluge_linux` identifier with `deluge_hal_linux` in the two moved crates' sources (NOT `deluge_linux_ui`). Run:

```bash
DST=~/GitHub/deluge-sdk/crates
grep -rlED 'deluge_linux([^_]|$)' "$DST/deluge-hal-linux/src" "$DST/deluge-linux-ui/src" 2>/dev/null \
  | xargs -r sed -i -E 's/deluge_linux([^_])/deluge_hal_linux\1/g; s/deluge_linux$/deluge_hal_linux/'
echo "--- remaining bare deluge_linux refs (should be none) ---"
grep -rnE 'deluge_linux([^_]|$)' "$DST/deluge-hal-linux/src" "$DST/deluge-linux-ui/src" 2>/dev/null || echo "  none (good)"
```
Expected: `none (good)`. (Comments mentioning the hyphenated `deluge-linux` repo/path are fine and not matched.)

- [ ] **Step 5: Commit**

```bash
cd ~/GitHub/deluge-sdk
git add crates/deluge-sys crates/deluge-hal-linux crates/deluge-linux-ui
git commit -q -m "crates: import libdeluge Rust bindings from deluge-linux-sdk

deluge-sys + deluge-hal-linux (was deluge-linux; renamed off the platform-repo
name collision) + deluge-linux-ui. deluge-linux-ui's cross-repo ../../../ path
deps collapse to in-workspace ../ paths. These are std/Linux crates; they are
wired into the workspace exclude list in the next task."
echo done
```
Expected: `done`.

---

## Task 2: Wire the crates into the workspace (excluded, not device members)

**Files:**
- Modify: `deluge-sdk/Cargo.toml` (`exclude` list)

- [ ] **Step 1: Add the three crates to `exclude`**

In `deluge-sdk/Cargo.toml`, extend the `exclude` array to include the three new crates, alongside `crates/deluge-sim-link`:

```toml
exclude = [
  "tools/cargo-deluge",
  "tools/deluge-simulator",
  "tools/wren-web",
  "tools/wren-web-debug",
  "tools/wren-analyzer-wasm",
  "crates/deluge-sim-link",
  "crates/deluge-dsp-test",
  "crates/deluge-sys",
  "crates/deluge-hal-linux",
  "crates/deluge-linux-ui",
]
```

- [ ] **Step 2: Verify the device workspace is intact (excluded crates don't build for the device)**

`cargo metadata` resolves the workspace **without compiling** (so no `-Zbuild-std` is needed) and *errors* if a crate dir under the root is neither a member nor excluded — exactly the failure the `exclude` entry prevents:

```bash
cd ~/GitHub/deluge-sdk
cargo metadata --format-version=1 --no-deps >/tmp/wsmeta.json 2>/tmp/wsmeta.err \
  && echo "metadata OK" || { echo "metadata FAILED:"; cat /tmp/wsmeta.err; }
grep -cE '"name":"deluge-(sys|hal-linux|linux-ui)"' /tmp/wsmeta.json | sed 's/^/excluded-as-members: /'
grep -cE '"name":"deluge-sdk"' /tmp/wsmeta.json | sed 's/^/device-facade-member: /'
```
Expected: `metadata OK`, then `excluded-as-members: 0` (the three are excluded → not workspace members), then `device-facade-member: 1` (the device workspace is intact). A `metadata FAILED` with a "current package believes it's in a workspace" message means an `exclude` entry is missing or misspelled.

- [ ] **Step 3: Commit**

```bash
cd ~/GitHub/deluge-sdk
git add Cargo.toml
git commit -q -m "workspace: exclude the std libdeluge crates (deluge-sys/hal-linux/linux-ui)

Like deluge-sim-link, these build for Linux (host / armv7-musl), not the device
no_std triple, so they must not be default workspace members."
echo done
```
Expected: `done`.

---

## Task 3: Verify the crates build against a host libdeluge

**Files:** none (build a host `libdeluge`, then compile the crates)

- [ ] **Step 1: Build + install a host `libdeluge` from `deluge-ndk`**

```bash
cd ~/GitHub/deluge-ndk
rm -rf build-host stage-host
cmake -S . -B build-host -DDELUGE_SDK_EXAMPLES=OFF -DDELUGE_SDK_TESTS=OFF >/dev/null
cmake --build build-host >/dev/null 2>&1
cmake --install build-host --prefix "$PWD/stage-host" >/dev/null
ls stage-host/lib/libdeluge.a stage-host/include/deluge/deluge.h stage-host/lib/pkgconfig/deluge.pc
```
Expected: the three host-install paths listed. (This is the host `libdeluge` `deluge-sys` binds against.)

- [ ] **Step 2: Build the crates for the host against it**

```bash
cd ~/GitHub/deluge-sdk
DELUGE_SDK_ROOT=~/GitHub/deluge-ndk/stage-host \
  cargo build --manifest-path crates/deluge-linux-ui/Cargo.toml 2>&1 | tail -12
```
Expected: `Finished`. Building `deluge-linux-ui` pulls in `deluge-sys` (its `build.rs` runs bindgen against `$DELUGE_SDK_ROOT/include/deluge` — proving the headers resolve), `deluge-hal-linux` (the rename compiles cleanly), and the in-workspace `deluge-ui-toolkit` + `deluge-grid-toolkit` (proving the collapsed path-deps resolve). A build error here means a rename miss or a path-dep typo — fix in Task 1.

- [ ] **Step 3: Confirm `deluge-hal-linux` builds standalone too**

```bash
cd ~/GitHub/deluge-sdk
DELUGE_SDK_ROOT=~/GitHub/deluge-ndk/stage-host \
  cargo build --manifest-path crates/deluge-hal-linux/Cargo.toml 2>&1 | tail -5
```
Expected: `Finished` (the renamed HAL crate + `deluge-sys` compile on their own).

- [ ] **Step 4: No commit (verification only)**

If Steps 1-3 are green, Stage 3a is done. If a compile fails, fix the offending Task-1 edit (rename/path) and re-verify. Do not proceed to Stage 3b until green.

---

## Self-Review

**Spec coverage (reorg spec §6 "→ deluge-sdk: Rust crates" + §4 collapse `../../../` path-deps):** `deluge-sys` moved ✓; `deluge-linux`→`deluge-hal-linux` renamed ✓ (Task 1 Steps 2/4); `deluge-linux-ui` moved with in-workspace path-deps ✓ (Task 1 Step 3); wired as excluded crates ✓ (Task 2); device build unaffected ✓ (Task 2 Step 2); builds against a `libdeluge` sysroot ✓ (Task 3). Examples + `cargo deluge linux` + cross/bundle build are **Stage 3b**, not here.

**Placeholder scan:** none — every path, dep, and command is concrete. The one env value (`DELUGE_SDK_ROOT=~/GitHub/deluge-ndk/stage-host`) is produced by Task 3 Step 1.

**Type/name consistency:** `deluge-hal-linux` / lib `deluge_hal_linux` is used identically across the package rename (Task 1.2), the code rename (Task 1.4), `deluge-linux-ui`'s dep (Task 1.3), and the `exclude` list (Task 2.1). `DELUGE_SDK_ROOT` semantics match `deluge-sys/build.rs` (`{root}/include`, `{root}/lib`).

**Sequencing/greenness:** Task 1 (copy+rename) before Task 2 (exclude references the new dirs) before Task 3 (build). `deluge-linux-sdk` untouched; device workspace verified intact in Task 2.2. The moved crates being `exclude`d means a bare device `cargo build` never attempts them.
