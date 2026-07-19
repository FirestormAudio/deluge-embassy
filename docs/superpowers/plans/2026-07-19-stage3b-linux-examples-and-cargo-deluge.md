# Stage 3b — Linux examples + `cargo deluge linux` — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Move the four native Linux example apps (`rust-app`, `snake`, `launcher`, `terminal`) into `deluge-sdk`, and add a `cargo deluge linux` subcommand that builds an app for the Deluge's Linux userland (armv7 musl, static, against the bundle's `libdeluge` sysroot) and packs it into an app image / bare binary with the bundle's `deluge-mkimage`.

**Architecture:** The examples are native `deluge-hal-linux` apps (the synchronous libdeluge API) — std/Linux binaries, so they live in a new `examples-linux/` dir and are workspace-`exclude`d like the crates. `cargo deluge linux` mirrors the retired `xtask` (minus the sysroot build, which the bundle now ships): it builds the current-dir package for `armv7-unknown-linux-musleabihf` with `DELUGE_SDK_ROOT` = the bundle sysroot and the bundle toolchain on `PATH`, then invokes `$DELUGE_BASE/tools/deluge-mkimage`.

**Tech Stack:** Rust (Cargo, musl cross target), `cargo-deluge` (pure std), the deluge-linux bundle.

## Global Constraints

- The examples are std/Linux binaries → new dir `deluge-sdk/examples-linux/<name>/`, each added to the workspace `exclude` list (they never build for the device triple).
- Example deps rewrite to the consolidated crates: `deluge-linux` → `deluge-hal-linux` (`../../crates/deluge-hal-linux`); `deluge-linux-ui` → `../../crates/deluge-linux-ui`; `../../../deluge-sdk/crates/deluge-ui-toolkit` → `../../crates/deluge-ui-toolkit`. Code: `use deluge_linux` → `use deluge_hal_linux` (word-boundary; never `deluge_linux_ui`).
- `cargo deluge linux` requires **`DELUGE_BASE`** (an unpacked bundle). It sets `DELUGE_SDK_ROOT=$DELUGE_BASE/toolchain/arm-buildroot-linux-musleabihf/sysroot/usr`, prepends `$DELUGE_BASE/toolchain/bin` to `PATH`, builds `--target armv7-unknown-linux-musleabihf` (**no** `-Zbuild-std` — musl std is a rustup target), then packs with `$DELUGE_BASE/tools/deluge-mkimage`.
- Linux builds need the musl target config: add `[target.armv7-unknown-linux-musleabihf]` (`linker = "arm-linux-gcc"`, `rustflags = ["-C","target-feature=+crt-static"]`) to `deluge-sdk/.cargo/config.toml`. It only applies when that triple is selected — device/host builds are unaffected.
- Prereq for the verify: the rustup target `armv7-unknown-linux-musleabihf` is installed, and a **complete** bundle (toolchain + `libdeluge` sysroot + `tools/deluge-mkimage`) exists — a fresh `make bundle` (the old `0.2.0` predates Stages 2a/2b).
- Repo touched: `deluge-sdk` only. `deluge-linux-sdk` read-only (its examples' history stays there until Stage 4). Device + host-sim builds must remain unaffected.

---

## Task 1: Move the four Linux examples into deluge-sdk

**Files:**
- Create: `deluge-sdk/examples-linux/{rust-app,snake,launcher,terminal}/` (copied, deps + code rewritten)
- Modify: `deluge-sdk/Cargo.toml` (`exclude`)

- [ ] **Step 1: Copy the examples**

```bash
SRC=~/GitHub/deluge-linux-sdk/examples; DST=~/GitHub/deluge-sdk/examples-linux
mkdir -p "$DST"
for e in rust-app snake launcher terminal; do cp -r "$SRC/$e" "$DST/$e"; rm -rf "$DST/$e/target"; done
ls -d "$DST"/*/
```
Expected: the four example dirs listed under `examples-linux/`.

- [ ] **Step 2: Rewrite each example's `Cargo.toml` dependencies**

For every `examples-linux/*/Cargo.toml`, apply these dependency path/name changes (leave `embedded-graphics`, `libc`, `vte` untouched):
- `deluge-linux = { path = "../../crates/deluge-linux" }` → `deluge-hal-linux = { path = "../../crates/deluge-hal-linux" }`
- `deluge-linux-ui = { path = "../../crates/deluge-linux-ui" }` → (path unchanged; it's already `../../crates/deluge-linux-ui`, which now resolves in-workspace)
- `deluge-ui-toolkit = { path = "../../../deluge-sdk/crates/deluge-ui-toolkit" }` → `deluge-ui-toolkit = { path = "../../crates/deluge-ui-toolkit" }`

Concretely: `rust-app` gets only the first change; `snake` and `launcher` get the first + third; `terminal` gets only the first.

- [ ] **Step 3: Rename `deluge_linux` code references in the examples**

```bash
DST=~/GitHub/deluge-sdk/examples-linux
grep -rlE 'deluge_linux([^_]|$)' "$DST" --include='*.rs' \
  | xargs -r sed -i -E 's/deluge_linux([^_])/deluge_hal_linux\1/g; s/deluge_linux$/deluge_hal_linux/'
grep -rnE 'deluge_linux([^_]|$)' "$DST" --include='*.rs' || echo "  none (good)"
```
Expected: `none (good)`.

- [ ] **Step 4: Exclude the examples from the device workspace**

In `deluge-sdk/Cargo.toml`, add the four example paths to the `exclude` array (after the three crates from Stage 3a):

```toml
  "examples-linux/rust-app",
  "examples-linux/snake",
  "examples-linux/launcher",
  "examples-linux/terminal",
```

Verify the workspace still resolves:
```bash
cd ~/GitHub/deluge-sdk
cargo metadata --format-version=1 --no-deps >/tmp/m.json 2>/tmp/m.err && echo "metadata OK" || cat /tmp/m.err
grep -cE '"name":"(snake|launcher|terminal|rust-app)"' /tmp/m.json | sed 's/^/linux-examples-as-members: /'
```
Expected: `metadata OK`, `linux-examples-as-members: 0`.

- [ ] **Step 5: Verify the examples compile (host, against a host libdeluge)**

Uses the host `libdeluge` from Stage 3a (`~/GitHub/deluge-ndk/stage-host`); the explicit `--target` overrides the forced device triple (as in 3a).

```bash
cd ~/GitHub/deluge-sdk
for e in rust-app snake launcher terminal; do
  echo "== $e =="
  DELUGE_SDK_ROOT=~/GitHub/deluge-ndk/stage-host \
    cargo build --manifest-path examples-linux/$e/Cargo.toml --target x86_64-unknown-linux-gnu 2>&1 | tail -3
done
```
Expected: each prints `Finished`. (If `~/GitHub/deluge-ndk/stage-host` is absent, rebuild it per Stage 3a Task 3 Step 1.) A failure means a dep-path/rename miss — fix Step 2/3.

- [ ] **Step 6: Commit**

```bash
cd ~/GitHub/deluge-sdk
git add examples-linux Cargo.toml
git commit -q -m "examples-linux: import the native libdeluge apps (snake/terminal/launcher/rust-app)

Native deluge-hal-linux (synchronous API) apps, moved from deluge-linux-sdk.
Deps rewired to the in-workspace consolidated crates; std/Linux binaries, so
workspace-excluded like the crates."
echo done
```
Expected: `done`.

---

## Task 2: Add the musl target to `.cargo/config.toml`

**Files:**
- Modify: `deluge-sdk/.cargo/config.toml`

- [ ] **Step 1: Add the `armv7-unknown-linux-musleabihf` section**

Append to `deluge-sdk/.cargo/config.toml` (a new `[target.…]` section — sibling of the existing `[target.armv7-unknown-linux-gnueabihf]` QEMU block):

```toml
# ---------------------------------------------------------------------------
# armv7-unknown-linux-musleabihf — the Deluge's Linux userland (deluge-linux).
# `cargo deluge linux` selects this triple and prepends the bundle toolchain's
# bin to PATH so `arm-linux-gcc` resolves; apps are fully static (crt-static)
# because they run from the SD card and must not depend on any image's rootfs.
# ---------------------------------------------------------------------------
[target.armv7-unknown-linux-musleabihf]
linker = "arm-linux-gcc"
rustflags = ["-C", "target-feature=+crt-static"]
```

- [ ] **Step 2: Verify it doesn't disturb device/host config resolution**

```bash
cd ~/GitHub/deluge-sdk
cargo metadata --format-version=1 --no-deps >/dev/null 2>/tmp/c.err && echo "config OK" || cat /tmp/c.err
grep -A2 'armv7-unknown-linux-musleabihf' .cargo/config.toml
```
Expected: `config OK`, and the new section shows the linker + crt-static. (The section is inert unless that triple is built.)

- [ ] **Step 3: Commit**

```bash
cd ~/GitHub/deluge-sdk
git add .cargo/config.toml
git commit -q -m "cargo: musl target config for deluge-linux app builds (linker + crt-static)"
echo done
```
Expected: `done`.

---

## Task 3: Add the `cargo deluge linux` subcommand

**Files:**
- Create: `deluge-sdk/tools/cargo-deluge/src/linux.rs`
- Modify: `deluge-sdk/tools/cargo-deluge/src/main.rs` (dispatch + `mod` + help)
- Modify: `deluge-sdk/tools/cargo-deluge/src/build.rs` (make `package_name`, `target_dir` `pub(crate)`)

**Interfaces:**
- Consumes: `build::package_name() -> Result<String,String>`, `build::target_dir() -> Result<PathBuf,String>`, `util::arg_value(&[String], &str) -> Option<String>`.

- [ ] **Step 1: Expose the two build helpers**

In `deluge-sdk/tools/cargo-deluge/src/build.rs`, change `fn package_name(` → `pub(crate) fn package_name(` and `fn target_dir(` → `pub(crate) fn target_dir(`. (Leave `extract_json_string` private.)

- [ ] **Step 2: Create `linux.rs`**

Create `deluge-sdk/tools/cargo-deluge/src/linux.rs`:

```rust
//! `cargo deluge linux`: build the current app for the Deluge's Linux userland
//! (armv7 musl, static) against the bundle's libdeluge sysroot, then pack it
//! into an app image — or a `--bare` binary — with the bundle's deluge-mkimage.
//!
//! Unlike the device path, this needs no `-Zbuild-std` (the musl std is a
//! rustup target) and no local sysroot build (the bundle ships libdeluge +
//! deluge-mkimage). Point it at an unpacked bundle with `DELUGE_BASE`.

use std::path::PathBuf;
use std::process::Command;

use crate::build::{package_name, target_dir};
use crate::util::arg_value;

/// The Deluge's Linux userland triple (musl, static).
const LINUX_TARGET: &str = "armv7-unknown-linux-musleabihf";

pub(crate) fn cmd_linux(args: &[String]) -> Result<(), String> {
    let debug = args.iter().any(|a| a == "--debug");
    let bare = args.iter().any(|a| a == "--bare");
    let out = arg_value(args, "--out");

    let base = std::env::var("DELUGE_BASE")
        .map_err(|_| "set DELUGE_BASE to an unpacked deluge-linux bundle".to_string())?;
    let sysroot = format!("{base}/toolchain/arm-buildroot-linux-musleabihf/sysroot/usr");
    let mkimage = PathBuf::from(&base).join("tools/deluge-mkimage");
    if !mkimage.is_file() {
        return Err(format!("bundle has no packer at {}", mkimage.display()));
    }
    // Prepend the bundle toolchain bin so the config's `arm-linux-gcc` resolves.
    let path = format!(
        "{base}/toolchain/bin:{}",
        std::env::var("PATH").unwrap_or_default()
    );

    let mut cmd = Command::new("cargo");
    cmd.args(["build", "--target", LINUX_TARGET]);
    if !debug {
        cmd.arg("--release");
    }
    cmd.env("DELUGE_SDK_ROOT", &sysroot).env("PATH", &path);
    if !cmd.status().map_err(|e| format!("cargo: {e}"))?.success() {
        return Err("build failed".to_string());
    }

    let name = package_name()?;
    let profile = if debug { "debug" } else { "release" };
    let bin = target_dir()?.join(LINUX_TARGET).join(profile).join(&name);
    if !bin.is_file() {
        return Err(format!("binary not found at {}", bin.display()));
    }

    let mut mk = Command::new(&mkimage);
    mk.arg(&bin);
    let product = if bare {
        let p = target_dir()?.join("bare").join(&name);
        if let Some(dir) = p.parent() {
            std::fs::create_dir_all(dir).ok();
        }
        mk.arg("--bare").arg("-o").arg(&p);
        p
    } else {
        let stem = out.unwrap_or_else(|| name.to_uppercase());
        let p = target_dir()?.join(format!("{stem}.ELF"));
        mk.arg("-o").arg(&p);
        p
    };
    if !mk.status().map_err(|e| format!("deluge-mkimage: {e}"))?.success() {
        return Err("deluge-mkimage failed".to_string());
    }
    println!("packed {}", product.display());
    Ok(())
}
```

- [ ] **Step 3: Wire it into `main.rs`**

In `deluge-sdk/tools/cargo-deluge/src/main.rs`: add `mod linux;` alongside the other `mod` lines, and add a dispatch arm `"linux" => linux::cmd_linux(rest),` alongside the others. Add a one-line entry to the help text near the `sim` line:

```
//! - `cargo deluge linux <app-dir> [--bare] [--out <NAME>]` — build the current
//!   app for the Deluge's Linux userland and pack it with the bundle's
//!   deluge-mkimage. Needs DELUGE_BASE set to an unpacked bundle.
```

- [ ] **Step 4: Confirm cargo-deluge still builds**

```bash
cd ~/GitHub/deluge-sdk
cargo build --manifest-path tools/cargo-deluge/Cargo.toml 2>&1 | tail -3
```
Expected: `Finished` (the tool compiles with the new subcommand).

- [ ] **Step 5: Commit**

```bash
cd ~/GitHub/deluge-sdk
git add tools/cargo-deluge/src/linux.rs tools/cargo-deluge/src/main.rs tools/cargo-deluge/src/build.rs
git commit -q -m "cargo-deluge: add \`linux\` subcommand (build musl + pack via bundle mkimage)

Builds the current app for armv7-unknown-linux-musleabihf against the bundle's
libdeluge sysroot (DELUGE_SDK_ROOT) with the bundle toolchain on PATH, then packs
it (image or --bare) with \$DELUGE_BASE/tools/deluge-mkimage. Replaces the retired
deluge-linux-sdk xtask; no local sysroot build (the bundle ships it)."
echo done
```
Expected: `done`.

---

## Task 4: End-to-end verify — build + pack an app from a fresh bundle

**Files:** none (build + pack)

**Prereq:** a complete bundle at `~/GitHub/deluge-linux/out/deluge-base-stage3` (from `make bundle VERSION=stage3`), and `rustup target add armv7-unknown-linux-musleabihf`.

- [ ] **Step 1: Confirm the bundle has the three pieces the subcommand needs**

```bash
B=~/GitHub/deluge-linux/out/deluge-base-stage3
ls "$B/toolchain/bin/arm-linux-gcc" \
   "$B/toolchain/arm-buildroot-linux-musleabihf/sysroot/usr/lib/libdeluge.a" \
   "$B/tools/deluge-mkimage"
```
Expected: all three listed. (If the dir is missing, the background `make bundle VERSION=stage3` has not finished — wait for it.)

- [ ] **Step 2: Ensure the musl rust target is installed**

```bash
rustup target add armv7-unknown-linux-musleabihf 2>&1 | tail -1
```
Expected: installed (or "up to date").

- [ ] **Step 3: `cargo deluge linux --bare` on `snake`**

Build `cargo-deluge` (a host tool) with an explicit host `--target` — the repo's forced `armv7a-none-eabihf` default can't build this `std` tool:

```bash
cd ~/GitHub/deluge-sdk
CD=$(pwd)/tools/cargo-deluge
cargo build --manifest-path "$CD/Cargo.toml" --target x86_64-unknown-linux-gnu -q
BIN="$CD/target/x86_64-unknown-linux-gnu/debug/cargo-deluge"
cd examples-linux/snake
DELUGE_BASE=~/GitHub/deluge-linux/out/deluge-base-stage3 "$BIN" linux --bare 2>&1 | tail -8
```
Expected: cargo builds `snake` for `armv7-unknown-linux-musleabihf`, then `packed …/bare/snake`. Confirm it is a static ARM binary:
```bash
BARE=$(cd ~/GitHub/deluge-sdk && cargo metadata --format-version=1 --no-deps --manifest-path examples-linux/snake/Cargo.toml 2>/dev/null | grep -o '"target_directory":"[^"]*"' | cut -d'"' -f4)/armv7-unknown-linux-musleabihf/release/snake
file "$BARE"
```
Expected: `ELF 32-bit LSB executable, ARM, EABI5 … statically linked`.

- [ ] **Step 4: `cargo deluge linux` image on `snake`**

```bash
cd ~/GitHub/deluge-sdk/examples-linux/snake
BIN=$(cd ~/GitHub/deluge-sdk && pwd)/tools/cargo-deluge/target/x86_64-unknown-linux-gnu/debug/cargo-deluge
DELUGE_BASE=~/GitHub/deluge-linux/out/deluge-base-stage3 "$BIN" linux 2>&1 | tail -6
```
Expected: `packed …/SNAKE.ELF`. That ELF is the appliance image `deluge-mkimage` baked from the bundle's rootfs + the snake binary — the full `cargo deluge linux` path working against a real bundle.

- [ ] **Step 5: No commit (verification only)**

If Steps 1-4 are green, Stage 3b — and the functional repo split — is complete. A failure in Step 3/4 that is a build issue routes to Task 1/2; a packing issue routes to Task 3 (or the bundle/Stage 2b).

---

## Self-Review

**Spec coverage (reorg spec §6 "→ deluge-sdk: examples + app-build/package half of xtask"):** the four Rust examples moved ✓ (Task 1); `cargo deluge linux` replaces xtask's build+package ✓ (Task 3), consuming the bundle sysroot + `mkimage` (no local sysroot build) ✓; musl target wired ✓ (Task 2); end-to-end image + bare verified against a real bundle ✓ (Task 4). `DELUGE_SDK_ROOT` repointed at the bundle sysroot ✓ (Task 3 Global Constraints).

**Placeholder scan:** none — `linux.rs` is complete; every path/dep/command concrete. The bundle path `deluge-base-stage3` is the artifact the background `make bundle VERSION=stage3` produces.

**Type/name consistency:** `LINUX_TARGET = "armv7-unknown-linux-musleabihf"` matches the `.cargo/config.toml` section (Task 2) and the rustup target (Task 4.2). `package_name`/`target_dir` are made `pub(crate)` (Task 3.1) exactly where `linux.rs` imports them (Task 3.2). `DELUGE_SDK_ROOT` = `$DELUGE_BASE/toolchain/arm-buildroot-linux-musleabihf/sysroot/usr` is identical in `linux.rs` and Stage 2a's verified sysroot location.

**Sequencing/greenness:** Task 1 (examples + exclude) is independent and host-verified. Task 2 (config) + Task 3 (subcommand) enable Task 4. Device/host-sim builds are untouched (examples excluded; musl config inert unless selected). `deluge-linux-sdk` untouched.
