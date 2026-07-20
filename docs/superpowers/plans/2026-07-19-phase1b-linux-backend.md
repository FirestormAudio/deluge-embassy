# Phase 1b — native Linux backend (`plat::linux`) + `oled_hello` on hardware — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add the third facade backend (`plat::linux`, over `libdeluge`/`deluge-hal-linux`) behind a `linux` cargo feature, so an existing async `#[deluge::app]` (`oled_hello`) builds via `cargo deluge linux --features linux` and runs on a real Deluge, drawing to the OLED.

**Architecture:** Phase 1a left `plat/mod.rs` with a `device | sim | linux` selection (linux gated on the `linux` feature). This phase declares that feature (swapping the sim deps out and `deluge-hal-linux` in), writes `plat/linux.rs` (OLED + input real; `sync_led`/`pic`/`oled_init` no-ops so `oled_hello` runs; the rest `unimplemented!()`), a `linux.rs` process-wide libdeluge handle (mirroring `host.rs`), `__rt::linux::run` (mirroring `__rt::host::run`, minus GUI/panel/ring — the executor runs on the main thread, libdeluge owns its input/audio threads), and the macro's third `main` arm. `cargo deluge linux` gains a `--no-default-features` + `--features` passthrough.

**Tech Stack:** Rust (std/musl), embassy (`platform-std`), `deluge-hal-linux` (libdeluge FFI), the deluge-linux bundle.

## Global Constraints

- **Feature discriminator (matches `plat/mod.rs` from Phase 1a):** `device = target_os="none"`; `linux = feature="linux"`; `sim = feature="sim"`. `default = ["sim"]` so existing device+sim workflows are unchanged.
- **`deluge-simulator` (iced) must NOT be in the linux build** (it can't cross-compile for musl-arm). Gate it out: make `deluge-sim-link` + `deluge-simulator` **optional**, enabled only by `sim`; `deluge-hal-linux` optional, enabled only by `linux`.
- **`plat::linux` op set = exactly the ops in `plat::sim`** (same names/signatures, ~35 fns — the crate must compile). For this milestone: `oled_flush` + `input_start_pump` are **real**; `oled_init_panel`, `pic_wait_ready`, `pic_ensure_started`, `sync_led_init/set/is_set_high/is_set_low` are **no-ops** (so `oled_hello` runs); **all others** are `unimplemented!("<op> is not on the linux backend yet")`.
- **Behavior-preserving for device+sim.** After every task, `cargo build-fw -p deluge-sdk` (device) and `cargo build -p deluge-sdk --target x86_64-unknown-linux-gnu` (sim, default features) + `cargo test` stay green.
- The linux build is `armv7-unknown-linux-musleabihf` with `DELUGE_SDK_ROOT=<bundle>/toolchain/arm-buildroot-linux-musleabihf/sysroot/usr` and `<bundle>/toolchain/bin` on `PATH` — exactly what `cargo deluge linux` already sets. Bundle: `~/GitHub/deluge-linux/out/deluge-base-stage3`.
- Repo: `deluge-sdk` only, branch `feat/linux-backend`.

---

## Task 1: Cargo features — optional backend deps + `sim`/`linux`

**Files:**
- Modify: `crates/deluge-sdk/Cargo.toml`
- Modify: `crates/deluge-sdk/src/plat/mod.rs`

- [ ] **Step 1: Make the backend deps optional + add features**

In `crates/deluge-sdk/Cargo.toml`'s `[target.'cfg(not(target_os = "none"))'.dependencies]`, mark the sim deps optional and add the linux dep:
```toml
deluge-sim-link = { path = "../deluge-sim-link", optional = true }
deluge-simulator = { path = "../../tools/deluge-simulator", optional = true }
deluge-hal-linux = { path = "../deluge-hal-linux", optional = true }
```
In `[features]`:
```toml
default = ["sim"]
## Host desktop simulator backend (cargo deluge sim).
sim = ["dep:deluge-sim-link", "dep:deluge-simulator"]
## Native Linux backend over libdeluge (cargo deluge linux).
linux = ["dep:deluge-hal-linux"]
```
(Leave `alloc`/`usb-log`/`audio-irq`/`rtt` as-is.)

- [ ] **Step 2: Point `plat::sim` at the `sim` feature**

In `crates/deluge-sdk/src/plat/mod.rs`, change the sim arm from `cfg(all(not(target_os = "none"), not(feature = "linux")))` to `cfg(feature = "sim")` (both the `mod sim;` and its `pub(crate) use sim::*;`). Device (`target_os="none"`) and linux (`feature="linux"`) arms are unchanged.

- [ ] **Step 3: Verify device + sim (default) still green**

```bash
cd ~/GitHub/deluge-sdk
cargo build-fw -p deluge-sdk 2>&1 | tail -3                                   # device (none): sim feature's optional deps are not-none-only → no-op here
cargo build -p deluge-sdk --target x86_64-unknown-linux-gnu 2>&1 | tail -3    # sim: default=["sim"]
cargo test -p deluge-sdk --target x86_64-unknown-linux-gnu 2>&1 | tail -3
```
Expected: all Finished/pass. (If the device build errors on `dep:deluge-sim-link` for the `none` target, fall back to `default = []` and have `cargo deluge sim` pass `--features sim` — note it in the report.)

- [ ] **Step 4: Commit** — `git add crates/deluge-sdk/Cargo.toml crates/deluge-sdk/src/plat/mod.rs && git commit -m "feat(linux): sim/linux cargo features; backend deps optional"`

---

## Task 2: The linux backend — handle, runtime, macro arm, `plat::linux`

**Files:**
- Create: `crates/deluge-sdk/src/linux.rs`, `crates/deluge-sdk/src/plat/linux.rs`
- Modify: `crates/deluge-sdk/src/lib.rs` (`mod linux;` gate + `__rt::linux::run` + re-gate `mod host`/`__rt::host`), `crates/deluge-sdk/src/plat/mod.rs`, `crates/deluge-sdk-macros/src/lib.rs` (third `main` arm)

**The final cfg partition** (apply everywhere a backend is selected): `device = cfg(target_os = "none")`; `sim = cfg(all(not(target_os = "none"), feature = "sim"))`; `linux = cfg(all(not(target_os = "none"), feature = "linux"))`. The `not(target_os = "none")` guard is required so an app that enables `sim` (or `linux`) for its **device** build doesn't double-compile a second backend.

- [ ] **Step 0: Finalize the sim/linux/device gating**
  - `crates/deluge-sdk/src/plat/mod.rs`: the **sim** arm is `cfg(all(not(target_os = "none"), feature = "sim"))` (Task 1 left it as bare `feature = "sim"` — add the guard); the **linux** arm is `cfg(all(not(target_os = "none"), feature = "linux"))` (add the guard to `mod linux;` + its `use`).
  - `crates/deluge-sdk/src/lib.rs`: re-gate the simulator-backend code from `cfg(not(target_os = "none"))` to `cfg(all(not(target_os = "none"), feature = "sim"))` — specifically `mod host;` and the `__rt::host` module (they use `deluge_sim_link`/`deluge_simulator`, which the linux build must not pull). **Leave** the genuinely-both-backends `cfg(not(target_os = "none"))` blocks alone — the host `StereoFrame` def (`audio.rs`), the host `SdError`/`FatError`/`Sd` def (`sd.rs`), and the crate-level `no_std`-only-on-device — sim *and* linux need those.

- [ ] **Step 1: `linux.rs` — process-wide libdeluge handle (mirror `host.rs`)**

```rust
//! Native Linux backend, active under `feature = "linux"` — the capability
//! modules drive real hardware through `libdeluge` (`deluge-hal-linux`) instead
//! of the simulator panel. Owns the process-wide handle, set up once by
//! [`crate::__rt::linux::run`] before any app code runs.
use std::sync::{Mutex, MutexGuard, OnceLock};

use deluge_hal_linux::Deluge;

static DELUGE: OnceLock<Mutex<Deluge>> = OnceLock::new();

pub(crate) fn init(d: Deluge) {
    let _ = DELUGE.set(Mutex::new(d));
}
pub(crate) fn dev() -> MutexGuard<'static, Deluge> {
    DELUGE
        .get()
        .expect("libdeluge not opened (run via `cargo deluge linux`)")
        .lock()
        .unwrap()
}
```
Gate its module decl in `lib.rs`: `#[cfg(all(not(target_os = "none"), feature = "linux"))] mod linux;` (next to `#[cfg(not(target_os="none"))] mod host;`).

- [ ] **Step 2: `plat/linux.rs` — the op set (OLED+input real; the rest no-op/unimplemented)**

Create with **every op `plat::sim` defines** (same names/signatures — copy the signatures from `plat/sim.rs`). Bodies:
- `oled_flush(fb: &FrameBuffer)` (async): `let _ = crate::linux::dev().oled_write(fb.as_bytes());`
- `input_start_pump(spawner)`: start libdeluge input delivery, mapping to `EVENTS` (see Step 3 note); `spawner` unused (libdeluge owns the thread).
- No-ops (so `oled_hello` runs): `oled_init_panel` (async, `{}`), `pic_wait_ready` (async, `{}`), `pic_ensure_started(_spawner)` (`{}`), `sync_led_init() -> bool` (`false`), `sync_led_set(_on)` (`{}`), `sync_led_is_set_high(state) -> bool` (`state`), `sync_led_is_set_low(state) -> bool` (`!state` — match `plat::sim`'s exact logic).
- Everything else — `audio_run`, `pads_flush`, `pads_set_brightness_interval`, `leds_set/clear/gold_knob`, `cv_gate_init`, `cv_set`, `gate_set`, `midi_init/send/recv/try_recv`, `clock_in_init/wait_edge/count/last_edge`, `jacks_*`, `sd_init_card/read/write` — body `unimplemented!("<op> is not on the linux backend yet")`. (For `-> !`/async/typed-return fns, `unimplemented!()` satisfies any signature.)

Add the needed `use` lines to match `plat::sim`'s imports (`FrameBuffer`, `PadLeds`, `Spawner`, `Instant`, `crate::sd::{SdError,FatError}`, `crate::audio::StereoFrame`).

- [ ] **Step 3: `input_start_pump` — libdeluge callback → `EVENTS`**

In `plat::linux::input_start_pump`, call `crate::linux::dev().input_start(cb)` where `cb: FnMut(deluge_hal_linux::Event) + Send + 'static` maps the hal `Event` (`kind,id,value,x,y,pressure`) to the SDK `crate::input::Event` and `try_send`s it into `crate::input::EVENTS` (the same static the device/sim pumps feed; `pub(crate)` since Phase 1a). Match the field mapping to how `route_pic_event`/`host_input_pump` build the SDK `Event`. If `input_start` returns `Err`, log and continue (no input device is non-fatal).

- [ ] **Step 4: `__rt::linux::run` (mirror `__rt::host::run`, no GUI/panel/ring)**

Add to the `__rt` module in `lib.rs`, gated `#[cfg(feature = "linux")]`:
```rust
#[cfg(all(not(target_os = "none"), feature = "linux"))]
pub mod linux {
    use super::Spawner;
    use embassy_executor::Executor;

    /// Open libdeluge, then run the app on a std executor on the MAIN thread
    /// (no GUI competes for it; libdeluge owns its own input/audio threads).
    pub fn run(setup: impl FnOnce(), spawn: impl FnOnce(Spawner) + Send + 'static) {
        let _ = env_logger::try_init();
        let dev = deluge_hal_linux::Deluge::open().expect("deluge_open failed");
        crate::linux::init(dev);
        setup();
        let executor: &'static mut Executor = Box::leak(Box::new(Executor::new()));
        executor.run(move |spawner| {
            crate::plat::input_start_pump(spawner);
            spawn(spawner);
        });
    }
}
```

- [ ] **Step 5: Macro — third `main` arm**

In `crates/deluge-sdk-macros/src/lib.rs`, re-gate the existing non-device `fn main` to the sim case and add the linux case:
```rust
#[cfg(all(not(target_os = "none"), feature = "sim"))]
fn main() { ::deluge::__rt::host::run(|| { #setup_call; }, |spawner| { spawner.spawn(__deluge_app_main(spawner).unwrap()); }) }

#[cfg(all(not(target_os = "none"), feature = "linux"))]
fn main() { ::deluge::__rt::linux::run(|| { #setup_call; }, |spawner| { spawner.spawn(__deluge_app_main(spawner).unwrap()); }) }
```

- [ ] **Step 6: Verify — linux musl build compiles + device/sim still green**

```bash
cd ~/GitHub/deluge-sdk
B=~/GitHub/deluge-linux/out/deluge-base-stage3
# linux backend compiles for musl against the bundle libdeluge sysroot:
DELUGE_SDK_ROOT="$B/toolchain/arm-buildroot-linux-musleabihf/sysroot/usr" \
PATH="$B/toolchain/bin:$PATH" \
  cargo build -p deluge-sdk --no-default-features --features linux \
    --target armv7-unknown-linux-musleabihf 2>&1 | tail -6
# device + sim unaffected:
cargo build-fw -p deluge-sdk 2>&1 | tail -2
cargo build -p deluge-sdk --target x86_64-unknown-linux-gnu 2>&1 | tail -2
```
Expected: the musl `--features linux` build reaches `Finished` (deluge-sys binds libdeluge from the bundle; `plat::linux` compiles; no `deluge-simulator`), and device+sim still Finished. A compile error in the musl build is a signature mismatch between `plat::linux` and the capability call sites, or a missing op — fix it.

- [ ] **Step 7: Commit** — `git add crates/deluge-sdk/src/linux.rs crates/deluge-sdk/src/plat/linux.rs crates/deluge-sdk/src/lib.rs crates/deluge-sdk-macros/src/lib.rs && git commit -m "feat(linux): plat::linux (OLED+input real, rest stubbed) + runtime + macro arm"`

---

## Task 3: Build `oled_hello` for linux via `cargo deluge linux`

**Files:**
- Modify: `crates/deluge-sdk/tools/cargo-deluge/src/linux.rs`, `examples/oled_hello/Cargo.toml`

- [ ] **Step 1: `cargo deluge linux` — `--no-default-features` + `--features` passthrough**

In `tools/cargo-deluge/src/linux.rs`'s `cmd_linux`, add `--no-default-features` to the `cargo build` args and forward a `--features <list>` value if present (parse via the existing `arg_value(args, "--features")`; pass `--features <list>` when non-empty). This lets a facade app select `linux` while a native `deluge-hal-linux` app builds with none. Rebuild the tool (`cargo build --manifest-path tools/cargo-deluge/Cargo.toml --target x86_64-unknown-linux-gnu`).

- [ ] **Step 2: `oled_hello` — feature wiring for the three backends**

Edit `examples/oled_hello/Cargo.toml`:
```toml
[dependencies]
deluge = { path = "../../crates/deluge-sdk", package = "deluge-sdk", default-features = false }
embassy-time = { workspace = true, features = ["tick-hz-1_000_000"] }
embedded-graphics = "0.8"

[features]
default = ["sim"]
sim = ["deluge/sim"]
linux = ["deluge/linux"]
rtt = ["deluge/rtt"]
```
(`default-features=false` on `deluge` so `cargo deluge linux --features linux` drops the sim backend; `default=["sim"]` keeps `cargo deluge sim` / device builds working.)

- [ ] **Step 3: Build the linux image**

```bash
cd ~/GitHub/deluge-sdk/examples/oled_hello
BIN=$(cd ~/GitHub/deluge-sdk && pwd)/tools/cargo-deluge/target/x86_64-unknown-linux-gnu/debug/cargo-deluge
DELUGE_BASE=~/GitHub/deluge-linux/out/deluge-base-stage3 "$BIN" linux --features linux 2>&1 | tail -8
# and the bare binary:
DELUGE_BASE=~/GitHub/deluge-linux/out/deluge-base-stage3 "$BIN" linux --features linux --bare 2>&1 | tail -6
```
Expected: `packed …/OLED_HELLO.ELF` (appliance image) and `packed …/bare/oled_hello`. Confirm the bare binary is a static ARM executable (`file …/armv7-unknown-linux-musleabihf/release/oled_hello` → `ELF 32-bit … ARM … statically linked`). This proves the whole facade→libdeluge→image pipeline builds.

- [ ] **Step 4: Verify device + sim of `oled_hello` still build**

```bash
cd ~/GitHub/deluge-sdk/examples/oled_hello
cargo deluge build 2>&1 | tail -2 || true          # device ELF (default=sim, no-op on none)
cargo deluge sim --help >/dev/null 2>&1 || true     # sim path intact
```
Expected: the device build produces an ELF (the feature rework didn't break the existing backends).

- [ ] **Step 5: Commit** — `git add crates/deluge-sdk/tools/cargo-deluge/src/linux.rs examples/oled_hello/Cargo.toml && git commit -m "feat(linux): cargo deluge linux --features passthrough; oled_hello linux feature"`

---

## Task 4: Run `oled_hello` on real hardware (maintainer, on-device)

**Files:** none (deploy + observe). **This is a manual on-device step — the first time this SDK runs on the Deluge-Linux userland.**

- [ ] **Step 1:** Copy the artifact to the Deluge SD card — the appliance `OLED_HELLO.ELF` to `/APPS/`, or the bare `oled_hello` to `/LINUX/APPS/` (launcher-run). Boot it.
- [ ] **Step 2:** Confirm **the OLED draws** what `oled_hello` renders. That is the end-to-end proof: macro linux `main` → std executor + time driver → `libdeluge` open → `plat::linux` OLED path → panel.
- [ ] **Step 3:** Note first-run friction separately from the backend (device nodes `/dev/deluge-*`, the fbdev/ALSA discovery in `deluge_open`, the launcher/appliance path). The `sync_led` toggle is a silent no-op this milestone (expected); input is wired but `oled_hello` doesn't read it.

---

## Self-Review

**Spec coverage (Phase 1 spec §3b–3d, §4):** `linux` feature + deps swap ✓ (Task 1); `linux.rs` handle + `__rt::linux::run` + macro arm ✓ (Task 2); `plat::linux` with OLED+input real, rest stubbed ✓ (Task 2, adjusted so `oled_hello`'s `sync_led`+`pic` are no-ops not panics — it uses both); `cargo deluge linux --features linux` builds+packs `oled_hello` ✓ (Task 3); on-device OLED draw ✓ (Task 4). Behavior-preserving for device+sim, verified each task.

**Placeholder scan:** none in the code — `linux.rs`, `__rt::linux::run`, and the macro arm are complete; `plat::linux`'s op list is fully enumerated (real / no-op / `unimplemented!()` each named). `unimplemented!()` bodies are the deliberate skeleton scope, not gaps.

**Type consistency:** `plat::linux` op names/signatures mirror `plat::sim` exactly (the crate won't compile otherwise — Task 2 Step 6 is that gate). `crate::linux::{init,dev}`, `deluge_hal_linux::{Deluge,Event}`, `EVENTS` (`pub(crate)`), and `DELUGE_SDK_ROOT`/bundle paths match their definitions. The macro's `#[cfg(all(not(none), not(feature="linux")))]` / `feature="linux"` arms partition the non-device case exactly.

**Sequencing/greenness:** Task 1 (features) verified device+sim before any linux code; Task 2 adds the whole backend and gates on the musl compile + device+sim; Task 3 packages + re-checks device; Task 4 is the manual run. `default=["sim"]` keeps every existing example building unchanged.
