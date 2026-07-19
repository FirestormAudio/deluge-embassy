# Phase 1a — `plat` module refactor (device+sim) — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Extract each capability module's inline `#[cfg]` backend arms into a single cfg-selected `plat` module (`plat::device` / `plat::sim`), with the capability methods delegating to `plat::<op>` — **behavior-preserving** for the device and sim backends — so the third backend (`plat::linux`, Phase 1b) is one file.

**Architecture:** New `crates/deluge-sdk/src/plat/`: `mod.rs` selects exactly one backend submodule by cfg and re-exports it; `device.rs` and `sim.rs` hold the per-backend op bodies moved verbatim from the capability modules' `#[cfg(target_os="none")]` / `#[cfg(not(target_os="none"))]` arms. Capability public APIs are unchanged; only method *bodies* become `plat::<op>(…)` calls. `linux.rs` is added in Phase 1b.

**Tech Stack:** Rust (no_std device + std sim), embassy, `deluge-bsp`, `deluge-sim-link`.

## Global Constraints

- **Pure refactor — zero behavior change.** Device build, `cargo deluge sim`, and the test suite must stay green after *every* task. Any diff that changes output/behavior is a bug, not the plan.
- **The move is mechanical:** a capability method's `#[cfg(target_os="none")] <A>` / `#[cfg(not(target_os="none"))] <B>` arms become `plat::<op>(…)`, with `<A>` moved verbatim into `plat::device::<op>` and `<B>` into `plat::sim::<op>`. Unify each op's signature to the superset (async if any backend awaits; owns the loop if any backend loops).
- **Types stay put.** Cross-backend types/consts/helpers (`deluge_bsp::oled::FrameBuffer`, `oled::VISIBLE_TOP`, `deluge_bsp::oled::text`, `StereoFrame`, the `Event` type, etc.) are NOT moved — only the backend-divergent *operation bodies* are.
- **`host.rs` stays** as the sim's process-wide state (`PANEL`/`AUDIO`); `plat::sim` ops call `crate::host::panel()`/`take_audio()`. `plat::sim` is gated `cfg(all(not(target_os="none"), not(feature="linux")))` from the start so Phase 1b slots `plat::linux` in with no re-split.
- **Discriminator lands now (no linux yet):** `plat/mod.rs` uses the final three-way cfg (`device` = `target_os="none"`; `sim` = `all(not(none), not(feature="linux"))`; `linux` = `feature="linux"` — the `linux` arm is declared but its `mod linux;` is `#[cfg(feature="linux")]`, and the `linux` feature does not exist yet, so it never compiles this phase).
- Repo: `deluge-sdk` only, branch `feat/linux-backend`.

---

## Task 1: Scaffold `plat` + convert OLED (the worked template)

**Files:**
- Create: `crates/deluge-sdk/src/plat/mod.rs`, `crates/deluge-sdk/src/plat/device.rs`, `crates/deluge-sdk/src/plat/sim.rs`
- Modify: `crates/deluge-sdk/src/lib.rs` (add `mod plat;`), `crates/deluge-sdk/src/oled.rs`

**Interfaces:**
- Produces: `plat::oled_init_panel() -> impl Future<Output=()>`, `plat::oled_flush(fb: &deluge_bsp::oled::FrameBuffer) -> impl Future<Output=()>`. Both backends provide them; capability calls them.

- [ ] **Step 1: Create `plat/mod.rs` with the three-way selection**

```rust
//! Backend selection. Each capability module delegates its backend-divergent
//! operations here; exactly one submodule compiles per build:
//!   device  — target_os = "none"                     (deluge-bsp peripherals)
//!   sim     — host, no `linux` feature                (deluge-sim-link panel)
//!   linux   — the `linux` feature                     (libdeluge; Phase 1b)
#[cfg(target_os = "none")]
mod device;
#[cfg(target_os = "none")]
pub(crate) use device::*;

#[cfg(all(not(target_os = "none"), not(feature = "linux")))]
mod sim;
#[cfg(all(not(target_os = "none"), not(feature = "linux")))]
pub(crate) use sim::*;

#[cfg(feature = "linux")]
mod linux;
#[cfg(feature = "linux")]
pub(crate) use linux::*;
```

- [ ] **Step 2: Register the module**

In `crates/deluge-sdk/src/lib.rs`, add `mod plat;` alongside the other `mod` declarations (near line 79-90).

- [ ] **Step 3: Move OLED's two ops into `plat::device` / `plat::sim`**

Create `plat/device.rs`:
```rust
//! Device backend ops (deluge-bsp peripherals). Bodies moved verbatim from the
//! capability modules' `#[cfg(target_os = "none")]` arms.
use deluge_bsp::oled::{self, FrameBuffer};

pub(crate) async fn oled_init_panel() {
    oled::init().await;
}
pub(crate) async fn oled_flush(fb: &FrameBuffer) {
    oled::send_frame(fb).await;
}
```

Create `plat/sim.rs`:
```rust
//! Host-simulator backend ops (the deluge-sim-link SharedPanel). Bodies moved
//! verbatim from the capability modules' `#[cfg(not(target_os = "none"))]` arms.
use deluge_bsp::oled::FrameBuffer;

pub(crate) async fn oled_init_panel() {}
pub(crate) async fn oled_flush(fb: &FrameBuffer) {
    crate::host::panel().set_display(fb.as_bytes());
}
```

- [ ] **Step 4: Make `oled.rs` delegate**

In `crates/deluge-sdk/src/oled.rs`, replace the two-arm `init_panel` (lines 6-14) with:
```rust
/// Run the panel init sequence (device: SSD1309 bring-up; sim/linux: no-op).
pub(crate) async fn init_panel() {
    crate::plat::oled_init_panel().await;
}
```
and the two-arm `flush` body (lines 78-83) with:
```rust
    #[inline]
    pub async fn flush(&self) {
        crate::plat::oled_flush(&self.fb).await;
    }
```
Remove the now-unused `#[cfg]` imports if any become dead. Leave everything else (`FrameBuffer`, `text`, constants, `DrawTarget`) untouched.

- [ ] **Step 5: Verify device + sim + tests green**

```bash
cd ~/GitHub/deluge-sdk
cargo build-fw -p deluge-sdk 2>&1 | tail -3               # device (armv7a-none-eabihf + build-std)
cargo build -p deluge-sdk --target x86_64-unknown-linux-gnu 2>&1 | tail -3   # sim/host
cargo test -p deluge-sdk --target x86_64-unknown-linux-gnu 2>&1 | tail -5
```
Expected: all three succeed. (`build-fw` is the repo's `-Zbuild-std=core` alias for the device triple.) If the host build fails on a moved body, the move wasn't verbatim.

- [ ] **Step 6: Commit**

```bash
cd ~/GitHub/deluge-sdk
git add crates/deluge-sdk/src/plat crates/deluge-sdk/src/lib.rs crates/deluge-sdk/src/oled.rs
git commit -q -m "refactor(plat): introduce cfg-selected plat module; convert OLED

Extract the device/sim backend arms into plat::{device,sim}; Oled delegates to
plat::oled_*. Behavior-preserving; the three-way cfg (device|sim|linux) is in
place so Phase 1b adds plat::linux as one file. Template for the remaining caps."
echo done
```
Expected: `done`.

---

## Tasks 2-12: Convert the remaining capabilities (one module per task)

Each task applies the **Task-1 pattern** to one capability module: for every method with backend-divergent `#[cfg]` arms, add a `plat::<op>(…)` pair (`device.rs` + `sim.rs`, bodies moved verbatim) and replace the method body with the `plat::<op>` call. Unify each op's signature to the superset. **Verify device + sim + tests green (Task-1 Step 5) and commit** after each. Do NOT change logic — this is relocation only.

For each: **Files** = `plat/device.rs`, `plat/sim.rs`, `src/<module>.rs`. Op inventory below is the set of backend-divergent operations to extract (derived from the module's current `#[cfg]` arm pairs); the implementer confirms against the actual arms.

- [ ] **Task 2 — `pads.rs`** (2 device / 2 sim arms). Ops: `pads_flush` (device: `self.leds.flush().await`; sim: copy grid → `panel().set_all_pads(buf)`), `pads_init`/frame setup if present. Commit `refactor(plat): pads`.

- [ ] **Task 3 — `leds.rs`** (4 device / 3 sim arms). Ops: the indicator/gold LED writes + any init. Device → `deluge_bsp` LED path; sim → `panel().set_led`/`set_knob_indicator`. Commit `refactor(plat): leds`.

- [ ] **Task 4 — `sync_led.rs`** (6 device / 4 sim arms). Ops: `sync_led_set`/`toggle`/init. Device → BSP SYNC LED; sim → `panel().set_synced_led`. Commit `refactor(plat): sync_led`.

- [ ] **Task 5 — `cv_gate.rs`** (3 device / 2 sim arms). Ops: `cv_set`, `gate_set`. Device → BSP CV/gate; sim → `panel().set_cv`/`set_gate`. Commit `refactor(plat): cv_gate`.

- [ ] **Task 6 — `midi.rs`** (7 device / 3 sim arms). Ops: `midi_send`/`recv`/init (DIN + any USB-MIDI split). Move each arm pair. Commit `refactor(plat): midi`.

- [ ] **Task 7 — `clock.rs`** (6 device / 3 sim arms). Ops: `clock_in_*` / `clock_out_*` (trigger in/out). Device → BSP trigger clock; sim → panel/no-op. Commit `refactor(plat): clock`.

- [ ] **Task 8 — `jacks.rs`** (10 device / 6 sim arms — the largest). Ops: the CV/gate jack routing set. Move each arm pair carefully; largest surface, so verify with extra care. Commit `refactor(plat): jacks`.

- [ ] **Task 9 — `sd.rs`** (3 device / 6 sim arms). Ops: SD open/read/write/dir. Device → SDHI/FAT; sim → file-backed disk image. Note the `SdError`/`FatError` re-export split (lib.rs lines ~114-121) — keep it; only the op bodies move. Commit `refactor(plat): sd`.

- [ ] **Task 10 — `input.rs`** (6 device / 3 sim arms). Ops: the **producer** side only — `Input::next()` reading `EVENTS` is already backend-agnostic and stays. Extract the pump start (`start_host_pump` / `route_pic_event`) into `plat::input_start_pump(spawner)` (device: PIC route setup; sim: `host_input_pump`). `__rt::{device,host}::run` call `plat::input_start_pump`. Commit `refactor(plat): input pump`.

- [ ] **Task 11 — `audio.rs`** (6 device / 2 sim arms). One op: `plat::audio_run(f) -> !` — the whole `process` loop moves per backend (device: Ticker/`audio-irq` DMA loop over `audio_block`; sim: the `BrainEnds` ring loop). `Audio::process(f)` becomes `plat::audio_run(f).await`. Keep `StereoFrame`'s per-backend definition where it is (device re-export vs sim local) OR move both behind `plat`; simplest is to keep `StereoFrame` in `audio.rs` selecting per backend as today. Commit `refactor(plat): audio`.

- [ ] **Task 12 — `pic_service.rs`** (6 device / 2 sim arms). Ops: `pic_ensure_started`/`pic_wait_ready` (device: UART+DMA bring-up; sim: no-op stubs). Commit `refactor(plat): pic_service`.

---

## Task 13: Whole-crate integration verify (all three targets)

**Files:** none (verification)

- [ ] **Step 1: Device + sim build + full test suite**

```bash
cd ~/GitHub/deluge-sdk
cargo build-fw -p deluge-sdk 2>&1 | tail -3
cargo deluge sim --help >/dev/null 2>&1 || true   # ensure the tool is present
cargo build -p deluge-sdk --target x86_64-unknown-linux-gnu 2>&1 | tail -3
cargo test -p deluge-sdk --target x86_64-unknown-linux-gnu 2>&1 | tail -6
grep -rnE 'cfg\(target_os = "none"\)|cfg\(not\(target_os = "none"\)\)' crates/deluge-sdk/src/*.rs | grep -vE 'plat/|StereoFrame|re-export' | wc -l
```
Expected: device + host build succeed, tests pass, and the count of leftover backend `#[cfg]` arms *outside* `plat/` (excluding legitimately-kept per-backend type re-exports like `StereoFrame`) is near zero — the backend logic now lives in `plat`.

- [ ] **Step 2: Run an app in the simulator (behavior unchanged)**

```bash
cd ~/GitHub/deluge-sdk/examples/oled_hello
DELUGE_HEADLESS=1 cargo deluge sim 2>&1 | tail -5 || true
```
Expected: the sim runs the app as before (headless run completes / draws). This is the behavior-preservation proof for the sim backend.

- [ ] **Step 3: No commit (verification only).** If green, Phase 1a is done and `plat::linux` (Phase 1b) drops in as one file.

---

## Self-Review

**Spec coverage (Phase 1 spec §3a):** the `plat` module with cfg-selected `device`/`sim` submodules ✓ (Task 1); all 12 capability modules delegated ✓ (Tasks 1-12); the three-way discriminator in place for Phase 1b ✓ (Task 1 Step 1); behavior-preserving, verified by device+sim+tests after each task ✓ (Task-1 Step 5, Task 13). Input's producer/consumer split handled ✓ (Task 10); audio's whole-loop op ✓ (Task 11).

**Placeholder scan:** the per-module tasks (2-12) reference the Task-1 worked template rather than reproducing each module's existing arm bodies — because this is a *move* of code that already exists, and each task names the exact ops + backends to relocate. Not a placeholder: the transformation is fully specified; the "code" is the module's current arms, relocated verbatim.

**Type consistency:** `plat::oled_flush(&FrameBuffer)` / `plat::oled_init_panel()` signatures match `oled.rs`'s calls; `plat::audio_run(f) -> !` matches `Audio::process`; `plat::input_start_pump(spawner)` matches the `__rt` callers. Ops are `async` where any backend awaits.

**Sequencing/greenness:** Task 1 establishes `plat` + the template; each of 2-12 is independent (one module) and re-verifies device+sim+tests; Task 13 is the whole-crate proof. The crate stays green throughout — a failing build after any task means a body wasn't moved verbatim.
