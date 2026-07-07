# Wavetable device upload — real-time-safe firmware `upload_table` Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Wire a real-time-safe inline `upload_table` into `wren-firmware` so a Wren script builds and plays its own wavetable on the device — by restructuring `audio_task` to drop its long-lived engine borrow and adding `audio::upload_table` (scoped pool access + the 3c IFFT build).

**Architecture:** `ENGINE` is eagerly initialized in `main` before any task is spawned; `audio_task` re-derives a fresh scoped `&mut Eng` inside its per-block closure (never across an `.await`); `audio::upload_table` uses the same scoped access to `pool_alloc` + `build_pyramid_into`. Sound because the single cooperative executor guarantees `audio_task`'s render and `vm_task`'s `Wavetable.from` never interleave and neither holds a borrow across a yield.

**Tech Stack:** Rust `no_std` firmware (`wren-firmware`, embassy, RZ/A1L), `deluge-wren-core`, `deluge-audio-graph` (Pool-in-Engine from 3b), `mipgen` (IFFT `build_pyramid_flat` from 3c). Device target `armv7a-none-eabihf`.

**Reference spec:** [device-upload design](../specs/2026-07-07-wavetable-device-upload-design.md).

## Global Constraints

- **The final gate is on-hardware validation by the maintainer** — this plan delivers a compiling, soundness-documented path + a device checklist (Task 3), NOT a hardware-verified result. Do not claim device correctness from a green host build.
- **Soundness rests on the single cooperative executor:** `audio_task`'s render closure and `vm_task`'s `Wavetable.from` foreign call are each fully synchronous and never interleave; no `&mut ENGINE` is held across an `.await`; no ISR touches `ENGINE`. Every `unsafe` access to `ENGINE` carries a `SAFETY:` comment pointing at the module `## Concurrency` docs.
- **`ENGINE` is eagerly initialized in `main` before spawning tasks** (removes the vm_task-spawned-first ordering hazard; no `ENGINE_READY` atomic needed). `audio_task` and `upload_table` only ever `assume_init_mut`.
- **Transparency:** the render path's behaviour is unchanged (Task 1 is a borrow-scoping refactor, not a logic change). All host tests (`deluge-wren-core`, etc.) and both goldens still pass. The firmware change is additive to the host feature (which stays as-is).
- **No new firmware deps:** `PYRAMID_LEN` comes from `deluge_wren_core`, so `wren-firmware` needs no direct `mipgen`/`deluge-fft` dependency.
- **Verification commands:** firmware compiles for the device target in BOTH audio modes:
  - `cargo check -p wren-firmware --target armv7a-none-eabihf -Zbuild-std=core -Zbuild-std-features=compiler-builtins-mem`
  - same with `--features audio-irq`
  - host regression: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --features test-support`
- Zero new warnings (pre-existing `neon`/`deluge-fft` warnings excepted). Commit after each task.

---

## Task 1: `PYRAMID_LEN` const + `audio_task` drops the long-lived borrow

**Files:**
- Modify: `crates/deluge-wren-core/src/lib.rs` (add `PYRAMID_LEN`), `wren-firmware/src/audio.rs` (borrow restructure + `init_engine` + `ENGINE` visibility + `## Concurrency` docs), `wren-firmware/src/main.rs` (call `audio::init_engine` before spawns).

**Interfaces:**
- Produces: `deluge_wren_core::PYRAMID_LEN: usize`; `wren_firmware::audio::init_engine()` (eager init, called from `main`); `audio_task` no longer holds `&mut Eng` across `.await`.
- Consumes: `mipgen::{N, LEVELS}` (in `deluge-wren-core`, already a dep).

- [ ] **Step 1: Add `PYRAMID_LEN` to `deluge-wren-core`**

In `crates/deluge-wren-core/src/lib.rs` (root), add:

```rust
/// Length (in f32) of one full wavetable mip pyramid: `mipgen::N * mipgen::LEVELS`.
/// A pooled wavetable region is exactly this long. Firmware sizes its `pool_alloc`
/// by this so it needs no direct `mipgen` dependency.
pub const PYRAMID_LEN: usize = mipgen::N * mipgen::LEVELS;
```

Verify it compiles: `cargo build --target x86_64-unknown-linux-gnu -p deluge-wren-core`. (No behaviour change.)

- [ ] **Step 2: Restructure `audio_task` + add eager `init_engine`**

In `wren-firmware/src/audio.rs`:
- Make `ENGINE` and `Eng` reachable from `init_engine`/`upload_table` (keep `ENGINE` module-private — do NOT make it `pub`). Change `audio_task` so it does NOT hold `eng` across `.await`.

Replace the current `static mut ENGINE` init-in-task + long-lived borrow with:

```rust
// SAFETY: ENGINE is written exactly once by `init_engine` (from `main`, before any
// task is spawned/polled) and thereafter only *scoped* `assume_init_mut` borrows are
// taken — inside `audio_task`'s synchronous per-block closure and inside
// `upload_table`, which run on the one cooperative executor and never interleave, and
// never hold a borrow across an `.await`. No ISR touches ENGINE. See `## Concurrency`.
static mut ENGINE: MaybeUninit<Eng> = MaybeUninit::uninit();

/// Initialize the audio engine. MUST be called once from `main` before spawning
/// `audio_task`/`vm_task`, so no task ever observes an uninitialized `ENGINE`.
pub fn init_engine() {
    // SAFETY: called once from `main` before any task runs; no other accessor yet.
    unsafe { (*addr_of_mut!(ENGINE)).write(Eng::new(SAMPLE_RATE)); }
}
```

Change `audio_task`'s body to NOT init and NOT hold a long-lived borrow — re-derive inside the closure:

```rust
#[embassy_executor::task]
pub async fn audio_task(audio: Audio) {
    audio
        .process(|block: &mut [deluge::StereoFrame]| {
            // SAFETY: ENGINE was initialized by `init_engine` in `main` before this
            // task could be polled. This closure is synchronous (no `.await` inside),
            // so the borrow never crosses a yield and never overlaps `upload_table`'s
            // (cooperative executor). See `## Concurrency`.
            let eng: &mut Eng = unsafe { (*addr_of_mut!(ENGINE)).assume_init_mut() };
            while let Some(c) = CMD_RING.lock(|r| r.borrow_mut().pop()) {
                eng.apply(c);
            }
            let mut scratch = [deluge_audio_graph::StereoFrame::default(); 32];
            for chunk in block.chunks_mut(32) {
                let out = &mut scratch[..chunk.len()];
                eng.render(out);
                for (dst, src) in chunk.iter_mut().zip(out.iter()) {
                    dst.l = src.l;
                    dst.r = src.r;
                }
            }
        })
        .await
}
```

Add `use core::ptr::addr_of_mut;` if not present. Update the module `## Concurrency` doc comment to state the eager-init + scoped-borrow invariant and the soundness argument (the five points from spec §2.3).

- [ ] **Step 3: Call `init_engine` from `main` before spawns**

In `wren-firmware/src/main.rs`, in the `#[deluge::app] async fn main`, BEFORE the `spawner.spawn(...)` block (specifically before `spawner.spawn(vm_task()...)` and `spawner.spawn(audio::audio_task(audio)...)`), add:

```rust
    // Initialize the audio engine before spawning any task, so neither audio_task
    // nor a boot-script `Wavetable.from` (vm_task is spawned first) can observe an
    // uninitialized ENGINE.
    audio::init_engine();
```

- [ ] **Step 4: Verify it compiles both modes; render behaviour unchanged**

Run: `cargo check -p wren-firmware --target armv7a-none-eabihf -Zbuild-std=core -Zbuild-std-features=compiler-builtins-mem`
Run: same with `--features audio-irq`
Expected: both compile, zero new warnings. The render logic is byte-identical (only the borrow scoping moved) — no functional change. If `deluge-sim-link` or another host-side firmware simulation exists and exercises `audio_task`, run it; otherwise the device-target compile is the gate (audio_task is firmware-only, not host-runnable).

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-wren-core/src/lib.rs wren-firmware/src/audio.rs wren-firmware/src/main.rs
git commit -m "refactor(wren-firmware): eager ENGINE init + scoped per-block borrow (enables upload_table)"
```

---

## Task 2: `audio::upload_table` + `FwHost::upload_table`

**Files:**
- Modify: `wren-firmware/src/audio.rs` (add `upload_table` + a `const` pool-capacity assertion), `wren-firmware/src/host.rs` (override `Host::upload_table`, remove the 3b `None`-stub note).

**Interfaces:**
- Produces: `wren_firmware::audio::upload_table(base: &[f32]) -> Option<deluge_audio_graph::PoolHandle>`; `FwHost::upload_table` forwarding to it.
- Consumes: `Eng::{pool_alloc, pool_slice_mut}`, `deluge_wren_core::{PYRAMID_LEN, build_pyramid_into}`, `deluge_audio_graph::PoolHandle`.

- [ ] **Step 1: Confirm the current stub**

`FwHost` currently does NOT override `upload_table` (inherits the trait default `None`), per the 3b revert. On device, `Wavetable.from` therefore returns an unbound handle → silence. This task makes it real. No failing host test exists (firmware isn't host-run); the gate is the device-target compile + the HW checklist (Task 3).

- [ ] **Step 2: Add `audio::upload_table` + capacity assertion**

In `wren-firmware/src/audio.rs`, add (near `submit`):

```rust
/// Build a band-limited mip pyramid from `base` into a freshly-allocated pool region
/// of the audio engine and return its handle. Called synchronously from `vm_task`'s
/// `Wavetable.from` foreign method (via `FwHost::upload_table`). `None` on pool
/// exhaustion. Blocks the executor for the (sub-millisecond, IFFT) build — see the
/// `## Concurrency` docs and the device-upload spec for why this fits the audio
/// write-ahead lead.
pub fn upload_table(base: &[f32]) -> Option<deluge_audio_graph::PoolHandle> {
    // SAFETY: ENGINE was initialized by `init_engine` in `main` before any task ran.
    // This runs synchronously inside a Wren foreign call on the one cooperative
    // executor; audio_task is parked at its `.await` holding no ENGINE borrow; no ISR
    // touches ENGINE — so no two `&mut ENGINE` coexist and none crosses a yield.
    let eng: &mut Eng = unsafe { (*addr_of_mut!(ENGINE)).assume_init_mut() };
    let h = eng.pool_alloc(deluge_wren_core::PYRAMID_LEN)?;
    deluge_wren_core::build_pyramid_into(base, eng.pool_slice_mut(h));
    Some(h)
}

// The firmware pool (PCAP in the `Eng` alias) must hold at least one full pyramid.
const _: () = assert!(90112 >= deluge_wren_core::PYRAMID_LEN);
```

(If the `Eng` alias's `PCAP` literal differs from `90112`, use the real value; the intent is `PCAP >= PYRAMID_LEN`. Since `PYRAMID_LEN` isn't `const`-comparable against a type-alias param directly, assert against the same literal used in the `Eng` alias.)

- [ ] **Step 3: Override `FwHost::upload_table`**

In `wren-firmware/src/host.rs`, replace the 3b `NOTE:` stub comment (the one explaining it inherits `None`) with an override:

```rust
    fn upload_table(&mut self, base: &[f32]) -> Option<deluge_audio_graph::PoolHandle> {
        crate::audio::upload_table(base)
    }
```

Update the surrounding doc comment: device-side `Wavetable.from` now builds + binds a real table (inline, real-time-safe under the cooperative executor — see `audio.rs` `## Concurrency` and the device-upload spec), replacing the previous silent stub.

- [ ] **Step 4: Verify both modes compile + host regression**

Run: `cargo check -p wren-firmware --target armv7a-none-eabihf -Zbuild-std=core -Zbuild-std-features=compiler-builtins-mem`
Run: same with `--features audio-irq`
Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --features test-support`
Expected: firmware compiles both modes (the `const _` assertion holds); host wren-core tests unchanged (this task doesn't touch the host path). Zero new warnings.

- [ ] **Step 5: Commit**

```bash
git add wren-firmware/src/audio.rs wren-firmware/src/host.rs
git commit -m "feat(wren-firmware): real-time-safe inline upload_table (device wavetable build)"
```

---

## Task 3: On-device validation checklist (handoff)

**Files:**
- Create: `wren-firmware/docs/wavetable-upload-validation.md` (or append to the firmware README) — the maintainer's on-hardware checklist.

This task ships NO code — it records the hardware validation the maintainer must perform, because the sub-project's correctness depends on a property (inline build time vs audio write-ahead lead) that cannot be measured off-device.

- [ ] **Step 1: Write the validation checklist**

Create `wren-firmware/docs/wavetable-upload-validation.md`:

```markdown
# Device validation — inline wavetable upload

The firmware `upload_table` builds a mip pyramid **synchronously** inside the Wren
`Wavetable.from` call, blocking the single cooperative executor for the build. This is
safe only if the build fits inside the audio TX write-ahead lead (~5.8 ms `audio-irq`,
~11.6 ms poll). That must be confirmed on hardware.

## Checklist
- [ ] `var w = Wavetable.from([ ...single cycle... ]); Out.patch(Osc.wavetable(w, 220))`
      → the node **plays the table** (audible tone), not silence.
- [ ] **No audible glitch/click on upload** (default poll mode). If possible, measure
      the build duration (e.g. toggle a GPIO / timestamp around `upload_table`) and
      confirm it is well under the write-ahead lead.
- [ ] Repeat with `--features audio-irq` (5.8 ms lead — tighter). Note any glitch;
      **poll mode (~11.6 ms lead) is the safer default for upload-heavy patches.**
- [ ] Rapid re-upload: build → `Node.free()` → build again, several times → no
      exhaustion, no corruption, tables still correct.
- [ ] A boot-script `Wavetable.from` (in MAIN.WREN) behaves (either plays, or silently
      degrades if it somehow precedes init — must never crash).

## If the inline build glitches
Fall back to the deferred **async/chunked build** (spec §4): `Wavetable.from` returns a
pending handle; spread the build across `vm_task` iterations, yielding to audio between
levels; bind when ready. This removes the long synchronous stall at the cost of an
async-readiness state machine.
```

- [ ] **Step 2: Commit**

```bash
git add wren-firmware/docs/wavetable-upload-validation.md
git commit -m "docs(wren-firmware): on-device wavetable-upload validation checklist"
```

---

## Self-review notes

- **Spec coverage:** `PYRAMID_LEN` + `audio_task` borrow-drop + eager init (Task 1); `audio::upload_table` + `FwHost` override + capacity assert (Task 2); the honest HW-validation handoff (Task 3). The `unsafe` soundness argument is documented at every site + the module docs.
- **Init hazard resolved by eager init:** `main` calls `audio::init_engine()` before any spawn, so no task (incl. a boot-script `Wavetable.from` on the first-spawned `vm_task`) can observe an uninitialized `ENGINE` — the spec's `ENGINE_READY` guard is unnecessary with eager init (chosen for cleanliness).
- **Transparency:** Task 1 is a pure borrow-scoping refactor (render logic byte-identical); host tests + goldens untouched; the host `Wavetable.from` feature is unchanged. Only the firmware gains device support.
- **Honest limit:** no host test can prove the real-time property; Task 3 is the explicit maintainer handoff. Do not report device correctness from a green build.
- **Known follow-ups:** async/chunked build (if HW glitches); Pool decoupling (only if a future preemptive executor breaks the cooperative-executor soundness argument); multi-frame morph.
