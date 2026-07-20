# Ring-free Linux Audio Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement `plat::linux::audio_run` so a `#[deluge::app]` DSP closure runs directly inside libdeluge's audio callback — no ring buffer, no cross-thread queue.

**Architecture:** libdeluge owns the audio thread and pushes 128-frame periods. The SDK adapts its split `(&[[f32;2]], &mut [[f32;2]])` callback to `Audio::process`'s in-place `&mut [StereoFrame]` contract via a pure `adapt_block` function, hands the closure to `audio_start`, and parks. A `Send + 'static` bound on `process` plus `!Send` capability handles make it a *compile error* for the DSP closure to touch hardware owned by the UI half.

**Tech Stack:** Rust (nightly), Embassy executor, `deluge-hal-linux` → `libdeluge` (C, static, musl), `armv7-unknown-linux-musleabihf` target, `cargo-deluge` build/upload tool.

**Spec:** `docs/superpowers/specs/2026-07-20-linux-audio-design.md`

## Global Constraints

- Branch: `feat/linux-backend`. Working tree is clean at start.
- Repo root: `/home/kate/GitHub/deluge-sdk`. All paths below are relative to it.
- `DELUGE_BASE=~/GitHub/deluge-linux/out/deluge-base-stage5` — the current bundle (first stage containing `deluge_leds_sync`). Do **not** rebuild Buildroot; this plan only consumes the bundle.
- `cargo-deluge` is host-built: `tools/cargo-deluge/target/x86_64-unknown-linux-gnu/debug/cargo-deluge`. Rebuild with `cargo build --manifest-path tools/cargo-deluge/Cargo.toml --target x86_64-unknown-linux-gnu` if missing.
- `deluge-sdk` features: `default = []`. `sim` and `linux` are **mutually exclusive** backends (see `crates/deluge-sdk/src/plat/mod.rs`); every build command must name one explicitly.
- Block size is fixed: `DELUGE_PERIOD == BLOCK_FRAMES == 128`.
- Tasks 1–3 are host-only and fully verifiable. Task 4 requires physical hardware.

---

### Task 1: Unit-testable frame adaptation

Extract the split→in-place buffer adaptation as a pure function and test it. This is the only logic in the feature that can be wrong in an interesting way, and it needs no libdeluge, so it lands first and independently.

**Files:**
- Modify: `crates/deluge-sdk/src/audio.rs` (add `adapt_block` + `#[cfg(test)] mod tests`)
- Modify: `tools/test.sh` (add `deluge-sdk` to the host bucket)

**Interfaces:**
- Produces: `pub(crate) fn adapt_block<F>(f: &mut F, inp: &[[f32; 2]], out: &mut [[f32; 2]]) where F: FnMut(&mut [StereoFrame])` — consumed by Task 3's `plat::linux::audio_run`.

- [ ] **Step 1: Write the failing tests**

Append to `crates/deluge-sdk/src/audio.rs`:

```rust
#[cfg(test)]
mod tests {
    use super::{adapt_block, StereoFrame};

    /// The closure must observe the *input* buffer, not `out`'s prior contents.
    #[test]
    fn input_reaches_the_closure() {
        let inp = [[1.0, 2.0], [3.0, 4.0]];
        let mut out = [[-9.0, -9.0]; 2];
        let mut seen = Vec::new();
        adapt_block(
            &mut |b: &mut [StereoFrame]| seen.extend(b.iter().map(|f| (f.l, f.r))),
            &inp,
            &mut out,
        );
        assert_eq!(seen, vec![(1.0, 2.0), (3.0, 4.0)]);
    }

    /// Whatever the closure leaves in the block is what libdeluge sends out.
    #[test]
    fn closure_writes_reach_out() {
        let inp = [[1.0, 2.0], [3.0, 4.0]];
        let mut out = [[0.0, 0.0]; 2];
        adapt_block(
            &mut |b: &mut [StereoFrame]| {
                for f in b {
                    f.l *= 10.0;
                    f.r *= 100.0;
                }
            },
            &inp,
            &mut out,
        );
        assert_eq!(out, [[10.0, 200.0], [30.0, 400.0]]);
    }

    /// A no-op closure is bit-exact passthrough — the `audio_passthru` contract.
    #[test]
    fn noop_closure_is_bit_exact_passthrough() {
        let inp = [[0.5, -0.25], [f32::MIN_POSITIVE, -0.0], [1.0, -1.0]];
        let mut out = [[9.9, 9.9]; 3];
        adapt_block(&mut |_: &mut [StereoFrame]| {}, &inp, &mut out);
        assert_eq!(out, inp);
        // -0.0 must survive as -0.0, not collapse to 0.0.
        assert!(out[1][1].is_sign_negative());
    }

    /// Degenerate but legal: no frames, no work, no panic.
    #[test]
    fn empty_block_is_a_noop() {
        let inp: [[f32; 2]; 0] = [];
        let mut out: [[f32; 2]; 0] = [];
        let mut called = false;
        adapt_block(
            &mut |b: &mut [StereoFrame]| {
                called = true;
                assert!(b.is_empty());
            },
            &inp,
            &mut out,
        );
        assert!(called);
    }

    /// The transmute in `adapt_block` is sound only under layout equality.
    #[test]
    fn stereoframe_is_layout_compatible_with_f32_pair() {
        assert_eq!(
            core::mem::size_of::<StereoFrame>(),
            core::mem::size_of::<[f32; 2]>()
        );
        assert_eq!(
            core::mem::align_of::<StereoFrame>(),
            core::mem::align_of::<[f32; 2]>()
        );
    }
}
```

- [ ] **Step 2: Run the tests to verify they fail**

```bash
cargo test -p deluge-sdk --features sim --target x86_64-unknown-linux-gnu 2>&1 | tail -20
```

Expected: FAIL to compile — `cannot find function `adapt_block` in module `super``.

- [ ] **Step 3: Implement `adapt_block`**

In `crates/deluge-sdk/src/audio.rs`, immediately after the `StereoFrame` definitions (before `fn ensure_init`), add:

```rust
/// Adapt libdeluge's split input/output buffers to [`Audio::process`]'s in-place
/// block contract.
///
/// libdeluge hands its callback two slices (input, output); the SDK's DSP
/// closure takes **one** slice pre-loaded with input, whose final contents are
/// sent to line-out. So seed `out` with the input and hand `out` to `f`.
///
/// Pure by design — no `libdeluge`, no hardware, no locks — so the Linux
/// backend's only interesting logic is unit-testable on the host. See
/// `plat::linux::audio_run`, its sole caller.
#[cfg(any(feature = "linux", test))]
#[inline]
pub(crate) fn adapt_block<F>(f: &mut F, inp: &[[f32; 2]], out: &mut [[f32; 2]])
where
    F: FnMut(&mut [StereoFrame]),
{
    // libdeluge passes the same period length for both (DELUGE_PERIOD). The
    // `copy_from_slice` below also panics on mismatch in release, which is the
    // correct failure for a broken ABI.
    debug_assert_eq!(inp.len(), out.len());
    out.copy_from_slice(inp);
    // SAFETY: `StereoFrame` is `#[repr(C)] { l: f32, r: f32 }`, layout-identical
    // to `[f32; 2]`. Asserted at compile time in `plat::linux::audio_run` and at
    // run time by `stereoframe_is_layout_compatible_with_f32_pair`.
    f(unsafe { core::mem::transmute::<&mut [[f32; 2]], &mut [StereoFrame]>(out) });
}
```

- [ ] **Step 4: Run the tests to verify they pass**

```bash
cargo test -p deluge-sdk --features sim --target x86_64-unknown-linux-gnu 2>&1 | tail -12
```

Expected: `test result: ok. 5 passed; 0 failed`.

- [ ] **Step 5: Add `deluge-sdk` to the host test bucket**

In `tools/test.sh`, in the `Host bucket` section, after the `deluge-sdk-macros` line, add:

```bash
# The SDK facade: `adapt_block` (the linux backend's frame adaptation) is pure
# logic and testable here. Needs an explicit backend feature — `sim` is the only
# one that builds on the host (`linux` requires the musl libdeluge sysroot).
# Host-only: `--features sim` pulls deluge-simulator -> cpal -> alsa-sys, which
# does not cross-compile to the QEMU ARM bucket's target.
cargo test --target "$HOST" -p deluge-sdk --features sim
```

- [ ] **Step 6: Run the full suite to confirm nothing regressed**

```bash
tools/test.sh 2>&1 | tail -15
```

Expected: ends with `==> All tests passed.`

- [ ] **Step 7: Commit**

```bash
git add crates/deluge-sdk/src/audio.rs tools/test.sh
git commit -m "feat(linux-audio): pure adapt_block + unit tests; add deluge-sdk to test.sh"
```

---

### Task 2: `Send + 'static` bound on `Audio::process`

Tighten the DSP closure contract on **all** backends, so a closure that is illegal on linux fails to compile on device too. Isolated from Task 3 because it is the change most likely to break an existing app.

**Files:**
- Modify: `crates/deluge-sdk/src/audio.rs:62` (the `process` signature + docs)
- Modify: `examples/audio_passthru/src/main.rs:27`

**Interfaces:**
- Consumes: nothing from Task 1.
- Produces: `Audio::process<F: FnMut(&mut [StereoFrame]) + Send + 'static>(self, f: F) -> !` — Task 3's `audio_run` relies on this bound to satisfy `Deluge::audio_start`.

- [ ] **Step 1: Add the bound and document why**

In `crates/deluge-sdk/src/audio.rs`, replace the `process` method (currently at line 52-64, the doc comment through the closing brace) with:

```rust
    /// Run `f` over every audio block, forever.
    ///
    /// `f` receives a `BLOCK`-length slice pre-loaded with codec input; whatever
    /// it leaves in the slice is sent to the codec. Never returns.
    ///
    /// `f` must be `Send + 'static` because the Linux backend runs it on
    /// libdeluge's audio thread rather than on the app's executor. The bound is
    /// uniform across backends on purpose: a closure that compiles on device but
    /// not on linux would hide the portability break until deploy time. Share
    /// state with the rest of the app through `static` atomics — see the
    /// `additive_osc` example.
    ///
    /// Capability handles ([`Oled`](crate::Oled), [`Pads`](crate::Pads),
    /// [`SyncLed`](crate::SyncLed), …) are deliberately `!Send`, so capturing one
    /// here is a compile error rather than a runtime audio stall.
    ///
    /// ```ignore
    /// dlg.audio().process(move |block| {
    ///     for f in block { f.l *= 0.5; f.r *= 0.5; }
    /// }).await
    /// ```
    pub async fn process<F: FnMut(&mut [StereoFrame]) + Send + 'static>(self, f: F) -> ! {
        crate::plat::audio_run(f).await
    }
```

- [ ] **Step 2: Update the `plat` signatures to match**

The three backend signatures must carry the same bound or the crate will not compile. In each of these files, change the `audio_run` signature:

`crates/deluge-sdk/src/plat/device.rs`:
```rust
pub(crate) async fn audio_run<F: FnMut(&mut [crate::audio::StereoFrame]) + Send + 'static>(
    mut f: F,
) -> ! {
```

`crates/deluge-sdk/src/plat/sim.rs`:
```rust
pub(crate) async fn audio_run<F: FnMut(&mut [crate::audio::StereoFrame]) + Send + 'static>(
    mut f: F,
) -> ! {
```

`crates/deluge-sdk/src/plat/linux.rs`:
```rust
pub(crate) async fn audio_run<F: FnMut(&mut [crate::audio::StereoFrame]) + Send + 'static>(
    mut f: F,
) -> ! {
```

Leave the bodies alone — Task 3 rewrites the linux one.

- [ ] **Step 3: Verify the bound actually bites**

```bash
cargo build -p deluge-sdk --features sim --target x86_64-unknown-linux-gnu 2>&1 | tail -5
cd examples/audio_passthru && cargo deluge build 2>&1 | tail -20; cd ../..
```

Expected: the SDK builds; `audio_passthru` **fails** with an error naming `drive` and a lifetime/`'static` requirement (its closure at `src/main.rs:27` borrows the local `drive`). That failure is the proof the bound is enforced.

- [ ] **Step 4: Fix `audio_passthru`**

In `examples/audio_passthru/src/main.rs`, change line 27 from `.process(|block: &mut [StereoFrame]| {` to:

```rust
        .process(move |block: &mut [StereoFrame]| {
```

- [ ] **Step 5: Verify every audio example still builds for device**

```bash
for ex in audio_passthru audio_passthru_irq additive_osc test_osc; do
  echo "--- $ex"
  (cd "examples/$ex" && cargo deluge build 2>&1 | tail -3)
done
```

Expected: each produces an ELF with no errors. `additive_osc` and `test_osc` already use `move` closures over owned state and `static` atomics, so they need no edit — if either fails, stop and report rather than adding `move` blindly.

- [ ] **Step 6: Commit**

```bash
git add crates/deluge-sdk/src/audio.rs crates/deluge-sdk/src/plat/ examples/audio_passthru/src/main.rs
git commit -m "feat(audio): require Send + 'static on Audio::process across all backends"
```

---

### Task 3: `!Send` capability handles

Make it a compile error for the DSP closure to capture a hardware handle, which would otherwise lock the process-wide `Mutex<Deluge>` from libdeluge's audio thread. This touches every backend, so it is isolated and verified before any linux-specific work.

**Files:**
- Modify: `crates/deluge-sdk/src/lib.rs` (add the shared `NotSend` marker + compile-time assertions)
- Modify: `crates/deluge-sdk/src/{cv_gate,jacks,midi,sd,leds,input,pads,oled,clock,sync_led}.rs`

**Interfaces:**
- Produces: `pub(crate) type NotSend = PhantomData<*const ()>;` and `pub(crate) const NOT_SEND: NotSend = PhantomData;` in `lib.rs`. Every capability handle gains a `_not_send: crate::NotSend` field.

- [ ] **Step 1: Add the shared marker to `lib.rs`**

In `crates/deluge-sdk/src/lib.rs`, near the other `pub(crate)` items, add:

```rust
/// Makes a capability handle `!Send`, so it cannot be captured by the
/// `Send + 'static` DSP closure passed to [`Audio::process`].
///
/// On the Linux backend that closure runs on libdeluge's audio thread; every
/// hardware op goes through `crate::linux::dev()`, a lock on the process-wide
/// `Mutex<Deluge>` the UI half also holds. Taking it from the audio thread
/// would stall the callback. Since every hardware op is a method on an owned
/// handle (there are no free functions), making the handles `!Send` turns that
/// stall into a compile error at the `.process()` call site.
///
/// Safe on all backends: Embassy's executor is single-threaded, so nothing in
/// the SDK requires these handles to be `Send`.
pub(crate) type NotSend = core::marker::PhantomData<*const ()>;
pub(crate) const NOT_SEND: NotSend = core::marker::PhantomData;
```

- [ ] **Step 2: Add the compile-time `!Send` assertions**

Also in `crates/deluge-sdk/src/lib.rs`, add this module. It compiles on **every** build of the crate, so it gates all three backends without needing a test harness:

```rust
/// Compile-time proof that capability handles are `!Send` (see [`NotSend`]).
///
/// If any listed type becomes `Send`, both blanket impls below apply and the
/// `AmbiguousIfSend<_>` inference in `assertions` fails with "type annotations
/// needed" — turning an accidental `Send` into a build error.
mod not_send_assertions {
    trait AmbiguousIfSend<A> {
        fn assert() {}
    }
    impl<T: ?Sized> AmbiguousIfSend<()> for T {}
    impl<T: ?Sized + Send> AmbiguousIfSend<u8> for T {}

    #[allow(dead_code)]
    fn assertions() {
        let _ = <crate::Cv as AmbiguousIfSend<_>>::assert;
        let _ = <crate::Gate as AmbiguousIfSend<_>>::assert;
        let _ = <crate::Jacks as AmbiguousIfSend<_>>::assert;
        let _ = <crate::Midi as AmbiguousIfSend<_>>::assert;
        let _ = <crate::Sd as AmbiguousIfSend<_>>::assert;
        let _ = <crate::Leds as AmbiguousIfSend<_>>::assert;
        let _ = <crate::Input as AmbiguousIfSend<_>>::assert;
        let _ = <crate::Pads as AmbiguousIfSend<_>>::assert;
        let _ = <crate::Oled as AmbiguousIfSend<_>>::assert;
        let _ = <crate::ClockIn as AmbiguousIfSend<_>>::assert;
        let _ = <crate::ClockOut as AmbiguousIfSend<_>>::assert;
        let _ = <crate::SyncLed as AmbiguousIfSend<_>>::assert;
    }
}
```

All 12 names are re-exported at the crate root (`lib.rs:101-111`), so these paths resolve as written — no module paths needed.

- [ ] **Step 3: Run the assertions against the *unmodified* handles to see them fail**

```bash
cargo build -p deluge-sdk --features sim --target x86_64-unknown-linux-gnu 2>&1 | grep -E "^error|annotations" | head -20
```

Expected: **12 errors**, one per handle, of the form `type annotations needed` at the `AmbiguousIfSend<_>` lines. This proves the assertion mechanism works before it is satisfied. If you see zero errors, the assertion is inert — fix it before continuing.

- [ ] **Step 4: Add the marker field to every handle**

For each handle, add the field and initialise it in its constructor. The handles that currently have `_private: ()` **replace** that field; the ones with real fields **gain** one.

`crates/deluge-sdk/src/cv_gate.rs` — `Cv` (line 21) and `Gate` (line 52):
```rust
pub struct Cv {
    _not_send: crate::NotSend,
}
// in Cv::new():   Self { _not_send: crate::NOT_SEND }

pub struct Gate {
    _not_send: crate::NotSend,
}
// in Gate::new(): Self { _not_send: crate::NOT_SEND }
```

`crates/deluge-sdk/src/jacks.rs` (`Jacks`, line 21), `midi.rs` (`Midi`, line 18), `leds.rs` (`Leds`, line 8), `input.rs` (`Input`, line 37) — identical shape:
```rust
pub struct Jacks {
    _not_send: crate::NotSend,
}
// in new():  Self { _not_send: crate::NOT_SEND }
```

`crates/deluge-sdk/src/sd.rs` — `Sd` has **two** cfg-gated definitions (lines ~33 and ~88). Both get the field and both `new()`s get the initialiser.

`crates/deluge-sdk/src/pads.rs` (`Pads`, line 16) — keeps `leds`:
```rust
pub struct Pads {
    leds: PadLeds,
    _not_send: crate::NotSend,
}
// in new(): Self { leds: ..., _not_send: crate::NOT_SEND }
```

`crates/deluge-sdk/src/oled.rs` (`Oled`, line 33) — keeps `fb`:
```rust
pub struct Oled {
    fb: FrameBuffer,
    _not_send: crate::NotSend,
}
```

`crates/deluge-sdk/src/clock.rs` — `ClockIn` (line 33) keeps `prev_ticks`; `ClockOut` (line 80) keeps `channel` and `pulse_width`. Both gain `_not_send`.

`crates/deluge-sdk/src/sync_led.rs` — **two** cfg-gated definitions: the device one (line 21) keeps `pin`, the host one (line 102) keeps `state`. Both gain `_not_send`.

- [ ] **Step 5: Verify the assertions now pass on sim**

```bash
cargo build -p deluge-sdk --features sim --target x86_64-unknown-linux-gnu 2>&1 | tail -5
```

Expected: builds clean, zero errors. The 12 errors from Step 3 are gone because every handle is now `!Send`.

- [ ] **Step 6: Verify the device backend — the real regression risk**

`!Send` handles held across an Embassy `spawn` would surface here and nowhere else.

```bash
for ex in blinky button_leds oled_hello oled_menu oled_hmenu pad_paint input_demo \
          midi_cv clock_jacks sd_demo audio_passthru audio_passthru_irq \
          additive_osc test_osc usb_log; do
  printf '%-22s ' "$ex"
  (cd "examples/$ex" && cargo deluge build >/dev/null 2>&1 && echo OK || echo FAIL)
done
```

Expected: every example `OK`. Any `FAIL` is a genuine finding — re-run that one without `>/dev/null` and report the error rather than reverting the `!Send` change.

- [ ] **Step 7: Run the test suite**

```bash
tools/test.sh 2>&1 | tail -8
```

Expected: `==> All tests passed.`

- [ ] **Step 8: Commit**

```bash
git add crates/deluge-sdk/src/
git commit -m "feat(sdk): make capability handles !Send so DSP closures cannot touch hardware"
```

---

### Task 4: `plat::linux::audio_run`

Replace the `unimplemented!()` stub with the real implementation: layout assert, `adapt_block` shim, `audio_start`, RT-priority confirmation, park.

**Files:**
- Modify: `crates/deluge-sdk/Cargo.toml` (add `libc` as a `linux`-only optional dep)
- Modify: `crates/deluge-sdk/src/plat/linux.rs:11-16` (the `audio_run` stub)

**Interfaces:**
- Consumes: `crate::audio::adapt_block` (Task 1); the `Send + 'static` bound (Task 2), which is what makes the closure acceptable to `Deluge::audio_start`.
- Consumes: `deluge_hal_linux::Deluge::audio_start(&mut self, cb: impl FnMut(&[[f32; 2]], &mut [[f32; 2]]) + Send + 'static) -> Result<(), Error>` (`crates/deluge-hal-linux/src/lib.rs:154`).

- [ ] **Step 1: Add the `libc` dependency**

In `crates/deluge-sdk/Cargo.toml`, in the `[target.'cfg(not(target_os = "none"))'.dependencies]` block, after the `deluge-hal-linux` line, add:

```toml
# `sched_getscheduler`, to confirm libdeluge's audio thread actually got
# SCHED_FIFO (it degrades to a stderr warning we never see). Linux backend only;
# already in the tree transitively via deluge-sys.
libc = { version = "0.2", optional = true }
```

And extend the `linux` feature in the `[features]` block:

```toml
linux = ["dep:deluge-hal-linux", "dep:libc"]
```

- [ ] **Step 2: Implement `audio_run`**

In `crates/deluge-sdk/src/plat/linux.rs`, replace the stub (lines 11-16, the doc comment through the closing brace) with:

```rust
/// Linux: hand the DSP closure to `libdeluge`, which runs it on its own audio
/// thread, then park.
///
/// This is the ring-free path: the app's closure executes *inside* libdeluge's
/// per-period callback, so there is no buffer or queue between the DSP and the
/// codec — only [`adapt_block`](crate::audio::adapt_block)'s single memcpy that
/// converts libdeluge's split slices to the SDK's in-place block contract.
pub(crate) async fn audio_run<F: FnMut(&mut [crate::audio::StereoFrame]) + Send + 'static>(
    mut f: F,
) -> ! {
    use core::sync::atomic::{AtomicBool, Ordering};

    // `adapt_block`'s transmute is sound only under this layout equality, and
    // libdeluge owns one side of it. Evaluated against the real armv7 musl
    // target at build time, so a 32-bit divergence cannot slip through.
    const _: () = assert!(
        core::mem::size_of::<crate::audio::StereoFrame>() == core::mem::size_of::<[f32; 2]>()
            && core::mem::align_of::<crate::audio::StereoFrame>()
                == core::mem::align_of::<[f32; 2]>()
    );

    static RT_CHECKED: AtomicBool = AtomicBool::new(false);

    let shim = move |inp: &[[f32; 2]], out: &mut [[f32; 2]]| {
        // libdeluge's audio thread self-elevates to SCHED_FIFO (audio.c), but
        // that fails *soft* to a stderr warning the appliance never shows. At
        // 128-frame periods a non-RT thread will xrun under load, so confirm it
        // once from inside the callback — this is that thread.
        if !RT_CHECKED.swap(true, Ordering::Relaxed) {
            // SAFETY: `sched_getscheduler(0)` queries the calling thread and has
            // no preconditions.
            if unsafe { libc::sched_getscheduler(0) } != libc::SCHED_FIFO {
                log::warn!(
                    "audio thread is NOT SCHED_FIFO — expect xruns under load \
                     (missing CAP_SYS_NICE?)"
                );
            }
        }
        crate::audio::adapt_block(&mut f, inp, out);
    };

    if let Err(e) = crate::linux::dev().audio_start(shim) {
        // Unlike the sync LED and input pump, missing audio is not survivable:
        // an app that called `.audio()` needs it, and silent no-sound is harder
        // to diagnose than an abort naming the cause.
        panic!("linux audio unavailable: {e} (is the ALSA 'Deluge' card present?)");
    }

    // The DSP now lives on libdeluge's thread. This task has no further work but
    // must not return (`-> !`).
    core::future::pending().await
}
```

- [ ] **Step 3: Remove the now-unused import if the compiler flags it**

The stub was the only user of nothing in particular, but `audio_run` now introduces `AtomicBool`/`Ordering` locally while the file already imports them at line 4 for `INPUT_PUMP_STARTED`. If `cargo` warns `unused import` or `duplicate`, delete the function-local `use core::sync::atomic::{AtomicBool, Ordering};` line and rely on the file-level import.

- [ ] **Step 4: Build for the Linux target**

```bash
BIN=$PWD/tools/cargo-deluge/target/x86_64-unknown-linux-gnu/debug/cargo-deluge
cd examples/audio_passthru
DELUGE_BASE=~/GitHub/deluge-linux/out/deluge-base-stage5 "$BIN" linux --features linux 2>&1 | tail -8
cd ../..
```

Expected: `packed .../AUDIO_PASSTHRU.ELF`. The `const _` layout assert is evaluated here, against `armv7-unknown-linux-musleabihf`.

- [ ] **Step 5: Confirm the artifact is a static ARM binary**

```bash
file examples/audio_passthru/target/armv7-unknown-linux-musleabihf/release/audio_passthru
```

Expected: `ELF 32-bit LSB executable, ARM, EABI5 ... statically linked`.

- [ ] **Step 6: Build `additive_osc` for linux too**

```bash
BIN=$PWD/tools/cargo-deluge/target/x86_64-unknown-linux-gnu/debug/cargo-deluge
cd examples/additive_osc
DELUGE_BASE=~/GitHub/deluge-linux/out/deluge-base-stage5 "$BIN" linux --features linux 2>&1 | tail -6
cd ../..
```

Expected: `packed .../ADDITIVE_OSC.ELF`. This proves a non-trivial DSP closure (owned per-voice state + `static` atomics + a C++ FFI call) satisfies `Send + 'static` and the `!Send` handle rule.

- [ ] **Step 7: Re-run device builds and the test suite**

```bash
(cd examples/audio_passthru && cargo deluge build 2>&1 | tail -2)
(cd examples/additive_osc && cargo deluge build 2>&1 | tail -2)
tools/test.sh 2>&1 | tail -5
```

Expected: both ELFs build; `==> All tests passed.`

- [ ] **Step 8: Commit**

```bash
git add crates/deluge-sdk/Cargo.toml crates/deluge-sdk/src/plat/linux.rs
git commit -m "feat(linux-backend): ring-free audio — run the DSP closure on libdeluge's RT thread"
```

---

### Task 5: Hardware bring-up (manual, maintainer)

**Requires physical hardware.** Everything before this is verifiable on the host; nothing here is.

**Files:** none — deploy and observe.

- [ ] **Step 1: Upload and run `audio_passthru`**

```bash
BIN=$PWD/tools/cargo-deluge/target/x86_64-unknown-linux-gnu/debug/cargo-deluge
cd examples/audio_passthru
DELUGE_BASE=~/GitHub/deluge-linux/out/deluge-base-stage5 "$BIN" linux --features linux --run
```

Put the unit on the boot menu first (the dev-upload CDC port must be present).

- [ ] **Step 2: Confirm audio passthrough**

Feed line-in, listen at line-out. Expected: clean passthrough at unity — this is the end-to-end proof of the callback path, the in-place contract, and the layout reinterpret.

If there is **no sound at all**, the app panicked — most likely `linux audio unavailable: ...`, meaning the ALSA `Deluge` card is absent. That is a bundle/kernel problem, not a defect in this work.

- [ ] **Step 3: Check for the RT warning — on the serial console**

Watch the **attached serial/boot console** as the app starts. That is the only channel that carries it: the warning goes to appliance stderr → boot console. It does **not** come back over `cargo deluge linux --run` (a serial upload path that returns once the image is sent), and `--log` tails the device-firmware USB-log CDC, which is unrelated.

Expected: **no** line matching `audio thread is NOT SCHED_FIFO`.

If it appears, the process did not get `CAP_SYS_NICE`/`RLIMIT_RTPRIO`. Resolve that in `deluge-linux` before judging audio quality — any glitching from here on is a privilege problem, not a defect in this code. A line reading `sched_getscheduler query failed` instead means the check itself could not run; that is inconclusive, not a pass.

If no console is attached, this step cannot be performed and Step 5's load test yields no verdict on scheduling — say so in the result rather than recording a clean run.

- [ ] **Step 4: Run `additive_osc` under load**

```bash
BIN=$PWD/tools/cargo-deluge/target/x86_64-unknown-linux-gnu/debug/cargo-deluge
cd examples/additive_osc
DELUGE_BASE=~/GitHub/deluge-linux/out/deluge-base-stage5 "$BIN" linux --features linux --run
```

Play pads. Expected: voices sound, and the OLED/pad UI stays responsive while audio runs — proving the DSP half and UI half coexist across the thread split. Note any glitching, with whether Step 3 was clean, so a scheduling problem is not misfiled as a shim bug.

- [ ] **Step 5: Record the result**

Update the `linux-backend-native` memory with the outcome (audio proven or not, and any hardware facts learned), matching how the OLED/sync-LED milestone was recorded.

---

## Self-Review

**Spec coverage:**
- §3a shim → Task 4 Step 2. `adapt_block` extraction → Task 1 Step 3.
- §3b `Send + 'static` → Task 2 Steps 1-2, with the `plat` signature triple explicitly listed (missing one is a compile error).
- §3c `!Send` handles → Task 3, all 12 definitions enumerated including the two cfg-gated pairs (`Sd`, `SyncLed`) that are easy to half-fix.
- §3d RT confirmation → Task 4 Step 2 (the check) and Task 5 Step 3 (the observation). The `libc` dep §3d calls out → Task 4 Step 1.
- §4.1 unit tests → Task 1 Steps 1-4; `test.sh` addition → Task 1 Step 5.
- §4.2 compile → Task 4 Steps 4-6. §4.3 cross-backend → Task 3 Step 6 (all 15 examples). §4.4/§4.5 hardware → Task 5.
- §2 non-goals: no task touches `audio_stop`/`xruns`, libdeluge scheduling, sample rates, USB audio, or the other `plat::linux` stubs.

**Placeholder scan:** none. Every code step shows complete code; every command has expected output. Task 3 Step 4 enumerates each file and field rather than saying "and the rest similarly".

**Type consistency:** `adapt_block(&mut f, inp, out)` — defined Task 1 Step 3, called Task 4 Step 2, same argument order and `&mut F` receiver. `NotSend`/`NOT_SEND` — defined Task 3 Step 1, used Task 3 Step 4 with the field name `_not_send` throughout. `audio_run`'s bound is identical in all three `plat` files (Task 2 Step 2) and unchanged when Task 4 rewrites the linux body.

**Sequencing:** Task 1 is standalone (pure logic, no backend). Task 2 changes an API and fixes its one caller. Task 3 is the highest-regression-risk change and is gated by a 15-example device sweep *before* any linux work. Task 4 is the only task that cannot be verified without the bundle. Task 5 is the only task that cannot be verified without hardware. Each task ends green and committed.

**Deliberate deviation from TDD:** Tasks 2-4 are gated by *compile-time* proofs rather than runtime tests — the `!Send` assertions (Task 3 Step 3 verifies they fail before they pass, preserving the red-green cycle), the `const _` layout assert, and the trait bounds themselves. This is not a shortcut: `deluge-sdk`'s hardware paths cannot execute on the host, and Task 1 already unit-tests the only logic that can be wrong without hardware.
