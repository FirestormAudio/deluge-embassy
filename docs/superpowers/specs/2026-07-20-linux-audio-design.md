# Ring-free audio on the native Linux backend — design & spec

Implement `plat::linux::audio_run` so a `#[deluge::app]` DSP closure runs on
libdeluge's audio thread with **no ring buffer and no cross-thread queue**
between the app and the codec. This is the headline capability remaining after
the Phase 1b walking skeleton (OLED + input + sync LED, proven on hardware
2026-07-19).

> **Status:** design proposal — awaiting sign-off before planning.
> **Branch:** `feat/linux-backend` (deluge-sdk).
> **Prereq (met):** Phase 1b complete — `plat::linux`, the linux runtime, and
> `cargo deluge linux --run` all exist and are hardware-proven.

---

## 1. Motivation

`plat/linux.rs:13` is a stub that panics. Every other backend runs the DSP
closure *on the Embassy executor*: the device backend polls DMA rings
(`plat/device.rs`), the sim backend polls the `deluge-sim-link` bridge. Neither
shape applies on Linux, because libdeluge owns the audio thread — it opens ALSA,
runs its own `pthread`, and calls back per period (`libdeluge/src/audio.c:84`).

The natural fit is therefore *inversion*: rather than the SDK pulling blocks,
libdeluge pushes them, and the app's closure executes directly inside that
callback. That is what "ring-free" means here — the DSP runs in the callback, so
there is nothing to buffer between.

Two measured facts make this clean rather than fiddly:

- **`DELUGE_PERIOD == 128 == BLOCK_FRAMES`** (`deluge/audio.h:27` vs
  `deluge-bsp/src/audio_block.rs:22`). The callback always gets exactly 128
  frames, so there is no variable-block problem and `additive_osc`'s
  `MAX_BLOCK: 256` assumption holds.
- **`StereoFrame` is `#[repr(C)] { l: f32, r: f32 }`** (`audio.rs:13-18`),
  layout-identical to libdeluge's `[f32; 2]`.

---

## 2. Scope

**In scope**

- `plat::linux::audio_run` — real implementation.
- A `Send + 'static` bound on `Audio::process`, applied to all three backends.
- Capability handles become `!Send`, statically preventing the DSP closure from
  touching hardware owned by the UI half.
- On-hardware verification with `audio_passthru` and `additive_osc`.

**Non-goals**

- `deluge_audio_stop()` / `deluge_audio_xruns()` bindings in `deluge-hal-linux`.
  Both exist in C and neither is bound; the app model here is start-once,
  run-forever, so neither is needed yet.
- **`SCHED_FIFO` on libdeluge's audio thread.** `src/audio.c:112` creates it with
  `pthread_create(..., NULL, ...)` — default scheduling, no RT priority. This is
  a genuine glitch risk under load, but the fix belongs in deluge-linux, not
  here. Recorded so it is not misdiagnosed as an SDK bug during bring-up.
- Sample-rate negotiation, USB audio, and the remaining `plat::linux` stubs
  (pads, LEDs, CV/gate, MIDI, SD, clock, jacks).

---

## 3. Design

### 3a. The shim

```rust
pub(crate) async fn audio_run<F>(mut f: F) -> !
where
    F: FnMut(&mut [crate::audio::StereoFrame]) + Send + 'static,
{
    // The transmute below is sound only under this layout equality. libdeluge
    // owns one side of it, so assert rather than trust the comment.
    const _: () = assert!(
        core::mem::size_of::<StereoFrame>() == core::mem::size_of::<[f32; 2]>()
            && core::mem::align_of::<StereoFrame>() == core::mem::align_of::<[f32; 2]>()
    );

    let shim = move |inp: &[[f32; 2]], out: &mut [[f32; 2]]| {
        // `Audio::process`'s contract is in-place: the block arrives pre-loaded
        // with input and whatever `f` leaves is sent to line-out. libdeluge
        // hands us split slices, so seed `out` with input and hand it over.
        debug_assert_eq!(inp.len(), out.len());
        out.copy_from_slice(inp);
        f(unsafe { core::mem::transmute::<&mut [[f32; 2]], &mut [StereoFrame]>(out) });
    };

    if let Err(e) = crate::linux::dev().audio_start(shim) {
        panic!("linux audio unavailable: {e}");
    }

    // `audio_start` moved the DSP onto libdeluge's thread; this task has no
    // further work but must not return (`-> !`) or drop the executor's future.
    core::future::pending().await
}
```

**Why `copy_from_slice` is the whole adaptation.** Both slices are 128 frames of
the same layout, so this is one 1 KiB memcpy per period — no scratch buffer, no
allocation, no queue. `debug_assert_eq!` documents the cross-repo length
invariant; `copy_from_slice` panics on mismatch in release too, which is the
correct failure for a broken ABI.

**Why panic on failure.** `Err` here is effectively `DELUGE_ERR_NODEV` — the
ALSA `Deluge` card is absent. An app that called `.audio()` needs audio; running
its UI silently with no sound is harder to diagnose than an abort naming the
cause. This matches the device backend, where codec init failure is also fatal,
and deliberately differs from `sync_led_init`/`input_start_pump`, where missing
hardware is genuinely optional.

**Why `pending()` rather than a poll loop.** Once `audio_start` returns, the SDK
has no per-block work — libdeluge drives everything. Parking is honest; a timer
loop would only burn wakeups.

### 3b. The `Send + 'static` bound

`Audio::process` becomes:

```rust
pub async fn process<F: FnMut(&mut [StereoFrame]) + Send + 'static>(self, f: F) -> !
```

Applied to **all** backends, not just linux. Device and sim don't need it — they
run the closure on the executor — but a bound that exists only on one backend
means an app compiles clean on device and fails on linux, which discovers the
portability break at the worst possible moment. A uniform contract is worth the
small cost of rejecting closures that would have been fine on device.

Churn is one line: `audio_passthru/src/main.rs:27` captures `drive` by
reference; adding `move` fixes it. `additive_osc` and `test_osc` already use
`move` closures over owned state and `static` atomics.

### 3c. Static RT isolation

On linux the closure runs on libdeluge's thread, while every sync SDK op
(`SyncLed::set`, `Gate::set`, `Midi::try_recv`, `Jacks::*`, `Sd::read`) goes
through `crate::linux::dev()` — a lock on the process-wide `Mutex<Deluge>`
(`linux.rs:9`) that the UI half also takes. Calling one from the DSP closure
would block the audio thread on the UI thread. Documentation alone would leave
that as a runtime glitch with no compile-time signal.

It can be made a compile error instead, at the cost of one field per handle.
Every hardware operation in the SDK is a method on an owned handle obtained from
`Deluge` — `SyncLed`, `Cv`, `Gate`, `Jacks`, `Midi`, `Sd`, `Leds` — and there
are no free functions. So the only way a closure can reach hardware is by
capturing a handle. Make the handles `!Send`:

```rust
pub struct Jacks {
    _private: PhantomData<*const ()>,   // was: ()
}
```

and the `Send` bound from §3b rejects any closure that captures one, at the
`.process()` call site. Apps keep the pattern `additive_osc` already models:
share state with the DSP half through `static` atomics.

This is safe to do because Embassy's executor is single-threaded — nothing in
the SDK requires these handles to be `Send` today. `Audio` itself stays `Send`:
`process(self, f)` consumes it, so it is never captured by the closure.

The escape hatch is a user's own `unsafe impl Send` wrapper. That is acceptable:
it makes bypassing the rule explicit and greppable.

---

## 4. Verification

Ordered so each step isolates one risk:

1. **Compile.** `cargo deluge linux --features linux` on `audio_passthru`. The
   layout `const _` assert and the musl build both gate here.
2. **Cross-backend regression.** Device and sim builds of every example stay
   green. §3b and §3c are the only changes touching all backends — if `!Send`
   breaks anything it will be an Embassy spawn interaction, and it surfaces
   here rather than after the hardware trip.
3. **`audio_passthru` on hardware.** Line-in reaches line-out. Proves the
   callback path, the in-place contract, and the layout reinterpret.
4. **`additive_osc` on hardware.** DSP under real load, with the UI half running
   concurrently. Proves the split survives contention — and is where the missing
   `SCHED_FIFO` (§2) would first show up as glitching under load.

---

## 5. Self-review

**Placeholders:** none. Every operation named is either implemented in §3a or
listed as a non-goal in §2.

**Consistency:** the `Send` bound in §3b and the `!Send` handles in §3c are one
mechanism, not two — §3c only works because §3b exists, and §3a's closure is the
thing both constrain. The panic in §3a is explicitly reconciled against the
log-and-continue precedent in `sync_led_init`.

**Scope:** one plan's worth. The shim is one function; the bound and the
`PhantomData` fields are mechanical; verification is four steps. Deliberately
excludes the remaining `plat::linux` stubs, which have no shared design with
this work.

**Ambiguity:** "ring-free" is defined in §1 (DSP executes inside libdeluge's
callback) rather than left to interpretation. The 128-frame equality is cited to
both source files rather than asserted from memory.
