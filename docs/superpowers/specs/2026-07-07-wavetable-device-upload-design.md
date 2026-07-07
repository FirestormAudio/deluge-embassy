# Wavetable device upload — real-time-safe firmware `upload_table` — design & spec

Completes the device story for [user-supplied dynamic wavetables](2026-07-07-osc-wavetable-design.md).
3b implemented `Wavetable.from` host-side and **deferred the firmware `upload_table`**
(it inherits the `Host` trait default `None`, so on device `Wavetable.from` yields an
unbound handle → silence). 3c made the mip build ~16× cheaper (IFFT). This cut wires
a **real-time-safe inline `upload_table`** into `wren-firmware`, so a Wren script can
build and play its own wavetable on the actual Deluge.

> **Status:** design proposal. Depends on merged 3b (Pool-in-Engine, `Host::upload_table`
> seam, `Wavetable.from`) and 3c (IFFT `build_pyramid_flat`). **Its final gate is
> on-hardware validation, which the maintainer performs** — this spec delivers a
> compiling, soundness-documented firmware path + a device checklist, not a
> hardware-verified result.

**Scope (chosen):** synchronous inline build on the device, fitting the audio
write-ahead slack; `audio_task` restructured to drop its long-lived engine borrow so
`vm_task` can legally reach the pool. **Deferred:** async/chunked build; moving the
Pool out of the Engine; multi-frame morph.

---

## 1. The constraints (from firmware recon)

- **One cooperative embassy executor.** `vm_task` and `audio_task` share it; there is
  no preemption and no second executor/priority. A Wren foreign call (`Wavetable.from`)
  runs synchronously to completion inside `wren_sys::interpret` and **cannot yield** —
  while it runs, `audio_task` cannot be polled.
- **Audio has write-ahead slack.** The SDK keeps the TX DMA filled ahead of playback:
  `TX_WRITE_AHEAD_FRAMES` = 512 (~11.6 ms) in poll mode, 256 (~5.8 ms) under the
  `audio-irq` feature. A synchronous stall shorter than that lead does **not** underrun.
- **3c made the build cheap.** `mipgen::build_pyramid_flat` = 1 forward + 11 inverse
  FFTs (O(N log N) each) ≈ sub-millisecond on the Cortex-A9 — plausibly well inside the
  write-ahead budget (the exact figure is a hardware measurement).
- **The real blocker is ownership, not concurrency.** `audio_task` derives `eng: &mut Eng`
  from `static mut ENGINE` and captures it in the `process` closure for the whole
  never-returning future. That long-lived `&mut` makes any second reference into
  `ENGINE` (from `vm_task`) undefined behaviour — even though the cooperative executor
  means the two never actually run at the same time.
- **`Cmd` can't carry samples.** The control ring is `[Cmd; 256]` (`Cmd: Copy`, largest
  variant `NewNode{[Input;3]}`) — bulk table data cannot ride it. So `upload_table`
  must touch the pool directly, not via a command.

---

## 2. Architecture

### 2.1 Part A — `audio_task` drops the long-lived borrow

Restructure so **no `&mut Eng` is ever held across an `.await`**. `ENGINE` is
initialized once (before `.process`), but the render closure re-derives a fresh,
tightly-scoped `&mut *addr_of_mut!(ENGINE)` **inside** the closure body each block and
drops it on return. The per-block render logic is otherwise unchanged.

```rust
// init once, keep no borrow:
unsafe { (*addr_of_mut!(ENGINE)).write(Eng::new(SAMPLE_RATE)); }
audio.process(|block| {
    // SAFETY (see module docs): single cooperative executor; this closure is
    // synchronous (no .await inside); ENGINE is touched only here and in
    // `upload_table`, both synchronous, never interleaved; no ISR touches ENGINE.
    let eng: &mut Eng = unsafe { (*addr_of_mut!(ENGINE)).assume_init_mut() };
    while let Some(c) = CMD_RING.lock(|r| r.borrow_mut().pop()) { eng.apply(c); }
    /* render chunks (unchanged) */
}).await
```

### 2.2 Part B — `audio::upload_table` + `FwHost` override

A new `pub fn upload_table(base: &[f32]) -> Option<PoolHandle>` in `audio.rs` (keeping
`ENGINE` private to the module that owns the safety argument):

```rust
pub fn upload_table(base: &[f32]) -> Option<deluge_audio_graph::PoolHandle> {
    // REQUIRED init guard: vm_task is spawned BEFORE audio_task, so its boot
    // script could call Wavetable.from before audio_task initializes ENGINE.
    // Return None (→ unbound handle → silence) until ENGINE is ready — never
    // read uninitialized memory.
    if !ENGINE_READY.load(Ordering::Acquire) { return None; }
    // SAFETY (see module docs): runs synchronously inside a Wren foreign call on the
    // one cooperative executor; audio_task is parked at its .await holding no ENGINE
    // borrow; no ISR touches ENGINE — so no two &mut ENGINE coexist. ENGINE is
    // initialized (guard above).
    let eng: &mut Eng = unsafe { (*addr_of_mut!(ENGINE)).assume_init_mut() };
    let h = eng.pool_alloc(deluge_wren_core::PYRAMID_LEN)?;
    deluge_wren_core::build_pyramid_into(base, eng.pool_slice_mut(h)); // 3c IFFT path
    Some(h)
}
```

`ENGINE_READY: AtomicBool` is set `true` by `audio_task` immediately after it writes
`ENGINE` (before its first render). This guard is **required** (not optional): the
firmware spawns `vm_task` before `audio_task`, so a boot-script `Wavetable.from` can be
polled first — the guard makes that case degrade to a silent (unbound) table rather than
UB. It also means the same `assume_init_mut()` in `audio_task`'s closure is only reached
after the write, and `upload_table`'s is only reached after `ENGINE_READY` — both sound.

`FwHost::upload_table` (replacing the 3b `None` stub) forwards:
`fn upload_table(&mut self, base: &[f32]) -> Option<PoolHandle> { crate::audio::upload_table(base) }`.

`deluge_wren_core` exposes `pub const PYRAMID_LEN: usize = mipgen::N * mipgen::LEVELS;`
so the firmware needs no direct `mipgen`/`deluge-fft` dependency.

### 2.3 The soundness argument (documented at both sites + module docs)

On this firmware:
1. One cooperative `embassy_executor::Executor`; tasks switch only at `.await`.
2. `audio_task`'s render closure is fully synchronous (no `.await` inside) and only
   borrows `ENGINE` within it.
3. `vm_task`'s `Wavetable.from` → `upload_table` is fully synchronous (a blocking
   `wren_sys::interpret` C call) and only borrows `ENGINE` within it.
4. The two are on the same executor, so they never interleave; neither holds a borrow
   across a yield.
5. No interrupt handler touches `ENGINE` (audio ISRs touch the DMA ring buffers, not
   the engine).

∴ no two live `&mut ENGINE` ever coexist — the accesses are sound. This argument is
specific to the single-cooperative-executor model; a future preemptive/multi-executor
change would require revisiting (a `Mutex`, or decoupling the Pool — the deferred
alternative).

---

## 3. Testing & the honest limit

**What this cut can verify (host / CI):**
- **Compiles for the device target:** `cargo check -p wren-firmware --target armv7a-none-eabihf -Zbuild-std=core` (both default poll mode and `--features audio-irq`), zero new warnings.
- **The build path itself is already host-tested** (unchanged): the EngineHost
  `Wavetable.from` round-trip (3b) + the IFFT equivalence (3c) prove
  `build_pyramid_into`/`build_pyramid_flat` produce a correct band-limited pyramid.
  This cut only changes *where* that runs on device, not the DSP.
- **`PYRAMID_LEN`** equals `mipgen::N * mipgen::LEVELS` = 22528, and the firmware pool
  (`PCAP = 90112` = 4 pyramids) has room — a `const` assertion in the firmware.
- **A soundness `SAFETY` comment** at each `unsafe` site + the module `## Concurrency`
  doc updated to state the invariant.

**What only on-hardware validation can confirm (maintainer checklist — shipped in the
plan/PR):**
1. `Wavetable.from([...])` on device returns a bound handle and the node **plays the
   table** (not silence).
2. **No audible glitch on upload** — the inline build fits within the TX write-ahead
   lead. Measure the actual build time if possible.
3. Repeated uploads reclaim pool memory (bind → `Node.free()` → rebuild) without
   exhaustion or corruption.
4. Behaviour under `audio-irq` (5.8 ms lead) vs poll (11.6 ms lead) — **poll mode has
   ~2× the margin and is the safer default for upload-heavy use**; note if `audio-irq`
   shows glitches.

This is stated plainly because the sub-project's correctness genuinely depends on a
hardware property (build-time vs write-ahead lead) that can't be measured off-device.

---

## 4. Deferred (tracked follow-ups)

- **Async/chunked build** — `Wavetable.from` returns a pending handle; the build spreads
  across `vm_task` iterations (yielding to audio between levels); the table binds when
  ready. The robust fallback **if hardware shows the inline build overruns the lead**.
- **Decoupling the Pool from the Engine** into a shared static (bitmap-locked alloc,
  lock-free immutable-data reads) — needed only if a future preemptive/multi-executor
  model breaks the §2.3 cooperative-executor soundness argument.
- **Multi-frame morphing wavetables** — the marquee expressive follow-on.
- Chunking the pool copy across blocks; measuring/reporting the on-device build time.

---

## 5. Open questions (resolved during implementation)

- Whether to also chunk the pool write (the 88 KB pyramid copy) — start whole; chunk
  only if HW shows the copy alone overruns.
- Exact placement of `PYRAMID_LEN` (a `pub const` in `deluge_wren_core`'s root vs the
  `host` module) — plan decides; root is simplest.
- The `ENGINE_READY` init guard is **required** (§2.2) because `vm_task` is spawned
  before `audio_task`; the only open detail is whether to instead eagerly initialize
  `ENGINE` in `main`/`setup` before spawning any task (which removes the ordering
  hazard entirely and lets the guard be dropped) — plan decides between the guard and
  eager init. Eager init is cleaner if `Eng::new` can run in that context.
