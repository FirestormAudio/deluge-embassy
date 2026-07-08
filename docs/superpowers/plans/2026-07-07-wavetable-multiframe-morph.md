# Multi-frame morphing wavetables (2D) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A wavetable becomes a stack of single-cycle frames with a `position` input that morphs across them (bilinear over the frame × mip axes), reaching 256-frame Serum-scale banks on the compact mip storage — with single-cycle tables (`FRAMES==1`) bit-exact unchanged.

**Architecture:** Extract the single-cycle per-sample read into `sample_one`; add `WtOsc::process_morph(region, frames, …)` that resolves only the 2 bracketing frames per sample from the flat region (never all frames) and linear-blends. The graph derives `FRAMES = region.len()/COMPACT_LEN`, routing `==1` to the existing `process` and `>1` to `process_morph` with `position` on port 2. Wren gains `.position` and `Wavetable.from2d` (nested list-read → per-frame compact build), plus a couple of named static 2D banks.

**Tech Stack:** Rust `no_std` (`deluge-dsp-kernels`, `mipgen`), `deluge-audio-graph`, `deluge-wren-core`, `wren-firmware`. QA via `deluge-dsp-test`. Host target `x86_64-unknown-linux-gnu`.

**Reference spec:** [multi-frame morph design](../specs/2026-07-07-wavetable-multiframe-morph-design.md).

## Global Constraints

- **`FRAMES==1` transparency:** a single-frame table routes to the unchanged `process` fast path and is BIT-EXACT with today; all merged single-cycle wavetable tests + the compaction fidelity gate + Osc-1/Osc-2 goldens pass unchanged.
- **Morph resolves only 2 frames per sample** (the `position` bracket) from the flat region via the kernel's `level_offset`/`level_len` — never materialize `FRAMES` MipSets. Frame count is DERIVED (`region.len()/COMPACT_LEN`), not stored.
- **Spectrally free:** linear blend of band-limited frames stays band-limited — no new anti-aliasing. QA gates continuity + bilinear correctness + band-limit-preserved.
- **`position`** is port 2 (audio-rate `In`), clamped `[0,1]`, mapped linearly to `[0, FRAMES-1]`. Port 2 = position only for `Kind::Wavetable` (square's `.width` is a different `Kind`).
- Kernel layout (`COMPACT_LEN`, `level_len`, `level_offset`) and `sample_one` are the single source of truth. `no_std`; zero warnings; commit per task.
- **Test commands:** `-p deluge-dsp-kernels`, `-p mipgen`, `-p deluge-audio-graph` (+`--features simd`), `-p deluge-wren-core --features test-support`; firmware `cargo check -p wren-firmware --target armv7a-none-eabihf -Zbuild-std=core -Zbuild-std-features=compiler-builtins-mem` (+ `--features deluge-sdk/audio-irq`).

## Reference values

- Kernel `wavetable.rs`: `WtOsc { phase }`, `MipSet { levels: &[&[f32]] }`, `process(mips, freq, pmod, dt, out)`, `interp_cubic`, `level_len`/`level_offset`/`COMPACT_LEN` (=6208), `static_table_flat(id) -> Option<&'static [f32]>`, `compact_levels(region) -> [&[f32]; LEVELS]`. `LEVELS=11`.
- `mipgen::build_pyramid_flat_compact(base, region)` (region `COMPACT_LEN`).
- `node.rs` `Kind::Wavetable` arm: derives a `MipSet` from the compact region (static via `static_table_flat`, pooled via the region) + `WtOsc::process(mips, ins[0], ins[1], dt, out)`.
- Wren: `.pm`/`.width`/`.feedback` setters (port pattern), `Wavetable.from` + `WtObj{tag,handle:Option<PoolHandle>}`, `Host::upload_table`, `build_pyramid_into`, `SlotApi::get_list_count/get_list_element`.

---

## Task 1: Kernel — `sample_one` refactor + `process_morph`

**Files:** Modify `crates/deluge-dsp-kernels/src/wavetable.rs`.

**Interfaces:**
- Produces: `fn sample_one(mips: &MipSet, ph: f32, dtp: f32) -> f32` (the extracted single-cycle inner read); `WtOsc::process_morph(&mut self, region: &[f32], frames: usize, freq: In, pmod: In, position: In, dt: f32, out: &mut [f32])`.
- `process` unchanged in behavior (now delegates the inner read to `sample_one`).

- [ ] **Step 1: Write failing tests**

Add to `wavetable.rs` tests (build frames via `mipgen::build_pyramid_flat_compact`):

```rust
    fn frame_region(bases: &[[f32; N]]) -> Vec<f32> { // host test; std Vec ok in tests
        let mut r = vec![0.0f32; bases.len() * COMPACT_LEN];
        for (f, b) in bases.iter().enumerate() {
            mipgen::build_pyramid_flat_compact(b, &mut r[f * COMPACT_LEN..(f + 1) * COMPACT_LEN]);
        }
        r
    }

    #[test]
    fn morph_frames1_matches_single_cycle() {
        // FRAMES==1 morph must equal the single-cycle process bit-for-bit.
        let mut saw = [0.0f32; N];
        for (i, s) in saw.iter_mut().enumerate() { *s = 2.0 * (i as f32 / N as f32) - 1.0; }
        let region = frame_region(&[saw]);
        let levels = compact_levels(&region);
        let (mut a, mut b) = (WtOsc::new(), WtOsc::new());
        let mut oa = [0.0f32; 256];
        let mut ob = [0.0f32; 256];
        a.process(MipSet { levels: &levels }, In::K(220.0), In::K(0.0), 1.0 / 48_000.0, &mut oa);
        b.process_morph(&region, 1, In::K(220.0), In::K(0.0), In::K(0.5), 1.0 / 48_000.0, &mut ob);
        assert_eq!(oa, ob); // bit-exact
    }

    #[test]
    fn morph_position_endpoints_and_midpoint() {
        // position=0 → frame 0; position=1 → frame 1; position=0.5 → average.
        let mut saw = [0.0f32; N];
        let mut sq = [0.0f32; N];
        for i in 0..N { saw[i] = 2.0 * (i as f32 / N as f32) - 1.0; sq[i] = if i < N/2 {1.0} else {-1.0}; }
        let region = frame_region(&[saw, sq]);
        let sr = 48_000.0f32;
        let render = |pos: f32| { let mut o=[0.0f32;256]; let mut w=WtOsc::new();
            w.process_morph(&region, 2, In::K(220.0), In::K(0.0), In::K(pos), 1.0/sr, &mut o); o };
        let f0 = render(0.0); let f1 = render(1.0); let mid = render(0.5);
        for i in 0..256 { assert!((mid[i] - 0.5*(f0[i]+f1[i])).abs() < 1e-4, "midpoint avg @ {i}"); }
    }

    #[test]
    fn morph_sweep_is_continuous() {
        let mut saw = [0.0f32; N]; let mut sq = [0.0f32; N];
        for i in 0..N { saw[i] = 2.0*(i as f32/N as f32)-1.0; sq[i] = if i<N/2 {1.0} else {-1.0}; }
        let region = frame_region(&[saw, sq]);
        let sr = 48_000.0f32; let mut prev: Option<f32> = None; let mut p = 0.0f32;
        while p <= 1.0 {
            let mut o = [0.0f32; 256]; let mut w = WtOsc::new();
            w.process_morph(&region, 2, In::K(220.0), In::K(0.0), In::K(p), 1.0/sr, &mut o);
            let rms = (o.iter().map(|s| s*s).sum::<f32>()/o.len() as f32).sqrt();
            if let Some(pr) = prev { assert!((rms-pr).abs() < 0.05, "morph rms jump @ pos {p}"); }
            prev = Some(rms); p += 0.05;
        }
    }
```

- [ ] **Step 2: Run, verify fail**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels`. Expected: FAIL (`process_morph` undefined).

- [ ] **Step 3: Extract `sample_one` + implement `process_morph`**

Extract the current per-sample inner read of `process` (the mip-select `flevel`, `lo`/`hi`, the two `interp_cubic` calls, the crossfade) into `fn sample_one(mips: &MipSet, ph: f32, dtp: f32) -> f32`, and make `process` call it. Then:

```rust
    pub fn process_morph(
        &mut self, region: &[f32], frames: usize,
        freq: In, pmod: In, position: In, dt: f32, out: &mut [f32],
    ) {
        // Region must be `frames * COMPACT_LEN`; each frame is a compact pyramid.
        if frames == 0 || region.len() < frames * COMPACT_LEN { return; }
        let last = frames - 1;
        for (i, s) in out.iter_mut().enumerate() {
            let dtp = freq.at(i) * dt;
            let mut ph = self.phase + pmod.at(i);
            ph -= floorf(ph);
            // Frame bracket from position in [0,1].
            let fpos = position.at(i).clamp(0.0, 1.0) * last as f32;
            let f0 = fpos as usize;
            let f0 = if f0 > last { last } else { f0 };
            let f1 = if f0 + 1 > last { last } else { f0 + 1 };
            let ffrac = fpos - f0 as f32;
            let m0 = compact_levels(&region[f0 * COMPACT_LEN..(f0 + 1) * COMPACT_LEN]);
            let y0 = sample_one(&MipSet { levels: &m0 }, ph, dtp);
            let y = if f1 == f0 { y0 } else {
                let m1 = compact_levels(&region[f1 * COMPACT_LEN..(f1 + 1) * COMPACT_LEN]);
                let y1 = sample_one(&MipSet { levels: &m1 }, ph, dtp);
                y0 + (y1 - y0) * ffrac.clamp(0.0, 1.0)
            };
            *s = y;
            self.phase += dtp; self.phase -= floorf(self.phase);
        }
    }
```

(`compact_levels` already exists — the same per-frame assembly the static/pooled readers use. Confirm its visibility to `process_morph` — if it's test-only, promote it to a `pub(crate) fn`.)

- [ ] **Step 4: Run, verify pass**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels`. Expected: PASS — the three morph tests + all existing single-cycle/compaction tests unchanged (`process` behavior identical after the `sample_one` extraction — verify the compaction fidelity + per-table gates still hold). Zero warnings.

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-dsp-kernels/src/wavetable.rs
git commit -m "feat(wavetable): sample_one refactor + WtOsc::process_morph (bilinear frame morph)"
```

---

## Task 2: Graph — frame-aware `Kind::Wavetable` + `position`

**Files:** Modify `crates/deluge-audio-graph/src/node.rs`.

**Interfaces:** Consumes `WtOsc::process_morph`, `COMPACT_LEN`. The `Kind::Wavetable` render arm derives `FRAMES` and routes.

- [ ] **Step 1: Write the failing pooled-morph test**

Add to `engine.rs` tests (build a 2-frame region into the pool, bind, set position, render):

```rust
    #[test]
    fn pooled_morph_wavetable_renders() {
        let mut e = E::new(48_000.0);
        let cl = deluge_dsp_kernels::wavetable::COMPACT_LEN;
        let h = e.pool_alloc(2 * cl).expect("pool");
        // build 2 frames (saw, square) into the region
        let mut saw = [0.0f32; mipgen::N]; let mut sq = [0.0f32; mipgen::N];
        for i in 0..mipgen::N { saw[i]=2.0*(i as f32/mipgen::N as f32)-1.0; sq[i]=if i<mipgen::N/2 {1.0} else {-1.0}; }
        { let r = e.pool_slice_mut(h);
          mipgen::build_pyramid_flat_compact(&saw, &mut r[..cl]);
          mipgen::build_pyramid_flat_compact(&sq, &mut r[cl..]); }
        e.create(NodeId(0), Kind::Wavetable);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(220.0); // freq
        *e.node_input_mut(NodeId(0), 2).unwrap() = Input::Const(0.5);   // position (port 2)
        e.apply(Cmd::BindTable { node: NodeId(0), src: TableSrc::Pooled(h) });
        e.render_block();
        let out = e.node_output(NodeId(0), 0);
        assert!(out.iter().all(|s| s.is_finite() && s.abs() <= 1.2));
        assert!(out.iter().any(|&s| s != 0.0));
    }
```

- [ ] **Step 2: Run, verify fail → implement the frame-aware arm**

In `node.rs`, the `Kind::Wavetable` arm resolves the flat region (static via `static_table_flat(id)`, pooled via `pool_region`), computes `frames = region.len() / COMPACT_LEN`, and routes:

```rust
                if let State::Wt(o) = &mut self.state {
                    // resolve `region: &[f32]` (static flat or pooled) as today...
                    if let Some(region) = /* resolved flat region */ {
                        let frames = region.len() / COMPACT_LEN;
                        if frames <= 1 {
                            let levels = compact_levels(region);
                            o.process(MipSet { levels: &levels }, ins[0], ins[1], dt, outs.port(0));
                        } else {
                            o.process_morph(region, frames, ins[0], ins[1], ins[2], dt, outs.port(0));
                        }
                    }
                }
```

Pooled guard: `region.len() % COMPACT_LEN == 0 && region.len() >= COMPACT_LEN` (else silence). (`compact_levels`/`COMPACT_LEN`/`process_morph` imported from the kernel.)

- [ ] **Step 3: Run, verify pass (both modes) + transparency**

Run: `cargo test -p deluge-audio-graph` and `--features simd`. Expected: PASS — the new pooled-morph test, the existing single-cycle pooled/static wavetable tests unchanged (FRAMES==1 path), both goldens unchanged. Zero warnings.

- [ ] **Step 4: Commit**

```bash
git add crates/deluge-audio-graph/src/node.rs
git commit -m "feat(audio-graph): frame-aware Kind::Wavetable morph (position on port 2)"
```

---

## Task 3: Wren — `.position` + `Wavetable.from2d`

**Files:** Modify `crates/deluge-wren-core/src/{bindings_audio.rs,bindings.rs,audio.rs,host.rs,test_support.rs}`, `wren/prelude.wren`, `tests/audio_bindings.rs`; `wren-firmware/src/audio.rs`+`host.rs` (upload_table_2d forward).

**Interfaces:** `.position` setter (SetInput port 2); `Wavetable.from2d(frames)` → nested read → per-frame compact build → `WtObj` bound to a `FRAMES*COMPACT_LEN` region; `Host::upload_table_2d(frames_flat, frames) -> Option<PoolHandle>` (or `build_pyramid_into` looped).

- [ ] **Step 1: Failing tests** — `.position` emits `SetInput{port:2}`; `Wavetable.from2d([[..],[..]])` (EngineHost `run_and_render`) renders finite/non-silent and morphs when `.position` sweeps. (Cmd-capture host: `upload_table_2d` returns None → unbound, assert graceful.)

- [ ] **Step 2: Implement `.position`** — `node_set_position_impl` → `audio::set_input(id, 2, arg_input)`, registered 4-place (impl+extern, register_audio, METHODS, prelude `foreign position=(v)`), mirroring `.width`.

- [ ] **Step 3: Implement `Wavetable.from2d`** — read outer list count (frames); `ensure_slots(4)`; for each frame `f`: `get_list_element(1, f, 2)` (frame list → slot 2), `get_list_count(2)`, read its samples into a `[f32; N]` scratch, then via the host build frame `f`'s pyramid into `region[f*COMPACT_LEN..]`. Grow the host seam: `Host::upload_table_2d(&mut self, frame_bases: impl Iterator<&[f32]> or a callback, frames)` OR simplest — `upload_table_2d` allocs `frames*COMPACT_LEN` then the binding fills each frame via a per-frame `build` call; mirror the 3b `upload_table` shape. Return `WtObj` bound to the multi-frame region. Register `Wavetable.from2d(_)` 4-place + prelude.

- [ ] **Step 4: Host seam + firmware** — add `upload_table_2d` to the `Host` trait (default `None`); implement on `EngineHost` (alloc `frames*COMPACT_LEN`, build each frame via `mipgen::build_pyramid_flat_compact`); firmware `FwHost::upload_table_2d` forwards to `audio::upload_table_2d` (same scoped-ENGINE pattern as `upload_table`; **note**: large dynamic banks stall the executor — documented as the deferred async-build case; small banks fine). Firmware capacity: `PCAP >= FRAMES*COMPACT_LEN` for supported frame counts — assert/limit.

- [ ] **Step 5: Verify** — `cargo test -p deluge-wren-core --features test-support` (from2d round-trip + `.position` + goldens unchanged); sys-backend build; firmware both modes compile. Zero warnings.

- [ ] **Step 6: Commit** — `feat(wren-core): .position setter + Wavetable.from2d (2D morph authoring)`.

---

## Task 4: Named static 2D banks

**Files:** Modify `crates/mipgen/src/bin/gen_tables.rs` (+ regenerate `wavetables_generated.rs`), `crates/deluge-dsp-kernels/src/wavetable.rs` (registry frame counts), `wren/prelude.wren` (`WT.*` ids), tests.

- [ ] **Step 1: Add two named 2D banks to `gen_tables`** — e.g. `HarmonicSweep` (frames sweeping 1→K harmonics of a saw) and `Formant` (frames shifting a formant peak). Each = `FRAMES` (e.g. 8–16) compact pyramids concatenated into a flat `[f32; FRAMES*COMPACT_LEN]`. Emit them + extend `TABLES` and the `WT` id set. Regenerate `wavetables_generated.rs`.

- [ ] **Step 2: Registry frame count** — `static_table_flat` already returns the flat slice; `frames = flat.len()/COMPACT_LEN` derives correctly for both single-cycle (1) and 2D banks (N) — confirm the render arm derives it uniformly, no per-table frame metadata needed.

- [ ] **Step 3: Test + prelude** — a named 2D bank renders + morphs (kernel or graph test: bind the static 2D table, sweep position, assert finite/non-silent/continuous). Add `WT.HarmonicSweep`/`WT.Formant` ids to the prelude `WT` holder + `Osc.wavetable(WT.HarmonicSweep, f)`.

- [ ] **Step 4: Verify all suites + firmware** — kernel/mipgen/graph/wren all green (single-cycle unchanged, 2D banks render), firmware compiles (regenerated tables fit; `PCAP >= largest static 2D bank`? — static tables are `&'static`, NOT pooled, so no PCAP constraint; confirm). Osc goldens unchanged.

- [ ] **Step 5: Commit** — `feat(wavetable): named static 2D morph banks (HarmonicSweep, Formant)`.

---

## Self-review notes

- **Spec coverage:** `sample_one`+`process_morph` (Task 1); frame-aware graph + position (Task 2); `.position`+`from2d`+host seam (Task 3); named static 2D banks (Task 4). QA: continuity/bilinear/frames1-transparency (Task 1), pooled morph (Task 2), from2d round-trip (Task 3), named-bank morph (Task 4).
- **`FRAMES==1` bit-exact:** the graph routes 1-frame tables to `process` (unchanged), and Task 1's `morph_frames1_matches_single_cycle` pins `process_morph(…,1,…)==process` too. `sample_one` extraction must not change `process` output — verified by the unchanged compaction/single-cycle tests.
- **2 frames per sample, not 256:** `process_morph` assembles only the f0/f1 `compact_levels` per sample from the flat region — no `FRAMES`-sized allocation. Frame count derived from `region.len()/COMPACT_LEN`.
- **Device honesty:** static 2D banks play on device (no runtime build); large *dynamic* `from2d` banks stall the cooperative executor (256× a single-cycle build) — Task 3 Step 4 documents this as the deferred async-build case; small dynamic banks are fine.
- **Transparency:** Osc-1/Osc-2 goldens untouched; single-cycle wavetable + compaction gates unchanged; only `wavetables_generated.rs` regenerates (Task 4, intended).
- **Known follow-ups:** async/chunked device upload of large dynamic 2D banks; cubic inter-frame interp; more named banks; SD streaming.
