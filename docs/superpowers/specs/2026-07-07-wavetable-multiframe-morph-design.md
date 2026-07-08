# Multi-frame morphing wavetables (2D) — design & spec

The marquee expressive feature of the [wavetable suite](2026-07-07-osc-wavetable-design.md):
a table becomes a **stack of single-cycle frames**, and a `position` input morphs the
timbre by interpolating across frames — a true **2D wavetable** (X = phase, Y =
position). Built directly on the merged single-cycle engine and the
[mip compaction](2026-07-07-wavetable-mip-compaction-design.md) that makes many-frame
banks affordable.

> **Status:** design proposal. Depends on merged 3a (`WtOsc`/`MipSet`), 3b (Pool,
> `Wavetable.from`), 3c (IFFT `mipgen`), device-upload, and mip-compaction (compact
> per-frame pyramids). Reaches Serum-scale (up to 256 frames) all-resident on compact
> mips (~6.4 MB/table).

**Scope (chosen):** bilinear morph over stacked compact pyramids; `position` on port 2;
`Wavetable.from2d` (host-complete) + a couple of **named static 2D banks** (device-
playable). **Deferred:** async/chunked device upload of large *dynamic* 2D banks (the
real-time-build problem, 256× a single-cycle upload).

---

## 1. Goals & non-goals

**Goals**

- **Bilinear morph.** At render, `position → frame bracket (f0,f1)`; `pitch → mip bracket
  (lo,hi)`; interpolate the 4 bracketing single-cycles and **bilinear-blend** (linear
  across mips × linear across frames). `position` is an audio-rate input in `[0,1]`.
- **Reuse, don't rewrite.** Extract the single-cycle per-sample read into `sample_one`;
  single-cycle = one call, morph = two calls (frames f0/f1) + a linear blend. Kernel
  state stays just `phase`.
- **Frame count is derived, not stored.** A morph table is `FRAMES` compact pyramids laid
  end-to-end (frame `f` at `f*COMPACT_LEN`); `FRAMES = region.len()/COMPACT_LEN` (pooled)
  or from the static registry. Single-cycle is `FRAMES==1` — the existing fast path.
- **Spectrally safe for free.** Each frame is band-limited (per-frame compact mips), and
  a convex (linear) blend of band-limited signals stays band-limited — so morph adds no
  aliasing; it inherits the single-cycle pitch-safety. QA gates continuity, not new
  anti-aliasing.
- **Two authoring paths.** `Wavetable.from2d([[frame],[frame],…])` (dynamic, host-
  complete) and named static 2D banks (device-playable, built at `gen_tables` time).
- **Transparency.** `FRAMES==1` reproduces today's single-cycle output exactly; the
  compaction goldens/threshold tests and Osc-1/Osc-2 goldens are unchanged.

**Non-goals (deferred / out of scope)**

- **Async/chunked device upload of large dynamic 2D banks** — a 256-frame
  `Wavetable.from2d` builds 256 pyramids (~100s of ms), far over the audio write-ahead
  lead; the device path for big *dynamic* banks needs the deferred async build. (Static
  2D banks and small dynamic ones are fine on device.)
- Higher-order (cubic) inter-frame interpolation — linear this cut (spectrally safe,
  cheap); cubic-across-frames is a later refinement.
- Frame-axis anti-aliasing of *discontinuous* frame content (linear blend is enough for
  band-limited frames; no BLEP across frames).
- Non-uniform frame spacing; per-frame variable length (all frames = one `COMPACT_LEN`).

---

## 2. Kernel (`deluge-dsp-kernels/src/wavetable.rs`)

Refactor the current per-sample read into a helper, then add the morph path:

```rust
/// One pyramid sampled at wrapped phase `ph` with per-sample increment `dtp`:
/// mip-select + cubic-interp within a level + linear crossfade between levels.
/// (This IS the current single-cycle inner loop, extracted verbatim.)
fn sample_one(mips: &MipSet, ph: f32, dtp: f32) -> f32 { /* moved from process */ }

impl WtOsc {
    // unchanged single-cycle path now calls sample_one once:
    pub fn process(&mut self, mips: MipSet, freq: In, pmod: In, dt: f32, out: &mut [f32]);

    /// Morph path: `frames` are the per-frame MipSets (frame 0..FRAMES-1); `position`
    /// in [0,1] maps across frames. Per sample: ph = frac(phase + pmod); fpos =
    /// position*(FRAMES-1); f0=floor(fpos), f1=f0+1 (clamped); frac = fpos-f0;
    /// out = lerp(sample_one(frames[f0],ph,dtp), sample_one(frames[f1],ph,dtp), frac).
    pub fn process_morph(
        &mut self, frames: &[MipSet], freq: In, pmod: In, position: In, dt: f32, out: &mut [f32],
    );
}
```

State stays `{ phase }` (position is an input). `MorphSet`/`&[MipSet]` is assembled by
the graph from the flat region (per-frame `compact_levels`). `FRAMES==1` → `process`
fast path (no frame blend). Clamp `position` to `[0,1]`; `FRAMES==1` ignores it.

---

## 3. Graph (`deluge-audio-graph`)

- `Kind::Wavetable` stays; the render arm derives `FRAMES` = `region.len()/COMPACT_LEN`
  (pooled) or the static table's frame count. If `FRAMES==1`, call `WtOsc::process`
  (ports 0=freq, 1=pmod). If `FRAMES>1`, assemble `[MipSet; FRAMES]` (each via the
  compact layout over `region[f*COMPACT_LEN..]`) and call `process_morph` with **port 2
  = position**.
- `TableSrc::Static(TableId)`/`Pooled(PoolHandle)` unchanged; the static registry entry
  gains a frame count (or it's `flat.len()/COMPACT_LEN`). The pooled guard becomes
  `region.len() % COMPACT_LEN == 0` and `>= COMPACT_LEN`.
- Assembling `[MipSet; FRAMES]` with `FRAMES` up to 256 at render time: build a small
  per-frame `[&[f32]; LEVELS]` on demand inside the sample loop for only the **two**
  bracketing frames (f0, f1) — not all 256 (only 2 frames are read per sample). So the
  render never materializes 256 MipSets; it resolves 2 per sample from the flat region.

**Port 2 note:** for `Kind::Wavetable`, port 2 = `position`; the classic-square `.width`
(also port 2) is a different `Kind`, so there's no conflict.

---

## 4. Wren surface (`deluge-wren-core`)

- **`.position` setter** → `SetInput` port 2 (mirrors `.pm`/`.width`).
- **`Wavetable.from2d([[f0…],[f1…],…])`** (dynamic): read the outer list (frame count),
  and for each frame read its inner sample list (nested use of the existing 3b
  `get_list_count`/`get_list_element` — fetch frame `f` into a slot, treat it as a list;
  `ensure_slots(4)` for depth), build its compact pyramid via `build_pyramid_flat_compact`
  into `region[f*COMPACT_LEN..]`, and return a `WtObj` bound to the full `FRAMES*COMPACT_LEN`
  region. The host `upload_table` seam grows a `upload_table_2d(frames)` (or `build_pyramid_into`
  loops per frame). `Osc.wavetable(w, freq)` binds it; `.position` sweeps.
- **Named static 2D banks:** add a couple to `gen_tables` (built at compile time from
  defined frame sets — e.g. a harmonic-sweep bank, a formant/vowel bank), exposed as
  `WT.*` ids that produce a multi-frame static table. Device-playable with no runtime
  build.

---

## 5. QA acceptance & testing

- **Frame-sweep continuity (the key gate).** Sweep `position` across a frame boundary
  (e.g. between two distinct frames) → output changes **smoothly**, no click/zipper
  (level + RMS continuity through the frame-bracket switch, like the mip-boundary test).
- **Bilinear correctness.** At `position` on an exact frame (`frac==0`), output equals
  that single frame's single-cycle render; at the midpoint, output is the average of the
  two frames' renders (within interp tolerance).
- **Band-limit preserved.** A morphed output at a swept position, at high pitch, still
  clears the `< -21 dB` alias gate (blend of band-limited frames stays band-limited).
- **`FRAMES==1` transparency.** A single-frame morph table renders **identically** to the
  single-cycle `WtOsc::process` path — the fast path is bit-exact, and all merged
  single-cycle wavetable tests pass unchanged.
- **Frame-count derivation.** `region.len()/COMPACT_LEN` yields the right `FRAMES`;
  a wrong-sized region (`% COMPACT_LEN != 0`) degrades to silence, no panic/OOB.
- **Authoring round-trips.** `Wavetable.from2d` (EngineHost, host) builds a 2-frame table
  that renders both frames + morphs between them; the nested list-read parses correctly.
  A named static 2D bank renders + morphs.
- **Determinism; no new anti-aliasing needed** (morph is a linear blend).

---

## 6. Deferred (tracked follow-ups)

- **Async/chunked device upload** of large *dynamic* 2D banks (build spread across
  `vm_task` iterations; bind when ready) — the real-time path for `Wavetable.from2d` with
  many frames on device.
- Cubic (higher-order) inter-frame interpolation.
- More named 2D banks; per-frame variable length; non-uniform frame spacing.
- 2D table streaming from SD for banks beyond resident SDRAM.

---

## 7. Open questions (resolved during implementation)

- Whether `Kind::Wavetable` derives `FRAMES` purely from `region.len()` or the static
  registry stores it explicitly (pooled must derive; static could do either — deriving
  from the flat length is simplest and unifies them).
- The `MorphSet` type: `&[MipSet]` vs a flat `&[f32]` + `FRAMES` resolved per-sample —
  favor resolving the 2 bracketing frames per sample from the flat region (never 256
  MipSets at once).
- `upload_table_2d` seam shape vs looping `build_pyramid_into` per frame (host side).
- Which named 2D banks to ship (content) + their `WT.*` ids.
- `position` mapping: `[0,1] → [0, FRAMES-1]` linear (chosen); whether to expose a
  wrap/clamp mode (clamp this cut).
