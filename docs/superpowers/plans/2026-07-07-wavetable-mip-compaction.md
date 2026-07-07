# Wavetable mip storage compaction Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Store each mip level at its own resolution (`~2N` total instead of `11N`) — a ~5.5× storage reduction on every wavetable, and the enabler for 256-frame morph banks — while keeping the audio band-limited-equivalent.

**Architecture:** Shared computed layout functions (`level_len`/`level_offset`/`COMPACT_LEN`) live in `deluge-dsp-kernels` and are used by the `mipgen` builder, the kernel `static_mipset`, the graph pooled-assembly, and `deluge-wren-core`. The kernel DSP is unchanged (`interp_cubic`/`MipSet` are already length-agnostic); only the stored per-level lengths and the flat-region offsets change. An oversampling factor `OS_FACTOR` is measured (1 vs 2) against the alias-floor gate.

**Tech Stack:** Rust `no_std` (`deluge-dsp-kernels`, `mipgen` over `deluge-fft`), `deluge-audio-graph`, `deluge-wren-core`, `wren-firmware`. QA via `deluge-dsp-test` realfft. Host target `x86_64-unknown-linux-gnu`.

**Reference spec:** [mip compaction design](../specs/2026-07-07-wavetable-mip-compaction-design.md).

## Global Constraints

- **Layout is computed, one source of truth** (`deluge-dsp-kernels`): `level_len(L)`, `level_offset(L)`, `COMPACT_LEN`, `N_MIN`, `OS_FACTOR`. Every builder/reader computes the same layout; no stored descriptor.
- **`level_len` invariants:** power of two; `level_len(0) == N` (=2048); non-increasing; `>= N_MIN` (=32, ≥4 for cubic). `COMPACT_LEN == level_offset(LEVELS)`.
- **Kernel DSP unchanged:** `WtOsc::process`, `interp_cubic` (uses `table.len()`), `MipSet { levels: &[&[f32]] }`, and mip-*select* are untouched — only the data lengths differ.
- **Transparency:** audio band-limited-equivalent (alias floors re-measured, must hold the existing `< -21 dB` per-table gate); the Osc-1/Osc-2 exact goldens (saw/LPF, not wavetables) are unchanged; only `wavetables_generated.rs` is regenerated (intended re-pin). The 3a/3b/3c wavetable tests are threshold-based → stay green (re-measure).
- **`OS_FACTOR` is measured, not predicted:** build at `F=1` (~2N) and `F=2` (~4N), measure `worst_alias_db`, pick the smallest `F` that holds the gate. Record both.
- **Test commands:** `-p deluge-dsp-kernels`, `-p mipgen`, `-p deluge-audio-graph` (+`--features simd`), `-p deluge-wren-core --features test-support`; firmware `cargo check -p wren-firmware --target armv7a-none-eabihf -Zbuild-std=core -Zbuild-std-features=compiler-builtins-mem` (+ `--features deluge-sdk/audio-irq`).
- Zero warnings. Commit after each task.

## Reference values (from prior wavetable work)

- `mipgen::N = 2048`, `LEVELS = 11`, `max_harmonic(L) = (N/2)>>L`. Current full pyramid = `N*LEVELS = 22528`; `deluge_wren_core::PYRAMID_LEN = 22528`.
- `mipgen` (post-3c): `build_all`=IFFT, `build_all_additive`=oracle, `build_level_ifft(spectrum, level, &mut [f32;N])` (DC-zero + zero>kmax + `RealFft::process_inverse` at N), `build_pyramid_flat(base, region)` (flat `N*LEVELS`), `pub use deluge_fft::Complex`.
- Kernel `wavetable.rs`: `WtOsc`, `MipSet{levels:&[&[f32]]}`, `TableId`, `static_mipset(id)->Option<MipSet<'static>>` (reads `wavetables_generated::TABLES: [&[&[f32]];6]`), `interp_cubic`. `wavetables_generated.rs` = `[[f32;2048];11]` per waveform (6 tables: Saw/Square/Sine/Tri/Organ/Formant).
- `node.rs` `Kind::Wavetable` pooled arm: assembles `[&[f32]; LEVELS]` from `region[l*N..(l+1)*N]`, guard `region.len() == N*LEVELS`.
- Pool `PCAP` in `Engine<...>` aliases: `engine.rs`/`cmd.rs` tests `45056`, `test_support.rs`/`wren-firmware` `90112`. Firmware `const _: () = assert!(90112 >= PYRAMID_LEN)`.

---

## Task 1: Layout functions + `mipgen` compact builder + measure `OS_FACTOR`

**Files:**
- Modify: `crates/deluge-dsp-kernels/src/wavetable.rs` (or a new `layout` module) — the layout consts/fns; `crates/mipgen/Cargo.toml` (+`deluge-dsp-kernels` dep) + `src/lib.rs` (compact builder).

**Interfaces:**
- Produces (in `deluge-dsp-kernels`, `pub`): `N_MIN: usize = 32`, `OS_FACTOR: usize` (set by measurement), `const fn level_len(level: usize) -> usize`, `const fn level_offset(level: usize) -> usize`, `const COMPACT_LEN: usize`.
- Produces (in `mipgen`): `build_pyramid_flat_compact(base: &[f32], region: &mut [f32])` (region len `COMPACT_LEN`).

- [ ] **Step 1: Add layout functions to `deluge-dsp-kernels`**

In `crates/deluge-dsp-kernels/src/wavetable.rs` (module-level, `pub`):

```rust
pub const N_MIN: usize = 32;
pub const OS_FACTOR: usize = 1; // measured in Step 6; 1 (~2N) or 2 (~4N)

/// Samples stored for mip level `L`. Level L carries `(N/2)>>L` harmonics, needing
/// `N>>L` critical samples; oversample by `OS_FACTOR`, clamp to `[N_MIN, N]`.
/// (`N>>L` and `OS_FACTOR∈{1,2}` are powers of two, so no rounding needed.)
pub const fn level_len(level: usize) -> usize {
    let crit = N >> level;                 // = 2*max_harmonic(level)
    let mut m = crit.saturating_mul(OS_FACTOR);
    if m > N { m = N; }
    if m < N_MIN { m = N_MIN; }
    m
}

/// f32 offset of level `L` in a flat compact pyramid (prefix sum of level_len).
pub const fn level_offset(level: usize) -> usize {
    let mut off = 0;
    let mut l = 0;
    while l < level { off += level_len(l); l += 1; }
    off
}

/// Total f32 in one compact pyramid.
pub const COMPACT_LEN: usize = level_offset(LEVELS);
```

(`N`/`LEVELS` already exist in `wavetable.rs` as `const N = 2048` and via `mipgen::LEVELS`=11 — define/confirm a local `pub const LEVELS: usize = 11` in the kernel if not present, so the layout is self-contained.)

- [ ] **Step 2: Write the failing layout unit tests**

In `wavetable.rs` tests:

```rust
    #[test]
    fn layout_invariants() {
        assert_eq!(level_len(0), N);
        for l in 0..LEVELS { assert!(level_len(l).is_power_of_two() && level_len(l) >= N_MIN); }
        for l in 1..LEVELS { assert!(level_len(l) <= level_len(l - 1)); }
        let mut sum = 0;
        for l in 0..LEVELS { assert_eq!(level_offset(l), sum); sum += level_len(l); }
        assert_eq!(COMPACT_LEN, sum);
        assert_eq!(COMPACT_LEN, level_offset(LEVELS));
    }
```

- [ ] **Step 3: Run, verify fail → implement → pass**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels`. Implement (Step 1), pass. Record `COMPACT_LEN` (expect ~4094 for F=1).

- [ ] **Step 4: Add `mipgen` compact builder**

Add `deluge-dsp-kernels` to `crates/mipgen/Cargo.toml` `[dependencies]` (path `../deluge-dsp-kernels`). **If cargo rejects the dev-dependency cycle** (the kernel dev-deps `mipgen`), fall back to a tiny leaf crate `crates/wavetable-layout` holding the layout, depended on by both — note which you did. Then in `mipgen/src/lib.rs`:

```rust
use deluge_dsp_kernels::wavetable::{level_len, level_offset, COMPACT_LEN};

/// Build a full compact pyramid into a flat `COMPACT_LEN` region. Each level is the
/// 3c band-limited IFFT level (built at N) decimated to `level_len(L)` — alias-free
/// because the level has ≤ `level_len(L)/2` harmonics.
pub fn build_pyramid_flat_compact(base: &[f32], region: &mut [f32]) {
    if region.len() < COMPACT_LEN { return; }
    let mut b = [0.0f32; N];
    let n = N.min(base.len());
    b[..n].copy_from_slice(&base[..n]);
    let mut spectrum = [Complex::ZERO; N / 2 + 1];
    deluge_fft::RealFft::<N, 4>::process(&b, &mut spectrum);
    let mut full = [0.0f32; N];
    for level in 0..LEVELS {
        build_level_ifft(&spectrum, level, &mut full); // existing 3c: DC-zero + zero>kmax + inverse@N
        let m = level_len(level);
        let step = N / m;
        let base_off = level_offset(level);
        for i in 0..m { region[base_off + i] = full[i * step]; }
    }
}
```

(Decimation `full[i*step]` of a signal band-limited to `m/2` harmonics is alias-free. The per-size-inverse-FFT optimization from the spec is deferred — decimate is simpler and reuses `build_level_ifft`.)

- [ ] **Step 5: Write the compact band-limit test (in the KERNEL crate)**

Place this in `crates/deluge-dsp-kernels/src/wavetable.rs` tests — the kernel has `WtOsc`, `In`, `deluge-dsp-test` (dev-dep), AND `mipgen` (dev-dep from 3a), so it can build compact via `mipgen` and render via `WtOsc` naturally with the real `In::K`:

```rust
    fn compact_levels(region: &[f32]) -> [&[f32]; LEVELS] {
        core::array::from_fn(|l| &region[level_offset(l)..level_offset(l) + level_len(l)])
    }

    #[test]
    fn compact_saw_is_band_limited() {
        let mut base = [0.0f32; N];
        for (i, s) in base.iter_mut().enumerate() { *s = 2.0 * (i as f32 / N as f32) - 1.0; }
        let mut region = [0.0f32; COMPACT_LEN];
        mipgen::build_pyramid_flat_compact(&base, &mut region);
        let levels = compact_levels(&region);
        let sr = 48_000.0f32;
        let mut osc = WtOsc::new();
        let mut buf = [0.0f32; deluge_dsp_test::FFT_N];
        osc.process(MipSet { levels: &levels }, In::K(5_000.0), In::K(0.0), 1.0 / sr, &mut buf);
        let wa = deluge_dsp_test::spectrum::analyze_buf(sr, &buf)
            .worst_alias_db(5_000.0, 3.0 * (sr / deluge_dsp_test::FFT_N as f32));
        assert!(wa < -21.0, "compact saw worst_alias {wa} dB");
    }
```

(Confirm the exact `In` path/import in `wavetable.rs`'s tests — it's the kernel's `In` used by the existing `WtOsc` tests.)

- [ ] **Step 6: MEASURE `OS_FACTOR` (F=1 vs F=2)**

Build the six named bases (saw/square/sine/tri/organ/formant) compact at `OS_FACTOR=1` then `=2`; render each at 5 kHz through `WtOsc`; record `worst_alias_db`. Set `deluge_dsp_kernels::wavetable::OS_FACTOR` to the smallest F whose worst floor across all six holds `< -21 dB`. Document the six measurements per F + the choice in a comment. If F=1 holds, keep it (5.5× win); else F=2. Re-run all tests with the chosen F.

- [ ] **Step 7: Commit**

```bash
git add crates/deluge-dsp-kernels/src/wavetable.rs crates/mipgen
git commit -m "feat(wavetable): per-level mip layout + compact mipgen builder (OS_FACTOR measured)"
```

---

## Task 2: Flip readers to compact — static tables, pooled assembly, ripples

**Files:**
- Modify: `crates/mipgen/src/bin/gen_tables.rs` (+ regenerate `crates/deluge-dsp-kernels/src/wavetables_generated.rs`), `crates/deluge-dsp-kernels/src/wavetable.rs` (`static_mipset`), `crates/deluge-audio-graph/src/node.rs` (pooled arm), `crates/deluge-wren-core/src/{lib.rs,host.rs}` (`PYRAMID_LEN`→`COMPACT_LEN`, `build_pyramid_into`), Pool `PCAP` aliases (`engine.rs`/`cmd.rs`/`test_support.rs`/`wren-firmware/src/audio.rs`).

**Interfaces:**
- `static_mipset` returns a `MipSet` sliced from a flat compact `[f32; COMPACT_LEN]` per table via `level_offset`/`level_len`.
- `node.rs` pooled arm assembles the `MipSet` from a `COMPACT_LEN` region via the layout.
- `deluge_wren_core::PYRAMID_LEN` = `deluge_dsp_kernels::wavetable::COMPACT_LEN`.

- [ ] **Step 1: Regenerate the static tables (compact, flat)**

Update `gen_tables.rs` to build each named base via `build_pyramid_flat_compact` into a `[f32; COMPACT_LEN]` and emit `pub static <NAME>: [f32; COMPACT_LEN] = [...]` (flat) + `pub static TABLES: [&[f32]; 6] = [&SAW, &SQUARE, ...]`. Regenerate:
`cargo run -p mipgen --bin gen_tables --target x86_64-unknown-linux-gnu > crates/deluge-dsp-kernels/src/wavetables_generated.rs`. The file shrinks ~5.5×.

- [ ] **Step 2: Update `static_mipset` for the flat layout**

In `wavetable.rs`, `static_mipset(id)` now slices the flat `TABLES[idx]: &[f32]` into `[&[f32]; LEVELS]` via `level_offset`/`level_len` and returns `MipSet`. Since `MipSet` borrows `&'a [&'a [f32]]`, the assembled array must outlive the borrow — return-by-value of a `MipSet` referencing a stack array won't work; instead have `static_mipset` return a small owned type or change the call site to assemble the level array. **Design:** add `static_mipset_into(id, out: &mut [&'static [f32]; LEVELS]) -> Option<MipSet<'_>>` OR make the render arms assemble the `[&[f32]; LEVELS]` locally (as the pooled arm already does) and call `WtOsc::process(MipSet{levels:&arr}, ..)`. Prefer: a helper `static_table_flat(id) -> Option<&'static [f32]>` + the render arm builds the level array via layout (unifying static + pooled assembly). Update `Kind::Wavetable`'s static branch accordingly.

- [ ] **Step 3: Update the pooled arm + guard**

In `node.rs`, the `Pooled` branch assembles `core::array::from_fn(|l| &region[level_offset(l)..level_offset(l)+level_len(l)])` and the size guard becomes `region.len() == COMPACT_LEN` (import the kernel layout fns). The static branch uses the same assembly over `static_table_flat(id)`.

- [ ] **Step 4: Ripple `PYRAMID_LEN` → `COMPACT_LEN` + `build_pyramid_into` + Pool**

- `crates/deluge-wren-core/src/lib.rs`: `pub const PYRAMID_LEN: usize = deluge_dsp_kernels::wavetable::COMPACT_LEN;` (was `mipgen::N*LEVELS`).
- `crates/deluge-wren-core/src/host.rs`: `build_pyramid_into` calls `mipgen::build_pyramid_flat_compact` (was `build_pyramid_flat`).
- Pool `PCAP` in the `Engine<...>` aliases: may shrink (compact tables are ~5.5× smaller) — keep or reduce, but ensure `PCAP >= COMPACT_LEN` (and ≥ a few for the tests that alloc 2). Update `engine.rs`/`cmd.rs` (`45056`), `test_support.rs`/`wren-firmware` (`90112`) if reducing; the firmware `const _: () = assert!(PCAP >= COMPACT_LEN)` must hold (update the literal if `PCAP` changes). Simplest: leave `PCAP` as-is (now holds ~20 compact tables) and just confirm the asserts/tests pass — reducing PCAP is optional cleanup.
- Any test that allocated `N*LEVELS` (e.g. the 3b `pooled_wavetable_renders_and_frees`, the device tests) now allocs `COMPACT_LEN`; update those literals/refs.

- [ ] **Step 5: Update the wavetable tests for compact + verify all green**

Update 3a/3b kernel/graph/wren tests that assumed `N*LEVELS` layout (the pooled-render test's manual pyramid build → use `build_pyramid_flat_compact`; the per-table band-limit test unchanged in spirit). Run:
- `cargo test -p deluge-dsp-kernels`, `-p mipgen`, `-p deluge-audio-graph` (+`--features simd`), `-p deluge-wren-core --features test-support` — all pass, both goldens unchanged, per-table alias floors hold `< -21 dB`.
- `cargo check -p wren-firmware --target armv7a-none-eabihf -Zbuild-std=core -Zbuild-std-features=compiler-builtins-mem` (+`--features deluge-sdk/audio-irq`) — compiles (COMPACT_LEN assert holds).
Zero warnings.

- [ ] **Step 6: Commit**

```bash
git add crates/deluge-dsp-kernels crates/mipgen/src/bin/gen_tables.rs crates/deluge-audio-graph crates/deluge-wren-core wren-firmware
git commit -m "feat(wavetable): flip static+pooled readers to compact layout; ~5.5x smaller tables"
```

---

## Self-review notes

- **Spec coverage:** layout functions (Task 1); compact builder + `OS_FACTOR` measurement (Task 1); flat static tables + pooled reader (Task 2); `PYRAMID_LEN`→`COMPACT_LEN` + Pool + `build_pyramid_into` ripple (Task 2); QA (band-limit + equivalence in Task 1, per-table + goldens in Task 2).
- **Kernel DSP untouched:** only storage layout + reader-assembly change; `WtOsc::process`/`interp_cubic`/mip-select are unchanged (verify `interp_cubic` handles `N_MIN=32`).
- **Measured, not predicted:** `OS_FACTOR` is chosen from the F=1/F=2 alias-floor measurement (Task 1 Step 6).
- **Transparency:** only `wavetables_generated.rs` regenerates (intended); Osc-1/Osc-2 exact goldens (saw/LPF) untouched; wavetable threshold tests re-measured and green.
- **Dependency note:** layout lives in `deluge-dsp-kernels`; `mipgen` deps it (dev-dep cycle via the kernel's test dep on `mipgen` — cargo permits; leaf-crate fallback noted in Task 1 Step 4).
- **The `static_mipset` borrow shape** (Task 2 Step 2) is the one fiddly bit — a `MipSet` borrowing a stack-assembled `[&[f32];LEVELS]` needs the array to live at the call site; unify static + pooled assembly in the render arm. Resolve as noted.
- **Known follow-ups:** multi-frame morph (built on this); per-size inverse FFT optimization; streaming for banks beyond SDRAM.
