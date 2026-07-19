# Native Linux SDK backend (Phase 1, walking skeleton) — design & spec

Add a **third platform backend** to the `deluge` facade so a `#[deluge::app]`
**async** app runs natively on the Deluge's Linux userland over `libdeluge`,
alongside the bare-metal Embassy (device) and in-process simulator (host)
backends. This milestone is a **walking skeleton**: get one existing app
(`oled_hello`) running on **real hardware** via `cargo deluge linux --features
linux`, on top of a **`plat` module refactor** that extracts the two existing
backends into `plat_device`/`plat_sim` so the new backend (and every future one)
is a single `plat_linux` file. Everything but the runtime + OLED (+ input pump)
is stubbed and filled in incrementally — **audio next**.

> **Status:** design proposal — awaiting sign-off before planning.
> **Branch:** `feat/linux-backend` (deluge-sdk; the Phase-0 reorg is merged to
> `main`). **Repo:** deluge-sdk only (consumes `deluge-hal-linux` + the bundle).
> **Prereq (met):** Phase 0 complete — `deluge-hal-linux` (libdeluge Rust HAL),
> `cargo deluge linux`, and a bundle shipping the `libdeluge` sysroot all exist.

---

## 1. Motivation

The facade already runs on two backends selected by one cfg predicate:
`target_os = "none"` (device, `deluge-bsp` peripherals) vs
`not(target_os = "none")` (host sim, the `deluge-sim-link` `SharedPanel`). The
original goal of this whole effort is a **third** backend: the *same* async app,
running as a native Linux process on the Deluge, driving real hardware through
`libdeluge` (the `deluge-hal-linux` crate). Phase 0 built the foundation (the
three-repo split, the crate, `cargo deluge linux`, the bundle); Phase 1 is the
backend itself.

Two forces shape this milestone:

- **De-risk the novel parts first (walking skeleton).** The genuinely unproven
  pieces are the *runtime bring-up* (a std executor + time driver driving the app
  while `libdeluge` owns its own threads) and the *build/run wiring* (feature
  selection, musl build, on-device launch). Prove those end-to-end with the
  smallest real app before investing in per-capability adapters.
- **One place per backend (the `plat` refactor).** Today each capability carries
  its device/sim logic as inline `#[cfg]` arms. A third backend would mean a
  third arm in all 12 modules and a re-split of every `not(none)` arm. Instead,
  extract each backend into a `plat` submodule so a new backend is **one file**
  (`plat_linux`) — chosen deliberately over inlining, for long-term clarity.

## 2. Goals & non-goals

**Goals**

- A **`plat` module** with a cfg-selected backend submodule
  (`plat::device | plat::sim | plat::linux`); the 12 capability modules delegate
  their backend-specific bodies to `plat::`. **Behavior-preserving for
  device+sim** — they build, the sim runs, and existing tests pass unchanged.
- A **`linux` cargo feature** on `deluge-sdk` and the three-way discriminator:
  `device = target_os="none"`, `linux = feature="linux"`,
  `sim = all(not(target_os="none"), not(feature="linux"))`.
- The **linux runtime**: a `linux.rs` (process-wide `deluge_hal_linux::Deluge`
  handle, mirroring `host.rs`) + `__rt::linux::run` (open libdeluge, run the app
  on a std `Executor`, start libdeluge's `input_start` → the existing `EVENTS`
  channel) + the macro's third `main` arm.
- **`plat::linux` implements OLED for real** (`Oled::flush` → libdeluge
  `oled_write`) and the **input pump**; every other capability operation is a
  `unimplemented!("… not on the linux backend yet")` stub so the crate compiles.
- **`oled_hello` runs on a real Deluge** via `cargo deluge linux --features
  linux` and draws to the screen — the end-to-end proof.

**Non-goals (this milestone)**

- Audio, pads, LEDs, CV/gate, MIDI, SD, clock, jacks on linux — stubbed now,
  implemented incrementally after (audio first). Their absence is a runtime
  `unimplemented!` panic with a clear message, not a compile error.
- A host/mock libdeluge (CI-runnable linux backend without hardware). Deferred;
  verification is on-device.
- Any device or sim behavior change. The `plat` refactor is a pure restructuring.
- USB, `usb-log`, `alloc`/`rtt` interplay with linux — out of scope here.

## 3. Architecture

### 3a. The `plat` module (foundation)

Introduce `crates/deluge-sdk/src/plat/` — `mod.rs` selects exactly one backend
submodule by cfg and re-exports it:

```rust
#[cfg(target_os = "none")] mod device; #[cfg(target_os = "none")] pub(crate) use device::*;
#[cfg(all(not(target_os = "none"), not(feature = "linux")))] mod sim; … pub(crate) use sim::*;
#[cfg(feature = "linux")] mod linux; #[cfg(feature = "linux")] pub(crate) use linux::*;
```

Each capability module keeps its **public API unchanged** (`Oled`, `Audio`,
`Input`, …) but its method *bodies* move to `plat`. Example — `Oled::flush`
today is `#[cfg(none)] send_frame(fb).await` / `#[cfg(not none)]
panel().set_display(fb)`; after the refactor it is `plat::oled_flush(fb).await`,
with the two current bodies living in `plat::device`/`plat::sim`. Signatures are
unified to the superset (e.g. `oled_flush` is `async` — the sim/linux impls
simply don't await hardware). The `__rt::{device,host}` entry points and
`host.rs` stay; `host.rs`'s `SharedPanel` statics become `plat::sim`'s concern.

This is the ~90-arm, 12-module refactor. It is mechanical and **verified by the
existing device build + `cargo deluge sim` + the test suite staying green** — no
new behavior, so any diff that changes output is a bug.

### 3b. The linux backend

- **Discriminator + deps.** `deluge-sdk/Cargo.toml`: a `linux` feature; the
  existing `[target.'cfg(not(target_os="none"))'.dependencies]` (embassy
  `platform-std`, `embassy-time/std`, `critical-section/std`, `env_logger`) stay
  for *both* sim and linux; `deluge-sim-link` + `deluge-simulator` move behind
  `not(feature="linux")`, and `deluge-hal-linux` is added behind `feature="linux"`.
- **`linux.rs`** — mirrors `host.rs`: a process-wide handle
  `static DELUGE: OnceLock<Mutex<deluge_hal_linux::Deluge>>`, `init()`, and an
  accessor `dev() -> MutexGuard<…>` that `plat::linux` uses.
- **`__rt::linux::run`** — mirrors `__rt::host::run`, minus the GUI/panel/ring:
  init `env_logger`; `deluge_hal_linux::Deluge::open()`; `linux::init(dev)`;
  start libdeluge input delivery (`Deluge::input_start(cb)`) with `cb` mapping
  `deluge_hal_linux::Event` → the SDK `Event` and pushing into the same
  `static EVENTS` channel the device/sim pumps feed; run the app on a std
  `Executor` (the main thread runs the executor directly — no GUI owns it).
- **Macro.** The non-device `main` arm branches on the feature:
  `#[cfg(all(not(target_os="none"), feature="linux"))]` → `__rt::linux::run(…)`;
  `#[cfg(all(not(target_os="none"), not(feature="linux")))]` → `__rt::host::run(…)`.

### 3c. `plat::linux` (this milestone)

- **OLED (real):** `oled_flush(fb)` → `plat::linux` calls
  `deluge_hal_linux::Deluge::oled_write(fb.as_bytes())` on the process handle.
  `oled_init()` is a no-op (libdeluge owns the panel), like the sim.
- **Input pump (real):** started in `__rt::linux::run` (§3b), so `Input::next()`
  is backend-agnostic (reads `EVENTS`) exactly as on device/sim.
- **Everything else (stub):** `audio_run`, `pads_flush`, `leds_*`, `cv_*`,
  `gate_*`, `midi_*`, `sd_*`, `clock_*`, `jacks_*`, `sync_led_*`,
  `pic_service_*` → `unimplemented!("<op> is not on the linux backend yet")`.
  The crate compiles; `oled_hello` (OLED-only) never hits a stub.

### 3d. Build & run

`oled_hello`'s crate enables `deluge/linux` (a `linux` feature on the example, or
`cargo deluge linux` passes `--features linux` for facade apps). `cargo deluge
linux` (Phase 0) already cross-builds for `armv7-unknown-linux-musleabihf`
against the bundle `libdeluge` sysroot and packs with `deluge-mkimage`. Deploy the
resulting image/bare binary to the Deluge and launch it.

## 4. Verification

- **Refactor (3a):** device build (`cargo build-fw -p …` equivalent) succeeds;
  `cargo deluge sim` runs an example with identical behavior; the test suite is
  green. No output changes.
- **Skeleton (3b–3d):** `cargo deluge linux --features linux` builds+packs
  `oled_hello`; on a real Deluge it **draws to the OLED**. That single result
  proves: the macro's linux `main`, the std executor + time driver, `libdeluge`
  open, the `plat::linux` OLED path, and the build/deploy pipeline.
- Fallback observable if OLED bring-up is fiddly: a boot log line or an LED, but
  OLED is the clear libdeluge-supported target.

## 5. Incremental roadmap (after the skeleton)

Each subsequent capability is a small, independently-verifiable step: replace its
`plat::linux` stub with a real libdeluge call, run an existing example on
hardware. Order: **audio** (the ring-free headline — `Audio::process`'s closure
handed to `deluge_hal_linux::Deluge::audio_start` on the RT thread, `process`
parks; `audio_passthru`/`additive_osc` on hardware), then pads/LEDs, CV/gate,
MIDI, clock/jacks, SD (libdeluge / filesystem). `Send + 'static` on the audio
closure is the one API nuance (device/sim run it on the app thread; linux runs it
on libdeluge's RT thread) — the atomics idiom the examples already use covers it.

## 6. Risks & open questions

- **Refactor blast radius.** 12 modules, ~90 arms — the plat extraction is large
  and must not change device/sim behavior. Mitigation: do it capability-by-
  capability, re-running the device build + sim + tests after each; it is
  mechanical (move body, unify signature) with no logic change.
- **Signature unification.** A few ops differ in async-ness/return shape across
  backends (OLED flush async-on-device; audio's closure-driven loop). Unify to
  the superset (async where any backend awaits; `-> !` loops owned by
  `plat::<backend>::audio_run`). Resolved per-op in the plan.
- **`oled_hello` fit.** Confirm it is OLED-only (no other capability) so no stub
  is hit; if it also uses input, that path is implemented too (the pump), so it
  is fine.
- **On-device loop.** Verification needs the `cargo deluge linux` → deploy →
  boot cycle on real hardware; first time anything from this SDK runs on the
  Deluge-Linux userland. Expect first-run bring-up friction (device nodes, the
  launcher/appliance path) separate from the backend code itself.
- **libdeluge audio binding.** The deluge-linux kernel recently moved to a
  standard `sound-dai`/ASoC binding; irrelevant to this OLED milestone, but note
  it before the audio step (the ALSA card/behaviour `audio_start` sees).
