# Three-repo topology reorg (platform / native SDK / Rust SDK) — design & spec

Split the current two-and-a-half-repo tangle into **three repos with one clean
seam** so that C, C++, and Rust are all first-class ways to write a Deluge app,
and both the bare-metal Embassy target and the Linux target are first-class SDK
platforms. The middle repo (`deluge-linux-sdk`) **dissolves along its grain**:
its C library goes *down* to the platform's build world, its Rust goes *up* to
the Rust SDK, and the two are joined only by the release **bundle** the platform
already publishes. This is **Phase 0** — a structural move that unblocks, but
does not itself implement, the native Linux SDK backend (Phase 1, separate
spec).

> **Status:** design proposal — scoped, awaiting sign-off before planning.
> **Spans three repos:** `deluge-linux` (BSP + image factory), a **new**
> `deluge-ndk` (C/C++ SDK, ex-`deluge-linux-sdk` C core), and `deluge-sdk`
> (Rust SDK). The old `deluge-linux-sdk` is retired.
> **Working names** `deluge-ndk` and the crate rename `deluge-linux` →
> `deluge-hal-linux` are provisional (see §7) — trivially find-replaceable.

---

## 1. Motivation

Three forces have converged that the current layout no longer fits:

1. **Bidirectional cross-repo path-deps.** `deluge-linux-sdk`'s `deluge-linux-ui`
   crate reaches *into* `deluge-sdk` by relative path
   (`../../../deluge-sdk/crates/deluge-ui-toolkit`), and the planned Linux backend
   makes `deluge-sdk` reach the other way into `deluge-linux-sdk`'s `deluge-linux`
   crate. Two workspaces that depend on each other by `../../../` are already one
   unit — they cannot be versioned or released independently, and every shared-crate
   API change is a lockstep dance. The boundary is providing an illusion of
   isolation, not the real thing.

2. **C / C++ / Rust are all first-class app-authoring languages.** Rust is the
   flagship — the richest experience (async, eventing, UI toolkits, three
   backends) and the language we steer users toward — but C and C++ are
   supported peers. A C++ wrapper over the C API is anticipated. That promotes
   `libdeluge` from "a thin lib the Rust bindings happen to link" into **a
   standalone C/C++ SDK** with its own consumers, build system, and release
   cadence.

3. **The Linux SDK backend is coming.** `deluge-sdk` is gaining a third platform
   backend (native Linux over `libdeluge`) alongside bare-metal Embassy and the
   host simulator. Building it against the *current* split would mean throwaway
   cross-repo wiring; building it against the *target* topology is clean. Phase 0
   pays for itself immediately at Phase 1.

The decisive design lens: **a repo boundary should coincide with a build-system +
release-cadence + consumer boundary.** A concern boundary that does not cross
those three is a *crate* boundary (a Cargo workspace member, a CMake target), not
a repo boundary. Applied here, all three lenses line up the same way:

| lens | `deluge-linux` | `deluge-ndk` | `deluge-sdk` |
| --- | --- | --- | --- |
| build system | Buildroot (Make) | CMake (C/C++) | Cargo |
| cadence | slow (ABI / kernel) | medium — C++ ergonomics iterate *independently of the ABI* | fast |
| consumer | device flashers | C / C++ app authors | Rust app authors |

Three build systems, three cadences, three consumer groups → three repos.

## 2. Goals & non-goals

**Goals**

- Retire `deluge-linux-sdk`, distributing its contents to the two repos that own
  its concerns, plus the new `deluge-ndk`.
- Establish the **release bundle as the single inter-repo interface**: the only
  thing any repo consumes from another is a published, version-pinned bundle
  (plus one Buildroot source-fetch, §5).
- Collapse the bidirectional `../../../` path-deps by landing *all* Rust in one
  `deluge-sdk` workspace with in-workspace paths.
- Make `libdeluge` a first-class **C/C++ SDK** (`deluge-ndk`) with its own CMake
  app-authoring surface, ready to host the future C++ wrapper with no structural
  change.
- Delete the `xtask`-builds-a-libdeluge-sysroot dance by shipping `libdeluge`
  (headers + `.a`) and `mkimage` *inside* the bundle.
- Preserve git history for moved files.
- Keep the tree green at every step: no window where a repo can't build.

**Non-goals**

- Implementing the native Linux backend itself (Phase 1, separate spec). This
  spec only guarantees the topology *serves* it (§10).
- Writing the C++ wrapper. The topology accommodates it (it is "more of
  `deluge-ndk`"); building it is later work.
- Any change to the kernel, drivers, ABI, U-Boot, or memory map. The bundle's
  *contents* grow (libdeluge sysroot + mkimage); its consumers and load
  addresses do not change.
- Merging into a monorepo (rejected — see §3) or splitting the shared Rust
  libraries into their own repos (rejected — they stay independently-publishable
  crates *within* `deluge-sdk`).

## 3. Alternatives considered

**Monorepo (all three welded together).** The only thing a split sacrifices is
atomic "change the ABI across C and Rust in one commit." But the ABI and its C
HAL (`libdeluge`) stay together in the platform's build world (§5), so an ABI
change *is* atomic where it matters, and `deluge-sys` re-runs bindgen from the
new bundle headers automatically. A monorepo would force Buildroot's heavy /
rare / Make world onto Cargo's fast / common / app-author world — an app author
would clone a Buildroot tree to build a blinky. Rejected: it punishes the common
consumer for the rare one with no atomicity gain the bundle doesn't already give.

**Two repos (fold `libdeluge` into `deluge-linux`).** This was the right answer
while `libdeluge` was a thin, Rust-only-consumed lib. Once C/C++ are first-class
and a C++ wrapper is in view, `libdeluge` acquires its own build system (CMake,
distinct from Buildroot), its own consumers (C/C++ app authors), and its own
cadence (ergonomics that move independently of the ABI). Folding it in would put
two build systems in the BSP repo and make a C++ convenience tweak a commit to
the kernel repo. Rejected once §1.2 landed.

**Four+ repos (C++ wrapper as its own repo).** Over-factored: the C++ wrapper
`#include`s the C headers directly and shares `libdeluge`'s CMake build. Same
build system, same ABI target, same consumers → same repo.

## 4. Target topology

| `deluge-linux` — BSP + image factory | `deluge-ndk` — C/C++ SDK | `deluge-sdk` — Rust SDK |
| --- | --- | --- |
| kernel, drivers, **the ABI** (`docs/drivers.md`), U-Boot, rootfs, toolchain | `libdeluge` **C library** (`src/*.c`) + public headers (`deluge/*.h`) | `deluge` facade + `#[deluge::app]` macro |
| `deluge-mkimage`, appliance / image assembly, memory map | **C++ wrapper** (future) over the C API | 3 backends: bare-metal (`rza1l-hal`/`deluge-bsp`), sim (`deluge-sim-link`/simulator), **linux** (`deluge-sys` + `deluge-hal-linux`) |
| **publishes the base bundle** (§5) | CMake integration: `deluge_add_app()`, `FindDeluge.cmake`, `deluge.pc` | shared publishable crates: `deluge-fixedpoint`, dsp-kernels, fft, fonts, ui/grid toolkits, image, protocol, wren-* |
| pulls `deluge-ndk` in as a **Buildroot package** → installs `libdeluge` into the rootfs *and* exports it into the bundle sysroot | C/C++ example apps + C/C++ app-authoring docs | `cargo deluge` (device / sim / **linux** subcommands); Rust examples incl. `launcher`/`snake`/`terminal`; bare-metal firmwares |
| pytest platform tests | ctest / pytest against `libdeluge` | Cargo tests (host + QEMU) |
| **Build: Buildroot + libdeluge-as-package. Consumes: `deluge-ndk` source (pinned).** | **Build: CMake. Consumes: bundle (toolchain + ABI headers).** | **Build: Cargo. Consumes: bundle (toolchain + `libdeluge` sysroot + `mkimage`).** |

Both Embassy and Linux end up first-class peers behind one `deluge` facade in
`deluge-sdk`; the C/C++ authoring experience is a peer product in `deluge-ndk`;
the platform owns everything that must not skew from the ABI.

## 5. The bundle contract (the spine)

`deluge-linux` already publishes a relocatable bundle
(`out/deluge-base-<ver>/`: cross-toolchain, kernel, rootfs, `DelugeToolchain.cmake`,
`manifest.json`). This reorg makes the bundle **the complete platform interface**
by adding two things:

- **`libdeluge` sysroot** — `include/deluge/*.h` + `lib/libdeluge.a` (+
  `lib/pkgconfig/deluge.pc`), produced by building the `deluge-ndk` Buildroot
  package during the platform build. This is what `DELUGE_SDK_ROOT` points at;
  `deluge-sys`'s `build.rs` already consumes exactly this shape.
- **`deluge-mkimage`** — the image-assembly tool, coupled to the platform's
  memory map / load addresses (see the streaming-app-loader spec), so both SDKs
  invoke *one* implementation.

The manifest gains version fields for the packaged `libdeluge` (the pinned
`deluge-ndk` rev) and `mkimage`, so a consumer can assert compatibility.

After this change, **the only thing any repo consumes from another is a
published, version-pinned bundle** — with the single exception of the Buildroot
source-fetch below.

### Artifact / dependency flow (a pipeline, not a cycle)

```
                 pinned rev (Buildroot source-fetch)
   deluge-ndk  ───────────────────────────────────────►  deluge-linux
   (C/C++ src)                                            (Buildroot build)
        ▲                                                      │
        │ bundle N-1 (toolchain + ABI headers)                 │ builds toolchain → kernel
        │ for standalone dev / CI                              │ → libdeluge (from ndk src)
        │                                                      │ → rootfs + sysroot
        └──────────────────────────────────────────────  publishes bundle N
                                                               │
                                   bundle N (toolchain +       │
                                   libdeluge sysroot +         ▼
                                   mkimage)              deluge-sdk
                              ─────────────────────────► (Cargo; linux backend
                                                          binds libdeluge via
                                                          the sysroot)
```

No cycle: within a platform build, Buildroot builds the toolchain and kernel
*before* the `libdeluge` package, so `deluge-ndk` compiles against in-build
staging, not a prior bundle. `deluge-ndk`'s *standalone* dev/CI build (and C/C++
app authors on x86-64) consume the *last published* bundle — pinned, one-way.
`deluge-sdk` consumes the published bundle sysroot. Every arrow targets a
published bundle or a pinned source rev; no repo depends on another's live
working tree.

## 6. Asset migration map

From the retiring `deluge-linux-sdk`:

| asset | destination | notes |
| --- | --- | --- |
| `src/*.c`, `include/`, lib `CMakeLists.txt`, `deluge.pc.in` | **`deluge-ndk`** | the C library proper |
| `cmake/FindDeluge.cmake`, `deluge_add_app()` | **`deluge-ndk`** | C/C++ app-authoring API; shells to the bundle's `mkimage` |
| C examples (`cv_demo.c`, `display_demo.c`, `midi_echo.c`, `sine_out.c`, `usb_*.c`, `examples/app`, …) | **`deluge-ndk`** | reference C apps |
| `building-an-app.md` (C/CMake portions) | **`deluge-ndk`** | Rust portions → `deluge-sdk` |
| `crates/deluge-sys` | **`deluge-sdk`** | unchanged; still binds via `DELUGE_SDK_ROOT` (now the bundle sysroot) |
| `crates/deluge-linux` | **`deluge-sdk`** as **`deluge-hal-linux`** | renamed off the repo-name collision (§7); becomes the Linux backend's safe HAL |
| `crates/deluge-linux-ui` | **`deluge-sdk`** | `../../../` paths collapse to in-workspace paths |
| Rust examples (`snake`, `terminal`, `launcher`, `rust-app`) | **`deluge-sdk`** examples | `launcher` binary is later packed into the LINUX image by `mkimage` |
| `xtask` (build app + sysroot + mkimage) | **split** | app-build+package logic → `deluge-sdk` `cargo deluge` linux subcommand; the sysroot-build step is *deleted* (bundle ships it) |
| `tools/deluge_mkimage/` | **`deluge-linux`** | image factory; shipped in the bundle |
| pytest/ctest for `libdeluge` | **`deluge-ndk`** | |

Confirmed to **stay put**: everything already in `deluge-sdk` (bare-metal HAL
`rza1l-hal`/`deluge-bsp`, DSP/toolkit crates, sim, `cargo-deluge`, `app-loader`,
firmwares); everything already in `deluge-linux` (Buildroot tree, bundle).
`deluge-linux` *gains* a Buildroot package definition that fetches `deluge-ndk`.

## 7. Naming

- **`deluge-ndk`** (Native Development Kit) for the C/C++ SDK repo/product —
  reads instantly as "C/C++ SDK," distinct from `deluge-sdk` (Rust) and from the
  `libdeluge` artifact. Runner-up: `deluge-native`. Provisional.
- **`libdeluge`** stays the *linkable artifact* name unchanged (`libdeluge.a`,
  `-ldeluge`, `deluge.pc`, headers under `deluge/`). **Resolved:** the "couple
  library by that name" collision is on the *product name*, not the linkable —
  so only the repo/product is renamed (`deluge-ndk`); `deluge-sys`'s
  `links = "deluge"` + bindgen allowlist and `deluge.pc` are untouched.
- **`deluge-linux` (crate) → `deluge-hal-linux`.** The safe Rust wrapper cannot
  keep a name identical to the *platform repo*. `deluge-sys` (the `-sys` FFI
  crate) keeps its name.

## 8. Build systems & how each repo is driven

- **`deluge-linux`** — `make` (Buildroot). Adds a package that cross-builds
  `deluge-ndk` at a pinned rev, installs `libdeluge` into rootfs + bundle
  sysroot. Publishes the bundle.
- **`deluge-ndk`** — `cmake -DCMAKE_TOOLCHAIN_FILE=<bundle>/DelugeToolchain.cmake`.
  Cross build (armv7 musl) for the artifact; host build for tests / x86-64 dev.
  `deluge_add_app()` produces an app + calls the bundle's `mkimage`.
- **`deluge-sdk`** — `cargo` with the existing multi-target discipline: default
  `[build] target = armv7a-none-eabihf` for device; explicit `--target` for host
  sim and for linux (`armv7-unknown-linux-musleabihf`, `+crt-static`). `build-std`
  stays out of global `[unstable]` (device uses it, linux/host do not). The
  linux backend is selected by a **cargo feature** (Phase 1), since device vs
  host-sim vs native-linux cannot be told apart by target triple alone
  (native-linux is `target_os = "linux"`, same as the sim's host build).
  `deluge-sys`'s `build.rs` points `DELUGE_SDK_ROOT` at the bundle sysroot.

## 9. Licensing & CI

- **License posture is now per-repo and must be set deliberately.** `libdeluge`
  is statically linked into *apps*, so `deluge-ndk` needs an explicit,
  app-author-friendly license (LGPL or permissive) rather than inheriting a
  GPL-by-association reading from living next to kernel code. `deluge-sdk` keeps
  `MIT OR Apache-2.0` with its GPL toolkits as opt-in crates. `deluge-linux`
  keeps its existing `LICENSE`; the `deluge-ndk` Buildroot package carries
  `deluge-ndk`'s own SPDX.
- **CI is per-repo**, joined by bundle pinning: `deluge-ndk` and `deluge-sdk` CI
  build against a pinned published bundle; `deluge-linux` CI builds the bundle
  (including the `deluge-ndk` package) and runs platform tests. A bundle bump is
  the unit of cross-repo integration.

## 10. Phase 1 preview — how the topology serves the Linux backend

The native Linux backend (own spec) is a **third platform backend** in
`deluge-sdk`, ring-free and libdeluge-direct:

- Audio: `Audio::process(|block| …)` hands the DSP closure straight to
  `libdeluge`'s `audio_start` on its RT thread; the future parks — no ringbuffer,
  no executor in the audio path.
- Input: `libdeluge`'s `input_start` callback feeds the same `static EVENTS`
  embassy channel the device/host pumps feed.
- Output (OLED/pads/LEDs/CV/gate/MIDI): direct blocking `libdeluge` writes from
  the async `flush`.
- Executor + time driver: reused from the existing host `platform-std` path.
- The per-method `#[cfg]` fork is refactored into a per-backend internal
  platform module (`plat_device | plat_sim | plat_linux`) so a third backend is
  one module, not a third arm scattered through ~12 files.

This topology is what makes that clean: `deluge-hal-linux`/`deluge-sys` live
*in* the `deluge-sdk` workspace (no cross-repo path-dep), and `libdeluge` arrives
as a bundle sysroot artifact (no in-repo C build). Phase 1 touches one repo.

## 10b. Deferred phase — packer unification (Stage 2c)

Stage 2b moves the `deluge_mkimage` packer into `deluge-linux`, where it meets
`deluge-linux`'s pre-existing `mk-app-elf.py` (the `make appelf` / `LINUX.ELF`
packer) — the two overlap on `memmap.py` + `uimage.py`. Stage 2b takes the
**share-one-module** path: a single `tools/memmap.py` + `tools/uimage.py` used by
both packers (this also fixes a **stale SDRAM ceiling** in `deluge-linux`'s
`memmap.py` — `0x10000000` where it must be `0x0FD20000` to match the app-loader
`SDRAM_HI`), with `deluge_mkimage` importing them flat.

**Stage 2c — unify the two packers into one. ✅ DONE (2026-07-19).** Took the
"thin caller" option: `deluge_mkimage.elf` is the single `pack`/`check`
implementation; `mk-app-elf.py` re-exports `elf.pack` and `check-app-elf.py`
wraps `elf.check`, deleting the duplicated bodies (−164 lines) with no interface
or behavior change (`make appelf`/`release.yml`/pytest untouched, `LINUX.ELF`
still packs + validates). Plan:
`docs/superpowers/plans/2026-07-19-stage2c-unify-elf-packers.md`.

## 11. Risks & open questions

- **Artifact name collision (resolved).** The `libdeluge` collision is on the
  repo/product name only — solved by `deluge-ndk`. The linkable artifact
  `libdeluge` is unchanged; `deluge-sys` and `deluge.pc` do not move.
- **Multi-repo coordination cost.** Three repos = 3 CIs and an ABI change that
  needs a `libdeluge` change is a two-step (land in `deluge-ndk` → bump the pin
  in `deluge-linux`'s package). Real for a small team; mitigated because most
  coupling collapses to "bump the pinned bundle/rev," and the build-system
  heterogeneity is a hard force independent of team size.
- **History preservation.** Moves use `git filter-repo` (or `git subtree split`)
  to carry history into `deluge-ndk` and into `deluge-sdk`, rather than a flat
  copy. Sequencing (proposed for the plan): (1) stand up `deluge-ndk` from the C
  core; (2) add the `deluge-ndk` Buildroot package to `deluge-linux` and teach
  the bundle to ship the `libdeluge` sysroot + `mkimage`; (3) move the Rust
  crates + examples into `deluge-sdk`, repoint `DELUGE_SDK_ROOT` at the bundle,
  rename the crate; (4) retire `deluge-linux-sdk`. Each step keeps all repos
  green.
- **Bundle size / build time.** Shipping `libdeluge` + `mkimage` in the bundle
  grows it slightly and adds the `deluge-ndk` package to the platform build;
  negligible against kernel + rootfs.
- **C++ wrapper (future).** Not built here; noted so `deluge-ndk`'s CMake layout
  reserves room for it (a `deluge/` C++ header set over the C API in the same
  build).
