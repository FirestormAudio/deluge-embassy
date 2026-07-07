# Device validation — inline wavetable upload

The firmware `upload_table` (`wren-firmware/src/audio.rs`) builds a mip pyramid
**synchronously** inside the Wren `Wavetable.from` call, blocking the single
cooperative embassy executor for the build. This is safe only if the build fits
inside the audio TX write-ahead lead (~5.8 ms under `audio-irq`, ~11.6 ms in the
default poll mode). That property can only be confirmed on hardware — everything
below is the maintainer's on-device checklist.

Design rationale: `docs/superpowers/specs/2026-07-07-wavetable-device-upload-design.md`.
The `unsafe` soundness argument (single cooperative executor; `audio_task`'s render
closure and `upload_table` are both synchronous and never interleave; `ENGINE`
init'd once before any task; no ISR touches `ENGINE`) is documented in `audio.rs`'s
`## Concurrency` section.

## Checklist

- [ ] **Plays, not silence.** `var w = Wavetable.from([ ...one single cycle... ])`
      then `Out.patch(Osc.wavetable(w, 220))` → an audible tone (the built table),
      not silence.
- [ ] **No glitch on upload (poll mode, default).** Trigger an upload during
      playback → no audible click/dropout. If possible, timestamp around
      `upload_table` (GPIO toggle / cycle counter) and confirm the build is well
      under the ~11.6 ms poll write-ahead lead.
- [ ] **`audio-irq` mode (tighter, ~5.8 ms lead).** Rebuild with
      `--features deluge-sdk/audio-irq` and repeat. Note any glitch. **Poll mode has
      ~2× the margin and is the safer default for upload-heavy patches.**
- [ ] **Reuse / no leak.** build → `Node.free()` → build again, several times →
      no pool exhaustion, no corruption, tables still correct. (Region is freed
      node-scoped via `Cmd::Free`; see the 3b lifetime note.)
- [ ] **Boot-script upload.** A `Wavetable.from` in `MAIN.WREN` behaves — plays, or
      silently degrades if it somehow precedes engine init — but never crashes.
      (Eager `ENGINE` init in `main` before task spawn should make it always ready.)

## If the inline build glitches

Fall back to the deferred **async/chunked build** (design spec §4):
`Wavetable.from` returns a *pending* handle immediately; spread the pyramid build
across `vm_task` loop iterations (yielding to `audio_task` between mip levels); bind
the table when all levels are ready (the node plays silence until then). This removes
the long synchronous stall entirely, at the cost of an async-readiness state machine.
