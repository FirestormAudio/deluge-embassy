# Aux / Mixer Sugar (IO-2d) Plan — executed inline
- [x] Add `class Aux` (reverb/hall/plate/delay/chorus presets) + `class Mixer` (construct new / master / channel) to prelude.wren.
- [x] Cmd-capture tests (Aux.reverb → NewNode{Room}+BusWriteGains; Mixer.channel → BusSend) + e2e (aux reverb wet over 4096 frames; mixer ch.gain halves) in tests/audio_bindings.rs.
- [x] Full wren-core green both configs; firmware armv7a clean.
