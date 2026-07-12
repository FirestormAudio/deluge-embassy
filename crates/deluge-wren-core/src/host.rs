//! The `Host` seam: everything the Wren bindings need from the outside world.
//!
//! The foreign methods run inside the VM (a single thread per target: the
//! firmware's `vm_task`, or the web simulator's audio worklet). They reach the
//! hardware — CV/gate jacks, MIDI, LEDs, OLED, the audio engine — only through
//! this trait, so the *same* bindings drive a real Deluge and a browser tab.
//!
//! ## Registration
//! Because Wren's foreign methods are bare `extern "C"` callbacks with no context
//! argument, the active host is held in a process-global. Each target constructs
//! one `'static` host and calls [`set_host`] once at boot, before running any
//! script. Access is single-threaded (the VM thread), matching the rest of the
//! binding state.

use core::ptr::addr_of_mut;

use deluge_audio_graph::Cmd;

/// Number of CV output jacks (`output[1]`, `output[2]`).
pub const CV_CHANNELS: usize = 2;
/// Number of gate output jacks (`gate[1]`..`gate[4]`).
pub const GATE_CHANNELS: usize = 4;

/// The outside world, as the Wren bindings see it. All control-rate / output
/// only — input events are pushed *into* the bindings by the host's own tasks
/// (see [`crate::midi_rx`], [`crate::input_dispatch`], [`crate::enc_turn`]).
pub trait Host {
    /// Current monotonic time in milliseconds (drives metro scheduling).
    fn now_ms(&mut self) -> u64;

    /// Set CV jack `ch` (0-based) to `volts`. Slew is computed in the bindings;
    /// this receives the already-slewed value. The host maps volts → hardware
    /// (e.g. a DAC code) or records them for display.
    fn cv_set(&mut self, ch: u8, volts: f32);
    /// Set gate jack `ch` (0-based) on/off.
    fn gate_set(&mut self, ch: u8, on: bool);

    /// Emit a complete DIN-MIDI message (1–3 bytes).
    fn midi_tx(&mut self, msg: &[u8]);

    /// Indicator LED by button id.
    fn led(&mut self, id: u8, on: bool);

    /// OLED frame-buffer drawing. `text` receives raw bytes (5×7 font, latin-1).
    fn oled_clear(&mut self);
    fn oled_text(&mut self, x: usize, y: usize, text: &[u8]);
    fn oled_pixel(&mut self, x: usize, y: usize, on: bool);
    fn oled_show(&mut self);

    /// Submit a control-rate audio-graph command. The host transports it to its
    /// `deluge_audio_graph::Engine` (firmware: a ring drained by the audio task;
    /// web: applied directly on the audio thread).
    fn audio_cmd(&mut self, cmd: Cmd);

    /// Build a band-limited mip pyramid from `base` (one single cycle) into a
    /// pool region owned by the host's `deluge_audio_graph::Engine`, and return
    /// its handle. `None` on exhaustion/bad input.
    ///
    /// Default: unsupported (no pool) → `None`, so hosts with no audio engine
    /// (e.g. [`crate::test_support::CmdCaptureHost`]) need not override this.
    fn upload_table(&mut self, base: &[f32]) -> Option<deluge_audio_graph::PoolHandle> {
        let _ = base;
        None
    }

    /// Build `nframes` band-limited mip pyramids into one contiguous pool
    /// region (`nframes * PYRAMID_LEN` long) and return its handle — the
    /// multi-frame counterpart of [`Host::upload_table`], used by
    /// `Wavetable.from2d`.
    ///
    /// `fill_frame(f, base)` is called once per frame, in ascending `f` order,
    /// synchronously and in-line (never stashed for later): it must fill
    /// `base` (`BASE_LEN` samples) with frame `f`'s single-cycle waveform
    /// before returning. The caller (the binding) owns reading the Wren list
    /// into `base`; this seam keeps VM slot access in the binding and pool
    /// access in the host, so implementers never touch the VM.
    ///
    /// Default: unsupported (no pool) → `None`, so hosts with no audio engine
    /// (e.g. [`crate::test_support::CmdCaptureHost`]) need not override this.
    fn upload_table_2d(
        &mut self,
        nframes: usize,
        fill_frame: &mut dyn FnMut(usize, &mut [f32]),
    ) -> Option<deluge_audio_graph::PoolHandle> {
        let _ = (nframes, fill_frame);
        None
    }

    /// Allocate a **zeroed** pool region of `len` f32s (a delay/effect ring
    /// buffer) in the host's `deluge_audio_graph::Engine` and return its
    /// handle. Unlike [`Host::upload_table`], no pyramid is built — the region
    /// is raw scratch the effect writes each block.
    ///
    /// Default: unsupported (no pool) → `None`, so hosts with no audio engine
    /// (e.g. [`crate::test_support::CmdCaptureHost`]) degrade to an unbound
    /// (dry-passthrough) effect rather than requiring an override.
    fn alloc_buffer(&mut self, len: usize) -> Option<deluge_audio_graph::PoolHandle> {
        let _ = len;
        None
    }

    /// Write a single f32 at `index` into the pool region backing `h` (a
    /// handle previously returned by [`Host::alloc_buffer`] et al.), in the
    /// host's `deluge_audio_graph::Engine`. Out-of-range `index` is silently
    /// ignored. Used by `SampleBuffer.from` to upload raw PCM verbatim.
    ///
    /// Default: unsupported (no pool) → no-op, so hosts with no audio engine
    /// (e.g. [`crate::test_support::CmdCaptureHost`]) need not override this.
    fn pool_set(&mut self, h: deluge_audio_graph::PoolHandle, index: usize, value: f32) {
        let _ = (h, index, value);
    }
}

/// Build a band-limited mip pyramid from `base` (one single cycle, padded or
/// truncated to `mipgen::N`) into `region`, using the flat, compact
/// (per-level-length) layout. `region` must be at least
/// `deluge_dsp_kernels::wavetable::COMPACT_LEN` (= [`crate::PYRAMID_LEN`])
/// long — shorter regions are left untouched (no panic). Shared by every
/// pool-backed [`Host::upload_table`] implementation so the embedder only has
/// to allocate the region and hand it here.
///
/// Uses the IFFT build path (forward FFT + per-level band-limit + inverse
/// FFT, then per-level decimation to its compact length), ~16× cheaper than
/// the previous additive (per-harmonic) build; the two are equivalent to f32
/// rounding.
pub fn build_pyramid_into(base: &[f32], region: &mut [f32]) {
    // IFFT + compact-decimate path (Osc 3c / wavetable-mip-compaction).
    mipgen::build_pyramid_flat_compact(base, region);
}

static mut HOST: Option<*mut (dyn Host + 'static)> = None;

/// Register the process-wide host. Call once at boot, before running any script.
pub fn set_host(host: &'static mut dyn Host) {
    // SAFETY: single-threaded VM context; called once during setup.
    unsafe { HOST = Some(host as *mut dyn Host) };
}

/// Borrow the registered host. Panics if [`set_host`] was never called.
#[inline]
pub(crate) fn host() -> &'static mut dyn Host {
    // SAFETY: the VM thread is the sole accessor; the pointer was set from a
    // `&'static mut` and is never reassigned, so reborrowing it is sound.
    unsafe {
        let p = (*addr_of_mut!(HOST)).expect("deluge_wren_core: host not registered");
        &mut *p
    }
}
