//! Audio DSP — a per-block callback over the codec.

#[cfg(target_os = "none")]
use core::sync::atomic::{AtomicBool, Ordering};

/// One stereo audio frame; samples in `[-1.0, 1.0]`. `l` = left, `r` = right.
#[cfg(target_os = "none")]
pub use deluge_bsp::audio_block::Frame as StereoFrame;

/// One stereo audio frame; samples in `[-1.0, 1.0]`. `l` = left, `r` = right.
/// Host simulator definition (mirrors `deluge_bsp::audio_block::Frame`).
#[cfg(not(target_os = "none"))]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct StereoFrame {
    pub l: f32,
    pub r: f32,
}

/// Adapt libdeluge's split input/output buffers to [`Audio::process`]'s in-place
/// block contract.
///
/// libdeluge hands its callback two slices (input, output); the SDK's DSP
/// closure takes **one** slice pre-loaded with input, whose final contents are
/// sent to line-out. So seed `out` with the input and hand `out` to `f`.
///
/// Pure by design — no `libdeluge`, no hardware, no locks — so the Linux
/// backend's only interesting logic is unit-testable on the host. See
/// `plat::linux::audio_run`, its sole caller.
// `adapt_block` reinterprets `&mut [[f32; 2]]` as `&mut [StereoFrame]`. That is
// sound only if the two are layout-identical. `StereoFrame` is SDK-local and
// `#[repr(C)]`, so this cannot drift from under us silently — but it CAN be
// edited, and these asserts are what turn such an edit into a build error.
//
// Note what this does NOT prove: field *types*. `{ l: i32, r: f32 }` would
// satisfy every assertion below and make the transmute type confusion. The
// `#[repr(C)] { l: f32, r: f32 }` declaration above is the real contract;
// these asserts guard its layout consequences.
#[cfg(any(feature = "linux", test))]
const _: () = {
    assert!(core::mem::size_of::<StereoFrame>() == core::mem::size_of::<[f32; 2]>());
    assert!(core::mem::align_of::<StereoFrame>() == core::mem::align_of::<[f32; 2]>());
    assert!(core::mem::offset_of!(StereoFrame, l) == 0);
    assert!(core::mem::offset_of!(StereoFrame, r) == core::mem::size_of::<f32>());
};

/// The block length (stereo frames per callback) every backend is built
/// against. Device (`deluge_bsp::audio_block::BLOCK_FRAMES`) and the desktop
/// simulator (`deluge_sim_link::audio::BLOCK_FRAMES`) each independently hard-code
/// 128; neither of those constants is reachable from here, though — the former is
/// `#[cfg(target_os = "none")]` (this module also compiles hosted, for `feature =
/// "linux"`), and the latter is gated behind the `sim` feature, not `linux`. This
/// is the SDK-side source of truth apps size fixed-length buffers against on the
/// Linux backend (e.g. `examples/additive_osc`'s `MAX_BLOCK`); keep it in sync
/// with the other two by hand if the period ever changes.
#[cfg(any(feature = "linux", test))]
pub(crate) const EXPECTED_BLOCK_FRAMES: usize = 128;

#[cfg(any(feature = "linux", test))]
#[inline]
pub(crate) fn adapt_block<F>(f: &mut F, inp: &[[f32; 2]], out: &mut [[f32; 2]])
where
    F: FnMut(&mut [StereoFrame]),
{
    // libdeluge passes the same period length for both (DELUGE_PERIOD). The
    // `copy_from_slice` below also panics on mismatch in release, which is the
    // correct failure for a broken ABI.
    debug_assert_eq!(inp.len(), out.len());
    out.copy_from_slice(inp);
    // SAFETY: `StereoFrame` is `#[repr(C)] { l: f32, r: f32 }`, layout-identical
    // to `[f32; 2]` — size, alignment, and both field offsets are asserted at
    // compile time immediately above this function, and again at run time by
    // `stereoframe_is_layout_compatible_with_f32_pair`.
    f(unsafe { core::mem::transmute::<&mut [[f32; 2]], &mut [StereoFrame]>(out) });
}

#[cfg(target_os = "none")]
fn ensure_init() {
    static DONE: AtomicBool = AtomicBool::new(false);
    if DONE.swap(true, Ordering::Relaxed) {
        return;
    }
    // SAFETY: runs once. Brings up the codec on the direct SSI TX+RX DMA path
    // (no SCUX — that's only for rate conversion); blocks ~5 ms internally.
    // Acquire `audio()` before the main loop. Owns the codec — incompatible with
    // the USB UAC2 device tasks.
    unsafe { deluge_bsp::audio::init() };
}

/// The codec audio path, taken once from [`Deluge::audio`](crate::Deluge::audio).
///
/// Run a DSP callback over every block with [`process`](Audio::process). The
/// block arrives pre-loaded with codec line-in; overwrite it with the output
/// sent to line-out — so the same API serves insert-effects and synths.
///
/// **Owns the codec path.** Do not also run a USB audio (UAC2) device stack; both
/// drive the same SSI/SCUX rings.
pub struct Audio {
    _private: (),
}

impl Audio {
    pub(crate) fn new() -> Self {
        #[cfg(target_os = "none")]
        ensure_init();
        Self { _private: () }
    }

    /// Run `f` over every audio block, forever.
    ///
    /// `f` receives a `BLOCK`-length slice pre-loaded with codec input; whatever
    /// it leaves in the slice is sent to the codec. Never returns.
    ///
    /// `f` must be `Send + 'static` because the Linux backend runs it on
    /// libdeluge's audio thread rather than on the app's executor. The bound is
    /// uniform across backends on purpose: a closure that compiles on device but
    /// not on linux would hide the portability break until deploy time. Share
    /// state with the rest of the app through `static` atomics — see the
    /// `additive_osc` example.
    ///
    /// Capability handles ([`Oled`](crate::Oled), [`Pads`](crate::Pads),
    /// [`SyncLed`](crate::SyncLed), …) are deliberately `!Send`, so capturing one
    /// here is a compile error rather than a runtime audio stall.
    ///
    /// **Dropping the returned future diverges by backend.** `process` returns
    /// `-> !`, so this is uncommon — but not impossible, e.g. racing it in a
    /// `select!` against a UI branch. On device and in the simulator, dropping
    /// stops the DSP. On Linux, `audio_start` has already moved `f` into a
    /// context owned by `libdeluge`; dropping the future does not reach it, so
    /// `f` keeps rendering audio blocks forever with no owning task left.
    ///
    /// **`!Send` guarantees no hardware access, not RT-safety.** Nothing stops
    /// `f` from allocating, taking a lock, logging, or blocking on I/O — all of
    /// which were merely slow on the app's executor but will xrun the codec
    /// once `f` runs on libdeluge's `SCHED_FIFO` thread.
    ///
    /// ```ignore
    /// dlg.audio().process(move |block| {
    ///     for f in block { f.l *= 0.5; f.r *= 0.5; }
    /// }).await
    /// ```
    pub async fn process<F: FnMut(&mut [StereoFrame]) + Send + 'static>(self, f: F) -> ! {
        crate::plat::audio_run(f).await
    }
}

#[cfg(test)]
mod tests {
    use super::{adapt_block, StereoFrame};

    /// The closure must observe the *input* buffer, not `out`'s prior contents.
    #[test]
    fn input_reaches_the_closure() {
        let inp = [[1.0, 2.0], [3.0, 4.0]];
        let mut out = [[-9.0, -9.0]; 2];
        let mut seen = Vec::new();
        adapt_block(
            &mut |b: &mut [StereoFrame]| seen.extend(b.iter().map(|f| (f.l, f.r))),
            &inp,
            &mut out,
        );
        assert_eq!(seen, vec![(1.0, 2.0), (3.0, 4.0)]);
    }

    /// Whatever the closure leaves in the block is what libdeluge sends out.
    #[test]
    fn closure_writes_reach_out() {
        let inp = [[1.0, 2.0], [3.0, 4.0]];
        let mut out = [[0.0, 0.0]; 2];
        adapt_block(
            &mut |b: &mut [StereoFrame]| {
                for f in b {
                    f.l *= 10.0;
                    f.r *= 100.0;
                }
            },
            &inp,
            &mut out,
        );
        assert_eq!(out, [[10.0, 200.0], [30.0, 400.0]]);
    }

    /// A no-op closure is bit-exact passthrough — the `audio_passthru` contract.
    #[test]
    fn noop_closure_is_bit_exact_passthrough() {
        let inp = [[0.5, -0.25], [f32::MIN_POSITIVE, -0.0], [1.0, -1.0]];
        let mut out = [[9.9, 9.9]; 3];
        adapt_block(&mut |_: &mut [StereoFrame]| {}, &inp, &mut out);
        assert_eq!(out, inp);
        // -0.0 must survive as -0.0, not collapse to 0.0.
        assert!(out[1][1].is_sign_negative());
    }

    /// Degenerate but legal: no frames, no work, no panic.
    #[test]
    fn empty_block_is_a_noop() {
        let inp: [[f32; 2]; 0] = [];
        let mut out: [[f32; 2]; 0] = [];
        let mut called = false;
        adapt_block(
            &mut |b: &mut [StereoFrame]| {
                called = true;
                assert!(b.is_empty());
            },
            &inp,
            &mut out,
        );
        assert!(called);
    }

    /// The transmute in `adapt_block` is sound only under layout equality.
    #[test]
    fn stereoframe_is_layout_compatible_with_f32_pair() {
        assert_eq!(
            core::mem::size_of::<StereoFrame>(),
            core::mem::size_of::<[f32; 2]>()
        );
        assert_eq!(
            core::mem::align_of::<StereoFrame>(),
            core::mem::align_of::<[f32; 2]>()
        );
    }
}
