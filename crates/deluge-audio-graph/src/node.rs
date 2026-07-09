//! The graph node: a uniform value wrapping one `deluge-dsp-kernels` struct plus
//! its input slots and its output-slot base. Dispatch is a `match` on `Kind`
//! (static, closed set). The engine resolves each input into an `In` and hands
//! the node an `OutView` to write its ports through; a `Custom(dyn Ugen)` escape
//! hatch is reserved for a future open set (not built in P0).

use crate::Input;
use deluge_dsp_kernels::{
    delay::{Delay, ModDelay},
    env::Ar, filter::OnePole, filter::{Modal, Moog, Ms20, Ms20Resp, Svf, SvfResp, Tb303, MODAL_MODES}, math,
    noise::Noise, noise::NoiseColor, osc::Osc, osc::SyncOsc, osc::Wave,
};
use deluge_dsp_kernels::wavetable::{
    level_len, level_offset, static_table_flat, MipSet, TableId, WtOsc, COMPACT_LEN, LEVELS,
};
pub use deluge_dsp_kernels::In;

/// Assemble a `MipSet`'s level-slice array from a flat, compact
/// (`COMPACT_LEN`-long) pyramid region via the kernel's `level_offset`/
/// `level_len` layout. Shared by the static and pooled `Kind::Wavetable`
/// arms below so both read the same per-level slicing.
fn compact_levels(region: &[f32]) -> [&[f32]; LEVELS] {
    core::array::from_fn(|l| &region[level_offset(l)..level_offset(l) + level_len(l)])
}

pub const MAX_INPUTS: usize = 3;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    Sine,
    Saw,
    Square,
    Tri,
    SyncSine,
    SyncSaw,
    SyncSquare,
    SyncTri,
    Noise,
    PinkNoise,
    BrownNoise,
    Env,
    Lpf,
    SvfLp,
    SvfHp,
    SvfBp,
    SvfNotch,
    Tb303,
    MoogLp4,
    MoogLp2,
    Ms20Lp,
    Ms20Hp,
    Modal,
    Mul,
    Add,
    Sub,
    Split2, // width-2 test node: input → both ports
    Pan,    // mono→stereo: port0 = L, port1 = R (constant-power)
    Wavetable,
    Delay,
    Chorus,
    Flanger,
}

/// Per-kind DSP state. Only the active variant's kernel is used.
#[derive(Clone, Copy)]
enum State {
    Osc(Osc),
    Sync(SyncOsc),
    Noise(Noise),
    Ar(Ar),
    OnePole(OnePole),
    Svf(Svf),
    Tb303(Tb303),
    Moog4(Moog<4>),
    Moog2(Moog<2>),
    Ms20(Ms20),
    Modal(Modal<MODAL_MODES>),
    Wt(WtOsc),
    Delay(Delay),
    Chorus(ModDelay<3>),
    Flanger(ModDelay<1>),
    Stateless,
}

/// Graph-local binding of a wavetable node's table source. The kernel only
/// knows about `MipSet`; this type names *where* a node's mipset comes from
/// so the graph can resolve it at render time. `Pooled` names an Engine-owned
/// pool region (type-erased flat `[f32]`); the engine resolves it into a
/// `MipSet` at render time (see `Node::process_resolved`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum TableSrc {
    Static(TableId),
    Pooled(crate::pool::PoolHandle),
}

#[derive(Clone, Copy)]
pub struct Node {
    pub(crate) kind: Kind,
    // The node's port base in the engine's output arena; read by
    // `Engine::render_block` to place/resolve this node's ports.
    pub(crate) out_base: u16,
    inputs: [Input; MAX_INPUTS],
    state: State,
    table: Option<TableSrc>,
}

impl Node {
    pub fn new(kind: Kind, out_base: u16) -> Node {
        let state = match kind {
            Kind::Sine | Kind::Saw | Kind::Square | Kind::Tri => State::Osc(Osc::new()),
            Kind::SyncSine | Kind::SyncSaw | Kind::SyncSquare | Kind::SyncTri => {
                State::Sync(SyncOsc::new())
            }
            Kind::Noise => State::Noise(Noise::seeded(0x2545_F491)),
            Kind::PinkNoise => State::Noise(Noise::seeded_color(0x2545_F491, NoiseColor::Pink)),
            Kind::BrownNoise => State::Noise(Noise::seeded_color(0x2545_F491, NoiseColor::Brown)),
            Kind::Env => State::Ar(Ar::new()),
            Kind::Lpf => State::OnePole(OnePole::new()),
            Kind::SvfLp | Kind::SvfHp | Kind::SvfBp | Kind::SvfNotch => State::Svf(Svf::new()),
            Kind::Tb303 => State::Tb303(Tb303::new()),
            Kind::MoogLp4 => State::Moog4(Moog::<4>::new()),
            Kind::MoogLp2 => State::Moog2(Moog::<2>::new()),
            Kind::Ms20Lp | Kind::Ms20Hp => State::Ms20(Ms20::new()),
            Kind::Modal => State::Modal(Modal::<MODAL_MODES>::new()),
            Kind::Mul | Kind::Add | Kind::Sub | Kind::Split2 | Kind::Pan => State::Stateless,
            Kind::Wavetable => State::Wt(WtOsc::new()),
            Kind::Delay => State::Delay(Delay::new()),
            Kind::Chorus => State::Chorus(ModDelay::<3>::new(0.020)),
            Kind::Flanger => State::Flanger(ModDelay::<1>::new(0.002)),
        };
        Node {
            kind,
            out_base,
            inputs: [Input::Const(0.0); MAX_INPUTS],
            state,
            table: None,
        }
    }

    pub fn out_width(kind: Kind) -> usize {
        match kind {
            Kind::Split2 | Kind::Pan | Kind::Chorus | Kind::Flanger => 2,
            _ => 1,
        }
    }

    pub fn input_mut(&mut self, port: u8) -> Option<&mut Input> {
        self.inputs.get_mut(port as usize)
    }

    pub fn gate(&mut self, on: bool) {
        if let State::Ar(a) = &mut self.state {
            a.gate(on);
        }
    }

    pub fn trigger(&mut self) {
        if let State::Ar(a) = &mut self.state {
            a.trigger();
        }
    }

    /// Set a non-signal scalar parameter. For oscillators, `param 0` = feedback.
    pub fn set_param(&mut self, param: u8, value: f32) {
        match &mut self.state {
            State::Osc(o) if param == 0 => o.set_feedback(value),
            State::Moog4(m) if param == 0 => m.set_drive(value),
            State::Moog2(m) if param == 0 => m.set_drive(value),
            State::Ms20(m) if param == 0 => m.set_drive(value),
            State::Modal(m) => match param {
                0 => m.set_structure(value),
                1 => m.set_brightness(value),
                2 => m.set_position(value),
                _ => {}
            },
            State::Delay(d) => match param {
                0 => d.set_mix(value),
                1 => d.set_damping(value),
                _ => {}
            },
            State::Chorus(md) => match param {
                0 => md.set_mix(value),
                1 => md.set_rate(value),
                2 => md.set_depth(value),
                3 => md.set_feedback(value),
                _ => {}
            },
            State::Flanger(md) => match param {
                0 => md.set_mix(value),
                1 => md.set_rate(value),
                2 => md.set_depth(value),
                3 => md.set_feedback(value),
                _ => {}
            },
            _ => {}
        }
    }

    pub fn inputs_snapshot(&self) -> [Input; MAX_INPUTS] {
        self.inputs
    }

    /// Bind this wavetable node's table source. No-op for other kinds.
    pub fn bind_table(&mut self, src: TableSrc) {
        self.table = Some(src);
    }

    /// This node's bound table source, if any (used by the engine to resolve
    /// a `Pooled` source's region before render, and to free it on `Cmd::Free`).
    pub fn table_src(&self) -> Option<TableSrc> {
        self.table
    }

    /// Render this node's ports. `ins[p]` is the already-resolved input for port
    /// `p` (the engine resolved every `Input` into an `In` before calling this).
    /// `pool_region` is the flat mip-pyramid region for a `TableSrc::Pooled`
    /// node, resolved by the engine (disjoint borrow of its `pool` field)
    /// before this call; ignored for `Static` and non-wavetable kinds.
    pub fn process_resolved(
        &mut self,
        ins: &[In; MAX_INPUTS],
        dt: f32,
        outs: &mut OutView,
        pool_region: Option<&mut [f32]>,
    ) {
        match self.kind {
            Kind::Sine | Kind::Saw | Kind::Square | Kind::Tri => {
                let wave = match self.kind {
                    Kind::Sine => Wave::Sine,
                    Kind::Saw => Wave::Saw,
                    Kind::Square => Wave::Square,
                    _ => Wave::Tri,
                };
                if let State::Osc(o) = &mut self.state {
                    o.process(wave, ins[0], ins[1], ins[2], dt, outs.port(0));
                }
            }
            Kind::SyncSine | Kind::SyncSaw | Kind::SyncSquare | Kind::SyncTri => {
                let wave = match self.kind {
                    Kind::SyncSine => Wave::Sine,
                    Kind::SyncSaw => Wave::Saw,
                    Kind::SyncSquare => Wave::Square,
                    _ => Wave::Tri,
                };
                if let State::Sync(so) = &mut self.state {
                    so.process(wave, ins[0], ins[1], dt, outs.port(0));
                }
            }
            Kind::Noise | Kind::PinkNoise | Kind::BrownNoise => {
                if let State::Noise(nz) = &mut self.state {
                    nz.process(outs.port(0));
                }
            }
            Kind::Env => {
                if let State::Ar(a) = &mut self.state {
                    a.process(ins[0], ins[1], dt, outs.port(0));
                }
            }
            Kind::Lpf => {
                if let State::OnePole(f) = &mut self.state {
                    f.process(ins[0], ins[1], dt, outs.port(0));
                }
            }
            Kind::SvfLp | Kind::SvfHp | Kind::SvfBp | Kind::SvfNotch => {
                let resp = match self.kind {
                    Kind::SvfLp => SvfResp::Lp,
                    Kind::SvfHp => SvfResp::Hp,
                    Kind::SvfBp => SvfResp::Bp,
                    _ => SvfResp::Notch,
                };
                if let State::Svf(f) = &mut self.state {
                    f.process(ins[0], ins[1], ins[2], resp, dt, outs.port(0));
                }
            }
            Kind::Tb303 => {
                if let State::Tb303(f) = &mut self.state {
                    f.process(ins[0], ins[1], ins[2], dt, outs.port(0));
                }
            }
            Kind::MoogLp4 => {
                if let State::Moog4(m) = &mut self.state {
                    m.process(ins[0], ins[1], ins[2], dt, outs.port(0));
                }
            }
            Kind::MoogLp2 => {
                if let State::Moog2(m) = &mut self.state {
                    m.process(ins[0], ins[1], ins[2], dt, outs.port(0));
                }
            }
            Kind::Ms20Lp | Kind::Ms20Hp => {
                let resp = if matches!(self.kind, Kind::Ms20Hp) { Ms20Resp::Hp } else { Ms20Resp::Lp };
                if let State::Ms20(f) = &mut self.state {
                    f.process(ins[0], ins[1], ins[2], resp, dt, outs.port(0));
                }
            }
            Kind::Modal => {
                if let State::Modal(m) = &mut self.state {
                    m.process(ins[0], ins[1], ins[2], dt, outs.port(0));
                }
            }
            Kind::Mul => math::mul(ins[0], ins[1], outs.port(0)),
            Kind::Add => math::add(ins[0], ins[1], outs.port(0)),
            Kind::Sub => math::sub(ins[0], ins[1], outs.port(0)),
            Kind::Split2 => {
                // Copy the resolved input into both ports.
                for p in 0..2 {
                    let port = outs.port(p);
                    for i in 0..port.len() {
                        port[i] = ins[0].at(i);
                    }
                }
            }
            Kind::Pan => {
                // Mono→stereo constant-power pan. ins[0] = input, ins[1] =
                // position (∈[-1,1], modulatable). port0 = L, port1 = R.
                // Compute both ports' lengths up front, then borrow each.
                let n = outs.port(0).len();
                for i in 0..n {
                    let (gl, gr) = math::pan_gains(ins[1].at(i));
                    let x = ins[0].at(i);
                    outs.port(0)[i] = x * gl;
                    outs.port(1)[i] = x * gr;
                }
            }
            Kind::Wavetable => {
                // Unbound table, an invalid static id, a missing pool region,
                // or a too-short pool region leaves the output untouched
                // (silence for a freshly-zeroed arena slot) — never panic.
                // Wavetable only reads its region; reborrow immutably so the
                // rest of this arm is unchanged by the `&mut` widening.
                let pool_region: Option<&[f32]> = pool_region.as_deref();
                if let State::Wt(o) = &mut self.state {
                    // Resolve the flat compact-pyramid region (static or
                    // pooled), then route by frame count: `FRAMES == 1`
                    // (single-cycle) must stay on the bit-exact `process`
                    // path; `FRAMES > 1` morphs across frames by `position`
                    // (port 2) via `process_morph`.
                    let region: Option<&[f32]> = match self.table {
                        Some(TableSrc::Static(id)) => static_table_flat(id),
                        Some(TableSrc::Pooled(_)) => {
                            // Require a region that's a whole, non-empty
                            // number of `COMPACT_LEN`-long frames (matching
                            // the kernel's flat compact pyramid layout); a
                            // short/mis-sized/partial region renders silence
                            // rather than risk an out-of-bounds slice or a
                            // bogus per-level length.
                            pool_region.filter(|r| {
                                r.len() >= COMPACT_LEN && r.len() % COMPACT_LEN == 0
                            })
                        }
                        None => None,
                    };
                    if let Some(region) = region {
                        let frames = region.len() / COMPACT_LEN;
                        if frames <= 1 {
                            let levels = compact_levels(region);
                            o.process(MipSet { levels: &levels }, ins[0], ins[1], dt, outs.port(0));
                        } else {
                            o.process_morph(region, frames, ins[0], ins[1], ins[2], dt, outs.port(0));
                        }
                    }
                }
            }
            Kind::Delay => {
                // Bound ring buffer (≥4 samples) → run the effect; otherwise
                // dry passthrough (out = input), never panic.
                let ran = if let Some(buf) = pool_region {
                    if buf.len() >= 4 {
                        if let State::Delay(d) = &mut self.state {
                            d.process(ins[0], ins[1], ins[2], dt, buf, outs.port(0));
                            true
                        } else {
                            false
                        }
                    } else {
                        false
                    }
                } else {
                    false
                };
                if !ran {
                    let port = outs.port(0);
                    for i in 0..port.len() {
                        port[i] = ins[0].at(i);
                    }
                }
            }
            Kind::Chorus | Kind::Flanger => {
                // Mono→stereo modulated delay. Bound ring (≥4) → run the
                // effect writing both ports; otherwise dry passthrough to both.
                let (out_l, out_r) = outs.port_pair();
                let ran = if let Some(buf) = pool_region {
                    if buf.len() >= 4 {
                        match &mut self.state {
                            State::Chorus(md) => { md.process(ins[0], dt, buf, out_l, out_r); true }
                            State::Flanger(md) => { md.process(ins[0], dt, buf, out_l, out_r); true }
                            _ => false,
                        }
                    } else {
                        false
                    }
                } else {
                    false
                };
                if !ran {
                    for i in 0..out_l.len() {
                        let x = ins[0].at(i);
                        out_l[i] = x;
                        out_r[i] = x;
                    }
                }
            }
        }
    }
}

/// Largest block size the engine will ask a node to render. The engine's const
/// `BLOCK` must be `<= MAX_BLOCK` (asserted in `Engine::new`).
pub const MAX_BLOCK: usize = 128;

// ── Output view a node writes its ports through ──────────────────────────────

/// A node's writable output ports (1 or 2 in P0). Built either from hand buffers
/// (unit tests) or from the engine's output arena (`from_arena`).
pub struct OutView<'a> {
    ports: [Option<&'a mut [f32]>; 2],
    width: usize,
}

impl<'a> OutView<'a> {
    pub fn single(p0: &'a mut [f32]) -> OutView<'a> {
        OutView { ports: [Some(p0), None], width: 1 }
    }
    pub fn pair(p0: &'a mut [f32], p1: &'a mut [f32]) -> OutView<'a> {
        OutView { ports: [Some(p0), Some(p1)], width: 2 }
    }

    /// Build the (1 or 2) port slices for the node owning `[base, base+width)` in
    /// the engine's output arena. Uses `split_at_mut` so the two ports are
    /// disjoint `&mut` slices with no unsafe.
    pub fn from_arena<const OUTS: usize, const BLOCK: usize>(
        arr: &'a mut [[f32; BLOCK]; OUTS],
        base: usize,
        width: usize,
    ) -> OutView<'a> {
        let (_, rest) = arr.split_at_mut(base);
        if width == 2 {
            let (a, b) = rest.split_at_mut(1);
            OutView { ports: [Some(&mut a[0][..]), Some(&mut b[0][..])], width: 2 }
        } else {
            OutView { ports: [Some(&mut rest[0][..]), None], width: 1 }
        }
    }

    pub fn width(&self) -> usize {
        self.width
    }
    pub fn port(&mut self, p: usize) -> &mut [f32] {
        self.ports[p].as_deref_mut().expect("port index in range")
    }

    /// Both output ports as disjoint mutable slices (for width-2 stereo nodes).
    /// Panics if this view has fewer than 2 ports.
    pub fn port_pair(&mut self) -> (&mut [f32], &mut [f32]) {
        let [a, b] = &mut self.ports;
        (
            a.as_deref_mut().expect("port 0 present"),
            b.as_deref_mut().expect("port 1 present"),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Input;
    use deluge_dsp_kernels::In;

    // Build a resolved-inputs array where every input is the given block of a
    // constant. For unit tests we only need constants.
    fn consts(a: f32, b: f32) -> ([f32; 4], [f32; 4]) {
        ([a; 4], [b; 4])
    }

    #[test]
    fn saw_node_writes_one_port() {
        let mut n = Node::new(Kind::Saw, 0);
        *n.input_mut(0).unwrap() = Input::Const(4.0); // freq
        assert_eq!(Node::out_width(Kind::Saw), 1);

        let (freq, zero) = consts(4.0, 0.0);
        let ins = [In::A(&freq), In::A(&zero), In::A(&zero)];
        let mut buf = [0.0f32; 4];
        {
            let mut outs = OutView::single(&mut buf);
            n.process_resolved(&ins, 1.0 / 16.0, &mut outs, None);
        }
        // Kernel-agnostic on purpose: this test verifies the node dispatched
        // to Kind::Saw and wrote its single output port, not the
        // oscillator's exact samples (naive vs. band-limited kernel shape
        // is covered by deluge-dsp-kernels).
        assert!(buf.iter().all(|s| s.is_finite() && *s >= -1.1 && *s <= 1.1));
        assert!(buf.iter().any(|&s| s != 0.0));
    }

    #[test]
    fn svf_lp_node_renders_bounded_nonsilent() {
        let mut n = Node::new(Kind::SvfLp, 0);
        assert_eq!(Node::out_width(Kind::SvfLp), 1);
        // process_resolved takes a resolved [In; MAX_INPUTS] array directly —
        // build it here (input signal, cutoff, res); do NOT use input_mut.
        let input = [0.7f32; 8];
        let cutoff = [1_000.0f32; 8];
        let res = [0.5f32; 8];
        let ins = [In::A(&input), In::A(&cutoff), In::A(&res)];
        let mut buf = [0.0f32; 8];
        {
            let mut outs = OutView::single(&mut buf);
            n.process_resolved(&ins, 1.0 / 48_000.0, &mut outs, None);
        }
        assert!(buf.iter().all(|s| s.is_finite() && s.abs() <= 4.0));
        assert!(buf.iter().any(|&s| s != 0.0)); // LP of a DC step responds (non-silent)
    }

    #[test]
    fn tb303_node_renders_bounded_nonsilent() {
        let mut n = Node::new(Kind::Tb303, 0);
        assert_eq!(Node::out_width(Kind::Tb303), 1);
        let input = [0.6f32; 16];
        let cutoff = [800.0f32; 16];
        let res = [0.7f32; 16];
        let ins = [In::A(&input), In::A(&cutoff), In::A(&res)];
        let mut buf = [0.0f32; 16];
        {
            let mut outs = OutView::single(&mut buf);
            n.process_resolved(&ins, 1.0 / 48_000.0, &mut outs, None);
        }
        assert!(buf.iter().all(|s| s.is_finite() && s.abs() <= 8.0)); // TB-303 bound is ±8 (resonance loop), not ±1
        assert!(buf.iter().any(|&s| s != 0.0));
    }

    #[test]
    fn moog_nodes_render_bounded_nonsilent() {
        for kind in [Kind::MoogLp4, Kind::MoogLp2] {
            let mut n = Node::new(kind, 0);
            assert_eq!(Node::out_width(kind), 1);
            let input = [0.6f32; 16];
            let cutoff = [1_000.0f32; 16];
            let res = [0.7f32; 16];
            let ins = [In::A(&input), In::A(&cutoff), In::A(&res)];
            let mut buf = [0.0f32; 16];
            {
                let mut outs = OutView::single(&mut buf);
                n.process_resolved(&ins, 1.0 / 48_000.0, &mut outs, None);
            }
            assert!(buf.iter().all(|s| s.is_finite() && s.abs() <= 8.0), "kind={kind:?}");
            assert!(buf.iter().any(|&s| s != 0.0), "kind={kind:?}");
        }
    }

    #[test]
    fn ms20_nodes_render_bounded_nonsilent() {
        for kind in [Kind::Ms20Lp, Kind::Ms20Hp] {
            let mut n = Node::new(kind, 0);
            assert_eq!(Node::out_width(kind), 1);
            let input = [0.6f32; 16];
            let cutoff = [1_000.0f32; 16];
            let res = [0.7f32; 16];
            let ins = [In::A(&input), In::A(&cutoff), In::A(&res)];
            let mut buf = [0.0f32; 16];
            {
                let mut outs = OutView::single(&mut buf);
                n.process_resolved(&ins, 1.0 / 48_000.0, &mut outs, None);
            }
            assert!(buf.iter().all(|s| s.is_finite() && s.abs() <= 8.0), "kind={kind:?}");
            assert!(buf.iter().any(|&s| s != 0.0), "kind={kind:?}");
        }
    }

    #[test]
    fn moog_drive_param_changes_output() {
        let mk = |drive: Option<f32>| {
            let mut n = Node::new(Kind::MoogLp4, 0);
            if let Some(d) = drive {
                n.set_param(0, d);
            }
            let input = [0.8f32; 32];
            let (c, r) = ([2_000.0f32; 32], [0.5f32; 32]);
            let ins = [In::A(&input), In::A(&c), In::A(&r)];
            let mut buf = [0.0f32; 32];
            {
                let mut outs = OutView::single(&mut buf);
                n.process_resolved(&ins, 1.0 / 48_000.0, &mut outs, None);
            }
            buf
        };
        assert!(mk(None) != mk(Some(4.0)), "drive param should change output");
    }

    #[test]
    fn split2_writes_both_ports_identically() {
        let mut n = Node::new(Kind::Split2, 0);
        assert_eq!(Node::out_width(Kind::Split2), 2);

        let (x, zero) = consts(0.75, 0.0);
        let ins = [In::A(&x), In::A(&zero), In::A(&zero)];
        let mut p0 = [0.0f32; 4];
        let mut p1 = [0.0f32; 4];
        {
            let mut outs = OutView::pair(&mut p0, &mut p1);
            n.process_resolved(&ins, 1.0, &mut outs, None);
        }
        assert_eq!(p0, [0.75; 4]);
        assert_eq!(p1, [0.75; 4]); // both ports carry the same input
    }

    #[test]
    fn modal_node_rings_bounded() {
        let mut n = Node::new(Kind::Modal, 0);
        assert_eq!(Node::out_width(Kind::Modal), 1);
        n.set_param(1, 0.9); // brightness
        let mut input = [0.0f32; 64];
        input[0] = 1.0; // strike
        let freq = [220.0f32; 64];
        let damping = [0.2f32; 64];
        let ins = [In::A(&input), In::A(&freq), In::A(&damping)];
        let mut buf = [0.0f32; 64];
        {
            let mut outs = OutView::single(&mut buf);
            n.process_resolved(&ins, 1.0 / 48_000.0, &mut outs, None);
        }
        assert!(buf.iter().all(|s| s.is_finite() && s.abs() <= 8.0));
        assert!(buf.iter().any(|&s| s != 0.0), "should ring");
    }

    #[test]
    fn delay_node_with_buffer_delays_and_passes_through() {
        // Bound buffer → the node runs the Delay kernel; no buffer → dry.
        let mut n = Node::new(Kind::Delay, 0);
        assert_eq!(Node::out_width(Kind::Delay), 1);
        // input impulse, time = 4 samples @ dt below, feedback 0.
        let dt = 1.0 / 48_000.0;
        let mut input = [0.0f32; 32];
        input[0] = 1.0;
        let time = [4.0 * dt; 32];
        let fb = [0.0f32; 32];
        let ins = [In::A(&input), In::A(&time), In::A(&fb)];
        n.set_param(0, 1.0); // mix = fully wet
        let mut ring = [0.0f32; 256];
        let mut buf = [0.0f32; 32];
        {
            let mut outs = OutView::single(&mut buf);
            n.process_resolved(&ins, dt, &mut outs, Some(&mut ring));
        }
        assert!(buf.iter().all(|s| s.is_finite() && s.abs() <= 2.0));
        // Impulse shows up around sample 4, not at 0.
        assert!(buf[4].abs() > 0.5, "delayed impulse missing: {:?}", &buf[..8]);
        assert!(buf[0].abs() < 1e-3, "wet output should be silent at t0");
    }

    #[test]
    fn delay_node_without_buffer_is_dry_passthrough() {
        let mut n = Node::new(Kind::Delay, 0);
        let dt = 1.0 / 48_000.0;
        let input = [0.6f32; 16];
        let time = [0.01f32; 16];
        let fb = [0.5f32; 16];
        let ins = [In::A(&input), In::A(&time), In::A(&fb)];
        let mut buf = [0.0f32; 16];
        {
            let mut outs = OutView::single(&mut buf);
            n.process_resolved(&ins, dt, &mut outs, None); // no ring buffer
        }
        // No buffer bound → dry passthrough (out == input), never panic.
        assert!(buf.iter().all(|&s| (s - 0.6).abs() < 1e-6), "expected dry: {buf:?}");
    }

    #[test]
    fn pan_node_writes_l_r_constant_power() {
        let mut n = Node::new(Kind::Pan, 0);
        assert_eq!(Node::out_width(Kind::Pan), 2);
        let x = [0.8f32; 8];
        // hard left
        let posl = [-1.0f32; 8];
        let ins = [In::A(&x), In::A(&posl), In::A(&[0.0; 8])];
        let mut p0 = [0.0f32; 8];
        let mut p1 = [0.0f32; 8];
        {
            let mut outs = OutView::pair(&mut p0, &mut p1);
            n.process_resolved(&ins, 1.0 / 48_000.0, &mut outs, None);
        }
        assert!((p0[0] - 0.8).abs() < 1e-5, "L port = input at hard-left: {}", p0[0]);
        assert!(p1[0].abs() < 1e-5, "R port silent at hard-left: {}", p1[0]);

        // center → both ≈ 0.8 * 0.7071, constant power
        let posc = [0.0f32; 8];
        let ins = [In::A(&x), In::A(&posc), In::A(&[0.0; 8])];
        let mut p0 = [0.0f32; 8];
        let mut p1 = [0.0f32; 8];
        {
            let mut outs = OutView::pair(&mut p0, &mut p1);
            n.process_resolved(&ins, 1.0 / 48_000.0, &mut outs, None);
        }
        let c = 0.8 * (0.5f32).sqrt();
        assert!((p0[0] - c).abs() < 1e-5 && (p1[0] - c).abs() < 1e-5);
        // total power preserved
        assert!((p0[0] * p0[0] + p1[0] * p1[0] - 0.8 * 0.8).abs() < 1e-4);
    }

    #[test]
    fn pan_node_position_modulates() {
        // A per-sample position sweep moves energy from L to R.
        let mut n = Node::new(Kind::Pan, 0);
        let x = [1.0f32; 4];
        let pos = [-1.0f32, -0.3, 0.3, 1.0]; // sweep L→R
        let ins = [In::A(&x), In::A(&pos), In::A(&[0.0; 4])];
        let mut p0 = [0.0f32; 4];
        let mut p1 = [0.0f32; 4];
        {
            let mut outs = OutView::pair(&mut p0, &mut p1);
            n.process_resolved(&ins, 1.0 / 48_000.0, &mut outs, None);
        }
        // L decreasing, R increasing across the block
        assert!(p0[0] > p0[3], "L should fall L→R: {:?}", p0);
        assert!(p1[0] < p1[3], "R should rise L→R: {:?}", p1);
    }

    #[test]
    fn chorus_node_renders_stereo_with_buffer() {
        let mut n = Node::new(Kind::Chorus, 0);
        assert_eq!(Node::out_width(Kind::Chorus), 2);
        n.set_param(0, 1.0); // mix = wet
        n.set_param(1, 2.0); // rate
        n.set_param(2, 0.6); // depth
        let dt = 1.0 / 48_000.0;
        let input: [f32; 64] = core::array::from_fn(|i| (i as f32 * 0.1).sin());
        let ins = [In::A(&input), In::A(&[0.0; 64]), In::A(&[0.0; 64])];
        let mut ring = [0.0f32; 4096];
        let mut p0 = [0.0f32; 64];
        let mut p1 = [0.0f32; 64];
        {
            let mut outs = OutView::pair(&mut p0, &mut p1);
            n.process_resolved(&ins, dt, &mut outs, Some(&mut ring));
        }
        assert!(p0.iter().all(|s| s.is_finite() && s.abs() <= 8.0));
        assert!(p1.iter().all(|s| s.is_finite() && s.abs() <= 8.0));
    }

    #[test]
    fn chorus_node_without_buffer_is_dry_both_ports() {
        let mut n = Node::new(Kind::Flanger, 0);
        assert_eq!(Node::out_width(Kind::Flanger), 2);
        let input = [0.4f32; 16];
        let ins = [In::A(&input), In::A(&[0.0; 16]), In::A(&[0.0; 16])];
        let mut p0 = [0.0f32; 16];
        let mut p1 = [0.0f32; 16];
        {
            let mut outs = OutView::pair(&mut p0, &mut p1);
            n.process_resolved(&ins, 1.0 / 48_000.0, &mut outs, None); // no ring
        }
        assert!(p0.iter().all(|&s| (s - 0.4).abs() < 1e-6), "L dry: {p0:?}");
        assert!(p1.iter().all(|&s| (s - 0.4).abs() < 1e-6), "R dry: {p1:?}");
    }
}
