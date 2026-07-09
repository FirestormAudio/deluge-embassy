//! The graph node: a uniform value wrapping one `deluge-dsp-kernels` struct plus
//! its input slots and its output-slot base. Dispatch is a `match` on `Kind`
//! (static, closed set). The engine resolves each input into an `In` and hands
//! the node an `OutView` to write its ports through; a `Custom(dyn Ugen)` escape
//! hatch is reserved for a future open set (not built in P0).

use crate::Input;
use deluge_dsp_kernels::{
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
    Wavetable,
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
            Kind::Mul | Kind::Add | Kind::Sub | Kind::Split2 => State::Stateless,
            Kind::Wavetable => State::Wt(WtOsc::new()),
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
            Kind::Split2 => 2,
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
        pool_region: Option<&[f32]>,
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
            Kind::Wavetable => {
                // Unbound table, an invalid static id, a missing pool region,
                // or a too-short pool region leaves the output untouched
                // (silence for a freshly-zeroed arena slot) — never panic.
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
}
