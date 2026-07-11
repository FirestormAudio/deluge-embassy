//! The graph node: a uniform value wrapping one `deluge-dsp-kernels` struct plus
//! its input slots and its output-slot base. Dispatch is a `match` on `Kind`
//! (static, closed set). The engine resolves each input into an `In` and hands
//! the node an `OutView` to write its ports through; a `Custom(dyn Ugen)` escape
//! hatch is reserved for a future open set (not built in P0).

use crate::Input;
use deluge_dsp_kernels::{
    delay::{Delay, ModDelay},
    drive::{Drive, Shape},
    dynamics::{Comp, Detector},
    env::{Adsr, Ar}, eq::{Eq, EqType}, filter::OnePole, filter::{Modal, Moog, Ms20, Ms20Resp, Svf, SvfResp, Tb303, MODAL_MODES}, lfo::Lfo, math,
    modutil::{SampleHold, Slew, Steps},
    noise::Noise, noise::NoiseColor, osc::Osc, osc::SyncOsc, osc::Wave,
    poly::{poly_add, poly_mul, voice_sum, voice_sum_stereo, PolyAdsr, PolyAr, PolyCtrl, PolyMoog, PolyMs20, PolyMtof, PolyNoise, PolyOsc, PolySlew, PolySvf, PolySync, PolyWt, VOICES},
    quant::{Mtof, QuantPitch, QuantStep},
    reverb::{Dattorro, Fdn8, Freeverb, HALL_BUF_SAMPLES, PLATE_BUF_SAMPLES, REVERB_BUF_SAMPLES},
    shape::{self, Ctrl},
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
    Adsr,
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
    Room,
    Hall,
    Plate,
    Drive,
    Comp,
    Eq,
    Lfo,
    SampleHold,
    Slew,
    Steps,
    Curve,
    QuantStep,
    QuantPitch,
    Mtof,
    Ctrl,
    PolyCtrl,
    PolyOsc,
    VoiceSum,
    /// Stereo voice sum: VOICES lanes → L/R (port0=L, port1=R) with per-lane
    /// unity-center pan + 1/√U gain. The Synth's stereo-spread collapse node.
    StereoVoiceSum,
    PolyAr,
    PolyAdsr,
    PolySvf,
    PolySlew,
    PolyMul,
    PolyMtof,
    PolyAdd,
    PolyNoise,
    PolyPink,
    PolyBrown,
    PolyMoogLp4,
    PolyMoogLp2,
    PolyMs20Lp,
    PolyMs20Hp,
    PolySyncSine,
    PolySyncSaw,
    PolySyncSquare,
    PolySyncTri,
    PolyWt,
    PolyWtMorph,
}

/// Per-kind DSP state. Only the active variant's kernel is used.
#[derive(Clone, Copy)]
enum State {
    Osc(Osc),
    Sync(SyncOsc),
    Noise(Noise),
    Ar(Ar),
    Adsr(Adsr),
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
    Room(Freeverb),
    Hall(Fdn8),
    Plate(Dattorro),
    Drive(Drive),
    Comp(Comp),
    Eq(Eq),
    Lfo(Lfo),
    SampleHold(SampleHold),
    Slew(Slew),
    Steps(Steps),
    QuantStep(QuantStep),
    QuantPitch(QuantPitch),
    Mtof(Mtof),
    Ctrl(Ctrl),
    PolyCtrl(PolyCtrl),
    PolyOsc(PolyOsc),
    PolyAr(PolyAr),
    PolyAdsr(PolyAdsr),
    PolySvf(PolySvf),
    PolySlew(PolySlew),
    PolyMtof(PolyMtof),
    PolyNoise(PolyNoise),
    PolyMoog4(PolyMoog<4>),
    PolyMoog2(PolyMoog<2>),
    PolyMs20(PolyMs20),
    PolySync(PolySync),
    PolyWt(PolyWt),
    VoiceSum(f32),
    StereoVoiceSum { gain: f32, pan: [f32; VOICES] },
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
            Kind::Adsr => State::Adsr(Adsr::new()),
            Kind::Lpf => State::OnePole(OnePole::new()),
            Kind::SvfLp | Kind::SvfHp | Kind::SvfBp | Kind::SvfNotch => State::Svf(Svf::new()),
            Kind::Tb303 => State::Tb303(Tb303::new()),
            Kind::MoogLp4 => State::Moog4(Moog::<4>::new()),
            Kind::MoogLp2 => State::Moog2(Moog::<2>::new()),
            Kind::Ms20Lp | Kind::Ms20Hp => State::Ms20(Ms20::new()),
            Kind::Modal => State::Modal(Modal::<MODAL_MODES>::new()),
            Kind::Mul | Kind::Add | Kind::Sub | Kind::Split2 | Kind::Pan | Kind::Curve | Kind::PolyMul | Kind::PolyAdd => State::Stateless,
            Kind::VoiceSum => State::VoiceSum(1.0),
            Kind::StereoVoiceSum => State::StereoVoiceSum { gain: 1.0, pan: [0.0; VOICES] },
            Kind::Wavetable => State::Wt(WtOsc::new()),
            Kind::Delay => State::Delay(Delay::new()),
            Kind::Chorus => State::Chorus(ModDelay::<3>::new(0.020)),
            Kind::Flanger => State::Flanger(ModDelay::<1>::new(0.002)),
            Kind::Room => State::Room(Freeverb::new()),
            Kind::Hall => State::Hall(Fdn8::new()),
            Kind::Plate => State::Plate(Dattorro::new()),
            Kind::Drive => State::Drive(Drive::new(Shape::Soft)),
            Kind::Comp => State::Comp(Comp::new(-20.0, 4.0, 0.01, 0.1, 6.0, 0.0, Detector::Rms)),
            Kind::Eq => State::Eq(Eq::new(EqType::Peak)),
            Kind::Lfo => State::Lfo(Lfo::new()),
            Kind::SampleHold => State::SampleHold(SampleHold::new()),
            Kind::Slew => State::Slew(Slew::new()),
            Kind::Steps => State::Steps(Steps::new()),
            Kind::QuantStep => State::QuantStep(QuantStep::new()),
            Kind::QuantPitch => State::QuantPitch(QuantPitch::new()),
            Kind::Mtof => State::Mtof(Mtof::new()),
            Kind::Ctrl => State::Ctrl(Ctrl::new()),
            Kind::PolyCtrl => State::PolyCtrl(PolyCtrl::new()),
            Kind::PolyOsc => State::PolyOsc(PolyOsc::new()),
            Kind::PolyAr => State::PolyAr(PolyAr::new()),
            Kind::PolyAdsr => State::PolyAdsr(PolyAdsr::new()),
            Kind::PolySvf => State::PolySvf(PolySvf::new()),
            Kind::PolySlew => State::PolySlew(PolySlew::new()),
            Kind::PolyMtof => State::PolyMtof(PolyMtof::new()),
            Kind::PolyNoise => State::PolyNoise(PolyNoise::new()),
            Kind::PolyPink => State::PolyNoise(PolyNoise::new_color(NoiseColor::Pink)),
            Kind::PolyBrown => State::PolyNoise(PolyNoise::new_color(NoiseColor::Brown)),
            Kind::PolyMoogLp4 => State::PolyMoog4(PolyMoog::<4>::new()),
            Kind::PolyMoogLp2 => State::PolyMoog2(PolyMoog::<2>::new()),
            Kind::PolyMs20Lp | Kind::PolyMs20Hp => State::PolyMs20(PolyMs20::new()),
            Kind::PolySyncSine | Kind::PolySyncSaw | Kind::PolySyncSquare | Kind::PolySyncTri => {
                State::PolySync(PolySync::new())
            }
            Kind::PolyWt | Kind::PolyWtMorph => State::PolyWt(PolyWt::new()),
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
            Kind::Split2 | Kind::Pan | Kind::Chorus | Kind::Flanger | Kind::Room | Kind::Hall | Kind::Plate | Kind::StereoVoiceSum => 2,
            Kind::PolyCtrl | Kind::PolyOsc | Kind::PolyAr | Kind::PolyAdsr | Kind::PolySvf | Kind::PolySlew | Kind::PolyMul
                | Kind::PolyMtof | Kind::PolyAdd | Kind::PolyNoise | Kind::PolyPink | Kind::PolyBrown
                | Kind::PolyMoogLp4 | Kind::PolyMoogLp2 | Kind::PolyMs20Lp | Kind::PolyMs20Hp
                | Kind::PolySyncSine | Kind::PolySyncSaw | Kind::PolySyncSquare | Kind::PolySyncTri
                | Kind::PolyWt | Kind::PolyWtMorph => VOICES,
            _ => 1,
        }
    }

    /// A poly node carries `VOICES` voice-lanes and is dispatched via
    /// `poly_process`, not `process_resolved`.
    pub fn is_poly(kind: Kind) -> bool {
        matches!(kind, Kind::PolyCtrl | Kind::PolyOsc | Kind::VoiceSum | Kind::StereoVoiceSum | Kind::PolyAr | Kind::PolyAdsr | Kind::PolySvf | Kind::PolySlew | Kind::PolyMul | Kind::PolyMtof | Kind::PolyAdd | Kind::PolyNoise | Kind::PolyPink | Kind::PolyBrown
            | Kind::PolyMoogLp4 | Kind::PolyMoogLp2 | Kind::PolyMs20Lp | Kind::PolyMs20Hp
            | Kind::PolySyncSine | Kind::PolySyncSaw | Kind::PolySyncSquare | Kind::PolySyncTri
            | Kind::PolyWt | Kind::PolyWtMorph)
    }

    /// Number of leading input ports that are poly edges (the rest are mono
    /// controls). Generalizes the Sy-1 single-poly-input model.
    pub fn poly_in_count(kind: Kind) -> usize {
        match kind {
            Kind::PolySvf | Kind::PolySlew | Kind::VoiceSum | Kind::StereoVoiceSum | Kind::PolyMtof
                | Kind::PolyMoogLp4 | Kind::PolyMoogLp2 | Kind::PolyMs20Lp | Kind::PolyMs20Hp
                | Kind::PolyWt | Kind::PolyWtMorph => 1,
            // PolyOsc: port 0 = pitch, port 1 = PWM width (unconnected ⇒
            // Const(0.0) ⇒ all-zero tile ⇒ 0.5 duty, bit-identical to the
            // pre-width PolyOsc). PolySync: port 0 = master, port 1 = slave.
            // PolyWt/PolyWtMorph: port 0 = pitch (poly edge); pmod (port 1)
            // and morph position (port 2) are trailing mono controls, matching
            // mono `Kind::Wavetable`'s port order.
            Kind::PolyOsc | Kind::PolyMul | Kind::PolyAdd
                | Kind::PolySyncSine | Kind::PolySyncSaw | Kind::PolySyncSquare | Kind::PolySyncTri => 2,
            _ => 0, // PolyCtrl, PolyAr, PolyNoise/PolyPink/PolyBrown, and all mono kinds
        }
    }

    pub fn input_mut(&mut self, port: u8) -> Option<&mut Input> {
        self.inputs.get_mut(port as usize)
    }

    pub fn gate(&mut self, on: bool) {
        match &mut self.state {
            State::Ar(a) => a.gate(on),
            State::Adsr(a) => a.gate(on),
            State::Lfo(l) if on => l.retrigger(),
            _ => {}
        }
    }

    pub fn trigger(&mut self) {
        match &mut self.state {
            State::Ar(a) => a.trigger(),
            State::Adsr(a) => a.trigger(),
            State::Lfo(l) => l.retrigger(),
            _ => {}
        }
    }

    pub fn gate_voice(&mut self, v: usize, on: bool) {
        match &mut self.state {
            State::PolyAr(a) => a.gate_voice(v, on),
            State::PolyAdsr(a) => a.gate_voice(v, on),
            _ => {}
        }
    }
    pub fn trigger_voice(&mut self, v: usize) {
        match &mut self.state {
            State::PolyAr(a) => a.trigger_voice(v),
            State::PolyAdsr(a) => a.trigger_voice(v),
            State::PolySlew(s) => s.trigger_voice(v),
            _ => {}
        }
    }

    /// Set a non-signal scalar parameter. For oscillators, `param 0` = feedback.
    pub fn set_param(&mut self, param: u8, value: f32) {
        match &mut self.state {
            State::Osc(o) if param == 0 => o.set_feedback(value),
            State::Adsr(a) if param == 0 => a.set_sustain(value),
            State::PolyAdsr(a) if param == 0 => a.set_sustain(value),
            State::PolySlew(s) if param == 0 => s.set_time(value),
            State::VoiceSum(g) if param == 0 => *g = value,
            State::StereoVoiceSum { gain, .. } if param == 0 => *gain = value,
            State::StereoVoiceSum { pan, .. } if (1..=VOICES).contains(&(param as usize)) => {
                pan[param as usize - 1] = value;
            }
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
            State::Room(fv) => match param {
                0 => fv.set_mix(value),
                1 => fv.set_damp(value),
                2 => fv.set_roomsize(value),
                3 => fv.set_width(value),
                _ => {}
            },
            State::Hall(f) => match param {
                0 => f.set_mix(value),
                1 => f.set_damp(value),
                2 => f.set_size(value),
                3 => f.set_width(value),
                _ => {}
            },
            State::Plate(d) => match param {
                0 => d.set_mix(value),
                1 => d.set_damp(value),
                2 => d.set_size(value),
                3 => d.set_width(value),
                _ => {}
            },
            State::Drive(d) => match param {
                0 => d.set_drive(value),
                1 => d.set_tone(value),
                2 => d.set_mix(value),
                3 => d.set_shape(value as u8),
                _ => {}
            },
            State::Comp(c) => match param {
                0 => c.set_threshold(value),
                1 => c.set_ratio(value),
                2 => c.set_attack(value),
                3 => c.set_release(value),
                4 => c.set_knee(value),
                5 => c.set_makeup(value),
                6 => c.set_detector(if value == 0.0 { Detector::Peak } else { Detector::Rms }),
                _ => {}
            },
            State::Eq(e) => match param {
                0 => e.set_freq(value),
                1 => e.set_gain(value),
                2 => e.set_q(value),
                3 => e.set_type(value as u8),
                _ => {}
            },
            State::Lfo(l) => match param {
                0 => l.set_shape(value as u8),
                1 => l.set_phase(value),
                _ => {}
            },
            State::Steps(s) => match param {
                0 => s.set_len(value as u8),
                k => s.set_value(k as usize, value),
            },
            State::QuantStep(q) => match param {
                0 => q.set_levels(value as u16),
                _ => {}
            },
            State::QuantPitch(q) => match param {
                0 => q.set_mask(value as u16),
                1 => q.set_root(value as u8),
                _ => {}
            },
            State::Mtof(m) => match param {
                0 => m.set_ref(value),
                _ => {}
            },
            State::Ctrl(c) => match param {
                0 => c.set_value(value),
                _ => {}
            },
            State::PolyCtrl(c) => c.set_voice(param as usize, value),
            State::PolyOsc(o) => match param {
                0 => o.set_shape(value as u8),
                _ => {}
            },
            State::PolyMtof(m) => match param {
                0 => m.set_ref(value),
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
            Kind::Adsr => {
                if let State::Adsr(a) = &mut self.state {
                    a.process(ins[0], ins[1], ins[2], dt, outs.port(0));
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
            Kind::Room => {
                // Mono→stereo reverb. Bound region must hold the full partitioned
                // layout (≥ REVERB_BUF_SAMPLES); otherwise dry passthrough both.
                let (out_l, out_r) = outs.port_pair();
                let ran = if let Some(buf) = pool_region {
                    if buf.len() >= REVERB_BUF_SAMPLES {
                        if let State::Room(fv) = &mut self.state {
                            fv.process(ins[0], dt, buf, out_l, out_r);
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
                    for i in 0..out_l.len() {
                        let x = ins[0].at(i);
                        out_l[i] = x;
                        out_r[i] = x;
                    }
                }
            }
            Kind::Hall => {
                let (out_l, out_r) = outs.port_pair();
                let ran = if let Some(buf) = pool_region {
                    if buf.len() >= HALL_BUF_SAMPLES {
                        if let State::Hall(f) = &mut self.state {
                            f.process(ins[0], dt, buf, out_l, out_r);
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
                    for i in 0..out_l.len() {
                        let x = ins[0].at(i);
                        out_l[i] = x;
                        out_r[i] = x;
                    }
                }
            }
            Kind::Plate => {
                let (out_l, out_r) = outs.port_pair();
                let ran = if let Some(buf) = pool_region {
                    if buf.len() >= PLATE_BUF_SAMPLES {
                        if let State::Plate(d) = &mut self.state {
                            d.process(ins[0], dt, buf, out_l, out_r);
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
                    for i in 0..out_l.len() {
                        let x = ins[0].at(i);
                        out_l[i] = x;
                        out_r[i] = x;
                    }
                }
            }
            Kind::Drive => {
                if let State::Drive(d) = &mut self.state {
                    d.process(ins[0], dt, outs.port(0));
                }
            }
            Kind::Comp => {
                if let State::Comp(c) = &mut self.state {
                    c.process(ins[0], dt, outs.port(0));
                }
            }
            Kind::Eq => {
                if let State::Eq(e) = &mut self.state {
                    e.process(ins[0], dt, outs.port(0));
                }
            }
            Kind::Lfo => {
                if let State::Lfo(l) = &mut self.state {
                    l.process(ins[0], dt, outs.port(0));
                }
            }
            Kind::SampleHold => {
                if let State::SampleHold(sh) = &mut self.state {
                    sh.process(ins[0], ins[1], outs.port(0));
                }
            }
            Kind::Slew => {
                if let State::Slew(s) = &mut self.state {
                    s.process(ins[0], ins[1], dt, outs.port(0));
                }
            }
            Kind::Steps => {
                if let State::Steps(s) = &mut self.state {
                    s.process(ins[0], outs.port(0)); // clock on port 0
                }
            }
            Kind::Curve => shape::curve(ins[0], ins[1], outs.port(0)),
            Kind::QuantStep => {
                if let State::QuantStep(q) = &mut self.state {
                    q.process(ins[0], outs.port(0));
                }
            }
            Kind::QuantPitch => {
                if let State::QuantPitch(q) = &mut self.state {
                    q.process(ins[0], outs.port(0));
                }
            }
            Kind::Mtof => {
                if let State::Mtof(m) = &mut self.state {
                    m.process(ins[0], outs.port(0));
                }
            }
            Kind::Ctrl => {
                if let State::Ctrl(c) = &mut self.state {
                    c.process(outs.port(0));
                }
            }
            Kind::PolyCtrl | Kind::PolyOsc | Kind::VoiceSum | Kind::StereoVoiceSum | Kind::PolyAr | Kind::PolyAdsr | Kind::PolySvf | Kind::PolySlew | Kind::PolyMul | Kind::PolyMtof | Kind::PolyAdd | Kind::PolyNoise | Kind::PolyPink | Kind::PolyBrown
                | Kind::PolyMoogLp4 | Kind::PolyMoogLp2 | Kind::PolyMs20Lp | Kind::PolyMs20Hp
                | Kind::PolySyncSine | Kind::PolySyncSaw | Kind::PolySyncSquare | Kind::PolySyncTri
                | Kind::PolyWt | Kind::PolyWtMorph => {
                // Poly kinds are dispatched via `poly_process`, not this path.
            }
        }
    }

    /// Dispatch a poly node. `ins` = the resolved mono control ports; `poly_in`
    /// = up to two voice-interleaved input tiles (`poly_in[j]` is `Some` for
    /// `j < poly_in_count`). `out` is the writable region (VOICES*BLOCK, or BLOCK
    /// for VoiceSum). `pool_region` mirrors `process_resolved`'s param: the flat
    /// mip-pyramid region for a `TableSrc::Pooled` `PolyWt`/`PolyWtMorph` node,
    /// resolved by the engine before this call; ignored by every other poly kind.
    pub fn poly_process(
        &mut self,
        ins: &[In; MAX_INPUTS],
        poly_in: [Option<&[f32]>; 2],
        dt: f32,
        out: &mut [f32],
        pool_region: Option<&mut [f32]>,
    ) {
        match self.kind {
            Kind::PolyCtrl => {
                if let State::PolyCtrl(c) = &mut self.state { c.process(out); }
            }
            Kind::PolyOsc => {
                // width port optional: unconnected ⇒ Const(0.0) ⇒ engine
                // broadcasts an all-zero tile ⇒ 0.5 duty (backward-compat).
                if let (State::PolyOsc(o), Some(pitch), Some(width)) =
                    (&mut self.state, poly_in[0], poly_in[1]) {
                    o.process(pitch, width, dt, out);
                }
            }
            Kind::VoiceSum => {
                if let (State::VoiceSum(gain), Some(pin)) = (&self.state, poly_in[0]) {
                    voice_sum(pin, out, *gain);
                }
            }
            Kind::StereoVoiceSum => {
                if let (State::StereoVoiceSum { gain, pan }, Some(pin)) = (&self.state, poly_in[0]) {
                    let half = out.len() / 2; // 2*BLOCK ⇒ BLOCK
                    let (l, r) = out.split_at_mut(half); // l = row base (port0=L), r = row base+1 (port1=R)
                    voice_sum_stereo(pin, l, r, *gain, pan);
                }
            }
            Kind::PolyAr => {
                if let State::PolyAr(a) = &mut self.state {
                    a.process(ins[0], ins[1], dt, out);
                }
            }
            Kind::PolyAdsr => {
                if let State::PolyAdsr(a) = &mut self.state {
                    a.process(ins[0], ins[1], ins[2], dt, out);
                }
            }
            Kind::PolySvf => {
                if let (State::PolySvf(s), Some(audio)) = (&mut self.state, poly_in[0]) {
                    s.process(audio, ins[1], ins[2], dt, out);
                }
            }
            Kind::PolySlew => {
                if let (State::PolySlew(s), Some(target)) = (&mut self.state, poly_in[0]) {
                    s.process(target, dt, out);
                }
            }
            Kind::PolyMul => {
                if let (Some(a), Some(b)) = (poly_in[0], poly_in[1]) {
                    poly_mul(a, b, out);
                }
            }
            Kind::PolyMtof => {
                if let (State::PolyMtof(m), Some(pin)) = (&mut self.state, poly_in[0]) {
                    m.process(pin, out);
                }
            }
            Kind::PolyAdd => {
                if let (Some(a), Some(b)) = (poly_in[0], poly_in[1]) {
                    poly_add(a, b, out);
                }
            }
            Kind::PolyNoise | Kind::PolyPink | Kind::PolyBrown => {
                if let State::PolyNoise(nz) = &mut self.state {
                    nz.process(out);
                }
            }
            Kind::PolyMoogLp4 => {
                if let (State::PolyMoog4(m), Some(audio)) = (&mut self.state, poly_in[0]) {
                    m.process(audio, ins[1], ins[2], dt, out);
                }
            }
            Kind::PolyMoogLp2 => {
                if let (State::PolyMoog2(m), Some(audio)) = (&mut self.state, poly_in[0]) {
                    m.process(audio, ins[1], ins[2], dt, out);
                }
            }
            Kind::PolyMs20Lp | Kind::PolyMs20Hp => {
                let resp = if matches!(self.kind, Kind::PolyMs20Hp) { Ms20Resp::Hp } else { Ms20Resp::Lp };
                if let (State::PolyMs20(m), Some(audio)) = (&mut self.state, poly_in[0]) {
                    m.process(audio, ins[1], ins[2], resp, dt, out);
                }
            }
            Kind::PolySyncSine | Kind::PolySyncSaw | Kind::PolySyncSquare | Kind::PolySyncTri => {
                let wave = match self.kind {
                    Kind::PolySyncSine => Wave::Sine,
                    Kind::PolySyncSaw => Wave::Saw,
                    Kind::PolySyncSquare => Wave::Square,
                    _ => Wave::Tri,
                };
                if let (State::PolySync(s), Some(master), Some(slave)) =
                    (&mut self.state, poly_in[0], poly_in[1]) {
                    s.process(master, slave, wave, dt, out);
                }
            }
            Kind::PolyWt | Kind::PolyWtMorph => {
                // Mirrors the mono `Kind::Wavetable` arm's table-region
                // resolution (static vs. pooled), but the frame-count-based
                // single/morph decision there is a per-Kind decision here
                // (PolyWt vs PolyWtMorph), since a poly node's Kind is fixed
                // at creation. `pool_region` is only Some for `TableSrc::Pooled`
                // (resolved by the engine, mirroring `process_resolved`'s param).
                let kind = self.kind;
                let pool_region: Option<&[f32]> = pool_region.as_deref();
                if let (State::PolyWt(w), Some(pitch)) = (&mut self.state, poly_in[0]) {
                    let region: Option<&[f32]> = match self.table {
                        Some(TableSrc::Static(id)) => static_table_flat(id),
                        Some(TableSrc::Pooled(_)) => {
                            pool_region.filter(|r| r.len() >= COMPACT_LEN && r.len() % COMPACT_LEN == 0)
                        }
                        None => None,
                    };
                    if let Some(region) = region {
                        let frames = region.len() / COMPACT_LEN;
                        let n = out.len() / VOICES;
                        let mut col = [0.0f32; MAX_BLOCK];
                        let mut ocol = [0.0f32; MAX_BLOCK];
                        if matches!(kind, Kind::PolyWtMorph) {
                            for v in 0..VOICES {
                                for i in 0..n { col[i] = pitch[i * VOICES + v]; }
                                w.process_voice_morph(v, region, frames, In::A(&col[..n]), ins[1], ins[2], dt, &mut ocol[..n]);
                                for i in 0..n { out[i * VOICES + v] = ocol[i]; }
                            }
                        } else {
                            let levels = compact_levels(region);
                            for v in 0..VOICES {
                                for i in 0..n { col[i] = pitch[i * VOICES + v]; }
                                w.process_voice(v, MipSet { levels: &levels }, In::A(&col[..n]), ins[1], dt, &mut ocol[..n]);
                                for i in 0..n { out[i * VOICES + v] = ocol[i]; }
                            }
                        }
                    }
                }
            }
            _ => {}
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
    extern crate std;
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

    #[test]
    fn room_node_renders_stereo_with_buffer() {
        use deluge_dsp_kernels::reverb::REVERB_BUF_SAMPLES;
        let mut n = Node::new(Kind::Room, 0);
        assert_eq!(Node::out_width(Kind::Room), 2);
        n.set_param(0, 1.0); // mix wet
        n.set_param(2, 0.7); // roomsize
        let input: [f32; 64] = core::array::from_fn(|i| if i == 0 { 1.0 } else { 0.0 });
        let ins = [In::A(&input), In::A(&[0.0; 64]), In::A(&[0.0; 64])];
        let mut ring = std::vec![0.0f32; REVERB_BUF_SAMPLES];
        let mut p0 = [0.0f32; 64];
        let mut p1 = [0.0f32; 64];
        {
            let mut outs = OutView::pair(&mut p0, &mut p1);
            n.process_resolved(&ins, 1.0 / 44_100.0, &mut outs, Some(&mut ring));
        }
        assert!(p0.iter().all(|s| s.is_finite() && s.abs() <= 16.0));
        assert!(p1.iter().all(|s| s.is_finite() && s.abs() <= 16.0));
    }

    #[test]
    fn room_node_without_buffer_is_dry_both_ports() {
        let mut n = Node::new(Kind::Room, 0);
        let input = [0.4f32; 16];
        let ins = [In::A(&input), In::A(&[0.0; 16]), In::A(&[0.0; 16])];
        let mut p0 = [0.0f32; 16];
        let mut p1 = [0.0f32; 16];
        {
            let mut outs = OutView::pair(&mut p0, &mut p1);
            n.process_resolved(&ins, 1.0 / 44_100.0, &mut outs, None);
        }
        assert!(p0.iter().all(|&s| (s - 0.4).abs() < 1e-6));
        assert!(p1.iter().all(|&s| (s - 0.4).abs() < 1e-6));
    }

    #[test]
    fn hall_node_renders_stereo_with_buffer() {
        use deluge_dsp_kernels::reverb::HALL_BUF_SAMPLES;
        let mut n = Node::new(Kind::Hall, 0);
        assert_eq!(Node::out_width(Kind::Hall), 2);
        n.set_param(0, 1.0); // mix wet
        n.set_param(2, 0.8); // size
        let input: [f32; 64] = core::array::from_fn(|i| if i == 0 { 1.0 } else { 0.0 });
        let ins = [In::A(&input), In::A(&[0.0; 64]), In::A(&[0.0; 64])];
        let mut ring = std::vec![0.0f32; HALL_BUF_SAMPLES];
        let mut p0 = [0.0f32; 64];
        let mut p1 = [0.0f32; 64];
        {
            let mut outs = OutView::pair(&mut p0, &mut p1);
            n.process_resolved(&ins, 1.0 / 44_100.0, &mut outs, Some(&mut ring));
        }
        assert!(p0.iter().all(|s| s.is_finite() && s.abs() <= 16.0));
        assert!(p1.iter().all(|s| s.is_finite() && s.abs() <= 16.0));
    }

    #[test]
    fn hall_node_without_buffer_is_dry_both_ports() {
        let mut n = Node::new(Kind::Hall, 0);
        let input = [0.4f32; 16];
        let ins = [In::A(&input), In::A(&[0.0; 16]), In::A(&[0.0; 16])];
        let mut p0 = [0.0f32; 16];
        let mut p1 = [0.0f32; 16];
        {
            let mut outs = OutView::pair(&mut p0, &mut p1);
            n.process_resolved(&ins, 1.0 / 44_100.0, &mut outs, None);
        }
        assert!(p0.iter().all(|&s| (s - 0.4).abs() < 1e-6));
        assert!(p1.iter().all(|&s| (s - 0.4).abs() < 1e-6));
    }

    #[test]
    fn plate_node_renders_stereo_with_buffer() {
        use deluge_dsp_kernels::reverb::PLATE_BUF_SAMPLES;
        let mut n = Node::new(Kind::Plate, 0);
        assert_eq!(Node::out_width(Kind::Plate), 2);
        n.set_param(0, 1.0);
        n.set_param(2, 0.8);
        let input: [f32; 64] = core::array::from_fn(|i| if i == 0 { 1.0 } else { 0.0 });
        let ins = [In::A(&input), In::A(&[0.0; 64]), In::A(&[0.0; 64])];
        let mut ring = std::vec![0.0f32; PLATE_BUF_SAMPLES];
        let mut p0 = [0.0f32; 64];
        let mut p1 = [0.0f32; 64];
        {
            let mut outs = OutView::pair(&mut p0, &mut p1);
            n.process_resolved(&ins, 1.0 / 44_100.0, &mut outs, Some(&mut ring));
        }
        assert!(p0.iter().all(|s| s.is_finite() && s.abs() <= 32.0));
        assert!(p1.iter().all(|s| s.is_finite() && s.abs() <= 32.0));
    }

    #[test]
    fn plate_node_without_buffer_is_dry_both_ports() {
        let mut n = Node::new(Kind::Plate, 0);
        let input = [0.4f32; 16];
        let ins = [In::A(&input), In::A(&[0.0; 16]), In::A(&[0.0; 16])];
        let mut p0 = [0.0f32; 16];
        let mut p1 = [0.0f32; 16];
        {
            let mut outs = OutView::pair(&mut p0, &mut p1);
            n.process_resolved(&ins, 1.0 / 44_100.0, &mut outs, None);
        }
        assert!(p0.iter().all(|&s| (s - 0.4).abs() < 1e-6));
        assert!(p1.iter().all(|&s| (s - 0.4).abs() < 1e-6));
    }

    #[test]
    fn drive_node_renders_mono_bounded() {
        let mut n = Node::new(Kind::Drive, 0);
        assert_eq!(Node::out_width(Kind::Drive), 1);
        n.set_param(0, 1.0); // drive
        n.set_param(2, 1.0); // mix
        n.set_param(3, 1.0); // shape = hard
        let input: [f32; 64] = core::array::from_fn(|i| 0.8 * (i as f32 * 0.2).sin());
        let ins = [In::A(&input), In::A(&[0.0; 64]), In::A(&[0.0; 64])];
        let mut buf = [0.0f32; 64];
        {
            let mut outs = OutView::single(&mut buf);
            n.process_resolved(&ins, 1.0 / 48_000.0, &mut outs, None);
        }
        assert!(buf.iter().all(|s| s.is_finite() && s.abs() <= 4.0));
        assert!(buf.iter().any(|&s| s != 0.0));
    }

    #[test]
    fn comp_node_wires_params_and_compresses() {
        assert_eq!(Node::out_width(Kind::Comp), 1);
        let mut n = Node::new(Kind::Comp, 0);
        n.set_param(0, -20.0); // threshold
        n.set_param(1, 8.0);   // ratio
        n.set_param(2, 0.001); // attack
        n.set_param(3, 0.05);  // release
        n.set_param(4, 0.0);   // knee (hard)
        n.set_param(5, 0.0);   // makeup
        n.set_param(6, 0.0);   // Peak
        // Loud constant 1.0 (0 dB, well above -20) on port 0; run several blocks to
        // settle the ballistics, then assert the output is well below the input.
        // Scaffolding copied from `drive_node_renders_mono_bounded` (node.rs:1399).
        let input = [1.0f32; 64];
        let ins = [In::A(&input), In::A(&[0.0; 64]), In::A(&[0.0; 64])];
        let mut buf = [0.0f32; 64];
        for _ in 0..20 {
            let mut outs = OutView::single(&mut buf);
            n.process_resolved(&ins, 1.0 / 48_000.0, &mut outs, None);
        }
        assert!(buf.iter().all(|s| s.is_finite()), "finite");
        // 0 dB through thr=-20/ratio=8 → ~15 dB GR → settled magnitude ≈ 0.18, well < 1.0.
        assert!(buf[buf.len() - 1].abs() < 0.5, "compressed well below input, got {}", buf[buf.len() - 1]);
    }

    #[test]
    fn drive_shape_param_changes_output() {
        let mk = |shape: f32| {
            let mut n = Node::new(Kind::Drive, 0);
            n.set_param(0, 1.0); // drive
            n.set_param(2, 1.0); // mix
            n.set_param(3, shape);
            let input = [0.9f32; 32];
            let ins = [In::A(&input), In::A(&[0.0; 32]), In::A(&[0.0; 32])];
            let mut buf = [0.0f32; 32];
            {
                let mut outs = OutView::single(&mut buf);
                n.process_resolved(&ins, 1.0 / 48_000.0, &mut outs, None);
            }
            buf
        };
        assert!(mk(0.0) != mk(2.0), "soft vs fold should differ");
    }

    #[test]
    fn eq_node_renders_mono_bounded() {
        let mut n = Node::new(Kind::Eq, 0);
        assert_eq!(Node::out_width(Kind::Eq), 1);
        n.set_param(0, 1000.0); // freq
        n.set_param(1, 12.0); // gain dB
        n.set_param(2, 1.0); // q
        n.set_param(3, 0.0); // peak
        let input: [f32; 64] = core::array::from_fn(|i| 0.5 * (i as f32 * 0.13).sin());
        let ins = [In::A(&input), In::A(&[0.0; 64]), In::A(&[0.0; 64])];
        let mut buf = [0.0f32; 64];
        {
            let mut outs = OutView::single(&mut buf);
            n.process_resolved(&ins, 1.0 / 48_000.0, &mut outs, None);
        }
        assert!(buf.iter().all(|s| s.is_finite() && s.abs() <= 32.0));
        assert!(buf.iter().any(|&s| s != 0.0));
    }

    #[test]
    fn eq_zero_gain_passes_through() {
        // 0 dB peak → identity biquad → output ≈ input.
        let mut n = Node::new(Kind::Eq, 0);
        n.set_param(0, 1000.0);
        n.set_param(1, 0.0); // 0 dB
        n.set_param(2, 1.0);
        let input = [0.4f32; 32];
        let ins = [In::A(&input), In::A(&[0.0; 32]), In::A(&[0.0; 32])];
        let mut buf = [0.0f32; 32];
        {
            let mut outs = OutView::single(&mut buf);
            n.process_resolved(&ins, 1.0 / 48_000.0, &mut outs, None);
        }
        // After settling, a DC input through a 0 dB EQ ≈ input.
        assert!((buf[31] - 0.4).abs() < 1e-3, "0 dB should pass through: {}", buf[31]);
    }

    #[test]
    fn lfo_node_renders_mono_bounded() {
        let mut n = Node::new(Kind::Lfo, 0);
        assert_eq!(Node::out_width(Kind::Lfo), 1);
        n.set_param(0, 2.0); // saw
        let rate = [4000.0f32; 64]; // fast so it varies over the block
        let ins = [In::A(&rate), In::A(&[0.0; 64]), In::A(&[0.0; 64])];
        let mut buf = [0.0f32; 64];
        {
            let mut outs = OutView::single(&mut buf);
            n.process_resolved(&ins, 1.0 / 48_000.0, &mut outs, None);
        }
        assert!(buf.iter().all(|s| s.is_finite() && s.abs() <= 1.0001));
        assert!(buf.iter().any(|&s| s != buf[0]), "should vary");
    }

    #[test]
    fn lfo_trigger_resets_phase() {
        let mut n = Node::new(Kind::Lfo, 0);
        n.set_param(0, 2.0); // saw, phase_offset 0
        let rate = [5.0f32; 500];
        let ins = [In::A(&rate), In::A(&[0.0; 500]), In::A(&[0.0; 500])];
        let mut buf = [0.0f32; 500];
        {
            let mut outs = OutView::single(&mut buf);
            n.process_resolved(&ins, 1.0 / 48_000.0, &mut outs, None);
        }
        n.trigger();
        let rate1 = [5.0f32; 1];
        let ins1 = [In::A(&rate1), In::A(&[0.0; 1]), In::A(&[0.0; 1])];
        let mut one = [0.0f32; 1];
        {
            let mut outs = OutView::single(&mut one);
            n.process_resolved(&ins1, 1.0 / 48_000.0, &mut outs, None);
        }
        assert!(one[0] < -0.99, "trigger resets saw to ~−1: {}", one[0]);
    }

    #[test]
    fn sample_hold_node_latches() {
        let mut n = Node::new(Kind::SampleHold, 0);
        assert_eq!(Node::out_width(Kind::SampleHold), 1);
        let input: [f32; 16] = core::array::from_fn(|i| i as f32 * 0.1);
        let clock: [f32; 16] = core::array::from_fn(|i| if (4..7).contains(&i) { 1.0 } else { -1.0 });
        let ins = [In::A(&input), In::A(&clock), In::A(&[0.0; 16])];
        let mut buf = [0.0f32; 16];
        {
            let mut outs = OutView::single(&mut buf);
            n.process_resolved(&ins, 1.0 / 48_000.0, &mut outs, None);
        }
        assert!(buf[2].abs() < 1e-6, "held 0 before edge");
        assert!((buf[8] - 0.4).abs() < 1e-6, "latched input at edge (sample 4 = 0.4)");
    }

    #[test]
    fn slew_node_glides_toward_input() {
        // Port 0 = input (a step to 1.0), port 1 = time (short → fast glide).
        // A port-swap in the render arm would treat 1.0 as the time constant
        // and ~0 as the input, so the output would crawl toward ~0 instead —
        // this test pins the input=0/time=1 wiring, not just boundedness.
        let mut n = Node::new(Kind::Slew, 0);
        assert_eq!(Node::out_width(Kind::Slew), 1);
        let input = [1.0f32; 32];
        let time = [0.0005f32; 32];
        let ins = [In::A(&input), In::A(&time), In::A(&[0.0; 32])];
        let mut buf = [0.0f32; 32];
        {
            let mut outs = OutView::single(&mut buf);
            n.process_resolved(&ins, 1.0 / 48_000.0, &mut outs, None);
        }
        assert!(buf[0] < 0.1, "starts near 0: {}", buf[0]);
        assert!(buf[31] > 0.5, "glides toward the input (1.0): {}", buf[31]);
        assert!(
            buf.windows(2).all(|w| w[1] >= w[0]),
            "monotonically rising toward the step"
        );
    }

    #[test]
    fn steps_node_sequences() {
        let mut n = Node::new(Kind::Steps, 0);
        assert_eq!(Node::out_width(Kind::Steps), 1);
        n.set_param(0, 2.0); // len
        n.set_param(1, 5.0); // values[0]
        n.set_param(2, 9.0); // values[1]
        let clock: [f32; 32] = core::array::from_fn(|i| if (8..11).contains(&i) || (16..19).contains(&i) { 1.0 } else { -1.0 });
        let ins = [In::A(&clock), In::A(&[0.0; 32]), In::A(&[0.0; 32])];
        let mut buf = [0.0f32; 32];
        {
            let mut outs = OutView::single(&mut buf);
            n.process_resolved(&ins, 1.0 / 48_000.0, &mut outs, None);
        }
        assert!((buf[4] - 5.0).abs() < 1e-6, "step 0");
        assert!((buf[12] - 5.0).abs() < 1e-6, "first edge keeps step 0");
        assert!((buf[20] - 9.0).abs() < 1e-6, "2nd edge → step 1");
    }

    #[test]
    fn curve_node_is_identity_at_k0() {
        let mut n = Node::new(Kind::Curve, 0);
        assert_eq!(Node::out_width(Kind::Curve), 1);
        let input: [f32; 8] = core::array::from_fn(|i| i as f32 / 7.0 * 2.0 - 1.0);
        let k = [0.0f32; 8];
        let ins = [In::A(&input), In::A(&k), In::A(&[0.0; 8])];
        let mut buf = [0.0f32; 8];
        {
            let mut outs = OutView::single(&mut buf);
            n.process_resolved(&ins, 1.0 / 48_000.0, &mut outs, None);
        }
        for i in 0..8 {
            assert!((buf[i] - input[i]).abs() < 1e-6, "k=0 identity at {i}");
        }
    }

    #[test]
    fn ctrl_node_holds_param_value() {
        let mut n = Node::new(Kind::Ctrl, 0);
        assert_eq!(Node::out_width(Kind::Ctrl), 1);
        n.set_param(0, 2.5);
        let ins = [In::A(&[0.0; 4]), In::A(&[0.0; 4]), In::A(&[0.0; 4])];
        let mut buf = [0.0f32; 4];
        {
            let mut outs = OutView::single(&mut buf);
            n.process_resolved(&ins, 1.0 / 48_000.0, &mut outs, None);
        }
        assert_eq!(buf, [2.5, 2.5, 2.5, 2.5]);
    }

    #[test]
    fn qstep_node_snaps_to_levels() {
        let mut n = Node::new(Kind::QuantStep, 0);
        n.set_param(0, 2.0); // {-1, +1}
        let input = [-0.4f32, 0.4, -0.9, 0.9];
        let ins = [In::A(&input), In::A(&[0.0; 4]), In::A(&[0.0; 4])];
        let mut buf = [0.0f32; 4];
        {
            let mut outs = OutView::single(&mut buf);
            n.process_resolved(&ins, 1.0 / 48_000.0, &mut outs, None);
        }
        assert_eq!(buf, [-1.0, 1.0, -1.0, 1.0]);
    }

    #[test]
    fn qpitch_node_snaps_major() {
        let mut n = Node::new(Kind::QuantPitch, 0);
        n.set_param(0, 2741.0); // 0b101010110101 major
        n.set_param(1, 0.0); // root 0
        let input = [1.0f32, 3.0, 6.4];
        let ins = [In::A(&input), In::A(&[0.0; 3]), In::A(&[0.0; 3])];
        let mut buf = [0.0f32; 3];
        {
            let mut outs = OutView::single(&mut buf);
            n.process_resolved(&ins, 1.0 / 48_000.0, &mut outs, None);
        }
        assert_eq!(buf, [2.0, 4.0, 7.0]);
    }

    #[test]
    fn mtof_node_converts_octaves() {
        let mut n = Node::new(Kind::Mtof, 0);
        n.set_param(0, 440.0);
        let input = [0.0f32, 12.0, -12.0];
        let ins = [In::A(&input), In::A(&[0.0; 3]), In::A(&[0.0; 3])];
        let mut buf = [0.0f32; 3];
        {
            let mut outs = OutView::single(&mut buf);
            n.process_resolved(&ins, 1.0 / 48_000.0, &mut outs, None);
        }
        assert!((buf[0] - 440.0).abs() < 1e-2 && (buf[1] - 880.0).abs() < 1e-2 && (buf[2] - 220.0).abs() < 1e-2);
    }

    #[test]
    fn poly_widths_and_predicates() {
        assert_eq!(Node::out_width(Kind::PolyCtrl), VOICES);
        assert_eq!(Node::out_width(Kind::PolyOsc), VOICES);
        assert_eq!(Node::out_width(Kind::VoiceSum), 1);
        assert!(Node::is_poly(Kind::PolyCtrl) && Node::is_poly(Kind::PolyOsc) && Node::is_poly(Kind::VoiceSum));
        assert!(!Node::is_poly(Kind::Saw));
        assert_eq!(Node::poly_in_count(Kind::PolyOsc), 2); // pitch (port 0) + PWM width (port 1)
        assert_eq!(Node::poly_in_count(Kind::VoiceSum), 1);
        assert_eq!(Node::poly_in_count(Kind::PolyCtrl), 0);
        assert_eq!(Node::poly_in_count(Kind::PolyMul), 2);
        assert_eq!(Node::poly_in_count(Kind::PolyAr), 0);
    }

    #[test]
    fn polyctrl_node_fills_and_voicesum_collapses() {
        // PolyCtrl(set voices) → tile → VoiceSum → mono sum.
        let n = 4;
        let mut ctrl = Node::new(Kind::PolyCtrl, 0);
        for v in 0..VOICES {
            ctrl.set_param(v as u8, (v + 1) as f32); // 1..=8
        }
        let mut tile = [0.0f32; VOICES * 4];
        let dummy: [In; MAX_INPUTS] = [In::K(0.0); MAX_INPUTS];
        ctrl.poly_process(&dummy, [None, None], 1.0 / 48_000.0, &mut tile, None);
        for i in 0..n {
            for v in 0..VOICES {
                assert_eq!(tile[i * VOICES + v], (v + 1) as f32);
            }
        }
        let mut sum = Node::new(Kind::VoiceSum, 0);
        let mut mono = [0.0f32; 4];
        sum.poly_process(&dummy, [Some(&tile), None], 1.0 / 48_000.0, &mut mono, None);
        let want: f32 = (1..=VOICES).map(|x| x as f32).sum(); // 36
        assert!(mono.iter().all(|&s| (s - want).abs() < 1e-4), "each sample sums to {want}");
    }

    #[test]
    fn voicesum_node_gain_defaults_unity_and_set_param_scales() {
        // PolyCtrl(set voices) → tile → VoiceSum → mono sum, mirroring
        // `polyctrl_node_fills_and_voicesum_collapses`.
        let mut ctrl = Node::new(Kind::PolyCtrl, 0);
        for v in 0..VOICES {
            ctrl.set_param(v as u8, (v + 1) as f32); // 1..=8
        }
        let mut tile = [0.0f32; VOICES * 4];
        let dummy: [In; MAX_INPUTS] = [In::K(0.0); MAX_INPUTS];
        ctrl.poly_process(&dummy, [None, None], 1.0 / 48_000.0, &mut tile, None);

        // Default gain 1.0 → plain sum.
        let mut n1 = Node::new(Kind::VoiceSum, 0);
        let mut mono1 = [0.0f32; 4];
        n1.poly_process(&dummy, [Some(&tile), None], 1.0 / 48_000.0, &mut mono1, None);
        let want: f32 = (1..=VOICES).map(|x| x as f32).sum(); // 36
        assert!(mono1.iter().all(|&s| (s - want).abs() < 1e-4), "default gain 1.0 = plain sum, got {mono1:?}");

        // set_param(0, 0.5) halves the summed output.
        let mut n2 = Node::new(Kind::VoiceSum, 0);
        n2.set_param(0, 0.5);
        let mut mono2 = [0.0f32; 4];
        n2.poly_process(&dummy, [Some(&tile), None], 1.0 / 48_000.0, &mut mono2, None);
        assert!(mono2.iter().all(|&s| (s - want * 0.5).abs() < 1e-4), "gain 0.5 halves the sum, got {mono2:?}");
    }

    #[test]
    fn stereovoicesum_node_center_and_pan_and_gain() {
        assert_eq!(Node::out_width(Kind::StereoVoiceSum), 2);
        assert!(Node::is_poly(Kind::StereoVoiceSum));
        assert_eq!(Node::poly_in_count(Kind::StereoVoiceSum), 1);

        const BLOCK: usize = 4;
        // A VOICES-lane tile, one sample: lane v = v+1.
        let mut tile = [0.0f32; VOICES * BLOCK];
        for v in 0..VOICES { tile[v] = (v + 1) as f32; }
        let sum: f32 = (1..=VOICES).map(|v| v as f32).sum();

        // out spans 2 rows (2*BLOCK); poly_process splits it L | R.
        let mut node = Node::new(Kind::StereoVoiceSum, 0);
        let ins: [In; MAX_INPUTS] = [In::K(0.0); MAX_INPUTS];
        let mut out = [0.0f32; 2 * BLOCK];
        // default (center, gain 1.0): L[0] == R[0] == sum.
        node.poly_process(&ins, [Some(&tile[..]), None], 0.0, &mut out, None);
        assert!((out[0] - sum).abs() < 1e-6, "L center == sum");
        assert!((out[BLOCK] - sum).abs() < 1e-6, "R center == sum");

        // set_param(0)=gain 0.5 halves both.
        node.set_param(0, 0.5);
        let mut out2 = [0.0f32; 2 * BLOCK];
        node.poly_process(&ins, [Some(&tile[..]), None], 0.0, &mut out2, None);
        assert!((out2[0] - 0.5 * sum).abs() < 1e-6 && (out2[BLOCK] - 0.5 * sum).abs() < 1e-6);

        // set_param(1)=pan lane 0 hard right (+1): lane 0 drops from L, stays in R.
        node.set_param(0, 1.0); // gain back to 1.0
        node.set_param(1, 1.0); // pan[0] = +1
        let mut out3 = [0.0f32; 2 * BLOCK];
        node.poly_process(&ins, [Some(&tile[..]), None], 0.0, &mut out3, None);
        let drop0: f32 = (2..=VOICES).map(|v| v as f32).sum(); // lanes 1..VOICES
        assert!((out3[0] - drop0).abs() < 1e-6, "lane0 absent from L");
        assert!((out3[BLOCK] - sum).abs() < 1e-6, "lane0 present in R");
    }

    #[test]
    fn polymtof_node_converts_semitones() {
        assert_eq!(Node::out_width(Kind::PolyMtof), VOICES);
        assert_eq!(Node::poly_in_count(Kind::PolyMtof), 1);
        let mut n = Node::new(Kind::PolyMtof, 0);
        n.set_param(0, 440.0);
        let mut semis = [0.0f32; VOICES * 2];
        for i in 0..2 {
            semis[i * VOICES] = 0.0; // voice 0 → 440
            semis[i * VOICES + 1] = 12.0; // voice 1 → 880
        }
        let dummy = [In::K(0.0); MAX_INPUTS];
        let mut out = [0.0f32; VOICES * 2];
        n.poly_process(&dummy, [Some(&semis), None], 1.0 / 48_000.0, &mut out, None);
        assert!((out[0] - 440.0).abs() < 1e-2 && (out[1] - 880.0).abs() < 1e-2);
    }

    #[test]
    fn polyslew_node_glides_and_snaps() {
        assert_eq!(Node::poly_in_count(Kind::PolySlew), 1);
        assert_eq!(Node::out_width(Kind::PolySlew), VOICES);
        assert!(Node::is_poly(Kind::PolySlew));

        let mut n = Node::new(Kind::PolySlew, 0);
        n.set_param(0, 0.05); // 50ms glide time
        let dt = 1.0 / 48_000.0;
        let dummy = [In::K(0.0); MAX_INPUTS];

        // Lane 0's target jumps from the initial 0.0 state to 1.0, held for
        // the whole block; the one-pole lag means it should move toward 1.0
        // without arriving there in a single 8-sample block.
        let n_samples = 8;
        let mut target = [0.0f32; VOICES * 8];
        for i in 0..n_samples {
            target[i * VOICES] = 1.0;
        }
        let mut out = [0.0f32; VOICES * 8];
        n.poly_process(&dummy, [Some(&target), None], dt, &mut out, None);
        let last = out[(n_samples - 1) * VOICES];
        assert!(last > 0.0 && last < 1.0, "lane 0 glides partway toward target, got {last}");

        // trigger_voice(0) arms a snap: the very next processed sample for
        // lane 0 must equal the (new) target exactly, no swoop from 0.72ish.
        n.trigger_voice(0);
        let mut target2 = [0.0f32; VOICES * 4];
        for i in 0..4 {
            target2[i * VOICES] = -0.5;
        }
        let mut out2 = [0.0f32; VOICES * 4];
        n.poly_process(&dummy, [Some(&target2), None], dt, &mut out2, None);
        assert!((out2[0] - (-0.5)).abs() < 1e-6, "lane 0 snaps to target, got {}", out2[0]);

        // Snap isn't sticky across calls: a later target change without a
        // fresh trigger_voice must glide again, not re-snap.
        let mut target3 = [0.0f32; VOICES * 4];
        for i in 0..4 {
            target3[i * VOICES] = 0.5;
        }
        let mut out3 = [0.0f32; VOICES * 4];
        n.poly_process(&dummy, [Some(&target3), None], dt, &mut out3, None);
        assert!(
            out3[0] > -0.5 && out3[0] < 0.5,
            "no snap-stick: lane 0 glides toward new target, got {}",
            out3[0]
        );
    }

    #[test]
    fn polyosc_shape_and_new_kinds() {
        assert_eq!(Node::poly_in_count(Kind::PolyAdd), 2);
        assert_eq!(Node::poly_in_count(Kind::PolyNoise), 0);
        assert_eq!(Node::out_width(Kind::PolyAdd), VOICES);
        assert_eq!(Node::out_width(Kind::PolyNoise), VOICES);
        // PolyOsc set_param(0, code) selects the shape (a saw is band-limited,
        // so a mid-phase sample differs from the sine at the same phase).
        let mut n = Node::new(Kind::PolyOsc, 0);
        n.set_param(0, 1.0); // Saw
        let pitch = [220.0f32; VOICES * 4];
        let width = [0.0f32; VOICES * 4]; // unconnected ⇒ 0.5 duty default
        let ins = [In::A(&[0.0; VOICES * 4]); MAX_INPUTS];
        let mut out = [0.0f32; VOICES * 4];
        n.poly_process(&ins, [Some(&pitch), Some(&width)], 1.0 / 48_000.0, &mut out, None);
        assert!(out.iter().all(|s| s.is_finite() && s.abs() <= 1.2));
        assert!(out.iter().any(|&s| s != 0.0), "saw renders");
    }

    #[test]
    fn poly_noise_node_renders_bounded() {
        let mut n = Node::new(Kind::PolyNoise, 0);
        let ins = [In::A(&[0.0; VOICES * 4]); MAX_INPUTS];
        let mut out = [0.0f32; VOICES * 4];
        n.poly_process(&ins, [None, None], 1.0 / 48_000.0, &mut out, None);
        assert!(out.iter().all(|&s| s.is_finite() && s.abs() <= 1.0) && out.iter().any(|&s| s != 0.0));
    }

    #[test]
    fn poly_pink_and_brown_noise_nodes_render_bounded() {
        // Mirrors poly_noise_node_renders_bounded: PolyPink/PolyBrown are pure
        // poly sources (poly_in_count 0, out_width VOICES) that construct
        // State::PolyNoise with the matching color.
        for kind in [Kind::PolyPink, Kind::PolyBrown] {
            assert_eq!(Node::poly_in_count(kind), 0);
            assert_eq!(Node::out_width(kind), VOICES);
            assert!(Node::is_poly(kind));
            let mut n = Node::new(kind, 0);
            let ins = [In::A(&[0.0; VOICES * 4]); MAX_INPUTS];
            let mut out = [0.0f32; VOICES * 4];
            n.poly_process(&ins, [None, None], 1.0 / 48_000.0, &mut out, None);
            assert!(out.iter().all(|&s| s.is_finite() && s.abs() <= 1.0), "kind={kind:?} bounded");
            assert!(out.iter().any(|&s| s != 0.0), "kind={kind:?} non-silent");
        }
    }

    #[test]
    fn polyadsr_node_sustains_at_set_level() {
        assert_eq!(Node::poly_in_count(Kind::PolyAdsr), 0);
        assert_eq!(Node::out_width(Kind::PolyAdsr), VOICES);
        assert!(Node::is_poly(Kind::PolyAdsr));

        let mut n = Node::new(Kind::PolyAdsr, 0);
        // inputs [attack, decay, release]
        *n.input_mut(0).unwrap() = Input::Const(0.001);
        *n.input_mut(1).unwrap() = Input::Const(0.001);
        *n.input_mut(2).unwrap() = Input::Const(0.5);
        n.set_param(0, 0.5); // sustain
        for v in 0..VOICES { n.gate_voice(v, true); }

        let dt = 1.0 / 48_000.0;
        let block = 16usize;
        let a = [0.001f32; 16];
        let d = [0.001f32; 16];
        let r = [0.5f32; 16];
        let ins = [In::A(&a), In::A(&d), In::A(&r)];
        let mut out = [0.0f32; VOICES * 16];
        // Render enough blocks that attack (1ms) + decay (1ms) settle into
        // sustain well before the end (16*100 samples @ 48kHz ≈ 33ms).
        for _ in 0..100 {
            n.poly_process(&ins, [None, None], dt, &mut out, None);
        }
        for v in 0..VOICES {
            let last = out[(block - 1) * VOICES + v];
            assert!((last - 0.5).abs() < 1e-3, "voice {v} steady level = {last}, want ~0.5");
        }
    }

    /// Regression for C1: `Node::gate` must dispatch to `State::Adsr`, not just
    /// `State::Ar`. Pre-fix, a mono `Kind::Adsr` node's `gate(true)` was a no-op
    /// (fell into `_ => {}`), so the envelope never left `Stage::Idle` and this
    /// node's output stayed 0.0 forever — `Osc * Env.adsr(...)` used outside a
    /// Synth rendered pure silence with no error. Mirrors
    /// `polyadsr_node_sustains_at_set_level` but drives the mono `gate`/
    /// `process_resolved` path instead of `gate_voice`/`poly_process`.
    #[test]
    fn adsr_node_gate_sustains_at_set_level() {
        let mut n = Node::new(Kind::Adsr, 0);
        // inputs [attack, decay, release]
        *n.input_mut(0).unwrap() = Input::Const(0.001);
        *n.input_mut(1).unwrap() = Input::Const(0.001);
        *n.input_mut(2).unwrap() = Input::Const(0.5);
        n.set_param(0, 0.6); // sustain
        n.gate(true);

        let dt = 1.0 / 48_000.0;
        let a = [0.001f32; 16];
        let d = [0.001f32; 16];
        let r = [0.5f32; 16];
        let ins = [In::A(&a), In::A(&d), In::A(&r)];
        let mut buf = [0.0f32; 16];
        // Render enough blocks that attack (1ms) + decay (1ms) settle into
        // sustain well before the end (16*100 samples @ 48kHz ≈ 33ms).
        for _ in 0..100 {
            let mut outs = OutView::single(&mut buf);
            n.process_resolved(&ins, dt, &mut outs, None);
        }
        let last = buf[15];
        assert!((last - 0.6).abs() < 1e-3, "steady level = {last}, want ~0.6 (pre-fix this was 0.0, stuck in Idle)");
    }

    /// Same gap as `adsr_node_gate_sustains_at_set_level`, driven via
    /// `Node::trigger` (the percussive AD path) rather than `gate`.
    #[test]
    fn adsr_node_trigger_is_non_zero() {
        let mut n = Node::new(Kind::Adsr, 0);
        *n.input_mut(0).unwrap() = Input::Const(0.001);
        *n.input_mut(1).unwrap() = Input::Const(0.001);
        *n.input_mut(2).unwrap() = Input::Const(0.5);
        n.set_param(0, 0.6); // sustain
        n.trigger();

        let dt = 1.0 / 48_000.0;
        let a = [0.001f32; 16];
        let d = [0.001f32; 16];
        let r = [0.5f32; 16];
        let ins = [In::A(&a), In::A(&d), In::A(&r)];
        let mut buf = [0.0f32; 16];
        for _ in 0..100 {
            let mut outs = OutView::single(&mut buf);
            n.process_resolved(&ins, dt, &mut outs, None);
        }
        assert!(buf[15] > 0.0, "trigger() should leave Idle and produce non-zero output, got {}", buf[15]);
    }

    #[test]
    fn poly_moog_and_ms20_voice_render_sound() {
        // Mirrors the PolySvf graph test (crate::engine::tests::full_voice_gated_per_voice):
        // PolyCtrl → PolyOsc → <filter> → PolyMul(·, PolyAr) → VoiceSum, one voice
        // gated via Cmd::GateVoice. Parametrized over the four new poly filter kinds.
        use crate::cmd::Cmd;
        use crate::engine::Engine;
        use crate::ids::NodeId;

        for fkind in [Kind::PolyMoogLp4, Kind::PolyMoogLp2, Kind::PolyMs20Lp, Kind::PolyMs20Hp] {
            type PE = Engine<64, 8, 48, 4, 45056, 2048>;
            let mut e = PE::new(48_000.0);
            // pitch source
            e.create(NodeId(0), Kind::PolyCtrl);
            for v in 0..VOICES {
                e.apply(Cmd::SetParam { node: NodeId(0), param: v as u8, value: (v as f32 + 1.0) * 110.0 });
            }
            // osc → filter
            e.create(NodeId(1), Kind::PolyOsc);
            *e.node_input_mut(NodeId(1), 0).unwrap() = Input::Node { node: NodeId(0), port: 0 };
            e.create(NodeId(2), fkind);
            *e.node_input_mut(NodeId(2), 0).unwrap() = Input::Node { node: NodeId(1), port: 0 };
            *e.node_input_mut(NodeId(2), 1).unwrap() = Input::Const(1200.0); // cutoff
            *e.node_input_mut(NodeId(2), 2).unwrap() = Input::Const(0.2);    // res
            // envelope + VCA
            e.create(NodeId(3), Kind::PolyAr);
            *e.node_input_mut(NodeId(3), 0).unwrap() = Input::Const(0.0005); // attack
            *e.node_input_mut(NodeId(3), 1).unwrap() = Input::Const(0.05);   // release
            e.create(NodeId(4), Kind::PolyMul);
            *e.node_input_mut(NodeId(4), 0).unwrap() = Input::Node { node: NodeId(2), port: 0 };
            *e.node_input_mut(NodeId(4), 1).unwrap() = Input::Node { node: NodeId(3), port: 0 };
            // sum → out
            e.create(NodeId(5), Kind::VoiceSum);
            *e.node_input_mut(NodeId(5), 0).unwrap() = Input::Node { node: NodeId(4), port: 0 };

            e.apply(Cmd::GateVoice { node: NodeId(3), voice: 0, on: true });
            e.render_block();
            let out = e.node_output(NodeId(5), 0);
            assert!(out.iter().all(|&s| s.is_finite() && s.abs() <= 8.1), "kind={fkind:?} bounded: {out:?}");
            assert!(out.iter().any(|&s| s.abs() > 1e-4), "kind={fkind:?} gated voice sounds");
        }
    }

    #[test]
    fn poly_sync_voice_renders_for_each_wave() {
        // Two poly pitch sources (master, slave) → PolySyncX → VoiceSum, for
        // each of the four sync-wave kinds. No gate/envelope involved (a sync
        // oscillator is always producing tone), so the collapsed mix should be
        // finite, bounded, and non-silent for every wave.
        use crate::cmd::Cmd;
        use crate::engine::Engine;
        use crate::ids::NodeId;

        for skind in [Kind::PolySyncSine, Kind::PolySyncSaw, Kind::PolySyncSquare, Kind::PolySyncTri] {
            assert_eq!(Node::poly_in_count(skind), 2); // master (port 0) + slave (port 1)
            assert_eq!(Node::out_width(skind), VOICES);
            assert!(Node::is_poly(skind));

            type PE = Engine<64, 8, 48, 4, 45056, 2048>;
            let mut e = PE::new(48_000.0);
            // master pitch source
            e.create(NodeId(0), Kind::PolyCtrl);
            for v in 0..VOICES {
                e.apply(Cmd::SetParam { node: NodeId(0), param: v as u8, value: (v as f32 + 1.0) * 110.0 });
            }
            // slave pitch source (a non-integer ratio above the master)
            e.create(NodeId(1), Kind::PolyCtrl);
            for v in 0..VOICES {
                e.apply(Cmd::SetParam { node: NodeId(1), param: v as u8, value: (v as f32 + 1.0) * 110.0 * 2.7 });
            }
            // sync osc: port 0 = master, port 1 = slave
            e.create(NodeId(2), skind);
            *e.node_input_mut(NodeId(2), 0).unwrap() = Input::Node { node: NodeId(0), port: 0 };
            *e.node_input_mut(NodeId(2), 1).unwrap() = Input::Node { node: NodeId(1), port: 0 };
            // sum → out
            e.create(NodeId(3), Kind::VoiceSum);
            *e.node_input_mut(NodeId(3), 0).unwrap() = Input::Node { node: NodeId(2), port: 0 };

            e.render_block();
            let out = e.node_output(NodeId(3), 0);
            assert!(out.iter().all(|&s| s.is_finite() && s.abs() <= 8.1), "kind={skind:?} bounded: {out:?}");
            assert!(out.iter().any(|&s| s.abs() > 1e-4), "kind={skind:?} non-silent");
        }
    }

    #[test]
    fn polywt_single_matches_mono_per_voice() {
        // Static table 0 (saw). A PolyWt-backed poly node with distinct
        // per-voice pitch must match 8 independent mono `Kind::Wavetable`
        // nodes, one per pitch — bit-identical, since `PolyWt::process_voice`
        // is a thin per-voice delegate to the same `WtOsc` kernel.
        //
        // Node-level (not `Engine`): builds the pitch tile directly in the
        // sample-major interleaved convention (`tile[i*VOICES+v]`) that
        // `poly_process` itself reads/writes, so `out[i*VOICES+v]` below is
        // exactly voice v's trace — no arena port-chunking ambiguity (the
        // engine's `node_output(id, port)` for a poly node returns a
        // BLOCK-chunk of the interleaved tile, not a single voice's trace,
        // when `port` doesn't divide evenly; going through `Node` directly
        // sidesteps that entirely).
        assert_eq!(Node::poly_in_count(Kind::PolyWt), 1);
        assert_eq!(Node::out_width(Kind::PolyWt), VOICES);
        assert!(Node::is_poly(Kind::PolyWt) && Node::is_poly(Kind::PolyWtMorph));

        let freqs: [f32; VOICES] = core::array::from_fn(|v| (v as f32 + 1.0) * 110.0);
        let dt = 1.0 / 48_000.0;
        let n = 64usize;

        let mut poly = Node::new(Kind::PolyWt, 0);
        poly.bind_table(TableSrc::Static(TableId(0)));
        let mut pitch = std::vec![0.0f32; VOICES * n];
        for i in 0..n {
            for v in 0..VOICES {
                pitch[i * VOICES + v] = freqs[v];
            }
        }
        let zero = std::vec![0.0f32; n];
        let ins = [In::A(&zero), In::A(&zero), In::A(&zero)];
        let mut out = std::vec![0.0f32; VOICES * n];
        poly.poly_process(&ins, [Some(&pitch), None], dt, &mut out, None);

        for v in 0..VOICES {
            let mut mono = Node::new(Kind::Wavetable, 0);
            mono.bind_table(TableSrc::Static(TableId(0)));
            let freq_buf = std::vec![freqs[v]; n];
            let mono_ins = [In::A(&freq_buf), In::A(&zero), In::A(&zero)];
            let mut mono_out = std::vec![0.0f32; n];
            {
                let mut outs = OutView::single(&mut mono_out);
                mono.process_resolved(&mono_ins, dt, &mut outs, None);
            }
            for i in 0..n {
                assert_eq!(out[i * VOICES + v], mono_out[i], "voice {v} sample {i}");
            }
            assert!(mono_out.iter().any(|&s| s != 0.0), "voice {v} renders");
        }
    }

    #[test]
    fn polywtmorph_matches_mono_per_voice() {
        // Static table 6 (HarmonicSweep) — a baked-in 2D (multi-frame) morph
        // bank (see `TableId`'s doc comment). Parallels
        // `polywt_single_matches_mono_per_voice`: a PolyWtMorph-backed poly
        // node with distinct per-voice pitch and a fixed, non-trivial
        // `position` (0.3, so the morph actually blends between frames
        // rather than landing exactly on one) must match 8 independent mono
        // `Kind::Wavetable` nodes bound to the same multi-frame table — bit-
        // identical, since `PolyWt::process_voice_morph` is a thin per-voice
        // delegate to the same `WtOsc::process_morph` kernel that mono
        // `Kind::Wavetable` routes to once it detects `frames > 1` (see
        // `process_resolved`'s `Kind::Wavetable` arm).
        //
        // Node-level (not `Engine`), for the same reason as the single-cycle
        // test above: builds the pitch tile directly in the sample-major
        // interleaved convention `poly_process` reads/writes, so
        // `out[i*VOICES+v]` is exactly voice v's trace.
        let freqs: [f32; VOICES] = core::array::from_fn(|v| (v as f32 + 1.0) * 110.0);
        let dt = 1.0 / 48_000.0;
        let n = 64usize;
        let position = 0.3f32;

        let mut poly = Node::new(Kind::PolyWtMorph, 0);
        poly.bind_table(TableSrc::Static(TableId(6)));
        let mut pitch = std::vec![0.0f32; VOICES * n];
        for i in 0..n {
            for v in 0..VOICES {
                pitch[i * VOICES + v] = freqs[v];
            }
        }
        let zero = std::vec![0.0f32; n];
        let pos = std::vec![position; n];
        let ins = [In::A(&zero), In::A(&zero), In::A(&pos)];
        let mut out = std::vec![0.0f32; VOICES * n];
        poly.poly_process(&ins, [Some(&pitch), None], dt, &mut out, None);

        for v in 0..VOICES {
            let mut mono = Node::new(Kind::Wavetable, 0);
            mono.bind_table(TableSrc::Static(TableId(6)));
            let freq_buf = std::vec![freqs[v]; n];
            let mono_ins = [In::A(&freq_buf), In::A(&zero), In::A(&pos)];
            let mut mono_out = std::vec![0.0f32; n];
            {
                let mut outs = OutView::single(&mut mono_out);
                mono.process_resolved(&mono_ins, dt, &mut outs, None);
            }
            for i in 0..n {
                assert_eq!(out[i * VOICES + v], mono_out[i], "voice {v} sample {i}");
            }
            assert!(mono_out.iter().any(|&s| s != 0.0), "voice {v} renders");
        }
    }

    #[test]
    fn polywtmorph_renders_finite_bounded_and_position_varies() {
        // Static table 6 (HarmonicSweep) is a baked-in 2D (multi-frame) morph
        // bank (see `TableId`'s doc comment). PolyWtMorph must render finite,
        // bounded, non-silent audio for every voice, and moving `position`
        // (port 2, a shared mono control) must change the output.
        let dt = 1.0 / 48_000.0;
        let n = 128usize;
        let mut pitch = std::vec![0.0f32; VOICES * n];
        for i in 0..n {
            for v in 0..VOICES {
                pitch[i * VOICES + v] = 220.0 + v as f32 * 15.0;
            }
        }
        let zero = std::vec![0.0f32; n];

        let render_at = |position: f32| -> std::vec::Vec<f32> {
            let mut node = Node::new(Kind::PolyWtMorph, 0);
            node.bind_table(TableSrc::Static(TableId(6)));
            let pos = std::vec![position; n];
            let ins = [In::A(&zero), In::A(&zero), In::A(&pos)];
            let mut out = std::vec![0.0f32; VOICES * n];
            node.poly_process(&ins, [Some(&pitch), None], dt, &mut out, None);
            out
        };

        let out_lo = render_at(0.0);
        let out_hi = render_at(1.0);
        assert!(out_lo.iter().all(|s| s.is_finite() && s.abs() <= 1.2));
        assert!(out_hi.iter().all(|s| s.is_finite() && s.abs() <= 1.2));
        assert!(out_lo.iter().any(|&s| s != 0.0), "position 0 renders");
        assert!(out_hi.iter().any(|&s| s != 0.0), "position 1 renders");
        assert!(out_lo != out_hi, "position must change the morph output");
    }

    #[test]
    fn polywt_pooled_table_renders_via_engine_voicesum() {
        // Exercises the ENGINE's pool_region threading for a poly wavetable
        // node (the Step-4 plumbing): `TableSrc::Pooled`, resolved by
        // `Engine::render_block`'s poly branch and passed into
        // `Node::poly_process`. PolyCtrl(pitch) → PolyWt(pooled saw pyramid)
        // → VoiceSum: non-silent, bounded, finite. `VoiceSum`'s width-1
        // output sidesteps the per-voice arena-chunking concern above.
        use crate::cmd::Cmd;
        use crate::engine::Engine;
        use crate::ids::NodeId;

        type PE = Engine<64, 8, 32, 4, 45056, 2048>;
        let mut e = PE::new(48_000.0);
        let compact_len = deluge_dsp_kernels::wavetable::COMPACT_LEN;
        let h = e.pool_alloc(compact_len).expect("pool room");
        let n = mipgen::N;
        let mut base = std::vec![0.0f32; n];
        for (i, s) in base.iter_mut().enumerate() {
            *s = 2.0 * (i as f32 / n as f32) - 1.0; // saw
        }
        mipgen::build_pyramid_flat_compact(&base, e.pool_slice_mut(h));

        e.create(NodeId(0), Kind::PolyCtrl);
        for v in 0..VOICES {
            e.apply(Cmd::SetParam { node: NodeId(0), param: v as u8, value: (v as f32 + 1.0) * 110.0 });
        }
        e.create(NodeId(1), Kind::PolyWt);
        *e.node_input_mut(NodeId(1), 0).unwrap() = Input::Node { node: NodeId(0), port: 0 };
        e.apply(Cmd::BindTable { node: NodeId(1), src: TableSrc::Pooled(h) });
        e.create(NodeId(2), Kind::VoiceSum);
        *e.node_input_mut(NodeId(2), 0).unwrap() = Input::Node { node: NodeId(1), port: 0 };

        e.render_block();
        let out = e.node_output(NodeId(2), 0);
        // 8 unison-ish saw voices can constructively peak up to ~8x a single
        // voice's ~1.2 bound (early-block phase alignment); 9.7 covers that
        // with margin while still catching genuine divergence.
        assert!(out.iter().all(|&s| s.is_finite() && s.abs() <= 9.7), "bounded: {out:?}");
        assert!(out.iter().any(|&s| s.abs() > 1e-4), "pooled poly wavetable sounds");

        // Free → pool region reclaimed (Cmd::Free's `table_src()`-driven
        // `pool.free` doesn't care whether the node is mono or poly).
        e.apply(Cmd::Free { node: NodeId(1) });
        assert_eq!(e.pool_alloc(compact_len), Some(h));
    }

    #[test]
    fn polywtmorph_pooled_table_renders_via_engine_voicesum() {
        // Same as above but `PolyWtMorph` with a 2-frame pooled bank (saw +
        // square) and a `position` control (port 2), driven through the full
        // engine (pool_region resolved by `render_block`'s poly branch).
        use crate::cmd::Cmd;
        use crate::engine::Engine;
        use crate::ids::NodeId;

        type PE = Engine<64, 8, 32, 4, 45056, 2048>;
        let mut e = PE::new(48_000.0);
        let compact_len = deluge_dsp_kernels::wavetable::COMPACT_LEN;
        let h = e.pool_alloc(2 * compact_len).expect("pool room");
        let n = mipgen::N;
        let mut saw = std::vec![0.0f32; n];
        let mut square = std::vec![0.0f32; n];
        for i in 0..n {
            saw[i] = 2.0 * (i as f32 / n as f32) - 1.0;
            square[i] = if i < n / 2 { 1.0 } else { -1.0 };
        }
        {
            let r = e.pool_slice_mut(h);
            mipgen::build_pyramid_flat_compact(&saw, &mut r[..compact_len]);
            mipgen::build_pyramid_flat_compact(&square, &mut r[compact_len..]);
        }

        e.create(NodeId(0), Kind::PolyCtrl);
        for v in 0..VOICES {
            e.apply(Cmd::SetParam { node: NodeId(0), param: v as u8, value: (v as f32 + 1.0) * 90.0 });
        }
        e.create(NodeId(1), Kind::PolyWtMorph);
        *e.node_input_mut(NodeId(1), 0).unwrap() = Input::Node { node: NodeId(0), port: 0 };
        *e.node_input_mut(NodeId(1), 2).unwrap() = Input::Const(0.5); // position
        e.apply(Cmd::BindTable { node: NodeId(1), src: TableSrc::Pooled(h) });
        e.create(NodeId(2), Kind::VoiceSum);
        *e.node_input_mut(NodeId(2), 0).unwrap() = Input::Node { node: NodeId(1), port: 0 };

        e.render_block();
        let out = e.node_output(NodeId(2), 0);
        // See `polywt_pooled_table_renders_via_engine_voicesum`'s comment: 8
        // unison-ish voices can constructively peak above a single voice's bound.
        assert!(out.iter().all(|&s| s.is_finite() && s.abs() <= 9.7), "bounded: {out:?}");
        assert!(out.iter().any(|&s| s.abs() > 1e-4), "pooled poly morph wavetable sounds");
    }
}
