//! Native DSP audio engine — the portable signal-graph core.
//!
//! A small fixed node graph rendered per-sample at 44.1 kHz. Wren scripts
//! build/patch/modulate the graph at control rate (via the `Node` bindings, which
//! emit [`Cmd`]s through the [`Host`](crate::Host)); they never touch engine
//! memory directly.
//!
//! This module is pure compute — no hardware, no concurrency primitives. Each
//! target owns the [`Engine`] and decides how control-rate [`Cmd`]s reach it: the
//! firmware enqueues them onto a critical-section ring drained by its audio task;
//! the web simulator applies them directly on the audio thread.
//!
//! ## Eval order
//! Nodes are evaluated in creation order. Wren builds graphs bottom-up (a node's
//! inputs are always created first → smaller ids), so creation order is a valid
//! topological order and no sorting is needed.

/// Maximum simultaneous nodes in the graph.
pub const MAX_NODES: usize = 64;
const SAMPLE_RATE: f32 = 44_100.0;
const DT: f32 = 1.0 / SAMPLE_RATE;
const PI: f32 = core::f32::consts::PI;

// Node kind codes (shared with the Wren bindings and the prelude's factory args).
pub const K_SINE: u8 = 0;
pub const K_SAW: u8 = 1;
pub const K_SQUARE: u8 = 2;
pub const K_TRI: u8 = 3;
pub const K_NOISE: u8 = 4;
pub const K_ENV: u8 = 5;
pub const K_LPF: u8 = 6;
pub const K_MUL: u8 = 7;
pub const K_ADD: u8 = 8;
pub const K_SUB: u8 = 9;

// Envelope stages.
const ST_IDLE: u8 = 0;
const ST_ATTACK: u8 = 1;
const ST_SUSTAIN: u8 = 2;
const ST_RELEASE: u8 = 3;

/// A node input: a constant, or another node's output (by id).
#[derive(Clone, Copy)]
pub enum Input {
    Const(f32),
    Node(u16),
}

impl Input {
    #[inline]
    fn eval(self, outs: &[f32; MAX_NODES]) -> f32 {
        match self {
            Input::Const(c) => c,
            Input::Node(i) => outs[i as usize],
        }
    }
}

#[derive(Clone, Copy)]
struct Node {
    kind: u8,
    a: Input, // freq / input / operand0
    b: Input, // release / cutoff / operand1
    phase: f32,
    z: f32, // filter state / env level
    stage: u8,
    oneshot: bool,
    rng: u32,
}

impl Node {
    const EMPTY: Node = Node {
        kind: 255,
        a: Input::Const(0.0),
        b: Input::Const(0.0),
        phase: 0.0,
        z: 0.0,
        stage: ST_IDLE,
        oneshot: false,
        rng: 0x2545_F491,
    };

    fn with(kind: u8, a: Input, b: Input) -> Node {
        Node { kind, a, b, ..Node::EMPTY }
    }

    #[inline]
    fn eval(&mut self, outs: &[f32; MAX_NODES]) -> f32 {
        match self.kind {
            K_SINE | K_SAW | K_SQUARE | K_TRI => {
                let f = self.a.eval(outs);
                self.phase += f * DT;
                self.phase -= libm_floorf(self.phase);
                match self.kind {
                    K_SINE => fast_sin(self.phase),
                    K_SAW => 2.0 * self.phase - 1.0,
                    K_SQUARE => {
                        if self.phase < 0.5 {
                            1.0
                        } else {
                            -1.0
                        }
                    }
                    _ => 1.0 - 4.0 * (self.phase - 0.5).abs(), // tri
                }
            }
            K_NOISE => {
                // xorshift32 → [-1, 1)
                let mut r = self.rng;
                r ^= r << 13;
                r ^= r >> 17;
                r ^= r << 5;
                self.rng = r;
                (r as i32 as f32) / (i32::MAX as f32)
            }
            K_ENV => {
                let atk = self.a.eval(outs).max(0.0001);
                let rel = self.b.eval(outs).max(0.0001);
                match self.stage {
                    ST_ATTACK => {
                        self.z += DT / atk;
                        if self.z >= 1.0 {
                            self.z = 1.0;
                            self.stage = if self.oneshot { ST_RELEASE } else { ST_SUSTAIN };
                        }
                    }
                    ST_SUSTAIN => self.z = 1.0,
                    ST_RELEASE => {
                        self.z -= DT / rel;
                        if self.z <= 0.0 {
                            self.z = 0.0;
                            self.stage = ST_IDLE;
                        }
                    }
                    _ => self.z = 0.0,
                }
                self.z
            }
            K_LPF => {
                let x = self.a.eval(outs);
                let fc = self.b.eval(outs).max(1.0);
                let c = (2.0 * PI * fc * DT).min(1.0);
                self.z += c * (x - self.z);
                self.z
            }
            K_MUL => self.a.eval(outs) * self.b.eval(outs),
            K_ADD => self.a.eval(outs) + self.b.eval(outs),
            K_SUB => self.a.eval(outs) - self.b.eval(outs),
            _ => 0.0,
        }
    }
}

/// Fast sine of a normalised phase `p` in `[0,1)` (≈ `sin(2π p)`), via the
/// classic parabola + correction — ~0.1% error, no `libm`/table.
#[inline]
fn fast_sin(p: f32) -> f32 {
    let mut x = 2.0 * PI * p;
    if x > PI {
        x -= 2.0 * PI;
    }
    const B: f32 = 4.0 / PI;
    const C: f32 = -4.0 / (PI * PI);
    let y = B * x + C * x * x.abs();
    0.225 * (y * y.abs() - y) + y
}

/// `floorf` without a libm dependency (phase is small + finite here).
#[inline]
fn libm_floorf(x: f32) -> f32 {
    let t = x as i32 as f32;
    if t > x { t - 1.0 } else { t }
}

// ── Control → audio command ──────────────────────────────────────────────────

/// A control-rate mutation of the graph, produced by the `Node` bindings and
/// applied to the [`Engine`] by whatever transport a target uses (firmware ring /
/// web direct apply). `Nop` is the ring-buffer filler.
#[derive(Clone, Copy)]
pub enum Cmd {
    Nop,
    NewNode { id: u16, kind: u8, a: Input, b: Input },
    SetInput { id: u16, port: u8, src: Input },
    Gate { id: u16, on: bool },
    Trigger { id: u16 },
    SetRoot { id: u16 },
    Reset,
}

// ── Engine ───────────────────────────────────────────────────────────────────

/// The signal graph. Owned by a single accessor per target (the firmware audio
/// task, or the web audio thread).
pub struct Engine {
    nodes: [Node; MAX_NODES],
    outs: [f32; MAX_NODES],
    live: usize,
    root: Option<u16>,
}

impl Default for Engine {
    fn default() -> Self {
        Self::new()
    }
}

impl Engine {
    pub const fn new() -> Self {
        Engine {
            nodes: [Node::EMPTY; MAX_NODES],
            outs: [0.0; MAX_NODES],
            live: 0,
            root: None,
        }
    }

    fn node_mut(&mut self, id: u16) -> Option<&mut Node> {
        let i = id as usize;
        if i < self.live { Some(&mut self.nodes[i]) } else { None }
    }

    /// Apply one control-rate command to the graph.
    pub fn apply(&mut self, cmd: Cmd) {
        match cmd {
            Cmd::NewNode { id, kind, a, b } => {
                let i = id as usize;
                if i < MAX_NODES {
                    self.nodes[i] = Node::with(kind, a, b);
                    if i + 1 > self.live {
                        self.live = i + 1;
                    }
                }
            }
            Cmd::SetInput { id, port, src } => {
                if let Some(n) = self.node_mut(id) {
                    if port == 0 {
                        n.a = src;
                    } else {
                        n.b = src;
                    }
                }
            }
            Cmd::Gate { id, on } => {
                if let Some(n) = self.node_mut(id) {
                    n.oneshot = false;
                    n.stage = if on { ST_ATTACK } else { ST_RELEASE };
                }
            }
            Cmd::Trigger { id } => {
                if let Some(n) = self.node_mut(id) {
                    n.oneshot = true;
                    n.stage = ST_ATTACK;
                }
            }
            Cmd::SetRoot { id } => self.root = Some(id),
            Cmd::Reset => {
                self.live = 0;
                self.root = None;
            }
            Cmd::Nop => {}
        }
    }

    /// Render one mono sample (the patched root's output, or silence).
    #[inline]
    pub fn render_frame(&mut self) -> f32 {
        for id in 0..self.live {
            // `nodes` and `outs` are disjoint fields → simultaneous borrows OK.
            let v = self.nodes[id].eval(&self.outs);
            self.outs[id] = v;
        }
        match self.root {
            Some(r) => self.outs[r as usize],
            None => 0.0,
        }
    }
}
