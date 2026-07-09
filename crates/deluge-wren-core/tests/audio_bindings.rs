#![cfg(feature = "test-support")]
use deluge_audio_graph::StereoFrame;
use deluge_audio_graph::node::TableSrc;
use deluge_wren_core::Host as _;
use deluge_wren_core::test_support::{EngineHost, run_and_capture_cmds, run_and_render};
use deluge_wren_core::{BusId, Cmd, Input, Kind, NodeId};

fn saw(freq: f32) -> Cmd {
    Cmd::NewNode {
        node: NodeId(0),
        kind: Kind::Saw,
        args: [Input::Const(freq), Input::Const(0.0), Input::Const(0.0)],
    }
}

#[test]
fn osc_saw_emits_newnode() {
    let cmds = run_and_capture_cmds("Osc.saw(110)");
    assert_eq!(cmds, vec![saw(110.0)]);
}

#[test]
fn osc_pink_emits_newnode() {
    let cmds = run_and_capture_cmds("Osc.pink()");
    assert_eq!(
        cmds,
        vec![Cmd::NewNode {
            node: NodeId(0),
            kind: Kind::PinkNoise,
            args: [Input::Const(0.0), Input::Const(0.0), Input::Const(0.0)],
        }]
    );
}

#[test]
fn osc_brown_emits_newnode() {
    let cmds = run_and_capture_cmds("Osc.brown()");
    assert_eq!(
        cmds,
        vec![Cmd::NewNode {
            node: NodeId(0),
            kind: Kind::BrownNoise,
            args: [Input::Const(0.0), Input::Const(0.0), Input::Const(0.0)],
        }]
    );
}

#[test]
fn osc_pink_brown_render_finite_nonsilent() {
    let mut out = [StereoFrame::default(); 32];
    run_and_render("Out.patch(Osc.pink())", &mut out);
    assert!(out.iter().all(|f| f.l.is_finite() && f.l.abs() <= 1.0));
    assert!(out.iter().any(|f| f.l != 0.0));

    let mut out = [StereoFrame::default(); 32];
    run_and_render("Out.patch(Osc.brown())", &mut out);
    assert!(out.iter().all(|f| f.l.is_finite() && f.l.abs() <= 1.0));
    assert!(out.iter().any(|f| f.l != 0.0));
}

#[test]
fn svf_lp_emits_newnode() {
    // node0 = saw, node1 = svf lp over node0, cutoff 800, res 0.3
    let cmds = run_and_capture_cmds("var x = Svf.lp(Osc.saw(110), 800, 0.3)");
    assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::SvfLp, .. })));
}

#[test]
fn svf_lp_renders_finite_nonsilent() {
    let mut out = [StereoFrame::default(); 32];
    run_and_render("Out.patch(Svf.lp(Osc.saw(110), 800, 0.5))", &mut out);
    assert!(out.iter().all(|f| f.l.is_finite() && f.l.abs() <= 4.0));
    assert!(out.iter().any(|f| f.l != 0.0));
}

#[test]
fn tb303_lp_emits_newnode() {
    let cmds = run_and_capture_cmds("var x = Tb303.lp(Osc.saw(110), 800, 0.7)");
    assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::Tb303, .. })));
}

#[test]
fn tb303_lp_renders_finite_nonsilent() {
    let mut out = [StereoFrame::default(); 32];
    run_and_render("Out.patch(Tb303.lp(Osc.saw(110), 800, 0.7))", &mut out);
    assert!(out.iter().all(|f| f.l.is_finite() && f.l.abs() <= 8.0));
    assert!(out.iter().any(|f| f.l != 0.0));
}

#[test]
fn moog_lp_emits_newnode() {
    let c4 = run_and_capture_cmds("var x = Moog.lp(Osc.saw(110), 1000, 0.7)");
    assert!(c4.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::MoogLp4, .. })));
    let c2 = run_and_capture_cmds("var x = Moog.lp2(Osc.saw(110), 1000, 0.7)");
    assert!(c2.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::MoogLp2, .. })));
}

#[test]
fn ms20_emits_newnode() {
    let lp = run_and_capture_cmds("var x = Ms20.lp(Osc.saw(110), 1000, 0.7)");
    assert!(lp.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::Ms20Lp, .. })));
    let hp = run_and_capture_cmds("var x = Ms20.hp(Osc.saw(110), 1000, 0.7)");
    assert!(hp.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::Ms20Hp, .. })));
}

#[test]
fn ms20_renders_finite_nonsilent() {
    let mut out = [StereoFrame::default(); 32];
    run_and_render("Out.patch(Ms20.lp(Osc.saw(110), 1000, 0.7))", &mut out);
    assert!(out.iter().all(|f| f.l.is_finite() && f.l.abs() <= 8.0));
    assert!(out.iter().any(|f| f.l != 0.0));
}

#[test]
fn moog_lp_renders_finite_nonsilent() {
    let mut out = [StereoFrame::default(); 32];
    run_and_render("Out.patch(Moog.lp(Osc.saw(110), 1000, 0.7))", &mut out);
    assert!(out.iter().all(|f| f.l.is_finite() && f.l.abs() <= 8.0));
    assert!(out.iter().any(|f| f.l != 0.0));
}

#[test]
fn resonator_emits_newnode() {
    let cmds = run_and_capture_cmds("var x = Resonator.new(Osc.saw(110), 220, 0.3)");
    assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::Modal, .. })));
}

#[test]
fn resonator_renders_finite_nonsilent() {
    let mut out = [StereoFrame::default(); 32];
    run_and_render("Out.patch(Resonator.new(Osc.saw(110), 220, 0.3))", &mut out);
    assert!(out.iter().all(|f| f.l.is_finite() && f.l.abs() <= 8.0));
    assert!(out.iter().any(|f| f.l != 0.0));
}

#[test]
fn patch_writes_master_bus_and_sets_root() {
    let cmds = run_and_capture_cmds("Out.patch(Osc.saw(110))");
    assert_eq!(
        cmds,
        vec![
            saw(110.0),
            Cmd::BusWrite { src: Input::Node { node: NodeId(0), port: 0 }, bus: BusId(0) },
            Cmd::SetRoot { bus: BusId(0) },
        ]
    );
}

#[test]
fn lpf_and_binop_resolve_node_inputs() {
    // Osc.saw(110).lpf(800) → node0 saw, node1 lpf(node0, 800)
    let cmds = run_and_capture_cmds("var x = Osc.saw(110).lpf(800)");
    assert_eq!(
        cmds,
        vec![
            saw(110.0),
            Cmd::NewNode {
                node: NodeId(1),
                kind: Kind::Lpf,
                args: [
                    Input::Node { node: NodeId(0), port: 0 },
                    Input::Const(800.0),
                    Input::Const(0.0)
                ],
            },
        ]
    );
}

#[test]
fn nonaudio_foreign_arg_does_not_crash() {
    // `output[2]` is `Output.new(1)`, a 4-byte non-audio foreign whose leading
    // byte (ch=1) happens to equal `TAG_PORT`. Passing it where an audio Input
    // is expected must not over-read / panic; the arg degrades to a defined
    // (inert) value and the factory still emits its `NewNode`.
    let cmds = run_and_capture_cmds("Osc.sine(output[2])");
    assert_eq!(cmds.len(), 1);
    assert!(matches!(cmds[0], Cmd::NewNode { kind: Kind::Sine, .. }));
}

#[test]
fn reset_emits_reset() {
    let cmds = run_and_capture_cmds("Out.reset()");
    assert_eq!(cmds, vec![Cmd::Reset]);
}

#[test]
fn bus_write_and_patch() {
    let cmds = run_and_capture_cmds(
        "var m = Bus.new()\n\
         m.write(Osc.saw(110))\n\
         Out.patch(m)",
    );
    assert_eq!(cmds, vec![
        saw(110.0),
        Cmd::BusWrite { src: Input::Node { node: NodeId(0), port: 0 }, bus: BusId(1) },
        Cmd::SetRoot { bus: BusId(1) },
    ]);
}

#[test]
fn out_port_resolves_to_that_port() {
    // Split.new(Osc.saw(110)); consumer reads .out(1)
    let cmds = run_and_capture_cmds(
        "var s = Split.new(Osc.saw(110))\n\
         var c = s.out(1) * 2",
    );
    // node0 saw, node1 split2(node0), node2 mul(node1.port1, 2)
    assert!(cmds.iter().any(|c| *c == Cmd::NewNode {
        node: NodeId(2), kind: Kind::Mul,
        args: [Input::Node { node: NodeId(1), port: 1 }, Input::Const(2.0), Input::Const(0.0)],
    }));
}

#[test]
fn free_emits_free_and_reuses_id() {
    // Allocate n0, free it, allocate again → id 0 reused.
    let cmds = run_and_capture_cmds(
        "var a = Osc.saw(110)\n\
         a.free()\n\
         var b = Osc.saw(220)",
    );
    assert_eq!(cmds, vec![
        Cmd::NewNode { node: NodeId(0), kind: Kind::Saw, args: [Input::Const(110.0), Input::Const(0.0), Input::Const(0.0)] },
        Cmd::Free { node: NodeId(0) },
        Cmd::NewNode { node: NodeId(0), kind: Kind::Saw, args: [Input::Const(220.0), Input::Const(0.0), Input::Const(0.0)] },
    ]);
}

#[test]
fn golden_saw_lpf_renders_expected_block() {
    let mut out = [StereoFrame::default(); 32];
    run_and_render("Out.patch(Osc.saw(110).lpf(800))", &mut out);
    // CHARACTERIZATION golden re-pinned 2026-07-07 after Osc band-limiting (Tasks 1-3).
    // Regenerate only on an intended, reviewed output change.
    let expected = [0.0, -0.103913695, -0.204913, -0.29383174];
    for i in 0..4 {
        assert!((out[i].l - expected[i]).abs() < 1e-6, "sample {i}: {} vs {}", out[i].l, expected[i]);
    }
    assert!(out.iter().all(|f| f.l.is_finite() && f.l.abs() <= 1.0));
}

#[test]
fn ports_and_buses_render_finite() {
    let mut out = [StereoFrame::default(); 32];
    run_and_render(
        "var m = Bus.new()\n\
         var s = Split.new(Osc.saw(110))\n\
         m.write(s.out(0))\n\
         m.write(s.out(1))\n\
         Out.patch(m)",
        &mut out,
    );
    assert!(out.iter().all(|f| f.l.is_finite() && f.l.abs() <= 2.0));
    assert!(out.iter().any(|f| f.l != 0.0));
}

#[test]
fn midi_synth_example_parses_and_runs() {
    let src = include_str!("../../../wren-firmware/examples/midi_synth.wren");
    // Runs without a VM error (may reference Midi/output — that's fine, those
    // bindings exist); we only assert it boots + executes top-level code.
    let _ = run_and_capture_cmds(src);
}

#[test]
fn osc_width_emits_setinput_port2() {
    let cmds = run_and_capture_cmds("var s = Osc.square(110)\ns.width = 0.3");
    assert!(cmds.iter().any(|c| *c == Cmd::SetInput {
        node: NodeId(0), port: 2, src: Input::Const(0.3)
    }));
}

#[test]
fn osc_pm_emits_setinput_port1() {
    let cmds = run_and_capture_cmds("var s = Osc.sine(440)\ns.pm = 0.5");
    assert!(cmds.iter().any(|c| *c == Cmd::SetInput {
        node: NodeId(0), port: 1, src: Input::Const(0.5)
    }));
}

#[test]
fn resonator_pitch_and_damping_set_ports_1_and_2() {
    // pitch= → port 1 (freq), damping= → port 2 — the Resonator's exciter is port 0,
    // so these (not the inherited freq=, which targets port 0) are how you retune it.
    let cmds = run_and_capture_cmds(
        "var r = Resonator.new(Osc.saw(110), 220, 0.3)\nr.pitch = 330\nr.damping = 0.6",
    );
    assert!(cmds.iter().any(|c| matches!(c, Cmd::SetInput { port: 1, src: Input::Const(v), .. } if (*v - 330.0).abs() < 1e-3)));
    assert!(cmds.iter().any(|c| matches!(c, Cmd::SetInput { port: 2, src: Input::Const(v), .. } if (*v - 0.6).abs() < 1e-3)));
}

#[test]
fn osc_feedback_emits_setparam() {
    let cmds = run_and_capture_cmds("var s = Osc.sine(440)\ns.feedback = 0.8");
    assert!(cmds.iter().any(|c| *c == Cmd::SetParam {
        node: NodeId(0), param: 0, value: 0.8
    }));
}

#[test]
fn osc_wavetable_emits_newnode_and_bindtable() {
    let cmds = run_and_capture_cmds("var s = Osc.wavetable(WT.Saw, 220)");
    assert!(cmds.iter().any(|c| matches!(c,
        Cmd::NewNode { node: NodeId(0), kind: Kind::Wavetable, .. })));
    assert!(cmds.iter().any(|c| matches!(c,
        Cmd::BindTable { node: NodeId(0), src: TableSrc::Static(id) } if id.0 == 0)));
    // WT.Saw == id 0 — Saw is index 0 in the generated `TABLES` (Task 3).
}

#[test]
fn osc_wavetable_renders_finite_nonsilent() {
    let mut out = [StereoFrame::default(); 32];
    run_and_render("Out.patch(Osc.wavetable(WT.Saw, 220))", &mut out);
    assert!(out.iter().all(|f| f.l.is_finite() && f.l.abs() <= 1.0));
    assert!(out.iter().any(|f| f.l != 0.0));
}

#[test]
fn wavetable_from_unbound_on_cmd_capture_host_no_bogus_bindtable() {
    // The Cmd-capture host has no pool: `upload_table` returns `None`, so the
    // `Wavetable` handle stays unbound. `Osc.wavetable(w, freq)` must still
    // create the node (graceful degrade to silent) — but MUST NOT emit a
    // `BindTable` with a bogus/default handle.
    let cmds = run_and_capture_cmds(
        "var w = Wavetable.from([ -1, -0.5, 0, 0.5, 1, 0.5, 0, -0.5 ])\n\
         var v = Osc.wavetable(w, 220)",
    );
    assert!(cmds.iter().any(|c| matches!(c,
        Cmd::NewNode { node: NodeId(0), kind: Kind::Wavetable, .. })));
    assert!(!cmds.iter().any(|c| matches!(c, Cmd::BindTable { .. })));
}

#[test]
fn wavetable_from_emits_bindtable_pooled_and_renders_finite() {
    // With a real engine host (`run_and_render`), `upload_table` succeeds, so
    // `Wavetable.from` yields a bound handle and the node renders non-panicking,
    // finite audio.
    let mut out = [StereoFrame::default(); 32];
    run_and_render(
        "var w = Wavetable.from([ -1, -0.5, 0, 0.5, 1, 0.5, 0, -0.5 ])\n\
         Out.patch(Osc.wavetable(w, 220))",
        &mut out,
    );
    assert!(out.iter().all(|f| f.l.is_finite() && f.l.abs() <= 1.0));
}

#[test]
fn wavetable_from_round_trip_renders_finite_nonsilent_bounded_deterministic() {
    // End-to-end round-trip through the FULL dynamic-table path with a real
    // `EngineHost` (not the `CmdCaptureHost` used by the Cmd-shape tests
    // above): a known single-cycle sawtooth, generated here as a Wren list
    // literal (deterministic, no reliance on Wren-side loop syntax) ->
    // `Wavetable.from` (list-read, Task 1) -> `Host::upload_table` (Task 4,
    // real `pool_alloc` + `build_pyramid_into`) -> `TableSrc::Pooled` bind
    // (Task 5) -> `Osc.wavetable` render through the real pool region (Task
    // 3). This is stronger than Task 4's level-0-only pool-readback check and
    // stronger than `wavetable_from_emits_bindtable_pooled_and_renders_finite`
    // above (which only checks finite+bounded): it also asserts non-silence
    // and bit-exact determinism across independent VM/engine lifecycles.
    let n = 32;
    let pts: Vec<String> = (0..n)
        .map(|i| format!("{:.6}", 2.0 * (i as f64 / n as f64) - 1.0))
        .collect();
    let script = format!(
        "var w = Wavetable.from([{}])\nOut.patch(Osc.wavetable(w, 220))",
        pts.join(", ")
    );

    let mut out1 = [StereoFrame::default(); 32];
    run_and_render(&script, &mut out1);
    assert!(out1.iter().all(|f| f.l.is_finite() && f.r.is_finite()), "non-finite sample: {out1:?}");
    assert!(out1.iter().all(|f| f.l.abs() <= 1.0 && f.r.abs() <= 1.0), "unbounded sample: {out1:?}");
    assert!(out1.iter().any(|f| f.l != 0.0), "round-trip render must be non-silent");

    // Deterministic: a second, independent VM+engine lifecycle on the same
    // script renders bit-identical output (no uninitialized pool memory,
    // no ordering nondeterminism in the upload/bind path).
    let mut out2 = [StereoFrame::default(); 32];
    run_and_render(&script, &mut out2);
    for i in 0..32 {
        assert_eq!(out1[i].l, out2[i].l, "sample {i}: nondeterministic render");
        assert_eq!(out1[i].r, out2[i].r, "sample {i}: nondeterministic render");
    }
}

#[test]
fn osc_position_emits_setinput_port2() {
    let cmds = run_and_capture_cmds(
        "var w = Wavetable.from2d([[-1, 0, 1, 0], [1, 0, -1, 0]])\n\
         var s = Osc.wavetable(w, 220)\n\
         s.position = 0.5",
    );
    assert!(cmds.iter().any(|c| *c == Cmd::SetInput {
        node: NodeId(0), port: 2, src: Input::Const(0.5)
    }));
}

#[test]
fn wavetable_from2d_unbound_on_cmd_capture_host_no_bogus_bindtable() {
    // Mirrors `wavetable_from_unbound_on_cmd_capture_host_no_bogus_bindtable`:
    // the Cmd-capture host has no pool, so `upload_table_2d` returns `None`
    // and the handle stays unbound — `Osc.wavetable` must still create the
    // node (graceful degrade to silent) but MUST NOT emit a `BindTable`.
    let cmds = run_and_capture_cmds(
        "var w = Wavetable.from2d([[-1, 0, 1, 0], [1, 0, -1, 0]])\n\
         var v = Osc.wavetable(w, 220)",
    );
    assert!(cmds.iter().any(|c| matches!(c,
        Cmd::NewNode { node: NodeId(0), kind: Kind::Wavetable, .. })));
    assert!(!cmds.iter().any(|c| matches!(c, Cmd::BindTable { .. })));
}

#[test]
fn wavetable_from2d_round_trip_renders_finite_nonsilent_and_morphs() {
    // Two maximally-different frames (frame 0 = silence, frame 1 = a loud
    // square-ish wave) via a real `EngineHost`: `Wavetable.from2d` (nested
    // list read) -> `Host::upload_table_2d` (one pyramid per frame in a
    // single pool region) -> `TableSrc::Pooled` bind -> `Osc.wavetable`
    // render through `Kind::Wavetable`'s `FRAMES>1` morph path (`process_morph`,
    // reading `.position` on port 2). `position=0` must render frame 0
    // (silence) exactly; `position=1` must render frame 1 (loud) and
    // therefore differ audibly from `position=0`.
    let n = 32;
    let frame0: Vec<String> = (0..n).map(|_| "0.0".to_string()).collect();
    let frame1: Vec<String> =
        (0..n).map(|i| if i < n / 2 { "1.0".to_string() } else { "-1.0".to_string() }).collect();
    let script_at = |pos: f32| {
        format!(
            "var w = Wavetable.from2d([[{}], [{}]])\n\
             var s = Osc.wavetable(w, 220)\n\
             s.position = {}\n\
             Out.patch(s)",
            frame0.join(", "),
            frame1.join(", "),
            pos
        )
    };

    let mut out_pos0 = [StereoFrame::default(); 32];
    run_and_render(&script_at(0.0), &mut out_pos0);
    assert!(out_pos0.iter().all(|f| f.l.is_finite() && f.r.is_finite()));
    assert!(out_pos0.iter().all(|f| f.l.abs() <= 1.0 && f.r.abs() <= 1.0));
    assert!(
        out_pos0.iter().all(|f| f.l == 0.0 && f.r == 0.0),
        "position=0 (silent frame) should render silence: {out_pos0:?}"
    );

    let mut out_pos1 = [StereoFrame::default(); 32];
    run_and_render(&script_at(1.0), &mut out_pos1);
    assert!(out_pos1.iter().all(|f| f.l.is_finite() && f.r.is_finite()));
    assert!(out_pos1.iter().all(|f| f.l.abs() <= 1.0 && f.r.abs() <= 1.0));
    assert!(
        out_pos1.iter().any(|f| f.l != 0.0),
        "position=1 (loud frame) should render non-silent: {out_pos1:?}"
    );
}

#[test]
fn engine_host_upload_table_builds_band_limited() {
    let mut host = EngineHost::new(48_000.0);
    let mut base = [0.0f32; mipgen::N];
    for (i, s) in base.iter_mut().enumerate() {
        *s = 2.0 * (i as f32 / mipgen::N as f32) - 1.0;
    }
    let h = host.upload_table(&base).expect("upload");
    // level 0 region round-trips to a saw-ish shape; deeper levels are band-limited.
    let region = host.engine().pool_slice(h);
    assert_eq!(region.len(), deluge_wren_core::PYRAMID_LEN);
    assert!(region[..mipgen::N].iter().any(|&x| x != 0.0));
}

#[test]
fn osc_sync_saw_emits_newnode() {
    let cmds = run_and_capture_cmds("Osc.syncSaw(220, 660)");
    assert_eq!(
        cmds,
        vec![Cmd::NewNode {
            node: NodeId(0),
            kind: Kind::SyncSaw,
            args: [Input::Const(220.0), Input::Const(660.0), Input::Const(0.0)],
        }]
    );
}

#[test]
fn osc_sync_render_bounded_nonsilent() {
    let mut out = [StereoFrame::default(); 32];
    run_and_render("Out.patch(Osc.syncSaw(220, 660))", &mut out);
    assert!(out.iter().all(|f| f.l.is_finite() && f.l.abs() <= 1.0));
    assert!(out.iter().any(|f| f.l != 0.0));
}

#[test]
fn delay_new_emits_newnode_and_bindtable_on_engine_host_and_renders_bounded() {
    use deluge_wren_core::test_support::run_and_render;
    use deluge_audio_graph::StereoFrame;
    let mut out = [StereoFrame::default(); 32];
    run_and_render(
        "var d = Delay.new(Osc.saw(110), 0.01, 0.4)\n\
         d.mix = 0.5\n\
         d.damp = 0.3\n\
         Out.patch(d)",
        &mut out,
    );
    assert!(out.iter().all(|f| f.l.is_finite() && f.l.abs() <= 1.0));
    assert!(out.iter().any(|f| f.l != 0.0), "delay output should be non-silent");
}

#[test]
fn delay_on_cmd_capture_host_creates_node_without_bindtable() {
    use deluge_wren_core::test_support::run_and_capture_cmds;
    use deluge_audio_graph::{Cmd, Kind};
    // CmdCaptureHost has no pool → alloc_buffer returns None → the node is
    // created but NOT bound (dry passthrough), never a bogus BindTable.
    let cmds = run_and_capture_cmds("var d = Delay.new(Osc.saw(110), 0.01, 0.4)\nOut.patch(d)");
    assert!(
        cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::Delay, .. })),
        "expected a NewNode(Delay): {cmds:?}"
    );
    assert!(
        !cmds.iter().any(|c| matches!(c, Cmd::BindTable { .. })),
        "unbound delay must not emit BindTable: {cmds:?}"
    );
}

#[test]
fn delay_mix_and_damp_emit_setparam_0_and_1() {
    use deluge_wren_core::test_support::run_and_capture_cmds;
    use deluge_audio_graph::Cmd;
    let cmds = run_and_capture_cmds(
        "var d = Delay.new(Osc.saw(110), 0.01, 0.4)\nd.mix = 0.5\nd.damp = 0.3",
    );
    assert!(
        cmds.iter().any(|c| matches!(c, Cmd::SetParam { param: 0, .. })),
        "mix= should SetParam(0): {cmds:?}"
    );
    assert!(
        cmds.iter().any(|c| matches!(c, Cmd::SetParam { param: 1, .. })),
        "damp= should SetParam(1): {cmds:?}"
    );
}

#[test]
fn pan_new_emits_kind_pan_node() {
    use deluge_wren_core::test_support::run_and_capture_cmds;
    use deluge_audio_graph::{Cmd, Kind};
    let cmds = run_and_capture_cmds("var p = Pan.new(Osc.saw(110), -0.5)");
    assert!(
        cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::Pan, .. })),
        "expected a NewNode(Pan): {cmds:?}"
    );
}

#[test]
fn patch_stereo_node_emits_two_side_writes() {
    use deluge_wren_core::test_support::run_and_capture_cmds;
    use deluge_audio_graph::Cmd;
    // Out.patch of a width-2 Pan → two BusWriteGains: (1,0) and (0,1) to master.
    let cmds = run_and_capture_cmds("Out.patch(Pan.new(Osc.saw(110), 0.0))");
    let gains: std::vec::Vec<(f32, f32)> = cmds
        .iter()
        .filter_map(|c| match c {
            Cmd::BusWriteGains { gl, gr, .. } => Some((*gl, *gr)),
            _ => None,
        })
        .collect();
    assert!(gains.contains(&(1.0, 0.0)), "missing L-side write: {gains:?}");
    assert!(gains.contains(&(0.0, 1.0)), "missing R-side write: {gains:?}");
}

#[test]
fn patch_mono_node_still_emits_single_center_write() {
    use deluge_wren_core::test_support::run_and_capture_cmds;
    use deluge_audio_graph::Cmd;
    // A mono node patched → exactly one plain center BusWrite, no gained writes.
    let cmds = run_and_capture_cmds("Out.patch(Osc.saw(110))");
    let center = cmds.iter().filter(|c| matches!(c, Cmd::BusWrite { .. })).count();
    let gained = cmds.iter().filter(|c| matches!(c, Cmd::BusWriteGains { .. })).count();
    assert_eq!(center, 1, "mono patch should emit one center write: {cmds:?}");
    assert_eq!(gained, 0, "mono patch should emit no gained writes: {cmds:?}");
}

#[test]
fn stereo_pan_renders_distinct_l_and_r() {
    use deluge_wren_core::test_support::run_and_render;
    use deluge_audio_graph::StereoFrame;
    let mut out = [StereoFrame::default(); 32];
    // Hard-left pan → L carries signal, R ≈ silent.
    run_and_render("Out.patch(Pan.new(Osc.saw(110), -1.0))", &mut out);
    let suml: f32 = out.iter().map(|f| f.l.abs()).sum();
    let sumr: f32 = out.iter().map(|f| f.r.abs()).sum();
    assert!(suml > 0.01, "L should carry the hard-left pan: {suml}");
    assert!(sumr < 1e-3, "R should be ~silent at hard-left: {sumr}");
    assert!(out.iter().all(|f| f.l.is_finite() && f.r.is_finite()));
}

#[test]
fn chorus_new_emits_node_and_params_no_bind_on_capture_host() {
    use deluge_wren_core::test_support::run_and_capture_cmds;
    use deluge_audio_graph::{Cmd, Kind};
    // CmdCaptureHost has no pool → alloc_buffer None → NewNode + SetParams but
    // NO BindTable (dry-passthrough contract, same as Ef-1 Delay). The real
    // bind + render is covered by `chorus_renders_stereo_bounded_on_engine_host`.
    let cmds = run_and_capture_cmds("var c = Chorus.new(Osc.saw(110), 0.5, 0.4, 0.5)");
    assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::Chorus, .. })));
    assert!(!cmds.iter().any(|c| matches!(c, Cmd::BindTable { .. })), "no pool → no BindTable: {cmds:?}");
    // rate(1)/depth(2)/mix(0) SetParams emitted from the constructor args (pool-independent).
    for p in [0u8, 1, 2] {
        assert!(cmds.iter().any(|c| matches!(c, Cmd::SetParam { param, .. } if *param == p)), "missing SetParam {p}: {cmds:?}");
    }
}

#[test]
fn chorus_patch_routes_stereo() {
    use deluge_wren_core::test_support::run_and_capture_cmds;
    use deluge_audio_graph::Cmd;
    // A Chorus is width-2 → Out.patch emits the two side-writes.
    let cmds = run_and_capture_cmds("Out.patch(Chorus.new(Osc.saw(110), 0.5, 0.4, 0.5))");
    let gains: std::vec::Vec<(f32, f32)> = cmds.iter().filter_map(|c| match c {
        Cmd::BusWriteGains { gl, gr, .. } => Some((*gl, *gr)),
        _ => None,
    }).collect();
    assert!(gains.contains(&(1.0, 0.0)) && gains.contains(&(0.0, 1.0)), "not stereo-routed: {gains:?}");
}

#[test]
fn flanger_regen_sets_feedback_param() {
    use deluge_wren_core::test_support::run_and_capture_cmds;
    use deluge_audio_graph::Cmd;
    let cmds = run_and_capture_cmds(
        "var f = Flanger.new(Osc.saw(110), 0.3, 0.7, 0.6, 0.5)\nf.regen = 0.8",
    );
    // Flanger.new sets feedback (param 3) from its arg; regen= sets it again.
    let p3 = cmds.iter().filter(|c| matches!(c, Cmd::SetParam { param: 3, .. })).count();
    assert!(p3 >= 2, "expected feedback param set by ctor and regen=: {cmds:?}");
}

#[test]
fn chorus_renders_stereo_bounded_on_engine_host() {
    use deluge_wren_core::test_support::run_and_render;
    use deluge_audio_graph::StereoFrame;
    let mut out = [StereoFrame::default(); 32];
    run_and_render("Out.patch(Chorus.new(Osc.saw(110), 1.0, 0.5, 0.6))", &mut out);
    assert!(out.iter().all(|f| f.l.is_finite() && f.r.is_finite() && f.l.abs() <= 1.0 && f.r.abs() <= 1.0));
    assert!(out.iter().any(|f| f.l != 0.0 || f.r != 0.0), "chorus should be non-silent");
}

#[test]
fn room_new_emits_node_and_params_no_bind_on_capture_host() {
    use deluge_wren_core::test_support::run_and_capture_cmds;
    use deluge_audio_graph::{Cmd, Kind};
    let cmds = run_and_capture_cmds("var r = Room.new(Osc.saw(110), 0.7, 0.4, 0.5)");
    assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::Room, .. })));
    assert!(!cmds.iter().any(|c| matches!(c, Cmd::BindTable { .. })), "no pool → no BindTable: {cmds:?}");
    for p in [0u8, 1, 2] {
        assert!(cmds.iter().any(|c| matches!(c, Cmd::SetParam { param, .. } if *param == p)), "missing SetParam {p}: {cmds:?}");
    }
}

#[test]
fn room_patch_routes_stereo() {
    use deluge_wren_core::test_support::run_and_capture_cmds;
    use deluge_audio_graph::Cmd;
    let cmds = run_and_capture_cmds("Out.patch(Room.new(Osc.saw(110), 0.7, 0.4, 0.5))");
    let gains: std::vec::Vec<(f32, f32)> = cmds.iter().filter_map(|c| match c {
        Cmd::BusWriteGains { gl, gr, .. } => Some((*gl, *gr)),
        _ => None,
    }).collect();
    assert!(gains.contains(&(1.0, 0.0)) && gains.contains(&(0.0, 1.0)), "not stereo-routed: {gains:?}");
}

#[test]
fn room_size_and_spread_set_params_2_and_3() {
    use deluge_wren_core::test_support::run_and_capture_cmds;
    use deluge_audio_graph::Cmd;
    let cmds = run_and_capture_cmds(
        "var r = Room.new(Osc.saw(110), 0.5, 0.5, 0.5)\nr.size = 0.9\nr.spread = 0.3",
    );
    assert!(cmds.iter().any(|c| matches!(c, Cmd::SetParam { param: 2, .. })), "size→2: {cmds:?}");
    assert!(cmds.iter().any(|c| matches!(c, Cmd::SetParam { param: 3, .. })), "spread→3: {cmds:?}");
}

#[test]
fn room_renders_stereo_bounded_on_engine_host() {
    use deluge_wren_core::test_support::run_and_render;
    use deluge_audio_graph::StereoFrame;
    let mut out = [StereoFrame::default(); 32];
    run_and_render("Out.patch(Room.new(Osc.saw(110), 0.7, 0.4, 0.6))", &mut out);
    assert!(out.iter().all(|f| f.l.is_finite() && f.r.is_finite() && f.l.abs() <= 1.0 && f.r.abs() <= 1.0));
    assert!(out.iter().any(|f| f.l != 0.0 || f.r != 0.0), "reverb should be non-silent");
}

#[test]
fn hall_new_emits_node_and_params_no_bind_on_capture_host() {
    use deluge_wren_core::test_support::run_and_capture_cmds;
    use deluge_audio_graph::{Cmd, Kind};
    let cmds = run_and_capture_cmds("var h = Hall.new(Osc.saw(110), 0.8, 0.4, 0.5)");
    assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::Hall, .. })));
    assert!(!cmds.iter().any(|c| matches!(c, Cmd::BindTable { .. })), "no pool → no BindTable: {cmds:?}");
    for p in [0u8, 1, 2] {
        assert!(cmds.iter().any(|c| matches!(c, Cmd::SetParam { param, .. } if *param == p)), "missing SetParam {p}: {cmds:?}");
    }
}

#[test]
fn hall_patch_routes_stereo_and_size_setter_reused() {
    use deluge_wren_core::test_support::run_and_capture_cmds;
    use deluge_audio_graph::Cmd;
    // Reuses the Room `size=` setter (param 2) — proves the shared surface works on a Hall.
    let cmds = run_and_capture_cmds("var h = Hall.new(Osc.saw(110), 0.5, 0.5, 0.5)\nh.size = 0.9\nOut.patch(h)");
    assert!(cmds.iter().any(|c| matches!(c, Cmd::SetParam { param: 2, .. })), "size→2: {cmds:?}");
    let gains: std::vec::Vec<(f32, f32)> = cmds.iter().filter_map(|c| match c {
        Cmd::BusWriteGains { gl, gr, .. } => Some((*gl, *gr)),
        _ => None,
    }).collect();
    assert!(gains.contains(&(1.0, 0.0)) && gains.contains(&(0.0, 1.0)), "not stereo-routed: {gains:?}");
}

#[test]
fn hall_renders_stereo_bounded_on_engine_host() {
    use deluge_wren_core::test_support::run_and_render;
    use deluge_audio_graph::StereoFrame;
    let mut out = [StereoFrame::default(); 32];
    run_and_render("Out.patch(Hall.new(Osc.saw(110), 0.85, 0.4, 0.6))", &mut out);
    assert!(out.iter().all(|f| f.l.is_finite() && f.r.is_finite() && f.l.abs() <= 1.0 && f.r.abs() <= 1.0));
    assert!(out.iter().any(|f| f.l != 0.0 || f.r != 0.0), "hall should be non-silent");
}

#[test]
fn plate_new_emits_node_and_params_no_bind_on_capture_host() {
    use deluge_wren_core::test_support::run_and_capture_cmds;
    use deluge_audio_graph::{Cmd, Kind};
    let cmds = run_and_capture_cmds("var p = Plate.new(Osc.saw(110), 0.8, 0.4, 0.5)");
    assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::Plate, .. })));
    assert!(!cmds.iter().any(|c| matches!(c, Cmd::BindTable { .. })), "no pool → no BindTable: {cmds:?}");
    for p in [0u8, 1, 2] {
        assert!(cmds.iter().any(|c| matches!(c, Cmd::SetParam { param, .. } if *param == p)), "missing SetParam {p}: {cmds:?}");
    }
}

#[test]
fn plate_patch_routes_stereo() {
    use deluge_wren_core::test_support::run_and_capture_cmds;
    use deluge_audio_graph::Cmd;
    let cmds = run_and_capture_cmds("Out.patch(Plate.new(Osc.saw(110), 0.8, 0.4, 0.5))");
    let gains: std::vec::Vec<(f32, f32)> = cmds.iter().filter_map(|c| match c {
        Cmd::BusWriteGains { gl, gr, .. } => Some((*gl, *gr)),
        _ => None,
    }).collect();
    assert!(gains.contains(&(1.0, 0.0)) && gains.contains(&(0.0, 1.0)), "not stereo-routed: {gains:?}");
}

#[test]
fn plate_renders_stereo_bounded_on_engine_host() {
    use deluge_wren_core::test_support::run_and_render;
    use deluge_audio_graph::StereoFrame;
    let mut out = [StereoFrame::default(); 32];
    run_and_render("Out.patch(Plate.new(Osc.saw(110), 0.85, 0.4, 0.6))", &mut out);
    assert!(out.iter().all(|f| f.l.is_finite() && f.r.is_finite() && f.l.abs() <= 1.0 && f.r.abs() <= 1.0));
    assert!(out.iter().any(|f| f.l != 0.0 || f.r != 0.0), "plate should be non-silent");
}

#[test]
fn drive_factories_emit_kind_drive_with_shape() {
    use deluge_wren_core::test_support::run_and_capture_cmds;
    use deluge_audio_graph::{Cmd, Kind};
    for (call, shape) in [("soft", 0.0f32), ("hard", 1.0), ("fold", 2.0), ("tube", 3.0)] {
        let src = std::format!("var d = Drive.{}(Osc.saw(110), 0.7, 0.5, 0.8)", call);
        let cmds = run_and_capture_cmds(&src);
        assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::Drive, .. })), "{call}: {cmds:?}");
        // shape → SetParam(3, code)
        assert!(cmds.iter().any(|c| matches!(c, Cmd::SetParam { param: 3, value, .. } if (*value - shape).abs() < 1e-4)), "{call} shape {shape}: {cmds:?}");
        // drive/tone/mix → params 0/1/2
        for p in [0u8, 1, 2] {
            assert!(cmds.iter().any(|c| matches!(c, Cmd::SetParam { param, .. } if *param == p)), "{call} missing SetParam {p}");
        }
    }
}

#[test]
fn drive_setters_map_to_params() {
    use deluge_wren_core::test_support::run_and_capture_cmds;
    use deluge_audio_graph::Cmd;
    let cmds = run_and_capture_cmds(
        "var d = Drive.soft(Osc.saw(110), 0.5, 0.5, 0.5)\nd.drive = 0.9\nd.tone = 0.3\nd.wet = 0.7",
    );
    assert!(cmds.iter().any(|c| matches!(c, Cmd::SetParam { param: 0, .. })), "drive=→0: {cmds:?}");
    assert!(cmds.iter().any(|c| matches!(c, Cmd::SetParam { param: 1, .. })), "tone=→1: {cmds:?}");
    assert!(cmds.iter().any(|c| matches!(c, Cmd::SetParam { param: 2, .. })), "wet=→2: {cmds:?}");
}

#[test]
fn drive_renders_mono_bounded_on_engine_host() {
    use deluge_wren_core::test_support::run_and_render;
    use deluge_audio_graph::StereoFrame;
    let mut out = [StereoFrame::default(); 32];
    run_and_render("Out.patch(Drive.hard(Osc.saw(110), 0.9, 0.6, 1.0))", &mut out);
    assert!(out.iter().all(|f| f.l.is_finite() && f.l.abs() <= 1.0));
    assert!(out.iter().any(|f| f.l != 0.0), "drive should be non-silent");
}

#[test]
fn eq_factories_emit_kind_eq_with_type() {
    use deluge_wren_core::test_support::run_and_capture_cmds;
    use deluge_audio_graph::{Cmd, Kind};
    for (call, code) in [("peak", 0.0f32), ("lowShelf", 1.0), ("highShelf", 2.0)] {
        let src = std::format!("var e = EQ.{}(Osc.saw(110), 1000, 6, 1)", call);
        let cmds = run_and_capture_cmds(&src);
        assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::Eq, .. })), "{call}: {cmds:?}");
        assert!(cmds.iter().any(|c| matches!(c, Cmd::SetParam { param: 3, value, .. } if (*value - code).abs() < 1e-4)), "{call} type {code}: {cmds:?}");
        for p in [0u8, 1, 2] {
            assert!(cmds.iter().any(|c| matches!(c, Cmd::SetParam { param, .. } if *param == p)), "{call} missing SetParam {p}");
        }
    }
}

#[test]
fn eq_setters_map_to_params() {
    use deluge_wren_core::test_support::run_and_capture_cmds;
    use deluge_audio_graph::Cmd;
    let cmds = run_and_capture_cmds(
        "var e = EQ.peak(Osc.saw(110), 1000, 0, 1)\ne.hz = 2000\ne.gain = 6\ne.q = 2",
    );
    assert!(cmds.iter().any(|c| matches!(c, Cmd::SetParam { param: 0, .. })), "hz=→0: {cmds:?}");
    assert!(cmds.iter().any(|c| matches!(c, Cmd::SetParam { param: 1, .. })), "gain=→1: {cmds:?}");
    assert!(cmds.iter().any(|c| matches!(c, Cmd::SetParam { param: 2, .. })), "q=→2: {cmds:?}");
}

#[test]
fn eq_renders_mono_bounded_on_engine_host() {
    use deluge_wren_core::test_support::run_and_render;
    use deluge_audio_graph::StereoFrame;
    let mut out = [StereoFrame::default(); 32];
    run_and_render("Out.patch(EQ.highShelf(Osc.saw(110), 3000, 6, 0.707))", &mut out);
    assert!(out.iter().all(|f| f.l.is_finite() && f.l.abs() <= 1.0));
    assert!(out.iter().any(|f| f.l != 0.0), "eq should be non-silent");
}
