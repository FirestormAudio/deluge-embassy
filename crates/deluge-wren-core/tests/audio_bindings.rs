#![cfg(feature = "test-support")]
use deluge_audio_graph::StereoFrame;
use deluge_audio_graph::node::TableSrc;
use deluge_wren_core::Host as _;
use deluge_wren_core::test_support::{
    EngineHost, run_and_capture_cmds, run_and_render, run_midi_capture_cmds, run_script_ok,
};
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
fn noise_pink_emits_newnode() {
    let cmds = run_and_capture_cmds("Noise.pink()");
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
fn noise_brown_emits_newnode() {
    let cmds = run_and_capture_cmds("Noise.brown()");
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
fn noise_pink_brown_render_finite_nonsilent() {
    let mut out = [StereoFrame::default(); 32];
    run_and_render("Out.patch(Noise.pink())", &mut out);
    assert!(out.iter().all(|f| f.l.is_finite() && f.l.abs() <= 1.0));
    assert!(out.iter().any(|f| f.l != 0.0));

    let mut out = [StereoFrame::default(); 32];
    run_and_render("Out.patch(Noise.brown())", &mut out);
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

#[test]
fn lfo_factories_emit_kind_lfo_with_shape() {
    use deluge_wren_core::test_support::run_and_capture_cmds;
    use deluge_audio_graph::{Cmd, Kind};
    for (call, code) in [("sine", 0.0f32), ("tri", 1.0), ("saw", 2.0), ("square", 3.0), ("sampleHold", 4.0), ("random", 5.0)] {
        let src = std::format!("var l = LFO.{}(2)", call);
        let cmds = run_and_capture_cmds(&src);
        assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::Lfo, .. })), "{call}: {cmds:?}");
        assert!(cmds.iter().any(|c| matches!(c, Cmd::SetParam { param: 0, value, .. } if (*value - code).abs() < 1e-4)), "{call} shape {code}: {cmds:?}");
    }
}

#[test]
fn lfo_to_builds_scaling_graph() {
    use deluge_wren_core::test_support::run_and_capture_cmds;
    use deluge_audio_graph::{Cmd, Kind};
    // `.to(200, 2000)` = this*900 + 1100 → a Mul then an Add node.
    let cmds = run_and_capture_cmds("var m = LFO.sine(1).to(200, 2000)");
    assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::Mul, .. })), "to→mul: {cmds:?}");
    assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::Add, .. })), "to→add: {cmds:?}");
}

#[test]
fn lfo_phase_setter_maps_to_param_1() {
    use deluge_wren_core::test_support::run_and_capture_cmds;
    use deluge_audio_graph::Cmd;
    let cmds = run_and_capture_cmds("var l = LFO.saw(1)\nl.phase = 0.25");
    assert!(cmds.iter().any(|c| matches!(c, Cmd::SetParam { param: 1, .. })), "phase=→1: {cmds:?}");
}

#[test]
fn lfo_renders_bounded_on_engine_host() {
    use deluge_wren_core::test_support::run_and_render;
    use deluge_audio_graph::StereoFrame;
    let mut out = [StereoFrame::default(); 32];
    run_and_render("Out.patch(LFO.tri(1000))", &mut out); // fast LFO so it moves
    assert!(out.iter().all(|f| f.l.is_finite() && f.l.abs() <= 1.0));
}

#[test]
fn sample_hold_and_slew_factories_emit_nodes() {
    use deluge_wren_core::test_support::run_and_capture_cmds;
    use deluge_audio_graph::{Cmd, Kind};
    let cmds = run_and_capture_cmds("var a = SampleHold.new(Noise.pink(), Osc.square(4))\nvar b = Slew.new(a, 0.05)");
    assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::SampleHold, .. })), "S&H: {cmds:?}");
    assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::Slew, .. })), "Slew: {cmds:?}");
}

#[test]
fn steps_factory_emits_len_and_values() {
    use deluge_wren_core::test_support::run_and_capture_cmds;
    use deluge_audio_graph::{Cmd, Kind};
    let cmds = run_and_capture_cmds("var s = Steps.new([10, 20, 30], Osc.square(2))");
    assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::Steps, .. })), "Steps: {cmds:?}");
    // len = 3 → SetParam(0, 3)
    assert!(cmds.iter().any(|c| matches!(c, Cmd::SetParam { param: 0, value, .. } if (*value - 3.0).abs() < 1e-4)), "len: {cmds:?}");
    // values → SetParam(1, 10) SetParam(2, 20) SetParam(3, 30)
    for (p, v) in [(1u8, 10.0f32), (2, 20.0), (3, 30.0)] {
        assert!(cmds.iter().any(|c| matches!(c, Cmd::SetParam { param, value, .. } if *param == p && (*value - v).abs() < 1e-4)), "value {p}={v}: {cmds:?}");
    }
}

#[test]
fn steps_renders_bounded_on_engine_host() {
    use deluge_wren_core::test_support::run_and_render;
    use deluge_audio_graph::StereoFrame;
    let mut out = [StereoFrame::default(); 32];
    run_and_render("Out.patch(Steps.new([0.2, -0.2], Osc.square(1000)))", &mut out);
    assert!(out.iter().all(|f| f.l.is_finite() && f.l.abs() <= 1.0));
}

#[test]
fn shaping_factories_emit_nodes() {
    let cmds = run_and_capture_cmds(
        "var m = Macro.new(0.5)\nvar c = Curve.exp(m)\nvar q = m.quantize(Scale.Major, 0)\nvar f = q.hz(220)\nvar s = m.steps(4)",
    );
    // Ctrl (macro) with SetParam(0, 0.5)
    assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::Ctrl, .. })), "Ctrl node");
    assert!(cmds.iter().any(|c| matches!(c, Cmd::SetParam { param: 0, value, .. } if (*value - 0.5).abs() < 1e-6)), "Ctrl value 0.5");
    // Curve node (from Curve.exp)
    assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::Curve, .. })), "Curve node");
    // QuantPitch with mask 2741 (Scale.Major) + root 0
    assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::QuantPitch, .. })), "QuantPitch node");
    assert!(cmds.iter().any(|c| matches!(c, Cmd::SetParam { param: 0, value, .. } if (*value - 2741.0).abs() < 0.5)), "major mask 2741");
    // Mtof with ref 220 + QuantStep with N 4
    assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::Mtof, .. })), "Mtof node");
    assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::QuantStep, .. })), "QuantStep node");
}

#[test]
fn macro_value_setter_emits_setparam() {
    let cmds = run_and_capture_cmds("var m = Macro.new(0.0)\nm.value = 0.75");
    // One NewNode(Ctrl) + SetParam(0, 0.0) at build, then SetParam(0, 0.75) from the setter.
    assert!(cmds.iter().any(|c| matches!(c, Cmd::SetParam { param: 0, value, .. } if (*value - 0.75).abs() < 1e-6)), "value= → SetParam(0, 0.75)");
}

#[test]
fn pitch_chain_renders_bounded_nonsilent() {
    // seq (a bipolar LFO) → range → quantize (minor) → hz → osc.freq
    let mut out = [StereoFrame::default(); 32];
    run_and_render(
        "var note = LFO.saw(4).to(0, 24).quantize(Scale.Minor, 0)\nOut.patch(Osc.saw(note.hz(110)))",
        &mut out,
    );
    assert!(out.iter().all(|f| f.l.is_finite() && f.l.abs() <= 1.0), "finite/bounded");
    assert!(out.iter().any(|f| f.l != 0.0), "non-silent");
}

#[test]
fn curve_sugar_renders_bounded() {
    let mut out = [StereoFrame::default(); 32];
    run_and_render("Out.patch(Osc.saw(110) * Env.ar(0.0, 0.1).curve(0.6))", &mut out);
    assert!(out.iter().all(|f| f.l.is_finite() && f.l.abs() <= 1.0), "finite/bounded");
}

#[test]
fn scaling_sugar_builds_arithmetic() {
    // The scale/offset sugar is thin wrappers over the * / + binops; confirm each
    // wraps into the arithmetic graph it claims (Mul for scale-like, Add for offset).
    let atten = run_and_capture_cmds("var a = LFO.sine(2).atten(0.5)");
    assert!(atten.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::Mul, .. })), "atten → Mul");
    let offset = run_and_capture_cmds("var a = LFO.sine(2).offset(0.25)");
    assert!(offset.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::Add, .. })), "offset → Add");
    // unipolar = this * 0.5 + 0.5 → both a Mul and an Add.
    let uni = run_and_capture_cmds("var a = LFO.sine(2).unipolar()");
    assert!(uni.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::Mul, .. })), "unipolar has Mul");
    assert!(uni.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::Add, .. })), "unipolar has Add");
}

#[test]
fn poly_factories_emit_poly_kinds() {
    let cmds = run_and_capture_cmds(
        "var p = Node.polyBegin_()\nvar o = Node.polyosc_(p, 0)\nvar f = Node.polysvf_(o, 1200, 0.2)\nvar e = Node.polyar_(0.01, 0.3)\nvar v = Node.polymul_(f, e)",
    );
    assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::PolyCtrl, .. })), "PolyCtrl");
    assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::PolyMtof, .. })), "PolyMtof");
    assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::PolyOsc, .. })), "PolyOsc");
    assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::PolySvf, .. })), "PolySvf");
    assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::PolyAr, .. })), "PolyAr");
    assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::PolyMul, .. })), "PolyMul");
}

#[test]
fn poly_mode_reflects_build_state() {
    // polyMode_ is 1 during a build, 0 after (script leaves it set; a follow-up
    // script starts fresh — poly_begin resets). Here: 1 right after polyBegin_.
    let cmds = run_and_capture_cmds("var m = Node.polyMode_\nvar p = Node.polyBegin_()");
    // Nothing to assert on m directly via Cmds; the render/error behavior in
    // Task 3 exercises polyMode_. This test just confirms polyBegin_ emits nodes.
    assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::PolyCtrl, .. })));
}

#[test]
fn synth_note_on_emits_pitch_and_gate() {
    let cmds = run_and_capture_cmds(
        "var p = Node.polyBegin_()\nvar v = Node.polymul_(Node.polysvf_(Node.polyosc_(p, 0), 1200, 0.2), Node.polyar_(0.01, 0.3))\nvar s = Node.polyEnd_(v)\ns.noteOn(69, 100)",
    );
    // VoiceSum built at polyEnd_.
    assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::VoiceSum, .. })), "VoiceSum");
    // note_on → SetParam(pitch lane 0 = note-69 = 0) + GateVoice(gate, 0, true).
    assert!(cmds.iter().any(|c| matches!(c, Cmd::SetParam { param: 0, value, .. } if value.abs() < 1e-6)), "pitch");
    assert!(cmds.iter().any(|c| matches!(c, Cmd::GateVoice { voice: 0, on: true, .. })), "gate on");
}

#[test]
fn synth_builds_the_poly_graph() {
    let cmds = run_and_capture_cmds(
        "var bass = Synth.new { |p| Osc.sine(p).lpf(1200) * Env.ar(0.01, 0.3) }",
    );
    for k in [Kind::PolyCtrl, Kind::PolyMtof, Kind::PolyOsc, Kind::PolySvf, Kind::PolyAr, Kind::PolyMul, Kind::VoiceSum] {
        assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind, .. } if *kind == k)), "missing {:?}", k);
    }
    // No mono Osc/Svf leaked in.
    assert!(!cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::Saw | Kind::Sine, .. })), "no mono osc");
}

#[test]
fn poly_mode_is_scoped_after_synth() {
    // After Synth.new returns, poly mode is cleared → Osc.saw builds mono Saw.
    let cmds = run_and_capture_cmds(
        "var b = Synth.new { |p| Osc.sine(p) * Env.ar(0.01, 0.3) }\nOut.patch(Osc.saw(110))",
    );
    assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::Saw, .. })), "mono Saw after Synth");
}

#[test]
fn synth_error_cases_abort() {
    // A no-env voice aborts; two envs abort.
    assert!(!run_script_ok("Synth.new { |p| Osc.sine(p) }"), "no Env.ar aborts");
    assert!(!run_script_ok("Synth.new { |p| Osc.sine(p) * Env.ar(0.01,0.3) * Env.ar(0.01,0.3) }"), "two Env.ar aborts");
    assert!(!run_script_ok("Synth.new { |p| Osc.sine(p) * 0.5 }"), "poly * scalar aborts");
    // Nested Synth aborts (the inner polyBegin sees poly_mode already set).
    assert!(!run_script_ok("Synth.new { |p| Synth.new { |q| Osc.sine(q) * Env.ar(0.01,0.3) } }"), "nested Synth aborts");
    // Poly pink/brown noise now work in a Synth (Sy-2d) — see
    // `synth_sources_render_sound` for the positive case.
    assert!(run_script_ok("Synth.new { |p| Noise.pink() * Env.ar(0.01,0.3) }"), "Noise.pink ok in Synth (Sy-2d)");
    assert!(run_script_ok("Synth.new { |p| Noise.brown() * Env.ar(0.01,0.3) }"), "Noise.brown ok in Synth (Sy-2d)");
    // Sanity: a valid Synth interprets fine.
    assert!(run_script_ok("Synth.new { |p| Osc.sine(p) * Env.ar(0.01,0.3) }"), "valid Synth ok");
}

#[test]
fn synth_saw_and_add_and_noise_build() {
    // Osc.saw → PolyOsc with SetParam(0,1); `+` → PolyAdd; Noise.new → PolyNoise.
    let cmds = run_and_capture_cmds(
        "var s = Synth.new { |p| (Osc.saw(p) + Noise.new()).lpf(1200) * Env.ar(0.01,0.3) }",
    );
    assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::PolyOsc, .. })), "PolyOsc");
    assert!(cmds.iter().any(|c| matches!(c, Cmd::SetParam { param: 0, value, .. } if (*value - 1.0).abs() < 1e-6)), "saw shape");
    assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::PolyAdd, .. })), "PolyAdd");
    assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::PolyNoise, .. })), "PolyNoise");
}

#[test]
fn synth_saw_renders_sound() {
    let mut out = [StereoFrame::default(); 32];
    run_and_render(
        "var b = Synth.new { |p| Osc.saw(p).lpf(2000) * Env.ar(0.001,0.05) }\nOut.patch(b.out)\nb.noteOn(69,100)",
        &mut out,
    );
    assert!(out.iter().all(|f| f.l.is_finite() && f.l.abs() <= 8.0), "bounded");
    assert!(out.iter().any(|f| f.l.abs() > 1e-3), "saw voice sounds");
}

#[test]
fn synth_add_and_noise_render_sound() {
    // A `+`-mixed voice and a noise voice both render non-silent end-to-end
    // (spec §5 — PolyAdd/PolyNoise through the full Synth→VoiceSum path).
    let mut mixed = [StereoFrame::default(); 32];
    run_and_render(
        "var b = Synth.new { |p| (Osc.sine(p) + Osc.saw(p)) * Env.ar(0.001,0.05) }\nOut.patch(b.out)\nb.noteOn(69,100)",
        &mut mixed,
    );
    assert!(mixed.iter().all(|f| f.l.is_finite() && f.l.abs() <= 8.0), "mix bounded");
    assert!(mixed.iter().any(|f| f.l.abs() > 1e-3), "`+` mix sounds");

    let mut noise = [StereoFrame::default(); 32];
    run_and_render(
        "var b = Synth.new { |p| Noise.new() * Env.ar(0.001,0.05) }\nOut.patch(b.out)\nb.noteOn(69,100)",
        &mut noise,
    );
    assert!(noise.iter().all(|f| f.l.is_finite() && f.l.abs() <= 8.0), "noise bounded");
    assert!(noise.iter().any(|f| f.l.abs() > 1e-3), "noise voice sounds");
}

#[test]
fn synth_note_on_renders_sound() {
    // Build a Synth, route it, play a note in-script, then render.
    let mut out = [StereoFrame::default(); 32];
    run_and_render(
        "var bass = Synth.new { |p| Osc.sine(p).lpf(2000) * Env.ar(0.001, 0.05) }\nOut.patch(bass.out)\nbass.noteOn(69, 100)",
        &mut out,
    );
    assert!(out.iter().all(|f| f.l.is_finite() && f.l.abs() <= 8.0), "bounded/finite");
    assert!(out.iter().any(|f| f.l.abs() > 1e-3), "note-on sounds");
}

#[test]
fn synth_plays_from_midi() {
    // `bindMidi()` wires `Midi.onNoteOn`/`onNoteOff` to `synth.noteOn`/`noteOff`
    // (see prelude.wren). A DIN note-on fed via `midi_rx_impl` should fire that
    // closure and drive the allocator: SetParam (pitch) + GateVoice (gate on).
    let cmds = run_midi_capture_cmds(
        "var bass = Synth.new { |p| Osc.sine(p).lpf(2000) * Env.ar(0.001, 0.05) }\nOut.patch(bass.out)\nbass.bindMidi()",
        0x90, 69, 100, // note-on A4 vel 100
    );
    assert!(cmds.iter().any(|c| matches!(c, Cmd::GateVoice { on: true, .. })), "MIDI note-on gates a voice");
    assert!(cmds.iter().any(|c| matches!(c, Cmd::SetParam { .. })), "MIDI note-on sets pitch");
}

#[test]
fn synth_moog_and_ms20_render_sound() {
    // Sy-2c: poly Moog/Ms20 voices render finite, bounded, non-silent audio
    // end-to-end through the full Synth → VoiceSum path.
    let mut moog = [StereoFrame::default(); 32];
    run_and_render(
        "var b = Synth.new { |p| Moog.lp(Osc.saw(p), 1200, 0.85) * Env.ar(0.01, 0.3) }\nOut.patch(b.out)\nb.noteOn(69,100)",
        &mut moog,
    );
    assert!(moog.iter().all(|f| f.l.is_finite() && f.l.abs() <= 8.1), "Moog.lp bounded");
    assert!(moog.iter().any(|f| f.l.abs() > 1e-4), "Moog.lp voice sounds");

    let mut ms20 = [StereoFrame::default(); 32];
    run_and_render(
        "var b = Synth.new { |p| Ms20.hp(Osc.saw(p), 1200, 0.9) * Env.ar(0.01, 0.3) }\nOut.patch(b.out)\nb.noteOn(69,100)",
        &mut ms20,
    );
    assert!(ms20.iter().all(|f| f.l.is_finite() && f.l.abs() <= 8.1), "Ms20.hp bounded");
    assert!(ms20.iter().any(|f| f.l.abs() > 1e-4), "Ms20.hp voice sounds");
}

#[test]
fn synth_sources_build_correct_kinds() {
    // Sy-2d: each flipped source route emits the poly Kind, not the mono one.
    let sync = run_and_capture_cmds(
        "var b = Synth.new { |p| Osc.syncSaw(p, p*1.5) * Env.ar(0.01,0.3) }",
    );
    assert!(sync.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::PolySyncSaw, .. })), "PolySyncSaw");
    assert!(!sync.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::SyncSaw, .. })), "no mono SyncSaw");

    let wt = run_and_capture_cmds(
        "var b = Synth.new { |p| Osc.wavetable(WT.Saw, p) * Env.ar(0.01,0.3) }",
    );
    assert!(wt.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::PolyWt, .. })), "PolyWt");
    assert!(!wt.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::Wavetable, .. })), "no mono Wavetable");

    let morph = run_and_capture_cmds(
        "var b = Synth.new { |p| Osc.wavetable(WT.HarmonicSweep, p) * Env.ar(0.01,0.3) }",
    );
    assert!(morph.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::PolyWtMorph, .. })), "PolyWtMorph");

    let pink = run_and_capture_cmds("var b = Synth.new { |p| Noise.pink() * Env.ar(0.01,0.3) }");
    assert!(pink.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::PolyPink, .. })), "PolyPink");

    let brown = run_and_capture_cmds("var b = Synth.new { |p| Noise.brown() * Env.ar(0.01,0.3) }");
    assert!(brown.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::PolyBrown, .. })), "PolyBrown");

    // `.width =` in poly mode targets the PolyOsc's poly width port (1), not
    // the mono port (2).
    // NOTE: a multi-statement Wren block needs an explicit `return` for its
    // value to reach `builder.call(pitch)` in `Synth.new` — only a
    // single-expression block implicitly returns its value.
    let pwm = run_and_capture_cmds(
        "var b = Synth.new { |p|\n  var o = Osc.square(p)\n  o.width = LFO.sine(4).to(0.2,0.8)\n  return o * Env.ar(0.01,0.3)\n}",
    );
    assert!(pwm.iter().any(|c| matches!(c, Cmd::SetInput { port: 1, .. })), "width= sets poly port 1");
    assert!(!pwm.iter().any(|c| matches!(c, Cmd::SetInput { port: 2, .. })), "width= does not touch mono port 2");
}

#[test]
fn synth_sources_render_sound() {
    // Sy-2d: poly sync/wavetable(single+morph)/pink/brown/PWM all render
    // finite, bounded, non-silent audio end-to-end through Synth → VoiceSum.
    let mut sync = [StereoFrame::default(); 32];
    run_and_render(
        "var b = Synth.new { |p| Osc.syncSaw(p, p*1.5).lpf(2000) * Env.ar(0.01,0.3) }\nOut.patch(b.out)\nb.noteOn(69,100)",
        &mut sync,
    );
    assert!(sync.iter().all(|f| f.l.is_finite() && f.l.abs() <= 8.1), "sync bounded");
    assert!(sync.iter().any(|f| f.l.abs() > 1e-4), "sync voice sounds");

    let mut wt = [StereoFrame::default(); 32];
    run_and_render(
        "var b = Synth.new { |p| Osc.wavetable(WT.Saw, p) * Env.ar(0.01,0.3) }\nOut.patch(b.out)\nb.noteOn(69,100)",
        &mut wt,
    );
    assert!(wt.iter().all(|f| f.l.is_finite() && f.l.abs() <= 8.1), "wavetable bounded");
    assert!(wt.iter().any(|f| f.l.abs() > 1e-4), "wavetable voice sounds");

    // 2D morph table (WT.HarmonicSweep, a named static bank — Task 5).
    let mut morph = [StereoFrame::default(); 32];
    run_and_render(
        "var b = Synth.new { |p| Osc.wavetable(WT.HarmonicSweep, p) * Env.ar(0.01,0.3) }\nOut.patch(b.out)\nb.noteOn(69,100)",
        &mut morph,
    );
    assert!(morph.iter().all(|f| f.l.is_finite() && f.l.abs() <= 8.1), "wavetable morph bounded");
    assert!(morph.iter().any(|f| f.l.abs() > 1e-4), "wavetable morph voice sounds");

    let mut pink = [StereoFrame::default(); 32];
    run_and_render(
        "var b = Synth.new { |p| Noise.pink() * Env.ar(0.01,0.3) }\nOut.patch(b.out)\nb.noteOn(69,100)",
        &mut pink,
    );
    assert!(pink.iter().all(|f| f.l.is_finite() && f.l.abs() <= 8.1), "pink bounded");
    assert!(pink.iter().any(|f| f.l.abs() > 1e-4), "pink voice sounds");

    let mut brown = [StereoFrame::default(); 32];
    run_and_render(
        "var b = Synth.new { |p| Noise.brown() * Env.ar(0.01,0.3) }\nOut.patch(b.out)\nb.noteOn(69,100)",
        &mut brown,
    );
    assert!(brown.iter().all(|f| f.l.is_finite() && f.l.abs() <= 8.1), "brown bounded");
    assert!(brown.iter().any(|f| f.l.abs() > 1e-4), "brown voice sounds");

    // PWM: a mono LFO drives the width port via the Task-1 mono→poly broadcast.
    let mut pwm = [StereoFrame::default(); 32];
    run_and_render(
        "var b = Synth.new { |p|\n  var o = Osc.square(p)\n  o.width = LFO.sine(4).to(0.2,0.8)\n  return o * Env.ar(0.01,0.3)\n}\nOut.patch(b.out)\nb.noteOn(69,100)",
        &mut pwm,
    );
    assert!(pwm.iter().all(|f| f.l.is_finite() && f.l.abs() <= 8.1), "PWM bounded");
    assert!(pwm.iter().any(|f| f.l.abs() > 1e-4), "PWM voice sounds");
}

#[test]
fn polyosc_no_width_unchanged_no_setinput_port1() {
    // Backward-compat (Task 3 gate): a PolyOsc voice that never sets `.width`
    // must not emit any SetInput on port 1 (the poly width port) — it stays
    // at its NewNode-time default (Const(0.0) ⇒ 0.5 duty), bit-identical to
    // the pre-Sy-2d PolyOsc (proven bit-exact at the engine level — see
    // deluge-audio-graph's node.rs tests).
    let cmds = run_and_capture_cmds(
        "var b = Synth.new { |p| Osc.square(p) * Env.ar(0.01, 0.3) }",
    );
    assert!(cmds.iter().any(|c| matches!(c, Cmd::NewNode { kind: Kind::PolyOsc, .. })), "PolyOsc built");
    assert!(
        !cmds.iter().any(|c| matches!(c, Cmd::SetInput { port: 1, .. })),
        "no SetInput on PolyOsc's width port when .width wasn't set"
    );

    // Sanity: the render is non-silent and bounded (same as the pre-existing
    // `synth_saw_renders_sound`-style checks), i.e. the width-port addition
    // didn't silently break the default 0.5-duty square.
    let mut out = [StereoFrame::default(); 32];
    run_and_render(
        "var b = Synth.new { |p| Osc.square(p) * Env.ar(0.001, 0.05) }\nOut.patch(b.out)\nb.noteOn(69,100)",
        &mut out,
    );
    assert!(out.iter().all(|f| f.l.is_finite() && f.l.abs() <= 8.1), "bounded");
    assert!(out.iter().any(|f| f.l.abs() > 1e-3), "square voice sounds");
}

#[test]
fn non_poly_classes_abort_inside_synth() {
    // Sy-2c footgun close: classes not yet poly-ified (or that are post-voice
    // effects) must Fiber.abort rather than silently building a mono/broken
    // voice inside a Synth.
    for src in [
        "Synth.new { |p| Tb303.lp(Osc.saw(p), 400, 0.9) * Env.ar(0.01, 0.3) }",
        "Synth.new { |p| Resonator.new(Osc.saw(p), 220, 0.4) * Env.ar(0.01, 0.3) }",
        "Synth.new { |p| Svf.lp(Osc.saw(p), 1200, 0.6) * Env.ar(0.01, 0.3) }",
        "Synth.new { |p| Delay.new(Osc.saw(p), 0.2, 0.4) * Env.ar(0.01, 0.3) }",
        // I-1: signal-input control/shaping classes missed by Sy-2c.
        "Synth.new { |p| Slew.new(Osc.saw(p), 0.01) * Env.ar(0.01, 0.3) }",
        "Synth.new { |p| Osc.saw(p).curve(0.5) * Env.ar(0.01, 0.3) }",
        "Synth.new { |p| SampleHold.new(Osc.saw(p), Osc.square(p)) * Env.ar(0.01, 0.3) }",
        "Synth.new { |p| Split.new(Osc.saw(p)) * Env.ar(0.01, 0.3) }",
        "Synth.new { |p| Curve.new(Osc.saw(p), 0.5) * Env.ar(0.01, 0.3) }",
        "Synth.new { |p| Osc.saw(p).hz(69) * Env.ar(0.01, 0.3) }",
    ] {
        assert!(!run_script_ok(src), "expected abort for: {src}");
    }
}
