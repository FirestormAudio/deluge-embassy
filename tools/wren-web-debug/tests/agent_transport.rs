//! Task 2.4: the transport-agnostic driver loop (`transport::serve`).
//!
//! `serve` is the same logic wren-dap's `spawn_driver`/`run_until_stop`
//! (`wren-dap/src/session.rs:48-179`, `262-281`) implement, generalized
//! from wren-dap's stdio/DAP-JSON transport to the `DebugTransport` trait
//! (`src/transport.rs`). This test proves the seam actually works end to
//! end with a real transport implementation: an in-memory `mpsc` pair.
//!
//! Threading mirrors wren-dap's own arrangement (VM runs on its own thread,
//! spawned inside `agent::debug_run`; the driver — here, `serve` — runs on
//! a second thread) — `serve` blocks on `recv_cmd`/`wait_event`, so it
//! can't run on the test's main thread while that same thread also needs to
//! feed commands and drain events.

mod common;

use std::collections::HashSet;
use std::sync::mpsc::{channel, Receiver, Sender};

use wren_web_debug::transport::{DebugCmd, DebugEvent, DebugTransport};

/// An in-memory [`DebugTransport`] backed by a pair of `mpsc` channels — the
/// test-only "native" transport the brief asks for. `recv_cmd`/`send_event`
/// are exactly the two operations wren-dap's stdio transport performs
/// (read a request, write a response/event); here they're just channel
/// ops instead of JSON-over-stdio.
struct MpscTransport {
    cmd_rx: Receiver<DebugCmd>,
    event_tx: Sender<DebugEvent>,
}

impl DebugTransport for MpscTransport {
    fn recv_cmd(&self) -> Option<DebugCmd> {
        self.cmd_rx.recv().ok()
    }

    fn send_event(&self, ev: DebugEvent) {
        // The receiving end (the test's main thread) may already have
        // stopped draining once it's seen `Terminated` — a dropped receiver
        // here is expected, not a bug, so this deliberately ignores the
        // send error rather than unwrapping.
        let _ = self.event_tx.send(ev);
    }
}

#[test]
fn stack_trace_then_continue_over_mpsc_transport() {
    let _g = common::VM_TEST_LOCK.lock().unwrap();

    // Breaks on line 2; nothing after resuming stops it again, so a
    // `Continue` after the initial stop should run straight to `Terminated`.
    let session = wren_web_debug::agent::debug_run(
        "var a = 1\nvar b = 2\nSystem.print(b)\n",
        vec![],
        HashSet::from([2]),
    );

    let (cmd_tx, cmd_rx) = channel::<DebugCmd>();
    let (event_tx, event_rx) = channel::<DebugEvent>();
    let transport = MpscTransport { cmd_rx, event_tx };

    // Queue both commands up front: `serve`'s command loop only starts
    // consuming them after its initial `run_until_stop` pump (the
    // `debug_run`-already-launched equivalent of wren-dap's post-`Launch`
    // pump), so there's no race between queuing and `serve` reading.
    cmd_tx.send(DebugCmd::StackTrace { thread_id: 1 }).unwrap();
    cmd_tx.send(DebugCmd::Continue).unwrap();

    // `serve` blocks (it's a driver loop) — run it on its own thread so
    // this thread is free to drain `event_rx` concurrently. `session` and
    // `transport` are moved in wholesale (not shared): the VM itself
    // already runs on the separate thread `debug_run` spawned internally,
    // so this thread is purely `serve`'s driver loop.
    let driver = std::thread::spawn(move || {
        wren_web_debug::transport::serve(&session, &transport);
    });

    // Drain events until `Terminated` (the last event `serve` sends before
    // returning after the `Continue`-triggered run-to-completion — see
    // `run_until_stop`'s `Terminated` arm, which sends `Terminated` then
    // `Exited`). Collecting stops here rather than at `Exited` so the
    // assertions below can check exactly the three events the brief
    // specifies without over-fitting to the `Exited` tail.
    let mut events = Vec::new();
    loop {
        match event_rx.recv() {
            Ok(ev) => {
                let is_terminated = matches!(ev, DebugEvent::Terminated);
                events.push(ev);
                if is_terminated {
                    break;
                }
            }
            Err(_) => panic!("event channel closed before Terminated; events so far: {events:?}"),
        }
    }

    // `serve` must fully return (having driven the session to completion)
    // before the test ends, or the VM thread could still be parked/running
    // when the next test's `debug_run` boots a second VM — see
    // `common::drive_to_end`'s doc comment on the single-VM-per-process
    // invariant this crate requires.
    driver.join().expect("serve thread panicked");

    // Expected sequence: the initial breakpoint stop (from `serve`'s pre-loop
    // `run_until_stop`), then the `StackTrace` response, then `Terminated`
    // (from the `Continue` command's post-resume `run_until_stop`, since
    // nothing else in this program stops the VM again).
    assert_eq!(events.len(), 3, "expected exactly 3 events, got {events:?}");

    match &events[0] {
        DebugEvent::Stopped { thread_id, line, reason, all_threads_stopped } => {
            assert_eq!(*thread_id, 1);
            assert_eq!(*line, 2);
            assert_eq!(reason, "breakpoint");
            assert!(*all_threads_stopped);
        }
        other => panic!("expected Stopped first, got {other:?}"),
    }

    match &events[1] {
        DebugEvent::StackTrace { stack_frames, total_frames } => {
            assert_eq!(*total_frames, stack_frames.len());
            assert!(!stack_frames.is_empty(), "expected at least one frame at the breakpoint");
            assert_eq!(stack_frames[0].line, 2);
        }
        other => panic!("expected StackTrace second, got {other:?}"),
    }

    assert_eq!(events[2], DebugEvent::Terminated, "expected Terminated third, got {:?}", events[2]);
}
