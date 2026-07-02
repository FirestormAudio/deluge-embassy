//! Task 2.4: transport-agnostic driver loop for a [`DebugSession`].
//!
//! `wren-dap`'s `spawn_driver` (`wren-dap/src/session.rs:48-179`) couples two
//! things that don't need to be coupled: the DAP request/response *shape*,
//! and the stdio/JSON transport that carries it. This module keeps the
//! shape (`DebugCmd`/`DebugEvent`, DAP-field-named so Phase 3 can serialize
//! them as JSON over a `SharedArrayBuffer`) but replaces the transport with
//! a trait, so the exact same `serve` loop runs over an in-process `mpsc`
//! (native tests, this task) or a SAB worker protocol (Phase 3) without any
//! change to the command/event dispatch logic.
//!
//! # Deviation from wren-dap: no `Launch` command
//! In `wren-dap`, `DriverCommand::Launch` is what *builds* the
//! [`DebugSession`] (`session.rs:62-65`, calling `launch(...)`) — the driver
//! owns an `Option<DebugSession>` that starts `None`. Here, [`debug_run`]
//! (Task 2.1) already builds and returns a live `DebugSession` before
//! `serve` is ever called (this crate's `debug_run` signature takes the
//! entry source directly, it doesn't round-trip through a transport to
//! learn the program path). So `serve` takes `&DebugSession` up front and
//! there is no `DebugCmd::Launch` — the equivalent of wren-dap's
//! post-Launch `run_until_stop` runs unconditionally as `serve`'s first
//! step, pumping whatever the session already has buffered (typically the
//! initial breakpoint stop) before the command loop starts.
//!
//! [`debug_run`]: crate::agent::debug_run

use serde::{Deserialize, Serialize};
use wren_core::vm::{DebugSession, DebugStop};

/// A command sent to the driver loop, mirroring wren-dap's `DriverCommand`
/// (`session.rs:31-45`) minus `Launch`/`Detach`/`Disconnect`/`ExceptionInfo`
/// (session lifecycle concerns this crate's single-VM-per-`debug_run` model
/// doesn't need — see the module docs). Field names match the DAP
/// `arguments` wren-dap reads out of the request JSON (`threadId`,
/// `frameId`, `variablesReference`, `expression`), just typed instead of
/// `serde_json::Value`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "command", rename_all = "camelCase")]
pub enum DebugCmd {
    /// DAP `continue`.
    Continue,
    /// DAP `threads`.
    Threads,
    /// DAP `stackTrace`.
    StackTrace {
        #[serde(rename = "threadId")]
        thread_id: i64,
    },
    /// DAP `scopes`.
    Scopes {
        #[serde(rename = "frameId")]
        frame_id: i64,
    },
    /// DAP `variables`.
    Variables {
        #[serde(rename = "variablesReference")]
        variables_reference: i64,
    },
    /// DAP `evaluate`.
    Evaluate {
        #[serde(rename = "frameId")]
        frame_id: i64,
        expression: String,
    },
    /// DAP `next` (step over).
    Next,
    /// DAP `stepIn`.
    StepIn,
    /// DAP `stepOut`.
    StepOut,
}

/// One DAP `Thread` body (`{ id, name }`), as emitted by wren-dap's
/// `Threads` handler (`session.rs:73-81`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ThreadDto {
    pub id: i64,
    pub name: String,
}

/// One DAP `StackFrame` body, trimmed to the fields `DebugSession::stack_trace`
/// can actually produce (`FrameInfo { name, line, module, id }`). wren-dap's
/// version additionally nests a `source: { name, path }` resolved through its
/// `ModuleRegistry` (file-path bookkeeping this crate has no equivalent of
/// yet); `module` is carried here as a plain field instead of inventing a
/// path this crate doesn't have.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct StackFrameDto {
    pub id: i64,
    pub name: String,
    pub line: i32,
    pub column: i32,
    pub module: String,
}

/// One DAP `Scope` body (`session.rs:107-109`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ScopeDto {
    pub name: String,
    pub variables_reference: i64,
    pub expensive: bool,
}

/// One DAP `Variable` body (`session.rs:115-117`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct VariableDto {
    pub name: String,
    pub value: String,
    pub variables_reference: i64,
}

/// An event emitted by the driver loop, mirroring the DAP events/response
/// bodies wren-dap writes in `spawn_driver`/`run_until_stop`. Unlike
/// wren-dap (which wraps every reply in a `response_with_seq` envelope with
/// a `request_seq`), these are just the *body* shapes — sequencing/envelope
/// concerns belong to whichever transport (SAB, in this crate's case, in
/// Phase 3) actually frames messages.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "event", rename_all = "camelCase")]
pub enum DebugEvent {
    /// DAP `stopped` (`session.rs:268-273`).
    Stopped {
        #[serde(rename = "threadId")]
        thread_id: i64,
        line: i32,
        reason: String,
        #[serde(rename = "allThreadsStopped")]
        all_threads_stopped: bool,
    },
    /// DAP `output` (`session.rs:265-267`).
    Output { category: String, output: String },
    /// DAP `terminated` (`session.rs:275`).
    Terminated,
    /// DAP `exited` (`session.rs:276`).
    Exited {
        #[serde(rename = "exitCode")]
        exit_code: i32,
    },
    /// Response body for `DebugCmd::Threads` (`session.rs:73-81`).
    Threads { threads: Vec<ThreadDto> },
    /// Response body for `DebugCmd::StackTrace` (`session.rs:82-103`).
    StackTrace {
        #[serde(rename = "stackFrames")]
        stack_frames: Vec<StackFrameDto>,
        #[serde(rename = "totalFrames")]
        total_frames: usize,
    },
    /// Response body for `DebugCmd::Scopes` (`session.rs:104-111`).
    Scopes { scopes: Vec<ScopeDto> },
    /// Response body for `DebugCmd::Variables` (`session.rs:112-119`).
    Variables { variables: Vec<VariableDto> },
    /// Successful response body for `DebugCmd::Evaluate` (`session.rs:120-133`).
    Evaluate {
        result: String,
        #[serde(rename = "variablesReference")]
        variables_reference: i64,
    },
    /// Failed response body for `DebugCmd::Evaluate` — wren-dap reports this
    /// via an `error_response` envelope (`session.rs:129`); here it's just a
    /// distinct event variant since this module doesn't have an envelope.
    EvaluateError { message: String },
}

/// A transport for driving a [`DebugSession`], decoupled from any particular
/// wire format. `recv_cmd`/`send_event` are the two primitives [`serve`]
/// needs; how commands physically arrive and events physically leave (an
/// `mpsc` channel pair for native tests, a `SharedArrayBuffer`-backed
/// protocol for the Phase 3 browser worker) is entirely up to the
/// implementation.
pub trait DebugTransport {
    /// Block until the next command arrives. Returns `None` once no more
    /// commands will ever arrive (e.g. the command channel's sender was
    /// dropped) — `serve` stops its command loop in that case, mirroring
    /// wren-dap's reader-thread EOF (`spawn_driver`'s `while let Ok(cmd) =
    /// rx.recv()`, `session.rs:60`).
    fn recv_cmd(&self) -> Option<DebugCmd>;

    /// Emit an event/response to the client.
    fn send_event(&self, ev: DebugEvent);
}

/// Drive `session` to completion over `transport`: a straight port of
/// wren-dap's `spawn_driver` command loop (`session.rs:60-176`) plus
/// `run_until_stop` (`session.rs:262-281`), generalized from the DAP/stdio
/// transport to [`DebugTransport`].
///
/// Blocks until the session terminates or `transport.recv_cmd()` returns
/// `None`. Mirrors wren-dap's control flow exactly: every command that can
/// change the VM's run state (`Continue`/`Next`/`StepIn`/`StepOut`) is
/// followed by a `run_until_stop` pump, exactly like wren-dap's `Continue`/
/// `Next`/`StepIn`/`StepOut` arms (`session.rs:66-72`, `134-154`); every
/// inspection command (`Threads`/`StackTrace`/`Scopes`/`Variables`/
/// `Evaluate`) emits exactly one reply and returns to `recv_cmd` (session.rs's
/// non-run-state arms, `73-133`).
///
/// The one structural difference from wren-dap (see the module docs) is
/// that there's no `Launch` command to gate the first `run_until_stop` on:
/// `session` is already live when `serve` is called, so `serve` pumps once,
/// unconditionally, before entering the command loop — the equivalent of
/// wren-dap's `Launch` arm (`session.rs:62-65`) having already happened by
/// the time `serve` is invoked.
pub fn serve(session: &DebugSession, transport: &impl DebugTransport) {
    // Equivalent of wren-dap's post-`Launch` `run_until_stop` call
    // (`session.rs:64`): pump whatever the session has already produced (a
    // breakpoint stop, in the expected usage) before waiting on commands.
    if !run_until_stop(session, transport) {
        return;
    }

    while let Some(cmd) = transport.recv_cmd() {
        match cmd {
            DebugCmd::Threads => {
                let threads = session.threads();
                // Mirrors wren-dap's fallback for an empty thread list
                // (`session.rs:75-79`): a session with no fibers reported
                // yet still has an implicit "main" thread.
                let dtos = if threads.is_empty() {
                    vec![ThreadDto { id: 1, name: "main".to_string() }]
                } else {
                    threads.into_iter().map(|t| ThreadDto { id: t.id, name: t.name }).collect()
                };
                transport.send_event(DebugEvent::Threads { threads: dtos });
            }
            DebugCmd::StackTrace { thread_id } => {
                let frames = session.stack_trace(thread_id);
                let stack_frames: Vec<StackFrameDto> = frames
                    .into_iter()
                    .map(|f| StackFrameDto { id: f.id, name: f.name, line: f.line, column: 0, module: f.module })
                    .collect();
                let total_frames = stack_frames.len();
                transport.send_event(DebugEvent::StackTrace { stack_frames, total_frames });
            }
            DebugCmd::Scopes { frame_id } => {
                let scopes = session.scopes(frame_id);
                let dtos = scopes
                    .into_iter()
                    .map(|s| ScopeDto { name: s.name, variables_reference: s.var_ref, expensive: false })
                    .collect();
                transport.send_event(DebugEvent::Scopes { scopes: dtos });
            }
            DebugCmd::Variables { variables_reference } => {
                let vars = session.variables(variables_reference);
                let dtos = vars
                    .into_iter()
                    .map(|v| VariableDto { name: v.name, value: v.value, variables_reference: v.var_ref })
                    .collect();
                transport.send_event(DebugEvent::Variables { variables: dtos });
            }
            DebugCmd::Evaluate { frame_id, expression } => {
                match session.evaluate(frame_id, &expression) {
                    Ok(out) => {
                        transport.send_event(DebugEvent::Evaluate { result: out.result, variables_reference: out.var_ref });
                    }
                    Err(message) => transport.send_event(DebugEvent::EvaluateError { message }),
                }
            }
            DebugCmd::Continue => {
                session.resume();
                if !run_until_stop(session, transport) {
                    return;
                }
            }
            DebugCmd::Next => {
                session.step_over();
                if !run_until_stop(session, transport) {
                    return;
                }
            }
            DebugCmd::StepIn => {
                session.step_in();
                if !run_until_stop(session, transport) {
                    return;
                }
            }
            DebugCmd::StepOut => {
                session.step_out();
                if !run_until_stop(session, transport) {
                    return;
                }
            }
        }
    }
}

/// Pump the session's event stream until a stop or termination — a direct
/// port of wren-dap's `run_until_stop` (`session.rs:262-281`). Forwards
/// `Output` events as-is, emits `Stopped` (with line/reason from the
/// session) on a breakpoint/step hit and returns `true` (there's more to
/// drive), or emits `Terminated` + `Exited` on program end and returns
/// `false` (the session is spent; callers must stop calling into it).
fn run_until_stop(session: &DebugSession, transport: &impl DebugTransport) -> bool {
    loop {
        match session.wait_event() {
            DebugStop::Output { text, category } => {
                transport.send_event(DebugEvent::Output { category: category.to_string(), output: text });
            }
            DebugStop::Stopped { line, reason } => {
                transport.send_event(DebugEvent::Stopped {
                    thread_id: 1,
                    line,
                    reason: reason.to_string(),
                    all_threads_stopped: true,
                });
                return true;
            }
            DebugStop::Terminated => {
                transport.send_event(DebugEvent::Terminated);
                transport.send_event(DebugEvent::Exited { exit_code: 0 });
                return false;
            }
        }
    }
}
