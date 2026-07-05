//! Wasm C-ABI wrapper around `wren_analyzer::analyze`, for the web editor's live
//! diagnostics. JS writes the script into the source buffer, calls `analyze_run`,
//! then reads a packed result buffer of diagnostics.
//!
//! Result layout (little-endian):
//! ```text
//! u32  count
//! repeated `count` times:
//!   u32 start_line   (1-based)
//!   u32 start_col    (1-based, UTF-16 code units, for Monaco)
//!   u32 end_line
//!   u32 end_col
//!   u8  severity     (0=error 1=warning 2=info 3=hint)
//!   u32 msg_len
//!   u8  msg[msg_len] (UTF-8)
//! ```
use std::ptr::addr_of_mut;

use wren_analyzer::AnalyzerDiagnostic;
use wren_analyzer::analyze;
use wren_analyzer::index::{SymKind, SymbolDef, SymbolIndex};
use wren_syntax::ast::Severity;
use wren_syntax::span::Span;

static mut SRC: Vec<u8> = Vec::new();
static mut OUT: Vec<u8> = Vec::new();
/// A scratch buffer for a module *name* passed to `add_module`.
static mut MOD_NAME: Vec<u8> = Vec::new();
/// The other project files, keyed by module name (path without `.wren`), so
/// `analyze_run` can validate cross-file imports (`import "lib/x" for Y`) live —
/// otherwise import errors only surface on Run. Rebuilt each analyze via
/// `clear_modules` + `add_module`.
static mut MODULES: Vec<(String, String)> = Vec::new();

/// The Deluge prelude (same file the firmware/VM compile), prepended when
/// building the symbol index so `Osc`/`Output`/`Midi`/`output[]`/… resolve for
/// hover, go-to-definition, and completion. Not used for diagnostics (which run
/// on the user source alone — the analyzer has no undefined-global rule).
const PRELUDE: &str = include_str!("../../../crates/deluge-wren-core/wren/prelude.wren");

/// Reserve `len` bytes of source buffer and return a pointer for JS to fill.
#[unsafe(no_mangle)]
pub extern "C" fn src_reserve(len: usize) -> *mut u8 {
    // SAFETY: single-threaded wasm; sole accessor.
    let s = unsafe { &mut *addr_of_mut!(SRC) };
    s.clear();
    s.resize(len, 0);
    s.as_mut_ptr()
}

/// Reserve `len` bytes of the module-name buffer for JS to fill (paired with
/// `add_module`).
#[unsafe(no_mangle)]
pub extern "C" fn mod_name_reserve(len: usize) -> *mut u8 {
    let n = unsafe { &mut *addr_of_mut!(MOD_NAME) };
    n.clear();
    n.resize(len, 0);
    n.as_mut_ptr()
}

/// Drop all registered project modules (call before re-registering for a fresh
/// analyze).
#[unsafe(no_mangle)]
pub extern "C" fn clear_modules() {
    unsafe { (*addr_of_mut!(MODULES)).clear() };
}

/// Register the module whose name is in `MOD_NAME[..name_len]` and whose source
/// is the current `SRC` buffer contents. (JS writes the name via
/// `mod_name_reserve`, the source via `src_reserve`, then calls this.)
#[unsafe(no_mangle)]
pub extern "C" fn add_module(name_len: usize) {
    let name = {
        let n = unsafe { &*addr_of_mut!(MOD_NAME) };
        core::str::from_utf8(&n[..name_len.min(n.len())]).unwrap_or("").to_string()
    };
    let src = {
        let s = unsafe { &*addr_of_mut!(SRC) };
        core::str::from_utf8(s).unwrap_or("").to_string()
    };
    unsafe { (*addr_of_mut!(MODULES)).push((name, src)) };
}

/// The top-level names a module source exports (classes, module variables, and
/// re-exported imports) — what an `import "M" for X` can pull out of module `M`.
fn module_exports(src: &str) -> Vec<String> {
    let module = wren_syntax::parser::parse(src).module;
    let index = SymbolIndex::build(&module);
    index
        .completion_candidates()
        .into_iter()
        .filter(|d| matches!(d.kind, SymKind::Class | SymKind::ModuleVariable | SymKind::ImportedSymbol))
        .map(|d| d.name.clone())
        .collect()
}

/// Validate the active source's `import "M" for A, B` against the registered
/// project modules: for each imported name a *known* module doesn't export,
/// emit an error at that name (mirrors the wren compiler's message). Imports of
/// modules we don't have (builtins / external) are left alone.
fn import_diagnostics(src: &str) -> Vec<AnalyzerDiagnostic> {
    let module = wren_syntax::parser::parse(src).module;
    let index = SymbolIndex::build(&module);
    let modules = unsafe { &*addr_of_mut!(MODULES) };
    let mut out = Vec::new();
    for imp in &index.imports {
        let Some((_, mod_src)) = modules.iter().find(|(n, _)| *n == imp.module_name) else { continue };
        let exports = module_exports(mod_src);
        for var in &imp.variables {
            if !exports.iter().any(|e| e == &var.name) {
                out.push(AnalyzerDiagnostic {
                    message: format!(
                        "Could not find a variable named '{}' in module '{}'.",
                        var.name, imp.module_name
                    ),
                    span: Span { start: var.name_span.start, end: var.name_span.end },
                    severity: Severity::Error,
                    related: None,
                });
            }
        }
    }
    out
}

/// Analyze the source currently in the buffer; returns the result byte length.
#[unsafe(no_mangle)]
pub extern "C" fn analyze_run() -> usize {
    let s = unsafe { &*addr_of_mut!(SRC) };
    let src = core::str::from_utf8(s).unwrap_or("");
    let mut diags = analyze(src);
    // Live cross-file import validation against the registered project modules.
    diags.extend(import_diagnostics(src));

    let out = unsafe { &mut *addr_of_mut!(OUT) };
    out.clear();
    out.extend_from_slice(&(diags.len() as u32).to_le_bytes());
    for d in &diags {
        let (sl, sc) = line_col(src, d.span.start);
        let (el, ec) = line_col(src, d.span.end);
        out.extend_from_slice(&sl.to_le_bytes());
        out.extend_from_slice(&sc.to_le_bytes());
        out.extend_from_slice(&el.to_le_bytes());
        out.extend_from_slice(&ec.to_le_bytes());
        out.push(severity_code(d.severity));
        let msg = d.message.as_bytes();
        out.extend_from_slice(&(msg.len() as u32).to_le_bytes());
        out.extend_from_slice(msg);
    }
    out.len()
}

#[unsafe(no_mangle)]
pub extern "C" fn result_ptr() -> *const u8 {
    unsafe { (*addr_of_mut!(OUT)).as_ptr() }
}

fn severity_code(s: Severity) -> u8 {
    match s {
        Severity::Error => 0,
        Severity::Warning => 1,
        Severity::Info => 2,
        Severity::Hint => 3,
    }
}

// ── Position queries (hover / definition / completion) ───────────────────────
// These build a SymbolIndex over `prelude + user source`. Query offsets are
// USER byte offsets; we add the prelude length before querying, and translate
// returned spans back to user coordinates (spans inside the prelude are flagged).

/// Build a symbol index over the prelude + the current source; returns the index
/// and the byte offset at which the user source begins.
fn build_index() -> (SymbolIndex, usize, String) {
    let user = {
        let s = unsafe { &*addr_of_mut!(SRC) };
        core::str::from_utf8(s).unwrap_or("").to_string()
    };
    let offset = PRELUDE.len() + 1;
    let mut combined = String::with_capacity(offset + user.len());
    combined.push_str(PRELUDE);
    combined.push('\n');
    combined.push_str(&user);
    let module = wren_syntax::parser::parse(&combined).module;
    let index = SymbolIndex::build(&module);
    (index, offset, user)
}

fn kind_label(k: &SymKind) -> &'static str {
    match k {
        SymKind::Class => "class",
        SymKind::Method => "method",
        SymKind::Constructor => "constructor",
        SymKind::Getter => "getter",
        SymKind::Setter => "setter",
        SymKind::StaticMethod => "static method",
        SymKind::Field => "field",
        SymKind::StaticField => "static field",
        SymKind::ModuleVariable => "variable",
        SymKind::LocalVariable => "local",
        SymKind::Parameter => "parameter",
        SymKind::LoopVariable => "loop variable",
        SymKind::Import => "import",
        SymKind::ImportedSymbol => "imported",
    }
}

/// Monaco CompletionItemKind bucket: 0=Class 1=Method 2=Field 3=Variable 4=Module.
fn kind_code(k: &SymKind) -> u8 {
    match k {
        SymKind::Class => 0,
        SymKind::Method | SymKind::StaticMethod | SymKind::Getter | SymKind::Setter | SymKind::Constructor => 1,
        SymKind::Field | SymKind::StaticField => 2,
        SymKind::ModuleVariable | SymKind::LocalVariable | SymKind::Parameter | SymKind::LoopVariable => 3,
        SymKind::Import | SymKind::ImportedSymbol => 4,
    }
}

/// Resolve the symbol at a combined-source offset (a definition, or a reference
/// pointing at one).
fn def_at<'a>(index: &'a SymbolIndex, off: usize) -> Option<&'a SymbolDef> {
    if let Some(d) = index.definition_at(off) {
        return Some(d);
    }
    let r = index.reference_at(off)?;
    index.find_definition(&r.name)
}

fn out() -> &'static mut Vec<u8> {
    unsafe { &mut *addr_of_mut!(OUT) }
}

/// Hover at the given USER byte offset. Writes a markdown string to OUT; returns
/// its byte length (0 if nothing under the cursor).
#[unsafe(no_mangle)]
pub extern "C" fn hover_at(user_off: usize) -> usize {
    let (index, offset, _user) = build_index();
    let out = out();
    out.clear();
    if let Some(def) = def_at(&index, user_off + offset) {
        let sig = def.detail.clone().unwrap_or_else(|| def.name.clone());
        let mut s = alloc_string_fmt(&sig);
        match &def.container {
            Some(c) => {
                s.push_str("  ·  ");
                s.push_str(kind_label(&def.kind));
                s.push_str(" in ");
                s.push_str(c);
            }
            None => {
                s.push_str("  ·  ");
                s.push_str(kind_label(&def.kind));
            }
        }
        if let Some(doc) = &def.doc {
            s.push_str("\n\n");
            s.push_str(doc);
        }
        out.extend_from_slice(s.as_bytes());
    }
    out.len()
}

fn alloc_string_fmt(sig: &str) -> String {
    let mut s = String::with_capacity(sig.len() + 4);
    s.push_str("**");
    s.push_str(sig);
    s.push_str("**");
    s
}

/// Definition location for the symbol at a USER byte offset. Writes a fixed
/// record `[found:u8][in_prelude:u8][startLine,startCol,endLine,endCol:u32]`
/// (10 bytes) or nothing (len 0). Lines/cols are 1-based, user coordinates.
#[unsafe(no_mangle)]
pub extern "C" fn definition_at(user_off: usize) -> usize {
    let (index, offset, user) = build_index();
    let out = out();
    out.clear();
    if let Some(def) = def_at(&index, user_off + offset) {
        let span = def.name_span;
        let in_prelude = span.start < offset;
        out.push(1); // found
        out.push(in_prelude as u8);
        if in_prelude {
            out.extend_from_slice(&[0u8; 16]);
        } else {
            let (sl, sc) = line_col(&user, span.start - offset);
            let (el, ec) = line_col(&user, span.end - offset);
            out.extend_from_slice(&sl.to_le_bytes());
            out.extend_from_slice(&sc.to_le_bytes());
            out.extend_from_slice(&el.to_le_bytes());
            out.extend_from_slice(&ec.to_le_bytes());
        }
    }
    out.len()
}

/// All top-level completion candidates (prelude classes + user-defined symbols).
/// Writes `u32 count` then per item `[kind:u8][label_len:u32][label][detail_len:u32][detail]`.
#[unsafe(no_mangle)]
pub extern "C" fn completions() -> usize {
    let (index, _offset, _user) = build_index();
    let cands = index.completion_candidates();
    let out = out();
    out.clear();
    // Only top-level names belong in unqualified completion (classes + module
    // vars, prelude or user); methods/params need a receiver and come from the
    // editor's static prelude list after a `.`.
    let top: Vec<&SymbolDef> = cands
        .into_iter()
        .filter(|d| matches!(d.kind, SymKind::Class | SymKind::ModuleVariable))
        .collect();
    out.extend_from_slice(&(top.len() as u32).to_le_bytes());
    for def in top {
        out.push(kind_code(&def.kind));
        let label = def.name.as_bytes();
        out.extend_from_slice(&(label.len() as u32).to_le_bytes());
        out.extend_from_slice(label);
        let detail = def.detail.as_deref().unwrap_or(kind_label(&def.kind));
        out.extend_from_slice(&(detail.len() as u32).to_le_bytes());
        out.extend_from_slice(detail.as_bytes());
    }
    out.len()
}

/// Byte offset → (1-based line, 1-based UTF-16 column) for Monaco.
fn line_col(src: &str, byte: usize) -> (u32, u32) {
    let byte = byte.min(src.len());
    let mut line = 1u32;
    let mut line_start = 0usize;
    for (i, &b) in src.as_bytes().iter().enumerate() {
        if i >= byte {
            break;
        }
        if b == b'\n' {
            line += 1;
            line_start = i + 1;
        }
    }
    let col = src.get(line_start..byte).map(|s| s.encode_utf16().count()).unwrap_or(0) as u32 + 1;
    (line, col)
}
