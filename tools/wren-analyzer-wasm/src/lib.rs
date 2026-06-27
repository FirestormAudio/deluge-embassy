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

use wren_analyzer::analyze;
use wren_syntax::ast::Severity;

static mut SRC: Vec<u8> = Vec::new();
static mut OUT: Vec<u8> = Vec::new();

/// Reserve `len` bytes of source buffer and return a pointer for JS to fill.
#[unsafe(no_mangle)]
pub extern "C" fn src_reserve(len: usize) -> *mut u8 {
    // SAFETY: single-threaded wasm; sole accessor.
    let s = unsafe { &mut *addr_of_mut!(SRC) };
    s.clear();
    s.resize(len, 0);
    s.as_mut_ptr()
}

/// Analyze the source currently in the buffer; returns the result byte length.
#[unsafe(no_mangle)]
pub extern "C" fn analyze_run() -> usize {
    let s = unsafe { &*addr_of_mut!(SRC) };
    let src = core::str::from_utf8(s).unwrap_or("");
    let diags = analyze(src);

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
