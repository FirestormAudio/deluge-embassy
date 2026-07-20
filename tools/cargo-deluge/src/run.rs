//! `cargo deluge run`: build, strip, and push the ELF to a Deluge over USB
//! (dev mode), then launch it from RAM.

use std::fs;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::time::Duration;

use crate::build::cmd_build;
use crate::frame::build_frame;
use crate::log::tail_log;
use crate::util::arg_value;
use crate::{DELUGE_PID, DELUGE_VID, UPLOAD_PRODUCT};

pub(crate) fn cmd_run(args: &[String]) -> Result<(), String> {
    // `run` is USB upload only; the SD build-and-copy path is `deploy`.
    if arg_value(args, "--dest").is_some() {
        return Err(
            "`run` uploads over USB; use `cargo deluge deploy --dest <sd-mount>` to copy \
             the ELF to /APPS/ instead."
                .to_string(),
        );
    }

    let elf = cmd_build(args)?;

    // Strip everything the on-device loader never reads (it parses only the ELF
    // program headers + entry point) before sending.  A debug build is mostly
    // `.debug_*`/symbol sections that never become a PT_LOAD segment, so they
    // bloat the transfer for zero on-device benefit.  `--no-strip` opts out.
    let stripped_elf = if args.iter().any(|a| a == "--no-strip") {
        elf.clone()
    } else {
        strip_for_upload(&elf)?
    };

    upload_elf(&stripped_elf, args)
}

/// Upload an already-built ELF (device-loadable: PT_LOAD segments + entry
/// point) to a Deluge over USB (dev mode) and launch it from RAM. Shared by
/// `run` (after stripping a freshly built device ELF) and `linux --run`
/// (an already segment-only appliance image, no stripping needed).
/// `cargo deluge upload <path> [--port <p>] [--log]` — stream an already-built
/// ELF / appliance image straight to the device, skipping the build+pack step.
/// Used to re-test a packed image without rebuilding (e.g. when the source tree
/// has moved on but the on-disk image is the one we want on the device).
pub(crate) fn cmd_upload(args: &[String]) -> Result<(), String> {
    let mut path = None;
    let mut it = args.iter();
    while let Some(a) = it.next() {
        if a == "--port" {
            it.next(); // skip the flag's value
            continue;
        }
        if a.starts_with("--") {
            continue; // other flags (e.g. --log)
        }
        path = Some(a.as_str());
        break;
    }
    let path = path.ok_or("usage: cargo deluge upload <elf-path> [--port <p>] [--log]")?;
    upload_elf(Path::new(path), args)
}

pub(crate) fn upload_elf(elf: &Path, args: &[String]) -> Result<(), String> {
    // The Deluge must be sitting on the boot menu with DEV MODE on (its
    // background CDC listener is what we upload to).
    let bytes = fs::read(elf).map_err(|e| format!("reading {}: {e}", elf.display()))?;
    if bytes.len() > MAX_UPLOAD_BYTES {
        return Err(format!(
            "image is {} bytes, larger than the SDRAM app region ({MAX_UPLOAD_BYTES} bytes) \
             — build with --release, or trim the app.",
            bytes.len(),
        ));
    }
    let frame = build_frame(&bytes);

    let port_path = match arg_value(args, "--port") {
        Some(p) => p,
        None => discover_upload_port()?,
    };
    println!(
        "uploading {} ({} bytes) to {port_path}",
        elf.display(),
        bytes.len()
    );

    let mut port = serialport::new(&port_path, 115_200)
        .timeout(Duration::from_secs(5))
        .open()
        .map_err(|e| format!("opening {port_path}: {e}"))?;

    upload(&mut *port, &frame)?;
    println!("\nuploaded — the Deluge is loading and launching it from RAM.");

    if args.iter().any(|a| a == "--log") {
        // The launched app re-enumerates as its own USB-log CDC; reconnect and
        // tail it. Best-effort: the port path may change after re-enumeration.
        drop(port);
        tail_log()?;
    }
    Ok(())
}

/// The device streams uploads straight to each segment's load address — there is
/// no whole-image scratch window anymore — so the only real bound is that the
/// segments fit in the SDRAM app region (`SDRAM_LO..SDRAM_HI`, `0x0C000000..
/// 0x0FD20000`). Catch an absurd image locally with a useful message instead of
/// streaming megabytes only for the device to reject a segment.
const MAX_UPLOAD_BYTES: usize = 0x0FD2_0000 - 0x0C00_0000; // SDRAM app-region span

/// Strip the ELF down to what the loader reads (PT_LOAD segments + entry point)
/// via [`util::strip_elf`], so `run` still works (just with a larger transfer)
/// if objcopy can't be located.
fn strip_for_upload(elf: &Path) -> Result<PathBuf, String> {
    crate::util::strip_elf(elf)
}

/// Write the framed image, showing a local progress bar (USB bulk flow-control
/// naturally paces this against how fast the device drains the data).
fn upload(port: &mut dyn serialport::SerialPort, frame: &[u8]) -> Result<(), String> {
    const CHUNK: usize = 4096;
    let mut written = 0usize;
    let total = frame.len();
    while written < total {
        let n = (total - written).min(CHUNK);
        port.write_all(&frame[written..written + n])
            .map_err(|e| format!("writing to port: {e}"))?;
        written += n;
        print_progress(written, total);
    }
    port.flush().map_err(|e| format!("flushing port: {e}"))?;
    Ok(())
}

/// Print/refresh a single-line `[####    ] 42%` progress bar on stderr.
fn print_progress(done: usize, total: usize) {
    let pct = (done * 100).checked_div(total).unwrap_or(100);
    let filled = pct * 30 / 100;
    let bar: String = (0..30)
        .map(|i| if i < filled { '#' } else { ' ' })
        .collect();
    eprint!("\r[{bar}] {pct:>3}%");
    let _ = std::io::stderr().flush();
}

/// Find the Deluge dev-upload CDC port: a USB serial port with the Deluge
/// VID/PID whose product string names the upload listener.  If exactly one
/// Deluge serial port is present we use it even without the product match (some
/// platforms don't expose product strings); ambiguity is reported.
fn discover_upload_port() -> Result<String, String> {
    use serialport::SerialPortType;
    let ports = serialport::available_ports()
        .map_err(|e| format!("enumerating serial ports: {e}"))?;

    let mut deluge: Vec<(String, Option<String>)> = Vec::new();
    for p in ports {
        if let SerialPortType::UsbPort(info) = &p.port_type
            && info.vid == DELUGE_VID
            && info.pid == DELUGE_PID
        {
            deluge.push((p.port_name.clone(), info.product.clone()));
        }
    }

    // Prefer a product-string match for the upload listener.
    if let Some((name, _)) = deluge
        .iter()
        .find(|(_, prod)| prod.as_deref().is_some_and(|s| s.contains(UPLOAD_PRODUCT)))
    {
        return Ok(name.clone());
    }

    match deluge.as_slice() {
        [] => Err(
            "no Deluge serial port found. Make sure the unit is on the boot menu \
             with DEV MODE: ON, connected over USB. Pass --port <path> to override."
                .to_string(),
        ),
        [(name, _)] => Ok(name.clone()),
        many => {
            let list = many
                .iter()
                .map(|(n, p)| format!("  {n}  ({})", p.as_deref().unwrap_or("?")))
                .collect::<Vec<_>>()
                .join("\n");
            Err(format!(
                "multiple Deluge serial ports found; pick one with --port <path>:\n{list}"
            ))
        }
    }
}
