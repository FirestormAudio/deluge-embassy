//! `cargo deluge` — build and scaffold Deluge SDK apps.
//!
//! A thin host-side cargo subcommand so app authors never touch `-Zbuild-std`,
//! linker flags, or the embedded target triple. Pure std, no dependencies.
//!
//! Subcommands:
//! - `cargo deluge new <name>`  — scaffold a new app crate.
//! - `cargo deluge build [--release]` — build the current app → ELF.
//! - `cargo deluge run [--release] [--port <path>] [--log]` — build, then upload
//!   the ELF straight to a Deluge over USB (dev mode) and launch it from RAM.
//! - `cargo deluge deploy [--release] [--dest <sd-mount>]` — copy the ELF to a
//!   mounted Deluge SD card's `/APPS/` instead.
//! - `cargo deluge log [--port <path>]` — connect to a running app's USB
//!   serial-log channel (the `usb-log` feature) and stream it to stdout.
//! - `cargo deluge linux [--bare] [--out <NAME>] [--features <list>] [--no-strip]`
//!   — build the current app for the Deluge's Linux userland and pack it with
//!   the bundle's deluge-mkimage. Needs DELUGE_BASE set to an unpacked bundle.
//! - `cargo deluge debug [--release] [-- <args>]` — build, then `probe-rs run`
//!   (J-Link).
//! - `cargo deluge trace [--release] [--flow] [--duration-ms N] [-- <args>]` —
//!   build, then `probe-rs read-trace` (J-Link, `trace-a9` fork).
//!
//! Apps are loaded by the app-loader as **ELF** and run from RAM; this tool only
//! ever emits an ELF (UF2 is the separate firmware-flash path).
//!
//! Each subcommand lives in its own module; this file is just argument dispatch,
//! the shared device constants, and the help text.

mod build;
mod deploy;
mod frame;
mod linux;
mod log;
mod new;
mod probe;
mod run;
mod sim;
mod util;

use std::env;
use std::process::ExitCode;

/// Embedded target triple for the Deluge (RZ/A1L, Cortex-A9).
pub(crate) const TARGET: &str = "armv7a-none-eabihf";
/// probe-rs chip identifier for the RZ/A1L on the Deluge.
pub(crate) const CHIP: &str = "R7S721020";

/// USB VID/PID the Deluge presents in every device mode (shared with the
/// app-loader / firmware — see `app-loader/src/devupload.rs`).
pub(crate) const DELUGE_VID: u16 = 0x16D0;
pub(crate) const DELUGE_PID: u16 = 0x0EDA;
/// Product string the dev-upload CDC listener advertises.
pub(crate) const UPLOAD_PRODUCT: &str = "Dev Upload";
/// Substring of the product string the SDK's USB-log CDC advertises
/// (`deluge::usb_debug` sets `"Deluge (SDK log)"`); used to single it out from
/// the dev-upload listener, which shares the same VID/PID.
pub(crate) const LOG_PRODUCT: &str = "SDK log";

fn main() -> ExitCode {
    let mut args: Vec<String> = env::args().skip(1).collect();
    // When invoked as `cargo deluge …`, cargo passes "deluge" as the first arg.
    if args.first().map(String::as_str) == Some("deluge") {
        args.remove(0);
    }

    let cmd = args.first().cloned().unwrap_or_default();
    let rest = &args[args.len().min(1)..];

    let result = match cmd.as_str() {
        "new" => new::cmd_new(rest),
        "build" => build::cmd_build(rest).map(|_| ()),
        "run" => run::cmd_run(rest),
        "deploy" => deploy::cmd_deploy(rest),
        "log" => log::cmd_log(rest),
        "sim" => sim::cmd_sim(rest),
        "linux" => linux::cmd_linux(rest),
        "debug" => probe::cmd_debug(rest),
        "trace" => probe::cmd_trace(rest),
        "" | "help" | "-h" | "--help" => {
            print_help();
            Ok(())
        }
        other => Err(format!("unknown subcommand `{other}`\n\n{HELP}")),
    };

    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("error: {e}");
            ExitCode::FAILURE
        }
    }
}

const HELP: &str = "\
Usage: cargo deluge <command>

Commands:
  new <name>                 Scaffold a new Deluge app crate
  build [--release]          Build the current app for the Deluge (-> ELF)
  run [--release] [opts]     Build, then upload the ELF to a Deluge over USB and
                             launch it from RAM (requires DEV MODE on the unit).
                             The ELF is stripped to its loadable segments first.
                               --port <path>  serial port override (else auto)
                               --log          tail the app's USB log after launch
                               --no-strip     send the full ELF (debug info etc.)
  deploy [--release] [opts]  Build, then copy the ELF to a mounted Deluge SD card.
                               --dest <dir>   copy to <dir>/APPS/ (else print how)
  log [--port <path>]        Connect to a running app's USB serial-log channel
                             (the `usb-log` feature) and stream it to stdout.
                               --port <path>  serial port override (else auto)
  sim [--release]            Build for the host and run the app in the desktop
                             simulator (OLED/pads/LEDs/audio), no hardware needed.
  linux [opts]               Build the current app for the Deluge's Linux userland
                             (armv7 musl, static) and pack it with the bundle's
                             deluge-mkimage. Needs DELUGE_BASE=<unpacked bundle>.
                               --bare           emit the bare /LINUX/APPS/ binary
                               --out N          image basename (else PACKAGE-NAME)
                               --features LIST  cargo --features to build with
                               --no-strip       keep debug info (larger image)
  debug [--release] [-- ...] Build, then `probe-rs run` over J-Link (--chip set)
  trace [--release] [opts]   Build, then `probe-rs read-trace` (trace-a9 fork):
                               --flow         compact execution-flow view
                               --duration-ms N capture window (default 2000)
  help                       Show this help";

fn print_help() {
    println!("{HELP}");
}
