//! Deluge desktop front-panel simulator (binary front-end).
//!
//! Drives an external brain over the [`deluge_protocol`] wire — point it at
//! DelugeFirmware's native `deluge_host` (whose `host_link` bridge listens on the
//! stream) for a true emulator, or omit `--connect` for a passive, blank panel.
//!
//! ```text
//! DELUGE_HOST_LINK=127.0.0.1:9000   ./build-sim/deluge_host   # the brain (TCP)
//! deluge-simulator --connect 127.0.0.1:9000                   # the panel
//! ```
//!
//! To run an SDK app *in the same process* instead, use `cargo deluge sim` (which
//! calls [`deluge_simulator::run_in_process`]).

use clap::Parser;

#[derive(Parser)]
#[command(
    name = "deluge-simulator",
    about = "Desktop front panel for a Deluge brain"
)]
struct Cli {
    /// Connect to a Deluge brain (e.g. DelugeFirmware's `deluge_host`, launched
    /// with `DELUGE_HOST_LINK=<target>`). Accepts a TCP endpoint (`host:port`,
    /// `:port`, or `tcp://host:port`) on any platform, or a Unix domain socket
    /// path on Unix. Omit to open a blank, passive panel.
    #[arg(long, value_name = "TARGET")]
    connect: Option<String>,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    let cli = Cli::parse();
    deluge_simulator::run_connected(cli.connect.as_deref())
}
