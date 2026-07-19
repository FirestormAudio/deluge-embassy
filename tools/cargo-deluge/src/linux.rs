//! `cargo deluge linux`: build the current app for the Deluge's Linux userland
//! (armv7 musl, static) against the bundle's libdeluge sysroot, then pack it
//! into an app image — or a `--bare` binary — with the bundle's deluge-mkimage.
//!
//! Unlike the device path, this needs no `-Zbuild-std` (the musl std is a
//! rustup target) and no local sysroot build (the bundle ships libdeluge +
//! deluge-mkimage). Point it at an unpacked bundle with `DELUGE_BASE`.

use std::path::PathBuf;
use std::process::Command;

use crate::build::{package_name, target_dir};
use crate::util::arg_value;

/// The Deluge's Linux userland triple (musl, static).
const LINUX_TARGET: &str = "armv7-unknown-linux-musleabihf";

pub(crate) fn cmd_linux(args: &[String]) -> Result<(), String> {
    let debug = args.iter().any(|a| a == "--debug");
    let bare = args.iter().any(|a| a == "--bare");
    let out = arg_value(args, "--out");

    let base = std::env::var("DELUGE_BASE")
        .map_err(|_| "set DELUGE_BASE to an unpacked deluge-linux bundle".to_string())?;
    let sysroot = format!("{base}/toolchain/arm-buildroot-linux-musleabihf/sysroot/usr");
    let mkimage = PathBuf::from(&base).join("tools/deluge-mkimage");
    if !mkimage.is_file() {
        return Err(format!("bundle has no packer at {}", mkimage.display()));
    }
    // Prepend the bundle toolchain bin so the config's `arm-linux-gcc` resolves.
    let path = format!(
        "{base}/toolchain/bin:{}",
        std::env::var("PATH").unwrap_or_default()
    );

    let mut cmd = Command::new("cargo");
    cmd.args(["build", "--target", LINUX_TARGET]);
    if !debug {
        cmd.arg("--release");
    }
    cmd.env("DELUGE_SDK_ROOT", &sysroot).env("PATH", &path);
    if !cmd.status().map_err(|e| format!("cargo: {e}"))?.success() {
        return Err("build failed".to_string());
    }

    let name = package_name()?;
    let profile = if debug { "debug" } else { "release" };
    let bin = target_dir()?.join(LINUX_TARGET).join(profile).join(&name);
    if !bin.is_file() {
        return Err(format!("binary not found at {}", bin.display()));
    }

    let mut mk = Command::new(&mkimage);
    mk.arg(&bin);
    let product = if bare {
        let p = target_dir()?.join("bare").join(&name);
        if let Some(dir) = p.parent() {
            std::fs::create_dir_all(dir).ok();
        }
        mk.arg("--bare").arg("-o").arg(&p);
        p
    } else {
        let stem = out.unwrap_or_else(|| name.to_uppercase());
        let p = target_dir()?.join(format!("{stem}.ELF"));
        mk.arg("-o").arg(&p);
        p
    };
    if !mk.status().map_err(|e| format!("deluge-mkimage: {e}"))?.success() {
        return Err("deluge-mkimage failed".to_string());
    }
    println!("packed {}", product.display());
    Ok(())
}
