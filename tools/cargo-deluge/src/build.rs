//! `cargo deluge build`: compile the current app to an ELF, plus the cargo
//! metadata / manifest parsing needed to locate the produced artifact.

use std::fs;
use std::path::PathBuf;
use std::process::Command;

use crate::TARGET;

/// Build the current app; returns the path to the produced ELF.
pub(crate) fn cmd_build(args: &[String]) -> Result<PathBuf, String> {
    let release = args.iter().any(|a| a == "--release");

    let mut cmd = Command::new("cargo");
    cmd.args([
        "build",
        "--target",
        TARGET,
        "-Zbuild-std=core",
        "-Zbuild-std-features=compiler-builtins-mem",
    ]);
    if release {
        cmd.arg("--release");
    }

    let status = cmd
        .status()
        .map_err(|e| format!("failed to run cargo: {e}"))?;
    if !status.success() {
        return Err("build failed".to_string());
    }

    let name = package_name()?;
    let profile = if release { "release" } else { "debug" };
    // Ask cargo where it actually put the artifacts instead of assuming
    // `./target`: inside a workspace the output goes to the *workspace-root*
    // target dir, not the member's directory, so the old `./target` guess
    // failed for every firmware in this repo's `firmwares/` workspace.
    let elf = target_dir()?.join(TARGET).join(profile).join(&name);
    if !elf.is_file() {
        return Err(format!("expected ELF not found at {}", elf.display()));
    }
    println!("built {}", elf.display());
    Ok(elf)
}

/// Resolve cargo's target directory (honours workspaces, `CARGO_TARGET_DIR`,
/// and `.cargo/config.toml` overrides) via `cargo metadata`.
///
/// Parsed without a JSON dependency — the tool deliberately keeps `serialport`
/// as its only dep — by pulling the single `"target_directory"` string field.
pub(crate) fn target_dir() -> Result<PathBuf, String> {
    let out = Command::new("cargo")
        .args(["metadata", "--format-version", "1", "--no-deps"])
        .output()
        .map_err(|e| format!("failed to run cargo metadata: {e}"))?;
    if !out.status.success() {
        return Err("cargo metadata failed".to_string());
    }
    let json = String::from_utf8_lossy(&out.stdout);
    extract_json_string(&json, "target_directory")
        .map(PathBuf::from)
        .ok_or_else(|| "cargo metadata did not report a target_directory".to_string())
}

/// Extract the string value of a top-level-ish `"key":"value"` pair from JSON
/// text, decoding the basic escapes cargo can emit in paths (`\\`, `\"`, `\/`).
/// Good enough for the flat metadata fields we read; not a general parser.
fn extract_json_string(json: &str, key: &str) -> Option<String> {
    let needle = format!("\"{key}\":\"");
    let start = json.find(&needle)? + needle.len();
    let mut out = String::new();
    let mut chars = json[start..].chars();
    while let Some(c) = chars.next() {
        match c {
            '"' => return Some(out),
            '\\' => match chars.next()? {
                '\\' => out.push('\\'),
                '"' => out.push('"'),
                '/' => out.push('/'),
                other => {
                    out.push('\\');
                    out.push(other);
                }
            },
            other => out.push(other),
        }
    }
    None
}

/// Read the package (bin) name from `./Cargo.toml` (minimal TOML scan).
pub(crate) fn package_name() -> Result<String, String> {
    let toml = fs::read_to_string("Cargo.toml")
        .map_err(|_| "no Cargo.toml in the current directory — run inside an app".to_string())?;
    parse_package_name(&toml).ok_or_else(|| "could not find package name in Cargo.toml".to_string())
}

/// Extract `name = "…"` from the `[package]` table of a Cargo manifest.
/// A `name` key in any other table (e.g. `[[bin]]`) is ignored.
pub(crate) fn parse_package_name(toml: &str) -> Option<String> {
    let mut in_package = false;
    for line in toml.lines() {
        let t = line.trim();
        if t.starts_with('[') {
            in_package = t == "[package]";
            continue;
        }
        if in_package
            && let Some(rest) = t.strip_prefix("name")
            && let Some(q1) = rest.find('"')
            && let Some(q2) = rest[q1 + 1..].find('"')
        {
            return Some(rest[q1 + 1..q1 + 1 + q2].to_string());
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn package_name_from_package_table() {
        let toml = "[package]\nname = \"cool-app\"\nversion = \"0.1.0\"\n";
        assert_eq!(parse_package_name(toml).as_deref(), Some("cool-app"));
    }

    #[test]
    fn package_name_ignores_bin_table_name() {
        // The [[bin]] name must not shadow the [package] name, even when the
        // [[bin]] table appears first.
        let toml = "[[bin]]\nname = \"the-bin\"\n\n[package]\nname = \"the-pkg\"\n";
        assert_eq!(parse_package_name(toml).as_deref(), Some("the-pkg"));
    }

    #[test]
    fn package_name_absent() {
        assert_eq!(parse_package_name("[dependencies]\nfoo = \"1\"\n"), None);
    }
}
