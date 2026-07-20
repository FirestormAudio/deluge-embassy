//! Small shared helpers used across subcommands.

use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

/// Return the value following `flag` in `args`, if present.
pub(crate) fn arg_value(args: &[String], flag: &str) -> Option<String> {
    let i = args.iter().position(|a| a == flag)?;
    args.get(i + 1).cloned()
}

/// Locate `llvm-objcopy`: prefer one on `PATH` (`llvm-objcopy`, then
/// cargo-binutils' `rust-objcopy`), else the binary shipped by the active
/// toolchain's `llvm-tools` component at `<sysroot>/lib/rustlib/<host>/bin`.
pub(crate) fn locate_objcopy() -> Option<PathBuf> {
    for cand in ["llvm-objcopy", "rust-objcopy"] {
        if Command::new(cand)
            .arg("--version")
            .output()
            .is_ok_and(|o| o.status.success())
        {
            return Some(PathBuf::from(cand));
        }
    }

    let sysroot = Command::new("rustc").args(["--print", "sysroot"]).output().ok()?;
    let sysroot = String::from_utf8(sysroot.stdout).ok()?;
    let verbose = Command::new("rustc").arg("-vV").output().ok()?;
    let verbose = String::from_utf8(verbose.stdout).ok()?;
    let host = verbose.lines().find_map(|l| l.strip_prefix("host: "))?;

    let path = Path::new(sysroot.trim())
        .join("lib")
        .join("rustlib")
        .join(host)
        .join("bin")
        .join("llvm-objcopy");
    path.is_file().then_some(path)
}

/// Strip `elf` down to a `.stripped` sibling with `llvm-objcopy --strip-all`,
/// returning its path. If objcopy can't be located, warns and returns `elf`
/// unchanged (caller proceeds with the larger, unstripped binary).
pub(crate) fn strip_elf(elf: &Path) -> Result<PathBuf, String> {
    let Some(objcopy) = locate_objcopy() else {
        eprintln!(
            "warning: llvm-objcopy not found (add it with \
             `rustup component add llvm-tools-preview`); using the unstripped ELF"
        );
        return Ok(elf.to_path_buf());
    };

    let out = elf.with_file_name(format!(
        "{}.stripped",
        elf.file_name().and_then(|n| n.to_str()).unwrap_or("app")
    ));
    let status = Command::new(&objcopy)
        .arg("--strip-all")
        .arg(elf)
        .arg(&out)
        .status()
        .map_err(|e| format!("running {}: {e}", objcopy.display()))?;
    if !status.success() {
        return Err("llvm-objcopy --strip-all failed".to_string());
    }

    if let (Ok(before), Ok(after)) = (fs::metadata(elf), fs::metadata(&out)) {
        println!(
            "stripped {} -> {} bytes ({}% smaller)",
            before.len(),
            after.len(),
            before
                .len()
                .checked_sub(after.len())
                .map(|d| d * 100 / before.len().max(1))
                .unwrap_or(0)
        );
    }

    Ok(out)
}

/// Write `contents` to `path`, creating any missing parent directories.
pub(crate) fn write(path: &Path, contents: &str) -> Result<(), String> {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent).map_err(|e| format!("creating {}: {e}", parent.display()))?;
    }
    fs::write(path, contents).map_err(|e| format!("writing {}: {e}", path.display()))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn arg_value_finds_and_misses() {
        let a: Vec<String> = ["run", "--dest", "/mnt/sd", "--release"]
            .iter()
            .map(|s| s.to_string())
            .collect();
        assert_eq!(arg_value(&a, "--dest"), Some("/mnt/sd".to_string()));
        assert_eq!(arg_value(&a, "--missing"), None);
    }

    #[test]
    fn arg_value_flag_without_value_is_none() {
        let a = vec!["--dest".to_string()];
        assert_eq!(arg_value(&a, "--dest"), None);
    }
}
