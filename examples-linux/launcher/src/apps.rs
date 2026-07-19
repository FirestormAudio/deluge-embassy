//! Enumerate runnable apps in the SD card's app folder. The card's
//! `LINUX/APPS/` folder is reached at `/sd/LINUX/APPS` at runtime, since the
//! appliance init mounts the SD at `/sd`.

use std::path::{Path, PathBuf};

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AppEntry {
    pub path: PathBuf,
    pub display_name: String,
}

/// The app directory: `$LAUNCHER_APPS_DIR` if set, else `/sd/LINUX/APPS`
/// (the card's `LINUX/APPS/` folder under the `/sd` SD mountpoint).
pub fn apps_dir() -> PathBuf {
    std::env::var_os("LAUNCHER_APPS_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("/sd/LINUX/APPS"))
}

/// List regular, executable files in `dir`, sorted by display name.
/// A missing or unreadable directory yields an empty list.
pub fn scan(dir: &Path) -> Vec<AppEntry> {
    use std::os::unix::fs::PermissionsExt;

    let mut out = Vec::new();
    let Ok(rd) = std::fs::read_dir(dir) else {
        return out;
    };
    for entry in rd.flatten() {
        let path = entry.path();
        let Ok(meta) = entry.metadata() else { continue };
        if !meta.is_file() {
            continue;
        }
        if meta.permissions().mode() & 0o111 == 0 {
            continue; // not executable by anyone
        }
        let Some(stem) = path.file_name().and_then(|s| s.to_str()) else {
            continue;
        };
        out.push(AppEntry {
            path: path.clone(),
            display_name: stem.to_uppercase(),
        });
    }
    out.sort_by(|a, b| a.display_name.cmp(&b.display_name));
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::os::unix::fs::PermissionsExt;

    fn write_exec(dir: &Path, name: &str) {
        let p = dir.join(name);
        std::fs::write(&p, b"#!/bin/sh\n").unwrap();
        std::fs::set_permissions(&p, std::fs::Permissions::from_mode(0o755)).unwrap();
    }

    fn tmpdir() -> PathBuf {
        use std::sync::atomic::{AtomicU64, Ordering};
        static N: AtomicU64 = AtomicU64::new(0);
        let uniq = format!(
            "{}-{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos(),
            N.fetch_add(1, Ordering::Relaxed),
        );
        let base = std::env::temp_dir().join(format!("launcher-apps-{uniq}"));
        let _ = std::fs::remove_dir_all(&base);
        std::fs::create_dir_all(&base).unwrap();
        base
    }

    #[test]
    fn lists_executables_uppercased_and_sorted() {
        let d = tmpdir();
        write_exec(&d, "spark");
        write_exec(&d, "wren");
        let apps = scan(&d);
        assert_eq!(apps.len(), 2);
        assert_eq!(apps[0].display_name, "SPARK");
        assert_eq!(apps[1].display_name, "WREN");
        assert_eq!(apps[0].path, d.join("spark"));
    }

    #[test]
    fn skips_non_executable_and_non_files() {
        let d = tmpdir();
        write_exec(&d, "runnable");
        std::fs::write(d.join("readme.txt"), b"hi").unwrap(); // not executable
        std::fs::create_dir(d.join("subdir")).unwrap(); // not a file
        let apps = scan(&d);
        assert_eq!(apps.len(), 1);
        assert_eq!(apps[0].display_name, "RUNNABLE");
    }

    #[test]
    fn missing_dir_yields_empty() {
        assert!(scan(Path::new("/no/such/dir/xyz")).is_empty());
    }

    #[test]
    fn default_apps_dir_is_under_sd_mount() {
        // The SD is mounted at /sd by the appliance init, so the card's
        // LINUX/APPS/ folder lives at /sd/LINUX/APPS at runtime. Only assert
        // the shipped default when no override is set.
        if std::env::var_os("LAUNCHER_APPS_DIR").is_none() {
            assert_eq!(apps_dir(), PathBuf::from("/sd/LINUX/APPS"));
        }
    }
}
