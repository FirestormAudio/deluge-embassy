//! Launch and supervise a child application.
//!
//! The child is put in its own session/process group (`setsid`) so the kill
//! chord can tear down the whole group with `kill(-pgid, SIGKILL)`.

use std::io;
use std::os::unix::process::{CommandExt, ExitStatusExt};
use std::path::Path;
use std::process::{Child, Command, ExitStatus};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Exit {
    Clean,
    Nonzero(i32),
    Signalled(i32),
}

pub fn classify(status: ExitStatus) -> Exit {
    if let Some(code) = status.code() {
        if code == 0 {
            Exit::Clean
        } else {
            Exit::Nonzero(code)
        }
    } else if let Some(sig) = status.signal() {
        Exit::Signalled(sig)
    } else {
        Exit::Nonzero(-1)
    }
}

pub struct Launched {
    pub pid: i32,
    pub child: Child,
}

pub fn launch(path: &Path) -> io::Result<Launched> {
    let mut cmd = Command::new(path);
    // SAFETY: setsid is async-signal-safe and touches no Rust heap state.
    unsafe {
        cmd.pre_exec(|| {
            if libc::setsid() < 0 {
                return Err(io::Error::last_os_error());
            }
            Ok(())
        });
    }
    let child = cmd.spawn()?;
    let pid = child.id() as i32; // setsid makes pgid == pid
    Ok(Launched { pid, child })
}

pub fn kill_group(pid: i32) {
    // Negative pid signals the whole process group. Ignore ESRCH (already gone).
    unsafe {
        libc::kill(-pid, libc::SIGKILL);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clean_exit_classifies_clean() {
        let mut l = launch(Path::new("/bin/true")).unwrap();
        let status = l.child.wait().unwrap();
        assert_eq!(classify(status), Exit::Clean);
    }

    #[test]
    fn nonzero_exit_classifies_nonzero() {
        let mut l = launch(Path::new("/bin/false")).unwrap();
        let status = l.child.wait().unwrap();
        assert_eq!(classify(status), Exit::Nonzero(1));
    }

    #[test]
    fn kill_group_signals_the_child() {
        use std::os::unix::process::CommandExt;
        let mut cmd = std::process::Command::new("/bin/sleep");
        cmd.arg("60");
        unsafe {
            cmd.pre_exec(|| {
                if libc::setsid() < 0 {
                    return Err(std::io::Error::last_os_error());
                }
                Ok(())
            });
        }
        let mut child = cmd.spawn().unwrap();
        let pid = child.id() as i32;
        std::thread::sleep(std::time::Duration::from_millis(50));
        kill_group(pid);
        let status = child.wait().unwrap();
        assert_eq!(classify(status), Exit::Signalled(libc::SIGKILL));
    }

    #[test]
    fn launching_missing_binary_errors() {
        assert!(launch(Path::new("/no/such/binary/zzz")).is_err());
    }
}
