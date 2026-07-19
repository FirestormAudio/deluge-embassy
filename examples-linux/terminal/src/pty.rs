//! Spawn a child on a pseudo-terminal and talk to it over the master fd.
//!
//! The appliance rootfs does not mount `/dev/pts`; [`ensure_devpts`] mounts it
//! (idempotently) before [`spawn`]. `forkpty` then opens `/dev/ptmx` (present on
//! the auto-mounted devtmpfs) and allocates a slave.

use std::ffi::CString;
use std::io;
use std::os::unix::io::RawFd;
use std::ptr;
use std::sync::atomic::{AtomicBool, Ordering};

pub struct Pty {
    pub master: RawFd,
    pub child: libc::pid_t,
    reaped: AtomicBool,
}

/// Best-effort mount of devpts at `/dev/pts`. Ignores "already exists / mounted".
pub fn ensure_devpts() {
    let dir = CString::new("/dev/pts").unwrap();
    unsafe {
        libc::mkdir(dir.as_ptr(), 0o755);
    }
    let src = CString::new("devpts").unwrap();
    let fstype = CString::new("devpts").unwrap();
    let opts = CString::new("mode=0620,ptmxmode=0666").unwrap();
    unsafe {
        libc::mount(
            src.as_ptr(),
            dir.as_ptr(),
            fstype.as_ptr(),
            0,
            opts.as_ptr() as *const libc::c_void,
        );
    }
}

pub fn spawn(cols: u16, rows: u16) -> io::Result<Pty> {
    spawn_cmd("/bin/sh", &["sh", "-i"], cols, rows)
}

pub fn spawn_cmd(prog: &str, argv: &[&str], cols: u16, rows: u16) -> io::Result<Pty> {
    // All allocation happens here, in the parent, before forkpty. The child
    // branch below must remain async-signal-safe (no malloc/CString/Vec),
    // since by the time `spawn` runs the process is multi-threaded (SDK
    // input thread + pty-reader thread), and another thread could hold the
    // allocator lock across fork(), deadlocking the child forever.
    let cprog = CString::new(prog).map_err(|e| io::Error::new(io::ErrorKind::InvalidInput, e))?;
    let cargs: Vec<CString> = argv
        .iter()
        .map(|a| CString::new(*a).map_err(|e| io::Error::new(io::ErrorKind::InvalidInput, e)))
        .collect::<io::Result<_>>()?;
    let mut argv_ptrs: Vec<*const libc::c_char> = cargs.iter().map(|c| c.as_ptr()).collect();
    argv_ptrs.push(ptr::null());

    // Set TERM in the parent's environment so the child inherits it via the
    // normal `environ` on exec, with no post-fork setenv needed. The
    // terminal app's own TERM is irrelevant, so mutating it once here is
    // fine; setenv is idempotent across restarts.
    unsafe {
        let term = CString::new("TERM").unwrap();
        let vt = CString::new("vt100").unwrap();
        libc::setenv(term.as_ptr(), vt.as_ptr(), 1);
    }

    let mut master: libc::c_int = 0;
    let ws = libc::winsize { ws_row: rows, ws_col: cols, ws_xpixel: 0, ws_ypixel: 0 };
    let pid = unsafe { libc::forkpty(&mut master, ptr::null_mut(), ptr::null(), &ws) };
    if pid < 0 {
        return Err(io::Error::last_os_error());
    }
    if pid == 0 {
        // Child: only async-signal-safe calls from here on. `cprog`/`cargs`/
        // `argv_ptrs` were built in the parent above and are COW-inherited,
        // remaining valid until execv.
        unsafe {
            libc::prctl(libc::PR_SET_PDEATHSIG, libc::SIGKILL as libc::c_ulong, 0, 0, 0);
            libc::execv(cprog.as_ptr(), argv_ptrs.as_ptr());
            libc::_exit(127);
        }
    }
    Ok(Pty { master, child: pid, reaped: AtomicBool::new(false) })
}

impl Pty {
    pub fn write_all(&self, mut buf: &[u8]) -> io::Result<()> {
        while !buf.is_empty() {
            let n = unsafe { libc::write(self.master, buf.as_ptr() as *const libc::c_void, buf.len()) };
            if n < 0 {
                return Err(io::Error::last_os_error());
            }
            buf = &buf[n as usize..];
        }
        Ok(())
    }

    pub fn read(&self, buf: &mut [u8]) -> io::Result<usize> {
        let n = unsafe { libc::read(self.master, buf.as_mut_ptr() as *mut libc::c_void, buf.len()) };
        if n < 0 {
            Err(io::Error::last_os_error())
        } else {
            Ok(n as usize)
        }
    }

    /// Reap the child if it exited; returns its raw wait status if so.
    pub fn try_reap(&self) -> Option<i32> {
        let mut status: libc::c_int = 0;
        let r = unsafe { libc::waitpid(self.child, &mut status, libc::WNOHANG) };
        if r == self.child {
            self.reaped.store(true, Ordering::SeqCst);
            Some(status)
        } else {
            None
        }
    }
}

impl Drop for Pty {
    fn drop(&mut self) {
        // Always close the master fd so it isn't leaked. Only kill+waitpid
        // the child if it hasn't already been reaped (e.g. via `try_reap`
        // on the normal exit path): by Drop time a reaped pid may have been
        // recycled by the kernel, and killing it again could hit an
        // unrelated process.
        unsafe {
            libc::close(self.master);
            if !self.reaped.load(Ordering::SeqCst) {
                libc::kill(self.child, libc::SIGKILL);
                let mut status: libc::c_int = 0;
                libc::waitpid(self.child, &mut status, 0);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::{Duration, Instant};

    #[test]
    fn cat_echoes_input() {
        // On the dev host /dev/pts is already mounted, so no ensure_devpts().
        let pty = spawn_cmd("/bin/cat", &["cat"], 25, 6).expect("spawn cat");
        pty.write_all(b"hello\n").expect("write");

        let mut acc = String::new();
        let deadline = Instant::now() + Duration::from_secs(2);
        while Instant::now() < deadline && !acc.contains("hello") {
            let mut buf = [0u8; 256];
            if let Ok(n) = pty.read(&mut buf) {
                if n > 0 {
                    acc.push_str(&String::from_utf8_lossy(&buf[..n]));
                }
            }
            std::thread::sleep(Duration::from_millis(20));
        }
        assert!(acc.contains("hello"), "pty did not echo; got {:?}", acc);
        unsafe { libc::kill(pty.child, libc::SIGKILL); }
    }
}
