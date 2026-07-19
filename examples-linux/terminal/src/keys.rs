//! Map decoded input to the byte stream written to the shell's PTY.

use deluge_linux_ui::controls;
use deluge_linux_ui::deluge_grid_toolkit::KeyPress;

/// Event kinds (mirror `include/deluge/input.h`).
pub const EV_BUTTON: u8 = 1;
pub const EV_ENCODER: u8 = 2;

/// Bytes for a resolved keyboard key. `Shift` returns `None` (the caller toggles
/// shift state instead of writing anything).
pub fn keypress_bytes(k: KeyPress) -> Option<Vec<u8>> {
    match k {
        KeyPress::Char(c) => {
            let mut b = [0u8; 4];
            Some(c.encode_utf8(&mut b).as_bytes().to_vec())
        }
        KeyPress::Backspace => Some(vec![0x7f]),
        KeyPress::Enter => Some(vec![b'\r']),
        KeyPress::Shift => None,
    }
}

/// Control byte(s) for a front-panel button id (on a `value == 1` press).
pub fn button_bytes(id: u8) -> Option<&'static [u8]> {
    match id {
        controls::button::BACK => Some(&[0x03]),          // Ctrl-C
        controls::button::SAVE => Some(&[0x04]),          // Ctrl-D (EOF)
        controls::button::LOAD => Some(&[0x09]),          // Tab
        controls::button::AFFECT_ENTIRE => Some(&[0x1b]), // Esc
        _ => None,
    }
}

/// Arrow-key escapes for an encoder rotation (`delta` signed detents).
pub fn encoder_bytes(id: u8, delta: i16) -> Vec<u8> {
    if delta == 0 {
        return Vec::new();
    }
    let arrow: &[u8] = match id {
        controls::encoder::SELECT => if delta > 0 { b"\x1b[A" } else { b"\x1b[B" }, // up / down
        controls::encoder::TEMPO => if delta > 0 { b"\x1b[C" } else { b"\x1b[D" },  // right / left
        _ => return Vec::new(),
    };
    arrow.repeat(delta.unsigned_abs() as usize)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn char_encodes_utf8() {
        assert_eq!(keypress_bytes(KeyPress::Char('a')).unwrap(), b"a");
        assert_eq!(keypress_bytes(KeyPress::Char('/')).unwrap(), b"/");
    }

    #[test]
    fn enter_is_cr_backspace_is_del() {
        assert_eq!(keypress_bytes(KeyPress::Enter).unwrap(), b"\r");
        assert_eq!(keypress_bytes(KeyPress::Backspace).unwrap(), vec![0x7f]);
    }

    #[test]
    fn shift_writes_nothing() {
        assert!(keypress_bytes(KeyPress::Shift).is_none());
    }

    #[test]
    fn back_is_ctrl_c() {
        assert_eq!(button_bytes(controls::button::BACK), Some(&[0x03u8][..]));
        assert_eq!(button_bytes(controls::button::LOAD), Some(&[0x09u8][..]));
        assert_eq!(button_bytes(controls::button::PLAY), None);
    }

    #[test]
    fn encoder_arrows_repeat_by_delta() {
        assert_eq!(encoder_bytes(controls::encoder::SELECT, 1), b"\x1b[A");
        assert_eq!(encoder_bytes(controls::encoder::TEMPO, -2), b"\x1b[D\x1b[D");
        assert!(encoder_bytes(controls::encoder::SELECT, 0).is_empty());
    }
}
