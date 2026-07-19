//! Pure launcher state machine. No I/O — every transition is unit-testable.

use crate::apps::AppEntry;
use crate::supervisor::Exit;
use std::path::PathBuf;

const TOAST_MS: u64 = 2500;

/// Which screen the launcher is showing while browsing (orthogonal to
/// `running`, which is only set while a child app owns the display).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Screen {
    Apps,
    Settings,
    ConfirmShutdown,
}

/// Settings menu items, index-aligned. Only SHUTDOWN for now; the list is
/// future-extensible (navigation and rendering both derive from it).
pub const SETTINGS_ITEMS: &[&str] = &["SHUTDOWN"];

pub struct Model {
    pub entries: Vec<AppEntry>,
    pub selected: usize,
    pub running: Option<String>,
    pub toast: Option<String>,
    toast_until: u64,
    screen: Screen,
    settings_selected: usize,
    shutdown_request: bool,
}

impl Model {
    pub fn new(entries: Vec<AppEntry>) -> Self {
        Self {
            entries,
            selected: 0,
            running: None,
            toast: None,
            toast_until: 0,
            screen: Screen::Apps,
            settings_selected: 0,
            shutdown_request: false,
        }
    }

    pub fn is_browsing(&self) -> bool {
        self.running.is_none()
    }

    pub fn screen(&self) -> Screen {
        self.screen
    }

    pub fn settings_selected(&self) -> usize {
        self.settings_selected
    }

    /// Apps → Settings (only while browsing and on the app list).
    pub fn open_settings(&mut self) {
        if self.is_browsing() && self.screen == Screen::Apps {
            self.screen = Screen::Settings;
            self.settings_selected = 0;
        }
    }

    /// Move the settings-list selection, clamped to `SETTINGS_ITEMS`.
    pub fn move_settings_selection(&mut self, delta: i32) {
        if self.screen != Screen::Settings || SETTINGS_ITEMS.is_empty() {
            return;
        }
        let last = (SETTINGS_ITEMS.len() - 1) as i32;
        let next = (self.settings_selected as i32 + delta).clamp(0, last);
        self.settings_selected = next as usize;
    }

    /// Step back one screen: ConfirmShutdown → Settings → Apps.
    pub fn back(&mut self) {
        self.screen = match self.screen {
            Screen::ConfirmShutdown => Screen::Settings,
            Screen::Settings => Screen::Apps,
            Screen::Apps => Screen::Apps,
        };
    }

    /// Activate the focused settings item. SHUTDOWN → confirm screen.
    pub fn activate_settings_item(&mut self) {
        if self.screen != Screen::Settings {
            return;
        }
        if SETTINGS_ITEMS.get(self.settings_selected) == Some(&"SHUTDOWN") {
            self.screen = Screen::ConfirmShutdown;
        }
    }

    /// Confirm shutdown from the confirm screen: raise the one-shot request
    /// that `main` consumes to run the actual poweroff.
    pub fn confirm_shutdown(&mut self) {
        if self.screen == Screen::ConfirmShutdown {
            self.shutdown_request = true;
        }
    }

    /// Consume the shutdown request: `true` at most once per confirm.
    pub fn take_shutdown_request(&mut self) -> bool {
        std::mem::take(&mut self.shutdown_request)
    }

    /// Force the screen back to the app list (used after a failed shutdown).
    pub fn return_to_apps(&mut self) {
        self.screen = Screen::Apps;
    }

    pub fn set_entries(&mut self, entries: Vec<AppEntry>) {
        self.entries = entries;
        if self.selected >= self.entries.len() {
            self.selected = self.entries.len().saturating_sub(1);
        }
    }

    pub fn move_selection(&mut self, delta: i32) {
        if !self.is_browsing() || self.entries.is_empty() {
            return;
        }
        let last = (self.entries.len() - 1) as i32;
        let next = (self.selected as i32 + delta).clamp(0, last);
        self.selected = next as usize;
    }

    pub fn selected_path(&self) -> Option<PathBuf> {
        if !self.is_browsing() {
            return None;
        }
        self.entries.get(self.selected).map(|e| e.path.clone())
    }

    pub fn begin_running(&mut self, name: String) {
        self.running = Some(name);
    }

    pub fn on_child_exit(&mut self, exit: Exit, killed_by_user: bool, now_ms: u64) {
        let name = self.running.take().unwrap_or_default();
        if killed_by_user {
            return; // silent
        }
        let msg = match exit {
            Exit::Clean => None,
            Exit::Nonzero(code) => Some(format!("{name} EXITED ({code})")),
            Exit::Signalled(sig) => Some(format!("{name} CRASHED ({sig})")),
        };
        if let Some(m) = msg {
            self.show_toast(m, now_ms);
        }
    }

    /// Show a transient toast that expires TOAST_MS after `now_ms`.
    pub fn show_toast(&mut self, msg: String, now_ms: u64) {
        self.toast = Some(msg);
        self.toast_until = now_ms + TOAST_MS;
    }

    pub fn tick(&mut self, now_ms: u64) {
        if self.toast.is_some() && now_ms >= self.toast_until {
            self.toast = None;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn entry(name: &str) -> AppEntry {
        AppEntry {
            path: PathBuf::from(format!("/LINUX/APPS/{name}")),
            display_name: name.to_uppercase(),
        }
    }

    fn model3() -> Model {
        Model::new(vec![entry("a"), entry("b"), entry("c")])
    }

    #[test]
    fn selection_clamps_at_both_ends() {
        let mut m = model3();
        m.move_selection(-1);
        assert_eq!(m.selected, 0);
        m.move_selection(1);
        m.move_selection(1);
        m.move_selection(1);
        assert_eq!(m.selected, 2);
    }

    #[test]
    fn selected_path_returns_current_entry() {
        let mut m = model3();
        m.move_selection(1);
        assert_eq!(m.selected_path(), Some(PathBuf::from("/LINUX/APPS/b")));
    }

    #[test]
    fn no_launch_when_empty() {
        let m = Model::new(vec![]);
        assert_eq!(m.selected_path(), None);
    }

    #[test]
    fn no_launch_or_move_while_running() {
        let mut m = model3();
        m.begin_running("A".into());
        assert!(!m.is_browsing());
        assert_eq!(m.selected_path(), None);
        m.move_selection(1);
        assert_eq!(m.selected, 0);
    }

    #[test]
    fn kill_returns_silently() {
        let mut m = model3();
        m.begin_running("A".into());
        m.on_child_exit(Exit::Signalled(9), true, 0);
        assert!(m.is_browsing());
        assert_eq!(m.toast, None);
    }

    #[test]
    fn clean_exit_no_toast() {
        let mut m = model3();
        m.begin_running("A".into());
        m.on_child_exit(Exit::Clean, false, 0);
        assert_eq!(m.toast, None);
    }

    #[test]
    fn crash_shows_toast_until_expiry() {
        let mut m = model3();
        m.begin_running("SPARK".into());
        m.on_child_exit(Exit::Signalled(11), false, 1000);
        assert_eq!(m.toast.as_deref(), Some("SPARK CRASHED (11)"));
        m.tick(1000 + 2499);
        assert!(m.toast.is_some());
        m.tick(1000 + 2500);
        assert_eq!(m.toast, None);
    }

    #[test]
    fn nonzero_exit_shows_exit_toast() {
        let mut m = model3();
        m.begin_running("WREN".into());
        m.on_child_exit(Exit::Nonzero(2), false, 0);
        assert_eq!(m.toast.as_deref(), Some("WREN EXITED (2)"));
    }

    #[test]
    fn show_toast_sets_and_expires() {
        let mut m = model3();
        m.show_toast("CAN'T RUN A".into(), 1000);
        assert_eq!(m.toast.as_deref(), Some("CAN'T RUN A"));
        m.tick(1000 + 2499);
        assert!(m.toast.is_some());
        m.tick(1000 + 2500);
        assert_eq!(m.toast, None);
    }

    #[test]
    fn set_entries_clamps_selection() {
        let mut m = model3();
        m.move_selection(1);
        m.move_selection(1); // selected = 2
        m.set_entries(vec![entry("only")]);
        assert_eq!(m.selected, 0);
    }

    #[test]
    fn defaults_to_apps_screen() {
        let m = model3();
        assert_eq!(m.screen(), Screen::Apps);
    }

    #[test]
    fn shift_select_opens_settings() {
        let mut m = model3();
        m.open_settings();
        assert_eq!(m.screen(), Screen::Settings);
        assert_eq!(m.settings_selected(), 0);
    }

    #[test]
    fn open_settings_ignored_while_running() {
        let mut m = model3();
        m.begin_running("A".into());
        m.open_settings();
        assert_eq!(m.screen(), Screen::Apps);
    }

    #[test]
    fn back_from_settings_returns_to_apps() {
        let mut m = model3();
        m.open_settings();
        m.back();
        assert_eq!(m.screen(), Screen::Apps);
    }

    #[test]
    fn settings_selection_clamps() {
        let mut m = model3();
        m.open_settings();
        m.move_settings_selection(-1);
        assert_eq!(m.settings_selected(), 0);
        m.move_settings_selection(5);
        assert_eq!(m.settings_selected(), SETTINGS_ITEMS.len() - 1);
    }

    #[test]
    fn settings_nav_does_not_move_app_selection() {
        let mut m = model3();
        m.open_settings();
        m.move_settings_selection(1);
        assert_eq!(m.selected, 0);
    }

    #[test]
    fn activate_shutdown_enters_confirm() {
        let mut m = model3();
        m.open_settings();
        m.activate_settings_item();
        assert_eq!(m.screen(), Screen::ConfirmShutdown);
    }

    #[test]
    fn confirm_sets_shutdown_request_once() {
        let mut m = model3();
        m.open_settings();
        m.activate_settings_item();
        m.confirm_shutdown();
        assert!(m.take_shutdown_request());
        assert!(!m.take_shutdown_request());
    }

    #[test]
    fn back_from_confirm_returns_to_settings() {
        let mut m = model3();
        m.open_settings();
        m.activate_settings_item();
        m.back();
        assert_eq!(m.screen(), Screen::Settings);
    }

    #[test]
    fn confirm_ignored_off_confirm_screen() {
        let mut m = model3();
        m.open_settings(); // Settings, not ConfirmShutdown
        m.confirm_shutdown();
        assert!(!m.take_shutdown_request());
    }

    #[test]
    fn return_to_apps_resets_screen() {
        let mut m = model3();
        m.open_settings();
        m.activate_settings_item();
        m.return_to_apps();
        assert_eq!(m.screen(), Screen::Apps);
    }
}
