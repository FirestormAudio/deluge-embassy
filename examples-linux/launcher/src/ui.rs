//! Render the launcher list to the OLED using the shared UI toolkit.

use deluge_linux_ui::OledTarget;
use deluge_ui_toolkit::components::Title;
use deluge_ui_toolkit::text::{draw_text, Font, TextStyle};
use deluge_ui_toolkit::{ListMenuView, RowIcon};
use embedded_graphics::{pixelcolor::BinaryColor, prelude::*, text::Alignment};

use crate::model::{Model, Screen, SETTINGS_ITEMS};

pub struct View {
    list: ListMenuView,
}

impl View {
    pub fn new() -> Self {
        Self {
            list: ListMenuView::new(),
        }
    }

    pub fn tick(&mut self, model: &Model, delta_ms: u32) {
        if model.screen() != Screen::Apps || model.entries.is_empty() {
            return;
        }
        let name = &model.entries[model.selected].display_name;
        self.list.tick(
            model.selected,
            name,
            RowIcon::None,
            model.entries.len(),
            delta_ms,
        );
    }

    pub fn render(&self, model: &Model, target: &mut OledTarget) {
        target.clear_frame();
        match model.screen() {
            Screen::Apps => self.render_apps(model, target),
            Screen::Settings => self.render_settings(model, target),
            Screen::ConfirmShutdown => Self::render_confirm(target),
        }
    }

    fn render_apps(&self, model: &Model, target: &mut OledTarget) {
        // Title bar: canonical Deluge style — plain title text + a 1px
        // underline separator (matches DelugeFirmware's drawScreenTitle and
        // spark's render_title), not the inverted-bar `Header`. ListMenuView's
        // rows (y=14/23/32) are laid out to sit under exactly this.
        let _ = Title::new("APPS").with_separator(true).draw(target);

        // The list (empty_message shows when there are no rows).
        let rows: Vec<(&str, RowIcon)> = model
            .entries
            .iter()
            .map(|e| (e.display_name.as_str(), RowIcon::None))
            .collect();
        self.list
            .render(target, &rows, model.selected, Some("NO APPS"));

        // Transient toast, drawn over the bottom row.
        if let Some(msg) = &model.toast {
            let style = TextStyle::new(Font::FontApple).with_color(BinaryColor::On);
            let _ = draw_text(target, msg, Point::new(3, 35), style);
        }
    }

    fn render_settings(&self, model: &Model, target: &mut OledTarget) {
        let _ = Title::new("SETTINGS").with_separator(true).draw(target);
        let rows: Vec<(&str, RowIcon)> =
            SETTINGS_ITEMS.iter().map(|s| (*s, RowIcon::None)).collect();
        self.list
            .render(target, &rows, model.settings_selected(), None);
    }

    fn render_confirm(target: &mut OledTarget) {
        let _ = Title::new("SHUTDOWN?").with_separator(true).draw(target);
        let style = TextStyle::new(Font::FontApple).with_color(BinaryColor::On);
        let _ = draw_text(target, "SELECT = OFF", Point::new(3, 20), style);
        let _ = draw_text(target, "BACK = CANCEL", Point::new(3, 32), style);
    }

    /// Progress frame shown while `deluge-shutdown prepare` syncs and unmounts
    /// the SD. Replaced by `render_safe_to_power_off` once the card is safe.
    pub fn render_shutting_down(&self, target: &mut OledTarget) {
        target.clear_frame();
        let _ = Title::new("SHUTTING DOWN").with_separator(true).draw(target);
    }

    /// Final resting frame: "SAFE TO / POWER OFF", two centred bold lines.
    ///
    /// This board (RZ/A1L) can't cut its own power, so shutdown ends in a CPU
    /// halt with the panel still energised. This is the last thing flushed, and
    /// it persists: the OLED self-refreshes from its own GDDRAM once the CPU
    /// parks, so the message stays lit until the user flips the hardware switch.
    pub fn render_safe_to_power_off(&self, target: &mut OledTarget) {
        target.clear_frame();
        // Centred on the 128px-wide panel; two 13px lines vertically centred in
        // the 43px visible height (y is the glyph top, glyphs draw downward).
        let style = TextStyle::new(Font::MetricBold13px).with_alignment(Alignment::Center);
        let cx = 64;
        let _ = draw_text(target, "SAFE TO", Point::new(cx, 7), style);
        let _ = draw_text(target, "POWER OFF", Point::new(cx, 23), style);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::apps::AppEntry;
    use std::path::PathBuf;

    fn entry(name: &str) -> AppEntry {
        AppEntry {
            path: PathBuf::from(name),
            display_name: name.to_uppercase(),
        }
    }

    /// True if any pixel is set in rows [y0, y1) — used to prove content was
    /// drawn below the header (the header fills the top ~12 rows unconditionally).
    fn any_set_in_rows(frame: &[u8], y0: usize, y1: usize) -> bool {
        frame[y0 * 16..y1 * 16].iter().any(|&b| b != 0)
    }

    #[test]
    fn renders_list_without_panicking() {
        let m = Model::new(vec![entry("spark"), entry("wren")]);
        let v = View::new();
        let mut target = OledTarget::new();
        v.render(&m, &mut target);
        // App-list rows are drawn below the header (which unconditionally
        // fills the top ~12 rows, so asserting on the whole frame wouldn't
        // prove the list itself rendered).
        assert!(any_set_in_rows(target.frame(), 14, 43));
    }

    #[test]
    fn renders_empty_state_without_panicking() {
        let m = Model::new(vec![]);
        let v = View::new();
        let mut target = OledTarget::new();
        v.render(&m, &mut target);
        // "NO APPS" is drawn in the list area below the header.
        assert!(any_set_in_rows(target.frame(), 14, 43));
    }

    #[test]
    fn renders_toast_without_panicking() {
        let mut m = Model::new(vec![entry("spark")]);
        m.begin_running("SPARK".into());
        m.on_child_exit(crate::supervisor::Exit::Signalled(11), false, 0);
        let v = View::new();
        let mut target = OledTarget::new();
        v.render(&m, &mut target);
        // The toast is drawn at y=35, well below the single list row (which
        // only occupies y≈14..23), so a set pixel in y=[34, 43) specifically
        // proves the toast was drawn, not just the row.
        assert!(any_set_in_rows(target.frame(), 34, 43));
    }

    #[test]
    fn renders_settings_without_panicking() {
        let mut m = Model::new(vec![entry("spark")]);
        m.open_settings();
        let v = View::new();
        let mut target = OledTarget::new();
        v.render(&m, &mut target);
        // The SHUTDOWN row is drawn below the header.
        assert!(any_set_in_rows(target.frame(), 14, 43));
    }

    #[test]
    fn renders_confirm_without_panicking() {
        let mut m = Model::new(vec![entry("spark")]);
        m.open_settings();
        m.activate_settings_item();
        let v = View::new();
        let mut target = OledTarget::new();
        v.render(&m, &mut target);
        // The confirm prompt is drawn below the header.
        assert!(any_set_in_rows(target.frame(), 14, 43));
    }

    #[test]
    fn renders_shutting_down_without_panicking() {
        let v = View::new();
        let mut target = OledTarget::new();
        v.render_shutting_down(&mut target);
        // The title fills the header rows.
        assert!(any_set_in_rows(target.frame(), 0, 14));
    }

    #[test]
    fn renders_safe_to_power_off_without_panicking() {
        let v = View::new();
        let mut target = OledTarget::new();
        v.render_safe_to_power_off(&mut target);
        // Both centred lines land in the body: the top line around y≈7..20 and
        // the bottom line around y≈23..36. Assert content in each band so a
        // single mispositioned line can't pass this on its own.
        assert!(any_set_in_rows(target.frame(), 7, 20), "top line 'SAFE TO'");
        assert!(any_set_in_rows(target.frame(), 23, 36), "bottom line 'POWER OFF'");
    }
}
