# Launcher SETTINGS menu + confirmed SHUTDOWN Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a SETTINGS screen to the launcher (opened with SHIFT+SELECT) whose SHUTDOWN item powers the device off after a confirm step.

**Architecture:** Extend the pure `Model` state machine with a `Screen` enum (`Apps`/`Settings`/`ConfirmShutdown`) and a one-shot shutdown-request flag; `View` gains render branches for the two new screens; `main.rs` tracks SHIFT, routes SELECT/BACK/encoder by screen, and execs the SD-safe `/sbin/deluge-shutdown` when a shutdown is requested. All state logic stays in the pure, unit-tested model; the model never performs I/O.

**Tech Stack:** Rust; `deluge-ui-toolkit` (`Title`, `ListMenuView`, `RowIcon`, `draw_text`, `Font`, `TextStyle`); `deluge-linux-ui` (`OledTarget`, `controls`); `embedded-graphics`.

## Global Constraints

- All changes are confined to `examples/launcher/src/` (`model.rs`, `ui.rs`, `main.rs`). No changes to overlays, the shutdown script, or the image build — `/sbin/deluge-shutdown` already exists in the launcher image.
- The `Model` is **pure / I/O-free**; every state transition must be unit-testable. All process execution lives in `main.rs`.
- Settings interactions are reachable only while **browsing** (`running.is_none()`); a running child owns the display and only the kill-chord is live.
- Shutdown command defaults to `/sbin/deluge-shutdown` (no args → poweroff), overridable via the `LAUNCHER_SHUTDOWN_CMD` env var (off-device test safety), mirroring the existing `LAUNCHER_APPS_DIR` pattern.
- Confirm-prompt copy is exactly: title `SHUTDOWN?`, lines `SELECT = OFF` and `BACK = CANCEL`. Terminal frame title: `SHUTTING DOWN`. Failure toast: `SHUTDOWN FAILED`.
- Tests build against the staged SDK sysroot: prefix cargo commands with `DELUGE_SDK_ROOT="$PWD/stage"`.
- Control ids (from `deluge_linux_ui::controls`): `button::SHIFT` = 8, `button::BACK` = 16, `encoder_button::SELECT` = 31 (click), `encoder::SELECT` = 5 (rotation).

---

## File Structure

- `examples/launcher/src/model.rs` — **Modify.** Add `Screen` enum, `SETTINGS_ITEMS`, three new fields, navigation + shutdown-request methods, getters. Owns all screen-transition logic and its unit tests.
- `examples/launcher/src/ui.rs` — **Modify.** Branch `View::render` on `model.screen()`; add `render_settings`, `render_confirm`, `render_shutting_down`; gate `tick` to the Apps screen. Add render-smoke tests.
- `examples/launcher/src/main.rs` — **Modify.** Track `shift_held`; route SELECT/BACK/encoder by screen; add `run_shutdown()` and the shutdown-request handling in the frame loop.

---

## Task 1: Model — Screen state + settings navigation

**Files:**
- Modify: `examples/launcher/src/model.rs`

**Interfaces:**
- Consumes: existing `Model` (fields `entries`, `selected`, `running`, `toast`, `toast_until`), `is_browsing()`.
- Produces:
  - `pub enum Screen { Apps, Settings, ConfirmShutdown }` (derives `Clone, Copy, PartialEq, Eq, Debug`)
  - `pub const SETTINGS_ITEMS: &[&str] = &["SHUTDOWN"];`
  - `pub fn screen(&self) -> Screen`
  - `pub fn settings_selected(&self) -> usize`
  - `pub fn open_settings(&mut self)`
  - `pub fn move_settings_selection(&mut self, delta: i32)`
  - `pub fn back(&mut self)`

- [ ] **Step 1: Write the failing tests**

Add these tests inside the existing `#[cfg(test)] mod tests` block in `examples/launcher/src/model.rs` (after `set_entries_clamps_selection`):

```rust
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
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `DELUGE_SDK_ROOT="$PWD/stage" cargo test -p launcher model:: 2>&1 | tail -20`
Expected: compile error — `Screen`, `SETTINGS_ITEMS`, `screen`, `open_settings`, `settings_selected`, `move_settings_selection`, `back` not found.

- [ ] **Step 3: Add the enum, constant, fields, and methods**

At the top of `examples/launcher/src/model.rs`, after the `use` lines and the `const TOAST_MS` line, add:

```rust
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
```

In `struct Model`, add three fields after `toast_until: u64,`:

```rust
    screen: Screen,
    settings_selected: usize,
    shutdown_request: bool,
```

In `Model::new`, add to the struct literal after `toast_until: 0,`:

```rust
            screen: Screen::Apps,
            settings_selected: 0,
            shutdown_request: false,
```

Add these methods inside `impl Model` (e.g. after `is_browsing`):

```rust
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
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `DELUGE_SDK_ROOT="$PWD/stage" cargo test -p launcher model:: 2>&1 | tail -20`
Expected: PASS — all model tests, including the six new ones, green.

- [ ] **Step 5: Commit**

```bash
git add examples/launcher/src/model.rs
git commit -m "launcher: model Screen state + settings navigation"
```

---

## Task 2: Model — shutdown confirm + one-shot request

**Files:**
- Modify: `examples/launcher/src/model.rs`

**Interfaces:**
- Consumes: `Screen`, `SETTINGS_ITEMS`, `settings_selected`, `open_settings`, `back` (Task 1).
- Produces:
  - `pub fn activate_settings_item(&mut self)` — SHUTDOWN row → `ConfirmShutdown`
  - `pub fn confirm_shutdown(&mut self)` — sets the one-shot request
  - `pub fn take_shutdown_request(&mut self) -> bool` — returns `true` once, then clears
  - `pub fn return_to_apps(&mut self)` — force screen back to `Apps`

- [ ] **Step 1: Write the failing tests**

Add inside `#[cfg(test)] mod tests` in `examples/launcher/src/model.rs`:

```rust
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
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `DELUGE_SDK_ROOT="$PWD/stage" cargo test -p launcher model:: 2>&1 | tail -20`
Expected: compile error — `activate_settings_item`, `confirm_shutdown`, `take_shutdown_request`, `return_to_apps` not found.

- [ ] **Step 3: Add the methods**

Add inside `impl Model` (after `back`):

```rust
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
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `DELUGE_SDK_ROOT="$PWD/stage" cargo test -p launcher model:: 2>&1 | tail -20`
Expected: PASS — all model tests green.

- [ ] **Step 5: Commit**

```bash
git add examples/launcher/src/model.rs
git commit -m "launcher: model shutdown confirm + one-shot request"
```

---

## Task 3: View — render SETTINGS, confirm, and shutting-down screens

**Files:**
- Modify: `examples/launcher/src/ui.rs`

**Interfaces:**
- Consumes: `Model`, `Screen`, `SETTINGS_ITEMS` from `crate::model`; `Title`, `ListMenuView`, `RowIcon`, `draw_text`, `Font`, `TextStyle` (already imported).
- Produces: `View::render` branches by screen; new `View::render_shutting_down(&self, target: &mut OledTarget)`.

- [ ] **Step 1: Write the failing tests**

Add inside `#[cfg(test)] mod tests` in `examples/launcher/src/ui.rs` (after `renders_toast_without_panicking`):

```rust
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
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `DELUGE_SDK_ROOT="$PWD/stage" cargo test -p launcher ui:: 2>&1 | tail -20`
Expected: compile error — `open_settings`/`activate_settings_item` referenced fine (from Tasks 1–2) but `render_shutting_down` not found, and the settings/confirm screens render as the Apps list (tests may not yet fail on content — the missing method guarantees a compile failure).

- [ ] **Step 3: Branch `render` and add the new render methods**

In `examples/launcher/src/ui.rs`, update the imports line:

```rust
use crate::model::{Model, Screen, SETTINGS_ITEMS};
```

(replacing the existing `use crate::model::Model;`)

Replace the whole `pub fn tick` body-guard so animation only runs on the app list. Change the first lines of `tick` from:

```rust
    pub fn tick(&mut self, model: &Model, delta_ms: u32) {
        if model.entries.is_empty() {
            return;
        }
```

to:

```rust
    pub fn tick(&mut self, model: &Model, delta_ms: u32) {
        if model.screen() != Screen::Apps || model.entries.is_empty() {
            return;
        }
```

Replace the existing `pub fn render` with a dispatcher plus per-screen helpers. Replace:

```rust
    pub fn render(&self, model: &Model, target: &mut OledTarget) {
        target.clear_frame();

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
```

with:

```rust
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

    /// Terminal frame shown just before the poweroff command runs.
    pub fn render_shutting_down(&self, target: &mut OledTarget) {
        target.clear_frame();
        let _ = Title::new("SHUTTING DOWN").with_separator(true).draw(target);
    }
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `DELUGE_SDK_ROOT="$PWD/stage" cargo test -p launcher ui:: 2>&1 | tail -20`
Expected: PASS — the three new render-smoke tests and the existing ones green.

- [ ] **Step 5: Commit**

```bash
git add examples/launcher/src/ui.rs
git commit -m "launcher: render SETTINGS, confirm, and shutting-down screens"
```

---

## Task 4: main.rs — SHIFT tracking, screen-aware input routing, shutdown exec

**Files:**
- Modify: `examples/launcher/src/main.rs`

**Interfaces:**
- Consumes: `Screen` and all Task 1–2 model methods; `View::render_shutting_down` (Task 3); `controls::{button, encoder_button, encoder}`.
- Produces: `fn run_shutdown() -> std::io::Result<std::process::ExitStatus>`; the wired event loop. (No unit tests — glue verified by `cargo build` + `cargo test` staying green and the manual check below.)

- [ ] **Step 1: Update imports and add SHIFT tracking**

In `examples/launcher/src/main.rs`, change:

```rust
use model::Model;
```

to:

```rust
use model::{Model, Screen};
```

After the line `let mut killed_by_user = false;` (the running-state vars), add:

```rust
    // SHIFT is a modifier for chords (SHIFT+SELECT opens SETTINGS).
    let mut shift_held = false;
```

- [ ] **Step 2: Route button input by screen**

Replace the entire `if ev.kind == EV_BUTTON { ... }` block (the one containing the kill-chord watch and the select-click launch) with:

```rust
                if ev.kind == EV_BUTTON {
                    let pressed = ev.value == 1;
                    // Track SHIFT for chords (SHIFT+SELECT → SETTINGS).
                    if ev.id == controls::button::SHIFT {
                        shift_held = pressed;
                    }
                    // Kill-chord watch runs in every mode.
                    if kw.on_button(ev.id, pressed, t) {
                        if let Some(pid) = running_pid {
                            killed_by_user = true;
                            supervisor::kill_group(pid);
                        }
                    }
                    // Screen-aware clicks, only while browsing.
                    if model.is_browsing() && pressed {
                        if ev.id == controls::encoder_button::SELECT {
                            match model.screen() {
                                Screen::Apps => {
                                    if shift_held {
                                        model.open_settings();
                                    } else {
                                        launch_selected(
                                            &mut model,
                                            &tx,
                                            &mut running_pid,
                                            &mut killed_by_user,
                                            t,
                                        );
                                    }
                                }
                                Screen::Settings => model.activate_settings_item(),
                                Screen::ConfirmShutdown => model.confirm_shutdown(),
                            }
                        } else if ev.id == controls::button::BACK {
                            model.back();
                        }
                    }
                } else if ev.kind == EV_ENCODER
                    && ev.id == controls::encoder::SELECT
                    && model.is_browsing()
                {
                    match model.screen() {
                        Screen::Apps => model.move_selection(ev.value as i32),
                        Screen::Settings => model.move_settings_selection(ev.value as i32),
                        Screen::ConfirmShutdown => {}
                    }
                }
```

Note: this replaces both the old `EV_BUTTON` block **and** the old `else if ev.kind == EV_ENCODER ...` block (which previously always called `move_selection`).

- [ ] **Step 3: Handle the shutdown request in the frame loop**

In the frame-cadence section, immediately after the line `model.tick(t);`, add:

```rust
        // A confirmed shutdown: show the terminal frame, then run the SD-safe
        // poweroff. On real hardware the command never returns; if it does
        // (bad override, exec failure), fall back to the app list with a toast.
        if model.take_shutdown_request() {
            view.render_shutting_down(&mut target);
            let _ = target.flush(&mut dlg);
            let failed = match run_shutdown() {
                Ok(status) => !status.success(),
                Err(_) => true,
            };
            model.return_to_apps();
            if failed {
                model.show_toast("SHUTDOWN FAILED".into(), now_ms());
            }
        }
```

- [ ] **Step 4: Add the `run_shutdown` helper**

At the end of `examples/launcher/src/main.rs` (after `launch_selected`), add:

```rust
/// Run the SD-safe shutdown command. Defaults to `/sbin/deluge-shutdown`
/// (no args → poweroff); overridable via `LAUNCHER_SHUTDOWN_CMD` so running the
/// launcher off-device (on a workstation) cannot accidentally power it off.
fn run_shutdown() -> std::io::Result<std::process::ExitStatus> {
    let cmd = std::env::var("LAUNCHER_SHUTDOWN_CMD")
        .unwrap_or_else(|_| "/sbin/deluge-shutdown".to_string());
    std::process::Command::new(cmd).status()
}
```

- [ ] **Step 5: Build and run the full suite**

Run: `DELUGE_SDK_ROOT="$PWD/stage" cargo test -p launcher 2>&1 | tail -25`
Expected: PASS — all model + ui tests green, binary compiles with no warnings about unused `Screen`/`shift_held`/`run_shutdown`.

- [ ] **Step 6: Manual off-device flow check (optional but recommended)**

This proves the SHIFT+SELECT → SHUTDOWN → confirm path fires the command exactly once without powering anything off. It runs the real binary, which needs the Deluge device (`Deluge::open`); if no device is attached, skip and rely on the unit tests. With a device attached:

```bash
# A harmless stub that records it ran instead of powering off.
printf '#!/bin/sh\necho "stub shutdown ran" >&2\n' > /tmp/fake-shutdown.sh
chmod +x /tmp/fake-shutdown.sh
DELUGE_SDK_ROOT="$PWD/stage" LAUNCHER_SHUTDOWN_CMD=/tmp/fake-shutdown.sh \
  cargo run -p launcher
# On the device: SHIFT+SELECT → SETTINGS, SELECT on SHUTDOWN → SHUTDOWN?,
# SELECT → "stub shutdown ran" prints once and a SHUTDOWN FAILED toast shows
# (expected with a stub); BACK at each step steps back instead.
```

Expected: `stub shutdown ran` printed exactly once per confirm; BACK cancels cleanly.

- [ ] **Step 7: Commit**

```bash
git add examples/launcher/src/main.rs
git commit -m "launcher: wire SHIFT+SELECT settings + confirmed shutdown"
```

---

## Self-Review

**Spec coverage:**
- SETTINGS reachable via SHIFT+SELECT → Task 1 (`open_settings`) + Task 4 (SHIFT tracking + routing). ✓
- SHUTDOWN item → Task 1 (`SETTINGS_ITEMS`) + Task 3 (`render_settings`). ✓
- Confirm step → Task 2 (`activate_settings_item`/`confirm_shutdown`) + Task 3 (`render_confirm`) + Task 4 (BACK cancels). ✓
- Shutdown execution via `/sbin/deluge-shutdown`, `LAUNCHER_SHUTDOWN_CMD` override → Task 4 (`run_shutdown`). ✓
- `SHUTTING DOWN` frame + `SHUTDOWN FAILED` fallback → Task 3 (`render_shutting_down`) + Task 4. ✓
- Reachable only while browsing → guards in Task 1 (`open_settings` checks `is_browsing()`) + Task 4 (`model.is_browsing()` gate). ✓
- Input isolation (settings nav vs app nav) → Task 1 test `settings_nav_does_not_move_app_selection` + Task 4 encoder match. ✓
- Model stays pure; exec in `main.rs` → enforced by task split. ✓
- Testing (model unit + view render-smoke) → Tasks 1–3. ✓

**Placeholder scan:** No TBD/TODO/"handle edge cases"; every code step shows full code. ✓

**Type consistency:** `Screen` variants (`Apps`/`Settings`/`ConfirmShutdown`), `SETTINGS_ITEMS`, and method names (`open_settings`, `move_settings_selection`, `activate_settings_item`, `confirm_shutdown`, `take_shutdown_request`, `return_to_apps`, `back`, `screen`, `settings_selected`, `render_shutting_down`, `run_shutdown`) are used identically across Tasks 1–4. ✓
