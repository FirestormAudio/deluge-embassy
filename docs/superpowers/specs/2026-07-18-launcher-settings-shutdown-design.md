# Launcher SETTINGS menu with confirmed SHUTDOWN

**Date:** 2026-07-18
**Status:** design — awaiting implementation plan

## Goal

Add a **SETTINGS** screen to the app launcher, reachable with **SHIFT + SELECT**
from the app list. Its first (and, for now, only) item is **SHUTDOWN**, which —
after a confirmation step — powers the device off cleanly.

The launcher is the reference Linux appliance (`examples/launcher`). It runs as
`/usr/bin/deluge-app` inside the generic `LINUX` image, lists `/sd/LINUX/APPS`,
and launches the selected bare app. This feature adds a settings surface without
disturbing the existing browse/launch/kill flows.

## Current state

- **`Model`** (`src/model.rs`) is a pure, I/O-free state machine. Today it has two
  implicit states: **browsing** (`running.is_none()`) and **running** a child
  (`running: Some(name)`). `is_browsing()` gates selection movement and launch.
  It also owns a transient `toast`.
- **`View`** (`src/ui.rs`) renders the `APPS` title (`Title` + separator) and the
  app list via the shared `ListMenuView`, plus an optional toast.
- **`main.rs`** owns the event loop and all I/O. It reads `EV_BUTTON` /
  `EV_ENCODER` events, runs the kill-chord watch (`KillWatch`) in every mode, and
  — while browsing — launches on `SELECT`-click and moves selection on encoder
  `SELECT` rotation.
- **Controls** (`deluge-linux-ui::controls`): `button::SHIFT` = 8,
  `button::BACK` = 16, `encoder_button::SELECT` = 31 (shaft-click),
  `encoder::SELECT` = 5 (rotation).
- **Shutdown already exists:** `/sbin/deluge-shutdown` ships in the `common`
  rootfs overlay (so it is present in the launcher image). Invoked with no
  arguments it defaults to `poweroff`: it syncs, unmounts the rw `/sd` (so the
  FAT is not left dirty), syncs again, and stops via `reboot(2)` (`-f`). PID 1 is
  `runsvdir`, so plain `busybox poweroff` cannot signal init cleanly — this
  script is the correct, SD-safe path.

## Architecture

One repo (`deluge-linux-sdk`), three touched files (`model.rs`, `ui.rs`,
`main.rs`), all under `examples/launcher/src/`. No changes outside the launcher.

### Screen state

Add an explicit screen enum to `Model`, meaningful only while **not running**:

```rust
enum Screen { Apps, Settings, ConfirmShutdown }
```

`Model` gains:
- `screen: Screen` (default `Screen::Apps`),
- `settings_selected: usize` — selection index within the settings list
  (future-extensible; clamps like `move_selection`),
- a one-shot **shutdown request** flag, consumed by `main.rs` via
  `take_shutdown_request() -> bool` (returns `true` exactly once after a confirmed
  shutdown, then clears). This keeps the model I/O-free — the model only *signals*
  intent; `main.rs` performs the exec.

`running` stays orthogonal to `screen`. While a child runs, the screen state is
unreachable (input is kill-chord only), exactly as today. Returning from a child
(`on_child_exit`) leaves `screen` at `Apps` (the launcher always returns to the
app list on child exit; entering settings then relaunching is not a path).

### Input mapping

All settings interactions are gated on **browsing** (`running.is_none()`); a
running child owns the display and only the kill-chord is live.

| Screen            | Input                          | Action                                             |
| ----------------- | ------------------------------ | -------------------------------------------------- |
| Apps              | `SHIFT` held + `SELECT`-click  | → Settings                                         |
| Apps              | `SELECT`-click (no shift)      | launch highlighted app *(unchanged)*               |
| Apps              | encoder `SELECT` rotate        | move app selection *(unchanged)*                   |
| Settings          | encoder `SELECT` rotate        | move `settings_selected`                           |
| Settings          | `SELECT`-click on SHUTDOWN     | → ConfirmShutdown                                  |
| Settings          | `BACK`                         | → Apps                                             |
| ConfirmShutdown   | `SELECT`-click                 | set shutdown request → `main.rs` runs the shutdown |
| ConfirmShutdown   | `BACK`                         | → Settings (cancel)                                |

`main.rs` tracks `shift_held: bool` from `SHIFT` (id 8) button down/up events (a
couple of lines, no new type). The SHIFT+SELECT check happens before the existing
no-shift launch branch, so a shifted SELECT-click on the app list opens settings
instead of launching.

SHIFT+SELECT only **opens** settings (idempotent); `BACK` is the way out. No
toggle-closed on a second SHIFT+SELECT (keeps the mapping unambiguous).

### Rendering

`View` reuses the existing toolkit (`Title` + `ListMenuView`), branching on
`model.screen`:

- **Apps** — unchanged (`APPS` title + app list + toast).
- **Settings** — title `SETTINGS`; a single-row list `["SHUTDOWN"]` rendered
  through `ListMenuView` with `settings_selected` highlighted. The list is a
  `&[&str]` so adding future items is trivial.
- **ConfirmShutdown** — title `SHUTDOWN?`; a short prompt line drawn below the
  header, e.g. `SELECT = OFF   BACK = CANCEL`, using the same text style as the
  toast.

`View::tick` (marquee animation) applies to the app list only; the settings and
confirm screens are static (no per-frame animation needed).

### Shutdown execution

On a confirmed shutdown, `main.rs`:
1. renders a final `SHUTTING DOWN` frame and flushes it (so the user sees the
   device acknowledge before the panel goes dark),
2. runs the shutdown command and blocks (`Command::status()`).

The command defaults to `/sbin/deluge-shutdown` (no args → poweroff). It is
**overridable via `LAUNCHER_SHUTDOWN_CMD`** for safe manual testing off-device —
mirroring the existing `LAUNCHER_APPS_DIR` override — so running the launcher on a
workstation cannot accidentally power it off. If the override is set, its value is
the program to exec (argv[0]); otherwise `/sbin/deluge-shutdown`.

Because `deluge-shutdown` powers the machine off via `reboot(2)`, the call does
not normally return. If it *does* return (e.g. the override is a stub, or exec
fails), the launcher returns to the `Apps` screen and shows a toast
(`SHUTDOWN FAILED`) rather than silently swallowing the failure.

## Data flow

```
SHIFT held ──┐
             ├─ SELECT-click (Apps) ─▶ Model.screen = Settings
app list ────┘
Settings ── SELECT-click on SHUTDOWN ─▶ Model.screen = ConfirmShutdown
ConfirmShutdown ── SELECT-click ─▶ Model sets shutdown_request
main.rs loop: if Model.take_shutdown_request() ─▶ render "SHUTTING DOWN",
              flush, Command(shutdown_cmd).status() ─▶ device powers off
BACK at any settings screen ─▶ step back (ConfirmShutdown→Settings→Apps)
```

## Testing

**Model (pure unit tests, `src/model.rs`):**
- `SHIFT`+SELECT enters Settings; plain SELECT on Apps does *not* (still launches).
- Settings → ConfirmShutdown on activating SHUTDOWN.
- ConfirmShutdown SELECT sets the shutdown request; `take_shutdown_request()`
  returns `true` once then `false`.
- `BACK` steps ConfirmShutdown→Settings and Settings→Apps.
- Input isolation: encoder rotation in Settings moves `settings_selected`, not the
  app `selected`; and vice-versa.
- Settings entry is impossible while `running` (guarded by `is_browsing()`).

**View (render-smoke tests, `src/ui.rs`):** render the Settings and
ConfirmShutdown screens and assert content is drawn below the header (same
`any_set_in_rows` approach already used).

**Manual / on-device:** with `LAUNCHER_SHUTDOWN_CMD` pointed at a harmless stub,
exercise the full SHIFT+SELECT → SHUTDOWN → confirm flow and verify the stub runs
exactly once and the confirm/cancel paths behave. On real hardware, confirm the
device powers off cleanly (no dirty FAT on next boot).

## Risks / notes

- **Accidental poweroff:** mitigated by the confirm step (two deliberate
  SELECT-clicks, with a visible `SHUTDOWN?` prompt and BACK-cancel).
- **Off-device safety:** the `LAUNCHER_SHUTDOWN_CMD` override prevents the example
  from powering off a developer workstation during UI testing.
- **SHIFT tracking:** `shift_held` must be cleared on SHIFT release so a later
  unshifted SELECT-click launches normally; covered by the loop logic (not the
  pure model).

## Out of scope

- Additional settings items (brightness, network, versions, etc.) — the Settings
  list is structured to accept them later, but only SHUTDOWN ships now.
- Reboot/restart (only poweroff).
- Any change to the `deluge-shutdown` script, the rootfs overlays, or the image
  build — the script already exists and is already in the launcher image.
