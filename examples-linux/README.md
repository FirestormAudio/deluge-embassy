# Linux example apps

Native `deluge-hal-linux` apps for the Deluge's Linux userland, built with
`cargo deluge linux` (see the deluge-ndk building-an-app guide for the toolchain).

## The launcher (reference appliance)

The generic `LINUX` image's app is the launcher (`examples-linux/launcher`).
Build it as `LINUX.ELF` from the `launcher/` directory:

    cargo deluge linux            # -> target/LINUX.ELF for /APPS/

Copy `LINUX.ELF` to the card's `/APPS/`, and drop bare app binaries
(built with `cargo deluge linux --bare` from each app's own directory) into
`/LINUX/APPS/`. On boot the app-loader launches `LINUX.ELF`; the launcher
lists `/LINUX/APPS/` on the OLED, the select encoder browses and launches, and
SHIFT+TRIPLETS+LEARN (held ~1s) always returns to it.

### On-device manual verification checklist

Deploy `target/LINUX.ELF` to `/APPS/` and one or more bare apps to
`/LINUX/APPS/`, boot it, and confirm:

- The app list renders (title "APPS", rows for each `/LINUX/APPS/` binary),
  and "NO APPS" shows when the folder is empty.
- Select-encoder rotation moves the highlight; the label scrolls when it
  overflows; the scrollbar appears with >3 apps.
- Select-encoder click launches the highlighted app (it takes over the OLED).
- Holding SHIFT+TRIPLETS+LEARN for ~1s kills the running app and returns to
  the list, regardless of what the app is doing; a shorter hold does not.
- A clean-exiting app returns silently; a crashing app shows the
  `… CRASHED (sig)` toast briefly.
- The OLED image is correct (not mirrored/inverted). If wrong within 8px
  groups, apply the bit-order fix from Task 2 Step 5.

## Snake (pad-grid bare app)

`examples-linux/snake` is the reference *non-appliance* app: a static binary
for `/LINUX/APPS/` that the launcher exec's. It reads the pad grid and tempo
encoder and drives the pad LEDs + OLED. Build the bare binary from the
`snake/` directory:

    cargo deluge linux --bare      # -> target/bare/snake  for /LINUX/APPS/

Copy `target/bare/snake` onto the SD card's `/LINUX/APPS/`, boot the launcher,
and select `SNAKE`. Steer with the tempo encoder (turn = relative left/right);
eating food grows the snake and speeds it up; hitting a wall or yourself ends
the game — press the tempo encoder to restart. SHIFT+TRIPLETS+LEARN (held ~1s)
returns to the launcher.

> The bare binary must be copied to the card manually — the dev-mode USB upload
> only streams the appliance `LINUX.ELF` into RAM; it cannot place files on the
> card.

## Terminal (PTY shell bare app)

`examples-linux/terminal` is a bare `/LINUX/APPS/` app that runs an
interactive BusyBox shell (`/bin/sh -i`) on a pseudo-terminal and renders it
as a 25×6 character terminal on the OLED. Input comes from the on-pad QWERTY
keyboard (`deluge-grid-toolkit`'s `TextKeyboardComponent`, driven through the
`deluge-linux-ui::pads` bridge) plus front-panel keys. Build the bare binary
from the `terminal/` directory:

    cargo deluge linux --bare      # -> target/bare/terminal  for /LINUX/APPS/

Copy `target/bare/terminal` onto the card's `/LINUX/APPS/`, boot the launcher,
and select `TERMINAL`. Type on the pad keyboard; `Enter` runs the line. Shift
(on-pad or the `SHIFT` button) gives capitals and symbols. `BACK` sends Ctrl-C,
`SAVE` sends Ctrl-D (EOF), `LOAD` sends Tab; the `SELECT` encoder scrolls shell
history. On `exit`/Ctrl-D the OLED shows `[exit N] ENTER=restart`.
SHIFT+TRIPLETS+LEARN (held ~1s) returns to the launcher.

Notes:
- The app mounts `devpts` at `/dev/pts` itself on startup (the appliance rootfs
  does not) before `forkpty`; the child sets `PR_SET_PDEATHSIG(SIGKILL)` so the
  shell is reaped if the launcher force-kills the app.
- The reduced pad keyboard exposes the shell-critical symbols
  (`/ \ | ; : ? < > _ " ! @ # $ % ^ & * ( )`); `~` and `` ` `` are omitted (no
  free pad — the last slot collides with the SHIFT pad). A future symbols layer
  could add them.
- Full-screen TUI programs (`vi`, `top`) are out of scope: only a minimal
  VT100 subset (cursor moves + erase) is emulated; colours are ignored (1bpp).
