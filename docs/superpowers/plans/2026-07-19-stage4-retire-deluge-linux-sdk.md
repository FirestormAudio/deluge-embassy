# Stage 4 — migrate remaining docs + retire deluge-linux-sdk — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Migrate the only unmigrated content left in `deluge-linux-sdk` — its docs — to the three repos that now own the corresponding code, then retire `deluge-linux-sdk` (a `git bundle` safety net, then delete the local checkout).

**Architecture:** The audit confirmed all code/crates/tools/tests/examples are already in `deluge-ndk` (C, with history), `deluge-linux` (mkimage), or `deluge-sdk` (Rust). Only docs remain: `building-an-app.md` and 10 `docs/superpowers/{specs,plans}` design docs. Move them to the repo that owns their subject, then bundle + delete.

**Tech Stack:** git, Markdown.

## Global Constraints

- Doc migration is **plain-copy** (the `git bundle` in Task 2 preserves `deluge-linux-sdk`'s full history as insurance).
- Doc → repo mapping:
  - `docs/building-an-app.md` → **`deluge-ndk/docs/building-an-app.md`**, with its Rust/`xtask` section rewritten to point at `deluge-sdk`'s `cargo deluge linux` (the `xtask` flow is retired).
  - `docs/superpowers/plans/2026-07-13-deluge-mkimage.md`, `docs/superpowers/specs/2026-07-13-contained-elf-overlay-design.md` → **`deluge-linux/docs/superpowers/{plans,specs}/`** (the packer's home).
  - the 4 app features — `app-launcher`, `snake-demo`, `launcher-settings-shutdown`, `terminal-shell` (each a spec + a plan, 8 files) → **`deluge-sdk/docs/superpowers/{specs,plans}/`** (the apps' home).
- `deluge-linux-sdk`'s root files (`Cargo.toml`, `.cargo/config.toml`, `ci.yml`, `README.md`, `CMakeLists.txt`, `deluge.pc.in`, `bundle.lock`) are **superseded** by the three repos' own versions — not migrated.
- Retirement: **`git bundle`** the repo to a durable file, then `rm -rf` the local checkout. The `rm -rf` is the one irreversible step — the controller runs it (Task 2), not a subagent, and only after the bundle is confirmed.

---

## Task 1: Migrate the docs to the three repos

**Files:**
- Create: `deluge-ndk/docs/building-an-app.md` (from `deluge-linux-sdk`, Rust section rewritten)
- Create: `deluge-linux/docs/superpowers/plans/2026-07-13-deluge-mkimage.md`, `deluge-linux/docs/superpowers/specs/2026-07-13-contained-elf-overlay-design.md`
- Create: `deluge-sdk/docs/superpowers/{specs,plans}/…` — the 8 app-feature docs

- [ ] **Step 1: mkimage/elf design docs → deluge-linux**

```bash
SRC=~/GitHub/deluge-linux-sdk/docs/superpowers; DL=~/GitHub/deluge-linux/docs/superpowers
mkdir -p "$DL/plans" "$DL/specs"
cp "$SRC/plans/2026-07-13-deluge-mkimage.md" "$DL/plans/"
cp "$SRC/specs/2026-07-13-contained-elf-overlay-design.md" "$DL/specs/"
git -C ~/GitHub/deluge-linux add docs/superpowers
git -C ~/GitHub/deluge-linux commit -q -m "docs: import mkimage + contained-elf-overlay design docs from deluge-linux-sdk

The packer lives here now (Stages 2b/2c); its design history moves with it."
echo done-linux
```
Expected: `done-linux`.

- [ ] **Step 2: app-feature design docs → deluge-sdk**

```bash
SRC=~/GitHub/deluge-linux-sdk/docs/superpowers; DS=~/GitHub/deluge-sdk/docs/superpowers
for f in app-launcher snake-demo launcher-settings-shutdown terminal-shell; do
  cp "$SRC/specs/"*"$f-design.md" "$DS/specs/" 2>/dev/null
done
cp "$SRC/plans/2026-07-17-app-launcher.md" "$SRC/plans/2026-07-17-snake-demo.md" \
   "$SRC/plans/2026-07-18-launcher-settings-shutdown.md" "$SRC/plans/2026-07-18-terminal-shell.md" "$DS/plans/"
git -C ~/GitHub/deluge-sdk add docs/superpowers
git -C ~/GitHub/deluge-sdk commit -q -m "docs: import launcher/snake/terminal app design docs from deluge-linux-sdk

These Rust apps live in examples-linux/ now (Stage 3b); their design history
moves with them."
echo done-sdk
```
Expected: `done-sdk`. Verify 8 files landed: `ls ~/GitHub/deluge-sdk/docs/superpowers/specs/*{app-launcher,snake-demo,launcher-settings-shutdown,terminal-shell}* ~/GitHub/deluge-sdk/docs/superpowers/plans/*{app-launcher,snake-demo,launcher-settings,terminal-shell}* | wc -l` → `8`.

- [ ] **Step 3: building-an-app.md → deluge-ndk, Rust section updated**

Copy it, then rewrite the **Rust** section (which documents the retired `cargo xtask image/bare` flow) to point at the new tool:

```bash
cp ~/GitHub/deluge-linux-sdk/docs/building-an-app.md ~/GitHub/deluge-ndk/docs/building-an-app.md
```

Then edit `~/GitHub/deluge-ndk/docs/building-an-app.md`: locate the `## Rust` section and replace its `cargo xtask image <app>` / `cargo xtask bare <app>` guidance with the current flow — building a Rust Linux app now uses `deluge-sdk`'s `cargo deluge linux`:

```markdown
## Rust

Rust apps for the Deluge's Linux userland live in the **[deluge-sdk]** repo and
build with `cargo deluge linux` (which cross-compiles for
`armv7-unknown-linux-musleabihf` against this bundle's `libdeluge` sysroot and
packs the image with `deluge-mkimage`):

    export DELUGE_BASE=~/GitHub/deluge-linux/out/deluge-base-<ver>
    cargo deluge linux            # -> target/<APP>.ELF   for /APPS/
    cargo deluge linux --bare     # -> target/bare/<app>  for /LINUX/APPS/

See the deluge-sdk repo for the Rust SDK, the native `deluge-hal-linux` API, and
the example apps under `examples-linux/`.

[deluge-sdk]: https://github.com/FirestormAudio/deluge-sdk
```

Leave the C/CMake authoring content (FetchContent, `deluge_add_app`, the two products, the static requirement) intact — that is exactly what `deluge-ndk` documents. Then commit:

```bash
git -C ~/GitHub/deluge-ndk add docs/building-an-app.md
git -C ~/GitHub/deluge-ndk commit -q -m "docs: import building-an-app guide; point the Rust path at cargo deluge linux

The C/CMake authoring guide belongs with libdeluge; the Rust section's retired
xtask flow is replaced with deluge-sdk's cargo deluge linux."
echo done-ndk
```
Expected: `done-ndk`.

---

## Task 2: Retire deluge-linux-sdk (bundle, then delete) — controller-run

**Files:** none (git bundle + delete). **The `rm -rf` is run by the controller, not a subagent.**

- [ ] **Step 1: Final completeness re-audit — nothing unique left un-migrated**

```bash
cd ~/GitHub/deluge-linux-sdk
git remote -v || echo "(no remote — local only)"
# Everything tracked should be either migrated (src/include/cmake/crates/examples/tests/tools/docs)
# or a superseded root/config file. List what remains as a sanity check:
git ls-files | grep -vE '^(src|include|cmake|crates|examples|tests|tools|docs)/' 
```
Expected: only superseded root files (`.gitignore`, `.cargo/config.toml`, `CMakeLists.txt`, `Cargo.toml`, `README.md`, `bundle.lock`, `deluge.pc.in`, `.github/workflows/ci.yml`) — all of which have replacements in the three repos. No surprise unique files.

- [ ] **Step 2: Bundle the full history to a durable file**

```bash
cd ~/GitHub/deluge-linux-sdk
git bundle create ~/GitHub/deluge-linux-sdk.bundle --all
git -C /tmp bundle verify ~/GitHub/deluge-linux-sdk.bundle 2>&1 | tail -2
ls -lh ~/GitHub/deluge-linux-sdk.bundle
```
Expected: `The bundle records a complete history` (or `is okay`), and the `.bundle` file exists. This single file can be `git clone`d back into a repo if the history is ever needed.

- [ ] **Step 3: Delete the local checkout**

```bash
rm -rf ~/GitHub/deluge-linux-sdk
ls -d ~/GitHub/deluge-linux-sdk 2>/dev/null && echo "STILL PRESENT (bad)" || echo "deleted (good)"
ls ~/GitHub/deluge-linux-sdk.bundle && echo "history preserved in bundle"
```
Expected: `deleted (good)`, then the bundle path confirming the history survives.

- [ ] **Step 4: Confirm the three repos are unaffected**

```bash
for r in deluge-ndk deluge-linux deluge-sdk; do
  echo "== $r =="; git -C ~/GitHub/$r status --short | head -3; git -C ~/GitHub/$r log --oneline -1 | cat
done
```
Expected: each repo clean/normal, none referencing the deleted path. (`deluge-linux-ui`'s deps were already collapsed to in-workspace `../` in Stage 3a, so nothing points at the old repo.)

---

## Self-Review

**Spec coverage (reorg spec §4 "deluge-linux-sdk retired / dissolves along its grain"):** the last unmigrated content (docs) is placed with its code — mkimage/elf design → `deluge-linux`, app design → `deluge-sdk`, authoring guide → `deluge-ndk` (Rust path updated to `cargo deluge linux`) ✓ (Task 1); the repo is retired with history preserved in a bundle, then the local checkout deleted ✓ (Task 2). The reorg (Phase 0) is complete after this.

**Placeholder scan:** none — the doc mapping is explicit; `<ver>`/`<app>`/`<APP>` in the migrated guide are user-facing template placeholders (correct for a how-to), not plan gaps.

**Type/name consistency:** the four app-feature slugs (`app-launcher`, `snake-demo`, `launcher-settings-shutdown`, `terminal-shell`) match the source filenames; the bundle path `~/GitHub/deluge-linux-sdk.bundle` is identical across Task 2 steps.

**Reversibility:** the only destructive step (`rm -rf`, Task 2 Step 3) is preceded by a verified `git bundle` (Step 2), so the deletion is recoverable. Nothing else is destructive. The three repos have no path-dependency on the deleted repo (verified Task 2 Step 4).
