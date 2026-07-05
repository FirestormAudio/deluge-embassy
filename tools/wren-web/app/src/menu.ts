// A small accessible popover menu for the topbar (the Project ▾ button). No
// dependency: a button toggles a role="menu" list; a branch item expands its
// submenu inline (accordion). Arrow keys / Home / End move focus among visible
// items, Enter/Space activate, Escape or an outside click closes.

export interface MenuItem {
  label: string;
  action?: () => void; // leaf: run, then close the menu
  submenu?: () => MenuItem[]; // branch: built lazily on each open (live state)
  danger?: boolean; // destructive styling
  onDelete?: () => void; // optional inline ✕ affordance on a row
}

export interface Menu {
  close(): void;
}

/// Wire `button` to open a popover built from `items()` (re-invoked on every
/// open, so dynamic entries like saved slots stay current). The popover is
/// appended to the button's parent.
export function createMenu(button: HTMLButtonElement, items: () => MenuItem[]): Menu {
  let panel: HTMLElement | null = null;

  const focusables = (): HTMLElement[] =>
    panel
      ? Array.from(panel.querySelectorAll<HTMLElement>('[role="menuitem"]')).filter(
          (el) => el.offsetParent !== null,
        )
      : [];

  const close = () => {
    if (!panel) return;
    panel.remove();
    panel = null;
    button.setAttribute("aria-expanded", "false");
    document.removeEventListener("pointerdown", onDocDown, true);
    document.removeEventListener("keydown", onKey, true);
  };

  const onKey = (e: KeyboardEvent) => {
    if (!panel) return;
    const f = focusables();
    const i = f.indexOf(document.activeElement as HTMLElement);
    if (e.key === "Escape") {
      close();
      button.focus();
      e.preventDefault();
    } else if (e.key === "ArrowDown") {
      f[(i + 1) % f.length]?.focus();
      e.preventDefault();
    } else if (e.key === "ArrowUp") {
      f[(i - 1 + f.length) % f.length]?.focus();
      e.preventDefault();
    } else if (e.key === "Home") {
      f[0]?.focus();
      e.preventDefault();
    } else if (e.key === "End") {
      f[f.length - 1]?.focus();
      e.preventDefault();
    }
  };

  const onDocDown = (e: PointerEvent) => {
    if (panel && !panel.contains(e.target as Node) && e.target !== button) close();
  };

  const renderList = (list: MenuItem[], depth: number): HTMLUListElement => {
    const ul = document.createElement("ul");
    ul.className = "menu-list";
    ul.setAttribute("role", "menu");
    for (const item of list) {
      const li = document.createElement("li");
      li.className = "menu-row" + (depth ? " menu-sub" : "");
      const btn = document.createElement("button");
      btn.type = "button";
      btn.className = "menu-item" + (item.danger ? " menu-danger" : "");
      btn.setAttribute("role", "menuitem");
      btn.textContent = item.label;
      if (item.submenu) {
        btn.setAttribute("aria-haspopup", "true");
        btn.setAttribute("aria-expanded", "false");
        const sub = renderList(item.submenu(), depth + 1);
        sub.hidden = true;
        btn.addEventListener("click", () => {
          const willOpen = sub.hidden;
          sub.hidden = !willOpen;
          btn.setAttribute("aria-expanded", String(willOpen));
        });
        li.append(btn, sub);
      } else {
        btn.addEventListener("click", () => {
          close();
          item.action?.();
        });
        li.append(btn);
        if (item.onDelete) {
          const del = document.createElement("button");
          del.type = "button";
          del.className = "menu-del";
          del.setAttribute("aria-label", `Delete ${item.label}`);
          del.textContent = "✕";
          del.addEventListener("click", (e) => {
            e.stopPropagation();
            item.onDelete!();
            close();
            open(); // rebuild from items() so the list reflects the deletion
          });
          li.append(del);
        }
      }
      ul.append(li);
    }
    return ul;
  };

  const open = () => {
    if (panel) {
      close();
      return;
    }
    panel = document.createElement("div");
    panel.className = "menu-popover";
    panel.append(renderList(items(), 0));
    (button.parentElement ?? document.body).append(panel);
    button.setAttribute("aria-expanded", "true");
    document.addEventListener("pointerdown", onDocDown, true);
    document.addEventListener("keydown", onKey, true);
    focusables()[0]?.focus();
  };

  button.setAttribute("aria-haspopup", "true");
  button.setAttribute("aria-expanded", "false");
  button.addEventListener("click", (e) => {
    e.stopPropagation();
    open();
  });

  return { close };
}
