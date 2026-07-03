// Draggable splitters for the left panel (file/debug sidebar) width and the
// console height. Both dimensions are CSS custom properties on `.bench`
// (`--left-w`, `--console-h`); dragging a handle updates the variable (Monaco's
// automaticLayout reflows the editor) and the size is persisted to localStorage.
const LEFT_KEY = "wren-deluge:left-w";
const CONSOLE_KEY = "wren-deluge:console-h";

const clamp = (v: number, lo: number, hi: number) => Math.min(Math.max(v, lo), hi);

export function setupResizers() {
  const bench = document.querySelector<HTMLElement>(".bench");
  const editorPane = document.querySelector<HTMLElement>(".editor-pane");
  const leftHandle = document.querySelector<HTMLElement>("#resize-left");
  const consoleHandle = document.querySelector<HTMLElement>("#resize-console");
  if (!bench || !editorPane) return;

  // Restore persisted sizes.
  const savedW = localStorage.getItem(LEFT_KEY);
  const savedH = localStorage.getItem(CONSOLE_KEY);
  if (savedW) bench.style.setProperty("--left-w", savedW);
  if (savedH) bench.style.setProperty("--console-h", savedH);

  // Left panel width: drag X → distance from the bench's left edge.
  if (leftHandle) {
    dragHandle(leftHandle, "col", (e) => {
      const w = clamp(e.clientX - bench.getBoundingClientRect().left, 150, 520);
      bench.style.setProperty("--left-w", `${w}px`);
      return `${w}px`;
    }, LEFT_KEY);
  }

  // Console height: drag Y → distance from the editor pane's bottom edge.
  if (consoleHandle) {
    dragHandle(consoleHandle, "row", (e) => {
      const pane = editorPane.getBoundingClientRect();
      const h = clamp(pane.bottom - e.clientY, 72, pane.height - 140);
      bench.style.setProperty("--console-h", `${h}px`);
      return `${h}px`;
    }, CONSOLE_KEY);
  }
}

/// Wire a splitter: pointer-drag applies `apply` (which returns the size string
/// to persist under `key`); Arrow keys nudge it for keyboard users.
function dragHandle(
  handle: HTMLElement,
  axis: "col" | "row",
  apply: (e: { clientX: number; clientY: number }) => string,
  key: string,
) {
  let value = "";

  handle.addEventListener("pointerdown", (e) => {
    e.preventDefault();
    handle.setPointerCapture(e.pointerId);
    handle.classList.add("dragging");
    document.body.classList.add(axis === "col" ? "resizing-col" : "resizing-row");

    const onMove = (ev: PointerEvent) => { value = apply(ev); };
    const onUp = () => {
      handle.classList.remove("dragging");
      document.body.classList.remove("resizing-col", "resizing-row");
      handle.removeEventListener("pointermove", onMove);
      handle.removeEventListener("pointerup", onUp);
      if (value) { try { localStorage.setItem(key, value); } catch { /* best-effort */ } }
    };
    handle.addEventListener("pointermove", onMove);
    handle.addEventListener("pointerup", onUp);
  });

  // Keyboard: the splitter is a role="separator" — arrows resize by 16px.
  handle.addEventListener("keydown", (e) => {
    const rect = handle.getBoundingClientRect();
    const step = 16;
    let d: { clientX: number; clientY: number } | null = null;
    if (axis === "col" && e.key === "ArrowLeft") d = { clientX: rect.left - step, clientY: rect.top };
    else if (axis === "col" && e.key === "ArrowRight") d = { clientX: rect.left + step, clientY: rect.top };
    else if (axis === "row" && e.key === "ArrowUp") d = { clientX: rect.left, clientY: rect.top - step };
    else if (axis === "row" && e.key === "ArrowDown") d = { clientX: rect.left, clientY: rect.top + step };
    if (d) {
      e.preventDefault();
      value = apply(d);
      try { localStorage.setItem(key, value); } catch { /* best-effort */ }
    }
  });
}
