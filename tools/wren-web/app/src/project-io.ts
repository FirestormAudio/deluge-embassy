// Wires the Project ▾ menu to the project lifecycle: New (blank/template),
// Open (a named slot), Import (a folder off disk), Save / Save As (named
// slots), and Download (a .zip for the SD card). Owns the current-slot name
// (shown in the button label), the destructive-action guard, the hidden folder
// <input>, and the download anchor.
import { createMenu, type MenuItem } from "./menu";
import { ProjectStore, blankProject, projectFromExample } from "./project";
import { type Example } from "./examples";
import { listSlots, readSlot, writeSlot, deleteSlot } from "./slots";
import { projectFromTree, type TreeEntry } from "./import-tree";
import { zipFiles } from "./zip";

export interface ProjectMenuOpts {
  button: HTMLButtonElement;
  store: ProjectStore;
  examples: Example[];
  setStatus: (msg: string) => void;
}

export function setupProjectMenu(opts: ProjectMenuOpts): void {
  const { button, store, examples, setStatus } = opts;
  let currentSlot: string | null = null;

  const label = () => {
    button.textContent = currentSlot ? `Project · ${currentSlot} ▾` : "Project ▾";
  };
  label();

  const guard = () =>
    confirm("Replace the current project? Save or Download it first to keep it.");

  const newBlank = () => {
    if (!guard()) return;
    store.replace(blankProject());
    currentSlot = null;
    label();
  };
  const newExample = (ex: Example) => {
    if (!guard()) return;
    store.replace(projectFromExample(ex));
    currentSlot = null;
    label();
  };
  const openSlot = (name: string) => {
    const p = readSlot(name);
    if (!p || !guard()) return;
    store.replace(p);
    currentSlot = name;
    label();
  };
  const saveAs = () => {
    const name = prompt("Save project as:", currentSlot ?? "")?.trim();
    if (!name) return;
    if (listSlots().includes(name) && !confirm(`Overwrite "${name}"?`)) return;
    if (writeSlot(name, store.project)) {
      currentSlot = name;
      label();
      setStatus(`saved "${name}"`);
    } else {
      setStatus("save failed (storage full?)");
    }
  };
  const save = () => {
    if (!currentSlot) return saveAs();
    setStatus(writeSlot(currentSlot, store.project) ? `saved "${currentSlot}"` : "save failed (storage full?)");
  };
  const download = () => {
    // Cast: zipFiles returns a Uint8Array whose buffer type is ArrayBufferLike,
    // which the DOM's BlobPart (ArrayBuffer-backed) doesn't accept structurally.
    const blob = new Blob([zipFiles(store.project.files) as BlobPart], { type: "application/zip" });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = `${currentSlot ?? "wren-project"}.zip`;
    document.body.append(a);
    a.click();
    a.remove();
    URL.revokeObjectURL(url);
  };

  // Hidden folder picker for Import.
  const picker = document.createElement("input");
  picker.type = "file";
  picker.id = "import-picker";
  picker.multiple = true;
  picker.hidden = true;
  // webkitdirectory is non-standard; set via attribute so TS doesn't complain.
  picker.setAttribute("webkitdirectory", "");
  picker.addEventListener("change", async () => {
    const list = picker.files;
    if (!list || list.length === 0) return;
    const entries: TreeEntry[] = [];
    for (const f of Array.from(list)) {
      const path = (f as File & { webkitRelativePath?: string }).webkitRelativePath || f.name;
      entries.push({ path, content: await f.text() });
    }
    picker.value = ""; // allow re-picking the same folder
    const project = projectFromTree(entries);
    if (!project) {
      setStatus("no .wren files found");
      return;
    }
    if (!guard()) return;
    store.replace(project);
    currentSlot = null;
    label();
    setStatus("imported from disk");
  });
  document.body.append(picker);

  const items = (): MenuItem[] => [
    {
      label: "New",
      submenu: () => [
        { label: "Blank", action: newBlank },
        ...examples.map((ex) => ({ label: ex.name, action: () => newExample(ex) })),
      ],
    },
    {
      label: "Open",
      submenu: () => {
        const names = listSlots();
        if (names.length === 0) return [{ label: "(no saved projects)" }];
        return names.map((n) => ({
          label: n,
          action: () => openSlot(n),
          onDelete: () => deleteSlot(n),
        }));
      },
    },
    { label: "Import from disk…", action: () => picker.click() },
    { label: "Save", action: save },
    { label: "Save As…", action: saveAs },
    { label: "Download .zip", action: download },
  ];

  createMenu(button, items);
}
