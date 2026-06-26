// Monaco editor wiring: worker env (Vite), a dark theme tuned to the chassis
// palette, the Wren language, and a helper to surface VM errors as markers.
import * as monaco from "monaco-editor";
import EditorWorker from "monaco-editor/esm/vs/editor/editor.worker?worker";
import { registerWren, WREN_ID } from "./wren-lang";

self.MonacoEnvironment = {
  getWorker: () => new EditorWorker(),
};

monaco.editor.defineTheme("chassis", {
  base: "vs-dark",
  inherit: true,
  rules: [
    { token: "comment", foreground: "5a6470", fontStyle: "italic" },
    { token: "keyword", foreground: "f2b549" },
    { token: "type.identifier", foreground: "8fe9ff" },
    { token: "string", foreground: "b6d98a" },
    { token: "number", foreground: "e08f6a" },
    { token: "delimiter.interpolation", foreground: "f2b549" },
  ],
  colors: {
    "editor.background": "#15171c",
    "editor.foreground": "#c9d2dd",
    "editorLineNumber.foreground": "#39404a",
    "editorLineNumber.activeForeground": "#8fe9ff",
    "editor.selectionBackground": "#2a3a44",
    "editor.lineHighlightBackground": "#1a1d23",
    "editorCursor.foreground": "#8fe9ff",
    "editorIndentGuide.background1": "#23272e",
  },
});

export function createEditor(host: HTMLElement, value: string) {
  registerWren(monaco);
  const editor = monaco.editor.create(host, {
    value,
    language: WREN_ID,
    theme: "chassis",
    fontFamily: "'IBM Plex Mono', monospace",
    fontSize: 13,
    lineHeight: 20,
    minimap: { enabled: false },
    scrollBeyondLastLine: false,
    padding: { top: 14 },
    renderLineHighlight: "line",
    smoothScrolling: true,
    automaticLayout: true,
  });
  return { editor, monaco };
}

/// Show (or clear) a VM error as a marker on the editor model.
export function setErrorMarker(model: monaco.editor.ITextModel, line: number, message: string) {
  if (line < 0 || !message) {
    monaco.editor.setModelMarkers(model, "wren-vm", []);
    return;
  }
  const lineContent = model.getLineContent(Math.max(1, Math.min(line, model.getLineCount())));
  monaco.editor.setModelMarkers(model, "wren-vm", [
    {
      severity: monaco.MarkerSeverity.Error,
      message,
      startLineNumber: line,
      startColumn: 1,
      endLineNumber: line,
      endColumn: lineContent.length + 1,
    },
  ]);
}
