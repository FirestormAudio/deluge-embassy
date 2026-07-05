// Monaco editor wiring: worker env (Vite), a dark theme tuned to the chassis
// palette, the Wren language, and a helper to surface VM errors as markers.
// The core editor API only — not the `monaco-editor` barrel, which bundles every
// built-in language's tokenizer + the TS/HTML/CSS/JSON language services (~3 MB).
// We register Wren ourselves, so none of that is needed.
import * as monaco from "monaco-editor/esm/vs/editor/editor.api";
// The bare `editor.api` ships NO editor contributions, so registered hover /
// completion / go-to-definition providers would never be invoked (there'd be no
// controller to call them — the app looked like hover was "broken"). Pull in
// just the three we use as side-effect imports; this keeps the lean bundle (we
// still avoid the full `monaco-editor` barrel that bundles every language's
// tokenizer + the TS/HTML/CSS/JSON language services).
import "monaco-editor/esm/vs/editor/contrib/hover/browser/hoverContribution";
import "monaco-editor/esm/vs/editor/contrib/suggest/browser/suggestController";
import "monaco-editor/esm/vs/editor/contrib/gotoSymbol/browser/goToCommands";
import EditorWorker from "monaco-editor/esm/vs/editor/editor.worker?worker";
import { registerWren, WREN_ID } from "./wren-lang";
import { registerWrenTextMate } from "./textmate";
import type { Analyzer } from "./analyzer";

self.MonacoEnvironment = {
  getWorker: () => new EditorWorker(),
};

monaco.editor.defineTheme("chassis", {
  base: "vs-dark",
  inherit: true,
  // Rules key on TextMate scopes (see src/textmate.ts); Monaco matches by
  // dotted-prefix, longest wins.
  rules: [
    { token: "comment", foreground: "5a6470", fontStyle: "italic" },
    { token: "keyword", foreground: "f2b549" },
    { token: "keyword.operator", foreground: "7c8794" },
    { token: "string", foreground: "b6d98a" },
    { token: "constant.numeric", foreground: "e08f6a" },
    { token: "constant.language", foreground: "e0a86a" },
    { token: "constant.character.escape", foreground: "6ad0e0" },
    { token: "entity.name.type", foreground: "8fe9ff" },
    { token: "support.class", foreground: "8fe9ff" },
    { token: "entity.name.function", foreground: "9fd6c4" },
    { token: "variable.language", foreground: "8fe9ff", fontStyle: "italic" },
    { token: "variable.other", foreground: "c9d2dd" },
    { token: "entity.other.attribute-name", foreground: "e0a86a" },
    { token: "punctuation.section.interpolation", foreground: "f2b549" },
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
    // Match the app's native scrollbars (see style.css).
    "scrollbar.shadow": "#00000000",
    "scrollbarSlider.background": "#2c333d99",
    "scrollbarSlider.hoverBackground": "#3b4654",
    "scrollbarSlider.activeBackground": "#8fe9ffaa",
  },
});

export function createEditor(host: HTMLElement, value: string) {
  registerWren(monaco);
  // Replace the Monarch highlighter with the reused TextMate grammar (async:
  // loads oniguruma wasm + the grammar; Monaco re-tokenizes once ready).
  void registerWrenTextMate(WREN_ID, `${import.meta.env.BASE_URL}wren.tmLanguage.json`);
  const editor = monaco.editor.create(host, {
    value,
    language: WREN_ID,
    theme: "chassis",
    fontFamily: "'IBM Plex Mono', monospace",
    fontSize: 13,
    lineHeight: 20,
    minimap: { enabled: false },
    scrollBeyondLastLine: false,
    glyphMargin: true, // breakpoint gutter (see debug/gutter.ts)
    // Keep the line-number column snug so the breakpoint glyph sits right next
    // to the numbers (VSCode-like) instead of floating far left; expands past
    // this minimum for longer files.
    lineNumbersMinChars: 2,
    folding: false, // no folding margin between numbers and code (unused for Wren)
    padding: { top: 14 },
    renderLineHighlight: "line",
    smoothScrolling: true,
    automaticLayout: true,
  });
  return { editor, monaco };
}

/// Register analyzer-backed language intelligence (hover, go-to-definition, and
/// symbol completion) for Wren. Queries run in the analyzer worker; results merge
/// with the static prelude completion from `registerWren`.
export function registerIntelligence(analyzer: Analyzer) {
  const completionKind = [
    monaco.languages.CompletionItemKind.Class,
    monaco.languages.CompletionItemKind.Method,
    monaco.languages.CompletionItemKind.Field,
    monaco.languages.CompletionItemKind.Variable,
    monaco.languages.CompletionItemKind.Module,
  ];

  monaco.languages.registerHoverProvider(WREN_ID, {
    async provideHover(model, position) {
      const md = await analyzer.hover(model.getValue(), model.getOffsetAt(position));
      if (!md) return null;
      return { contents: [{ value: md }] };
    },
  });

  monaco.languages.registerDefinitionProvider(WREN_ID, {
    async provideDefinition(model, position) {
      const def = await analyzer.definition(model.getValue(), model.getOffsetAt(position));
      if (!def || def.inPrelude) return null; // builtins have no in-file location
      return {
        uri: model.uri,
        range: {
          startLineNumber: def.startLine,
          startColumn: def.startCol,
          endLineNumber: def.endLine,
          endColumn: def.endCol,
        },
      };
    },
  });

  monaco.languages.registerCompletionItemProvider(WREN_ID, {
    async provideCompletionItems(model, position) {
      const word = model.getWordUntilPosition(position);
      const range = {
        startLineNumber: position.lineNumber,
        endLineNumber: position.lineNumber,
        startColumn: word.startColumn,
        endColumn: word.endColumn,
      };
      const items = await analyzer.completions(model.getValue());
      return {
        suggestions: items.map((it) => ({
          label: it.label,
          kind: completionKind[it.kind] ?? monaco.languages.CompletionItemKind.Variable,
          detail: it.detail,
          insertText: it.label,
          range,
        })),
      };
    },
  });
}

/// Current analyzer markers on the model (for inspection/testing).
export function analyzerMarkers(model: monaco.editor.ITextModel) {
  return monaco.editor.getModelMarkers({ owner: "wren-analyzer", resource: model.uri });
}

/// Replace the analyzer's diagnostics on the model (owner "wren-analyzer", kept
/// separate from the VM's run-time errors). `diags` use 1-based line/col.
export function setAnalyzerMarkers(
  model: monaco.editor.ITextModel,
  diags: { startLine: number; startCol: number; endLine: number; endCol: number; severity: number; message: string }[],
) {
  const sev = [
    monaco.MarkerSeverity.Error,
    monaco.MarkerSeverity.Warning,
    monaco.MarkerSeverity.Info,
    monaco.MarkerSeverity.Hint,
  ];
  monaco.editor.setModelMarkers(
    model,
    "wren-analyzer",
    diags.map((d) => ({
      severity: sev[d.severity] ?? monaco.MarkerSeverity.Info,
      message: d.message,
      startLineNumber: d.startLine,
      startColumn: d.startCol,
      endLineNumber: d.endLine,
      // Ensure a visible range even for zero-width spans.
      endColumn: d.endLine === d.startLine && d.endCol <= d.startCol ? d.startCol + 1 : d.endCol,
    })),
  );
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
