// Wren language support for Monaco.
//
// First-cut highlighting via a Monarch tokenizer + a completion provider seeded
// from the Deluge prelude's classes/methods. The plan's longer-term path is to
// run the reused TextMate grammar (public/wren.tmLanguage.json) through
// vscode-textmate for byte-identical highlighting to the VSCode extension; this
// Monarch grammar keeps the editor self-contained until that pipeline lands.
import * as monaco from "monaco-editor/esm/vs/editor/editor.api";

export const WREN_ID = "wren";

// Deluge prelude surface, for completion (see deluge-wren-core/wren/prelude.wren).
const PRELUDE_API: Record<string, string[]> = {
  Osc: ["sine(f)", "saw(f)", "square(f)", "tri(f)"],
  Env: ["ar(attack, release)"],
  Noise: ["new()"],
  Out: ["patch(node)", "reset()"],
  Midi: ["noteOn(ch, note, vel)", "noteOff(ch, note, vel)", "cc(ch, num, val)", "onNoteOn", "onNoteOff", "onCC"],
  Pads: ["onPress", "onRelease"],
  Buttons: ["onPress", "onRelease"],
  Enc: ["onTurn"],
  Led: ["on(id)", "off(id)"],
  Oled: ["clear()", "text(x, y, s)", "pixel(x, y, on)", "show()"],
  Metro: ["new()", "start(fn, seconds)", "stop()", "time"],
};

export function registerWren(m: typeof monaco) {
  m.languages.register({ id: WREN_ID });

  m.languages.setLanguageConfiguration(WREN_ID, {
    comments: { lineComment: "//", blockComment: ["/*", "*/"] },
    brackets: [["{", "}"], ["[", "]"], ["(", ")"]],
    autoClosingPairs: [
      { open: "{", close: "}" },
      { open: "[", close: "]" },
      { open: "(", close: ")" },
      { open: '"', close: '"' },
    ],
    surroundingPairs: [
      { open: "{", close: "}" },
      { open: "[", close: "]" },
      { open: "(", close: ")" },
      { open: '"', close: '"' },
    ],
  });

  m.languages.setMonarchTokensProvider(WREN_ID, {
    keywords: [
      "break", "class", "construct", "continue", "else", "false", "for", "foreign",
      "if", "import", "in", "is", "null", "return", "static", "super", "this", "true",
      "var", "while",
    ],
    builtins: Object.keys(PRELUDE_API).concat(["System", "Fn", "List", "Map", "Num", "String", "Bool", "Range", "Fiber", "Random"]),
    operators: ["+", "-", "*", "/", "%", "=", "==", "!=", "<", ">", "<=", ">=", "&&", "||", "!", "..", "...", "?", ":"],
    tokenizer: {
      root: [
        [/\/\/.*$/, "comment"],
        [/\/\*/, "comment", "@comment"],
        [/"/, "string", "@string"],
        [/[A-Z][A-Za-z0-9_]*/, { cases: { "@builtins": "type.identifier", "@default": "type.identifier" } }],
        [/[a-z_][A-Za-z0-9_]*/, { cases: { "@keywords": "keyword", "@default": "identifier" } }],
        [/0x[0-9a-fA-F]+|\d+\.?\d*([eE][-+]?\d+)?/, "number"],
        [/%\(/, "delimiter.interpolation", "@interp"],
        [/[{}()\[\]]/, "@brackets"],
        [/[-+*/%=<>!&|?:.]+/, { cases: { "@operators": "operator", "@default": "" } }],
      ],
      comment: [
        [/[^/*]+/, "comment"],
        [/\*\//, "comment", "@pop"],
        [/[/*]/, "comment"],
      ],
      string: [
        [/%\(/, "delimiter.interpolation", "@interp"],
        [/[^"%]+/, "string"],
        [/%/, "string"],
        [/"/, "string", "@pop"],
      ],
      interp: [
        [/\)/, "delimiter.interpolation", "@pop"],
        { include: "root" },
      ],
    },
  });

  m.languages.registerCompletionItemProvider(WREN_ID, {
    triggerCharacters: ["."],
    provideCompletionItems(model, position) {
      const word = model.getWordUntilPosition(position);
      const range = { startLineNumber: position.lineNumber, endLineNumber: position.lineNumber, startColumn: word.startColumn, endColumn: word.endColumn };
      const suggestions: monaco.languages.CompletionItem[] = [];
      for (const [cls, methods] of Object.entries(PRELUDE_API)) {
        suggestions.push({ label: cls, kind: m.languages.CompletionItemKind.Class, insertText: cls, range });
        for (const meth of methods) {
          suggestions.push({ label: `${cls}.${meth}`, kind: m.languages.CompletionItemKind.Method, insertText: meth, range });
        }
      }
      return { suggestions };
    },
  });
}
