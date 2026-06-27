// TextMate highlighting for Monaco, using the reused VSCode Wren grammar
// (public/wren.tmLanguage.json) via vscode-textmate + vscode-oniguruma — so the
// highlighting is byte-identical to the wren-rs VSCode extension. Replaces the
// hand-written Monarch tokenizer.
import * as monaco from "monaco-editor/esm/vs/editor/editor.api";
import { Registry, parseRawGrammar, INITIAL, type StateStack } from "vscode-textmate";
import { loadWASM, createOnigScanner, createOnigString } from "vscode-oniguruma";
import onigWasmUrl from "vscode-oniguruma/release/onig.wasm?url";

const SCOPE = "source.wren";

let wasmReady: Promise<void> | null = null;
function ensureOniguruma(): Promise<void> {
  if (!wasmReady) {
    wasmReady = fetch(onigWasmUrl)
      .then((r) => r.arrayBuffer())
      .then((buf) => loadWASM(buf));
  }
  return wasmReady;
}

// Bridge vscode-textmate's immutable StateStack to Monaco's IState.
class TMState implements monaco.languages.IState {
  constructor(readonly stack: StateStack) {}
  clone(): monaco.languages.IState {
    return new TMState(this.stack);
  }
  equals(other: monaco.languages.IState): boolean {
    return other instanceof TMState && other.stack === this.stack;
  }
}

/// Install the TextMate tokenizer for `languageId`. Async (loads the oniguruma
/// wasm + the grammar); Monaco re-tokenizes once the provider is set.
export async function registerWrenTextMate(languageId: string, grammarUrl: string): Promise<boolean> {
  await ensureOniguruma();
  const registry = new Registry({
    onigLib: Promise.resolve({ createOnigScanner, createOnigString }),
    loadGrammar: async (scopeName) => {
      if (scopeName !== SCOPE) return null;
      const text = await fetch(grammarUrl).then((r) => r.text());
      return parseRawGrammar(text, "wren.tmLanguage.json");
    },
  });

  const grammar = await registry.loadGrammar(SCOPE);
  if (!grammar) return false;

  monaco.languages.setTokensProvider(languageId, {
    getInitialState: () => new TMState(INITIAL),
    tokenize: (line, state) => {
      const result = grammar.tokenizeLine(line, (state as TMState).stack);
      const tokens = result.tokens.map((t) => ({
        startIndex: t.startIndex,
        // Deepest (most specific) scope; Monaco's theme matcher resolves it by
        // dotted-prefix, longest match wins.
        scopes: t.scopes[t.scopes.length - 1],
      }));
      return { tokens, endState: new TMState(result.ruleStack) };
    },
  });
  return true;
}
