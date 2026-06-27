import { defineConfig } from "vite";

// Cross-origin isolation enables SharedArrayBuffer (used for the lock-free audio
// command ring between the main thread and the AudioWorklet — see src/audio.ts).
// require-corp is the widely-supported value; all resources are same-origin
// (fonts are self-hosted, see src/fonts.ts), so it just works. The app falls back
// to postMessage when isolation is unavailable (e.g. a host that can't set these
// headers).
const crossOriginIsolation = {
  "Cross-Origin-Opener-Policy": "same-origin",
  "Cross-Origin-Embedder-Policy": "require-corp",
};

export default defineConfig({
  base: "./",
  build: { target: "es2022", outDir: "dist" },
  worker: { format: "es" },
  server: { headers: crossOriginIsolation },
  preview: { headers: crossOriginIsolation },
});
