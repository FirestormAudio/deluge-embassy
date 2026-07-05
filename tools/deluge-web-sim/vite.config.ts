import { defineConfig } from "vite";

// Standalone faceplate emulator: no SharedArrayBuffer (no audio in MVP), so no
// cross-origin-isolation headers are needed — unlike tools/wren-web/app.
export default defineConfig({
  base: "./",
  build: { target: "es2022", outDir: "dist" },
});
