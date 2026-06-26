import { defineConfig } from "vite";

// The wasm core links wasi-libc, so the wasm is served as a static asset from
// `public/` and instantiated with a browser WASI shim at runtime (see src/sim.ts).
export default defineConfig({
  base: "./",
  build: { target: "es2022", outDir: "dist" },
  worker: { format: "es" },
});
