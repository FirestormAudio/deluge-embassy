import { defineConfig, devices } from "@playwright/test";

// Task 3.3: e2e for the browser DebugController. The app must be served
// cross-origin-isolated (COOP/COEP) for SharedArrayBuffer — vite's preview
// server emits those headers (see vite.config.ts). We build then preview so the
// production bundle's worker/nested-worker handling is what's exercised.
const PORT = 4173;

export default defineConfig({
  testDir: "./tests",
  fullyParallel: false,
  workers: 1,
  reporter: [["list"]],
  use: {
    baseURL: `http://localhost:${PORT}`,
    trace: "on-first-retry",
  },
  projects: [{ name: "chromium", use: { ...devices["Desktop Chrome"] } }],
  webServer: {
    command: `npm run build && npm run preview -- --port ${PORT} --strictPort`,
    url: `http://localhost:${PORT}`,
    reuseExistingServer: !process.env.CI,
    timeout: 180_000,
  },
});
