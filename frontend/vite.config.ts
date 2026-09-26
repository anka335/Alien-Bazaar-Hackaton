import react from "@vitejs/plugin-react";
import { defineConfig } from "vite";

// The sorter serves the build from src/sorter/dashboard/web (not in git). `npm run dev` proxies
// the API to a sorter running on :8000.
const backend = "http://127.0.0.1:8000";

export default defineConfig({
  plugins: [react()],
  build: {
    outDir: "../src/sorter/dashboard/web",
    emptyOutDir: true,
    chunkSizeWarningLimit: 1000, // three.js
  },
  server: {
    proxy: {
      "/api": backend,
      "/stream": backend,
      "/snapshot": backend,
      "/twin-assets": backend,
      "/ws": { target: backend, ws: true },
    },
  },
});
