import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";
import cesium from "vite-plugin-cesium";

// vite-plugin-cesium copies Cesium's static assets (Workers, Assets, Widgets)
// into the build and sets CESIUM_BASE_URL, so the viewer is fully self-hosted
// (no Cesium Ion / CDN dependency) — required for the air-gapped profile.
export default defineConfig({
  plugins: [react(), cesium()],
  server: { host: true, port: 5173 },
  build: { target: "es2022", chunkSizeWarningLimit: 4000 },
});
