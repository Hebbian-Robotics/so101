import { resolve } from "node:path";
import { defineConfig } from "vite";

export default defineConfig({
  root: resolve(import.meta.dirname, "web"),
  base: "./",
  publicDir: false,
  build: {
    outDir: resolve(import.meta.dirname, "../src/so101_setup/assets/web"),
    emptyOutDir: true,
    target: "es2023",
    chunkSizeWarningLimit: 1500,
  },
});
