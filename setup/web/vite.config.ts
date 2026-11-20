import { resolve } from "node:path";
import tailwindcss from "@tailwindcss/vite";
import { defineConfig } from "vite";

export default defineConfig({
  root: resolve(import.meta.dirname, "web"),
  base: "./",
  plugins: [tailwindcss()],
  resolve: {
    alias: { "@": resolve(import.meta.dirname, "src") },
  },
  publicDir: false,
  build: {
    outDir: resolve(import.meta.dirname, "../src/so101_setup/assets/web"),
    emptyOutDir: true,
    target: "es2023",
    chunkSizeWarningLimit: 1500,
    rolldownOptions: {
      onLog(logLevel, log, handler) {
        // These dependencies already run exclusively in the browser.
        if (
          log.code === "MODULE_LEVEL_DIRECTIVE" &&
          log.message.includes('"use client"') &&
          /node_modules\/(?:@base-ui\/|lucide-react\/)/.test(
            log.id ?? log.loc?.file ?? log.message,
          )
        )
          return;
        handler(logLevel, log);
      },
    },
  },
});
