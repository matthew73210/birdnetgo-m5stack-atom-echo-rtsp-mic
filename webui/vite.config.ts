import { defineConfig } from "vite";
import preact from "@preact/preset-vite";

export default defineConfig({
  plugins: [preact()],
  build: {
    assetsInlineLimit: 1000000,
    cssCodeSplit: false,
    minify: "esbuild",
    rollupOptions: {
      output: {
        manualChunks: undefined
      }
    }
  },
  server: {
    proxy: {
      "/api": "http://atoms3mic.local",
      "/streamer": "http://atoms3mic.local"
    }
  }
});
