import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

export default defineConfig(({ mode }) => {
  const isRelease = mode === "production";
  return {
    plugins: [react()],
    server: {
      port: 5173,
      proxy: {
        "/api": "http://127.0.0.1:8787",
      },
    },
    build: {
      outDir: isRelease ? "../../../bin/x64/web" : "../../../bin/x64d/web",
      emptyOutDir: true,
    },
  };
});
