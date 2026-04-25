# Preact Web UI

This is the Vite + Preact source tree for the embedded Web UI.

```bash
cd webui
npm install
npm run dev
npm run build
cd ..
python3 scripts/build_webui.py
```

The dev server proxies `/api` and `/streamer` to `http://atoms3mic.local`. Change the proxy target in `vite.config.ts` if your device uses a different hostname.

After `npm run build`, `scripts/build_webui.py` emits `src/WebUIAssets.h`. `src/WebUI.cpp` serves that generated asset as the root index page.
