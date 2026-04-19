# Preact Web UI

This is the Vite + Preact source tree for the next embedded Web UI.

```bash
cd webui
npm install
npm run dev
npm run build
cd ..
python3 scripts/build_webui.py
```

The dev server proxies `/api` and `/streamer` to `http://atoms3mic.local`. Change the proxy target in `vite.config.ts` if your device uses a different hostname.

The firmware still serves the embedded C++ UI today. After `npm run build`, `scripts/build_webui.py` emits `src/WebUIAssets.h`, which can be included by `src/WebUI.cpp` when you are ready to flip the served index page to the generated asset.
