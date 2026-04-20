# Frontend and Streaming Review

## Current State

- The firmware build is healthy, and the served index page now comes from a generated Preact/Vite asset header.
- RTSP now uses a bounded session table. Core 0 negotiates pending clients, then Core 1 owns every active streaming socket after `PLAY`.
- The device already exposes multiple audio access shapes:
  - RTSP/RTP L16 mono PCM at `rtsp://<device>:8554/audio`
  - Browser WebAudio PCM frames at `/api/web_audio_pcm`
  - Bounded WAV PCM chunks at `/api/web_audio_wav`
  - JSON PCM diagnostics frames at `/api/web_audio`
- UDP RTSP remains disabled by default. The active multi-client path is TCP interleaved RTP.

## Code Review Findings

### 1. RTSP Multi-client Model

The old globals (`rtspClient`, `rtpSequence`, `rtpTimestamp`, `activeRtspTransport`, UDP ports) described one stream. They have been replaced by a fixed two-slot `RtspSession` table with per-client RTP sequence, timestamp, SSRC, parse buffer, write-failure budget, and keepalive parser state.

Current safety model:

1. Core 0 accepts clients into free session slots.
2. Core 0 handles `OPTIONS`, `DESCRIBE`, and `SETUP` for sessions that are not streaming.
3. On `PLAY`, Core 0 sends the response, marks that session streaming, and stops touching that socket.
4. Core 1 sends RTP to all streaming sessions and handles keepalive/teardown commands on those sockets.
5. A slow or disconnected client is removed by Core 1 without stopping the other session.

Default: `MAX_RTSP_CLIENTS = 2`, with TCP interleaved RTP only.

### 2. Multi-format Streaming Should Stay PCM-first

BirdNET-Go wants clean PCM. L16 over RTSP and WebAudio PCM are the right base formats for this hardware.

Feasible next formats:

- RTP PCMU as a compatibility mode for legacy RTSP clients, but only as an optional degraded-quality format.

Avoid on this ESP32 path unless there is a hard requirement:

- Opus or AAC encode on-device. The compute, memory, latency, and dependency cost are a poor fit for the current always-on audio pipeline.

### 3. Frontend Build Recommendation

Implemented: **Preact + Vite**, built into a generated C++ header.

Why:

- Preact is small enough for an embedded control panel while still giving components, hooks, and a familiar React-like model.
- Vite gives fast local development and a production `dist/` output that can be transformed into PROGMEM string assets.
- The current UI already behaves like a small reactive app: polling status endpoints, updating controls, drawing graphs, and keeping local edit locks.

Repo layout:

```text
webui/
  package.json
  index.html
  src/
    app.tsx
    styles.css
scripts/
  build_webui.py
src/
  WebUIAssets.h
```

Build flow:

1. `npm --prefix webui run build`
2. `python3 scripts/build_webui.py`
3. `src/WebUI.cpp` serves `WEBUI_INDEX_HTML` from `src/WebUIAssets.h`.

## Immediate Changes Made

- Added a Preact/Vite frontend source tree and generated firmware asset header.
- Wired the generated frontend as the served index page.
- Added bounded two-client RTSP TCP interleaved streaming with per-session RTP state.
- Added a Streams & Clients panel/API fields to report connected and active RTSP clients.
- Added `/api/stream_options` so clients can discover supported formats and current RTSP policy.
- Added `/api/web_audio_wav` for short WAV PCM chunks from the diagnostics ring buffer.
- Fixed the read-only I2S Shift markup so it renders as valid HTML.
- Closed the Status card correctly so the UI no longer nests major cards inside it.
- Modernized the palette, spacing, card radius, and action styling while keeping the page fully embedded.
- Removed current deprecation warnings in project code where possible.
