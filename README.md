# AtomS3 Lite PDM RTSP Microphone for BirdNET-Go

Firmware for turning an [M5Stack AtomS3 Lite ESP32-S3 Dev Kit](https://shop.m5stack.com/products/atoms3-lite-esp32s3-dev-kit) plus an [M5Stack PDM MEMS Microphone Unit (SPM1423)](https://shop.m5stack.com/products/pdm-microphone-unit-spm1423?variant=35694843134116) into a small RTSP microphone for [BirdNET-Go](https://github.com/tphakala/birdnet-go), VLC, ffplay, or any client that can consume RTSP/RTP L16 mono PCM.

This repository is a fork of [stedrow/birdnetgo-m5stack-atom-echo-rtsp-mic](https://github.com/stedrow/birdnetgo-m5stack-atom-echo-rtsp-mic). The upstream project targets the all-in-one M5Stack Atom Echo / Atom Voice style hardware. This fork is tuned for a two-piece setup: AtomS3 Lite as the ESP32-S3 controller, with the external Unit Mini PDM microphone on the HY2.0-4P connector.

## Target Hardware

Required:

- [AtomS3 Lite ESP32S3 Dev Kit](https://shop.m5stack.com/products/atoms3-lite-esp32s3-dev-kit) - ESP32-S3FN8, 8 MB flash, USB-C, built-in WS2812C RGB LED.
- [PDM MEMS Microphone Unit (SPM1423)](https://shop.m5stack.com/products/pdm-microphone-unit-spm1423?variant=35694843134116) - digital MEMS microphone with PDM CLK/DAT output.
- HY2.0-4P Grove cable, usually included with the PDM Unit.

Wiring with the standard HY2.0 cable:

| PDM Unit wire | PDM Unit signal | AtomS3 Lite pin | Firmware define |
| --- | --- | --- | --- |
| Black | GND | GND | - |
| Red | 5V | 5V | - |
| Yellow | Data | G2 | `I2S_DATA_IN_PIN = 2` |
| White | Clock | G1 | `I2S_CLK_PIN = 1` |

The AtomS3 Lite HY2.0 pinout maps yellow to `G2` and white to `G1`; the PDM Unit maps yellow to data and white to clock. That is why this firmware uses **CLK=G1** and **DATA=G2**.

Important: `i2sShiftBits` is fixed at `0` for this PDM path. Do not make it configurable. These PDM samples are already 16-bit values; shifting them right can turn normal audio into zeros.

## What Changed From Upstream

| Area | Upstream stedrow project | This fork |
| --- | --- | --- |
| Main hardware | M5Stack Atom Echo / Atom Voice style device | AtomS3 Lite plus external Unit Mini PDM |
| PlatformIO board | `m5stack-atom` | `m5stack-atoms3` |
| MCU family | ESP32-PICO-D4 class Atom hardware | ESP32-S3 AtomS3 Lite |
| Microphone pins | Onboard Atom Echo/Voice mic pins | External PDM Unit: `G1` clock, `G2` data |
| LED handling | M5Atom display helpers / Atom LED pin | Arduino RGB LED helper on AtomS3 Lite WS2812 pin `G35` |
| Hostname | `atomecho.local` | `atoms3mic.local` |
| Uploads | USB serial | USB serial first, then Arduino OTA on port `3232` |
| RTSP transport | Atom Echo-oriented single target | TCP interleaved RTP, bounded two-client mode, clean UDP rejection |
| Web UI | Basic status, settings, logs | Rebuilt diagnostics UI, audio level, FFT, telemetry history, browser PCM/WAV preview, thermal controls |
| Audio defaults | Echo-oriented gain/HPF defaults | Conservative gain, HPF on at 180 Hz, PDM centering, limiter, optional noise suppressor and AGC |

This is not a drop-in Atom Echo binary. Atom Echo / Atom Voice has its own onboard SPM1423 microphone, speaker amp, LED, and ESP32-PICO-D4 pinout. To support it cleanly from this branch, add a separate board/pin profile rather than flashing the AtomS3 Lite build unchanged.

## Features

- RTSP server on port `8554`, serving mono 16-bit L16 PCM over RTP.
- TCP interleaved RTP by default; UDP SETUP requests are rejected so clients can retry TCP cleanly.
- Up to two bounded RTSP playback clients.
- mDNS discovery at `atoms3mic.local`.
- WiFiManager captive portal for first Wi-Fi setup.
- Arduino OTA updates after the first USB flash.
- Dual-core audio design: Core 1 owns capture, processing, RTP writes, and streaming sockets; Core 0 handles Web UI, RTSP negotiation, and diagnostics.
- Web UI for levels, Wi-Fi, RTSP clients, audio settings, logs, CPU/temperature history, FFT waterfall, and thermal protection.
- Browser preview endpoints for HTTP L16, AIFF, PCM, WAV chunks, and JSON PCM diagnostics.
- Persistent settings in ESP32 Preferences namespace `audioPrefs`.
- Thermal protection with latch/acknowledge flow.
- LED modes: off, static, or audio level.

## Quick Start

### 1. Flash Over USB

Connect the AtomS3 Lite over USB-C and run:

```bash
pio run -e m5stack-atoms3-lite -t upload
```

If the board does not enter download mode automatically, hold the reset button for about two seconds until the internal green LED lights, then flash again.

### 2. Configure Wi-Fi

On first boot, connect to the `ESP32-RTSP-Mic-AP` access point and choose your Wi-Fi network in the captive portal.

When the device is ready, the LED turns blue and the firmware advertises:

```text
http://atoms3mic.local/
rtsp://atoms3mic.local:8554/audio
```

### 3. Stream Audio

```bash
vlc rtsp://atoms3mic.local:8554/audio
```

```bash
ffplay -rtsp_transport tcp rtsp://atoms3mic.local:8554/audio
```

For BirdNET-Go, set the audio source to:

```text
rtsp://atoms3mic.local:8554/audio
```

The browser UI is available at:

```text
http://atoms3mic.local/
```

The lightweight browser streamer page is:

```text
http://atoms3mic.local/streamer
```

### 4. Use OTA After First Flash

Once the USB-flashed firmware is running on Wi-Fi, PlatformIO can upload over the network:

```bash
pio run -e m5stack-atoms3-lite-ota -t upload
```

The OTA target defaults to `atoms3mic.local:3232`. If you change the hostname or mDNS is unreliable on your network, update `upload_port` in `platformio.ini` or use the current device IP address.

### 5. Flash a prebuilt `.bin` with ESPHome Web

GitHub Actions builds ready-to-download firmware for users who do not want to install PlatformIO.

1. Open this repository on GitHub.
2. Go to **Actions** -> **Build firmware**.
3. Open the latest successful run, or use a tagged GitHub Release when available.
4. Download and unzip the `atoms3-lite-pdm-rtsp-mic-firmware` artifact.
5. Open [ESPHome Web](https://web.esphome.io/) in Chrome or Edge.
6. Connect the AtomS3 Lite over USB-C and flash `atoms3-lite-pdm-rtsp-mic-factory.bin`.

Use `atoms3-lite-pdm-rtsp-mic-factory.bin` for first-time browser flashing. It is a single merged image containing the bootloader, partition table, `boot_app0`, and firmware at the correct ESP32-S3 offsets. The plain `atoms3-lite-pdm-rtsp-mic-firmware.bin` is app-only at offset `0x10000`, so keep that one for update/OTA workflows rather than blank-device flashing in ESPHome Web.

## Defaults

| Setting | Default | Notes |
| --- | --- | --- |
| Hostname | `atoms3mic` | mDNS URL is `atoms3mic.local` |
| RTSP URL | `rtsp://atoms3mic.local:8554/audio` | L16 mono PCM over RTP |
| RTSP transport | TCP interleaved | UDP is disabled by default |
| Sample rate | 48000 Hz | Supported PDM rates: 16000, 24000, 32000, 48000 Hz |
| Gain | 1.0x | Start conservative, raise only after checking the meter |
| Buffer | 1024 samples | RTP chunking is capped internally at 1024 samples |
| High-pass filter | On | 180 Hz cutoff by default; can be disabled for baseline testing |
| Noise filter | Off | Optional steady-noise suppressor |
| AGC | Off | Optional final gain rider for distant birds |
| CPU | 160 MHz | Good thermal/performance balance |
| Wi-Fi TX power | 19.5 dBm | Adjustable in supported ESP32 radio steps |
| Thermal protection | On, 80 C | Stops streaming and requires acknowledge after a latched trip |
| I2S shift | 0 bits | Fixed for PDM; do not change |

## Audio Path

The live stream path is:

```text
PDM PCM -> stable DC normalize when needed -> 2nd-order high-pass by default -> optional noise bed suppressor -> manual gain -> limiter -> optional AGC -> RTP/Web preview
```

Key points:

- PDM capture uses ESP32 I2S PDM RX mode, 16-bit mono.
- Low-amplitude unsigned PDM blocks are centered with a slow DC tracker before gain.
- The limiter catches sudden peaks before full-scale clipping.
- Short I2S read stalls are bridged with de-clicked concealment blocks, then the next real block is ramped in to avoid a hard edge.
- AGC runs after the cleanup stages and is intentionally capped so it does not turn quiet mic bed noise into constant hiss.

## Web/API Endpoints

Useful URLs:

| Endpoint | Purpose |
| --- | --- |
| `/` | Full Web UI |
| `/streamer` | Browser PCM preview |
| `/api/status` | Network, RTSP, heap, packet, and hardware profile status |
| `/api/audio_status` | Audio settings, live meter, raw PDM link diagnostics |
| `/api/stream_options` | RTSP and browser preview format list |
| `/api/web_audio_pcm` | Binary PCM frames from diagnostics ring |
| `/api/web_audio_l16` | HTTP L16 mono PCM chunk |
| `/api/web_audio_aiff` | AIFF PCM chunk |
| `/api/web_audio_wav` | WAV PCM chunk |
| `/api/web_audio` | JSON PCM frame data |
| `/api/fft` | 128-bin spectrum data from the 256-point FFT diagnostics path |
| `/api/telemetry_history` | CPU load and temperature history |
| `/api/thermal` | Thermal protection status |
| `/api/logs` | Text log buffer |

Example diagnostic check:

```bash
curl -s http://atoms3mic.local/api/audio_status
```

Important fields:

- `i2s_reads_ok` should increase after boot, even before an RTSP client starts PLAY.
- `i2s_link_ok` should be `true` when raw samples are changing.
- `i2s_raw_peak` and `i2s_raw_rms` should rise when you speak or clap near the mic.
- `i2s_raw_min` and `i2s_raw_max` should not stay identical.
- `i2s_hint` gives a quick wiring/capture clue.
- `peak_dbfs` is the most useful tuning meter for practical audio level.

## Troubleshooting

### No Route To Host

This is a network path problem, not a microphone problem. From the same machine that runs VLC/BirdNET-Go:

```bash
ping -c 3 <device-ip>
nc -vz <device-ip> 8554
curl -s http://<device-ip>/api/audio_status
```

If ping or TCP port `8554` fails, check VLANs, client isolation, firewall rules, VPN routing, and whether your computer is on the same subnet.

### Silent Or Flat Audio

Check `/api/audio_status`.

- If `i2s_reads_ok` does not increase, the I2S driver is not capturing.
- If `i2s_link_ok` is `false`, re-check the PDM Unit cable: black GND, red 5V, yellow Data/G2, white Clock/G1.
- If raw values move but `peak_dbfs` is very low, raise gain gradually: `1.0x`, `2.0x`, `4.0x`.
- If levels vary strongly outdoors, enable AGC after the manual gain is in a sane range.

### Whispery, Gritty, Or Pumping Audio

Return to a clean baseline:

- Sample rate `24000` or `16000` Hz if the stream sounds rough at `48000` Hz.
- Gain around `1.0x`.
- AGC off.
- Noise filter off.
- Temporarily disable high-pass if you need the cleanest raw baseline.
- Confirm that `i2sShiftBits` is still `0`.

Then re-enable features one at a time. The noise filter and AGC are useful outdoors, but they can exaggerate a weak or poorly centered signal if enabled too early.
If `/api/audio_status` shows `i2s_unsigned_pcm: true`, the firmware is recentering an all-positive PDM stream before gain; the lower rates above are the cleanest baseline for checking whether the raspiness is coming from the mic path versus later DSP stages.

### Clipping

If the Web UI meter is red or `clip_count` rises quickly:

- Lower gain.
- Disable AGC until the manual gain is reasonable.
- Keep the target around `-20 dBFS` to `-10 dBFS` for loud nearby test sounds.

### Tapping Or Stutter

This firmware does not toggle a microphone-enable GPIO during normal capture. If tapping lines up with meter drops, look at `i2s_fallback_blocks`, `i2s_last_gap_ms`, Wi-Fi RSSI, and audio-pipeline load in the Web UI. Short capture stalls are concealed, but repeated stalls usually point to timing, CPU load, or network pressure.

## Building

```bash
pio run -e m5stack-atoms3-lite
pio run -e m5stack-atoms3-lite -t upload
pio run -e m5stack-atoms3-lite-ota -t upload
pio device monitor -b 115200
```

### GitHub Build Artifacts

The **Build firmware** workflow publishes these files:

| File | Use |
| --- | --- |
| `atoms3-lite-pdm-rtsp-mic-factory.bin` | Single merged image for [ESPHome Web](https://web.esphome.io/) and first-time USB browser flashing |
| `atoms3-lite-pdm-rtsp-mic-firmware.bin` | App-only firmware image at `0x10000` for update/OTA workflows |
| `manifest.json` | ESP Web Tools style manifest for hosts that serve the factory binary next to the manifest |
| `bootloader.bin`, `partitions.bin`, `boot_app0.bin` | Raw components used to build the merged factory image |

Tagged pushes such as `v3.0.0` also attach the same files to a GitHub Release so users can download a `.bin` without browsing workflow artifacts.

Dependencies are declared in `platformio.ini`:

```ini
lib_deps =
    tzapu/WiFiManager @ ^2.0.17
```

The upstream `m5stack/M5Atom` dependency is intentionally not used in this fork because AtomS3 Lite LED handling is done directly through Arduino-ESP32's RGB LED/RMT helper.

## Project Layout

| Path | Purpose |
| --- | --- |
| `src/esp32_rtsp_mic_birdnetgo.ino` | Main firmware, I2S PDM capture, RTSP, RTP, Wi-Fi, OTA, LED, thermal logic |
| `src/WebUI.cpp` | Embedded Web UI and JSON/PCM/WAV API endpoints |
| `src/WebUIAssets.h` | Generated Web UI asset bundle |
| `webui/` | Vite/Preact source for the newer UI bundle |
| `scripts/build_webui.py` | Converts the built web UI into `src/WebUIAssets.h` |
| `platformio.ini` | AtomS3 Lite USB and OTA build environments |
| `partitions_ota.csv` | OTA-capable partition table |
| `docs/DETAILS.md` | Deeper architecture and troubleshooting notes |

## Acknowledgments

- [stedrow/birdnetgo-m5stack-atom-echo-rtsp-mic](https://github.com/stedrow/birdnetgo-m5stack-atom-echo-rtsp-mic), the upstream Atom Echo project this fork came from.
- [Sukecz/birdnetgo-esp32-rtsp-mic](https://github.com/Sukecz/birdnetgo-esp32-rtsp-mic), credited by upstream as the original ESP32 RTSP microphone foundation.
- [BirdNET-Go](https://github.com/tphakala/birdnet-go) and its community.
- M5Stack for the AtomS3 Lite and Unit Mini PDM hardware.
