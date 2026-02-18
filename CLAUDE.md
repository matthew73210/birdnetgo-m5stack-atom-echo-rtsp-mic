# M5Stack Atom Echo - RTSP Microphone Project

## Project Overview
ESP32-based RTSP audio streaming server for M5Stack Atom Echo with SPM1423 PDM microphone. Streams live audio to BirdNET-Go or any RTSP-compatible client.

## Hardware
- **Board**: M5Stack Atom Echo (ESP32-PICO-D4)
- **CPU**: 240 MHz Dual Core
- **Microphone**: SPM1423 PDM MEMS microphone (built-in)
- **Audio Format**: 16-bit PCM, 16kHz (configurable 8-48kHz)
- **Streaming**: RTSP/RTP over TCP on port 8554

## Pin Configuration (PDM Microphone)
```cpp
I2S_BCLK_PIN    = 19  // Bit Clock
I2S_LRCLK_PIN   = 33  // Word Select / Left-Right Clock
I2S_DATA_IN_PIN = 23  // Microphone Data Input (PDM)
I2S_DATA_OUT_PIN = 22 // Speaker Data Output (not used)
```

## I2S Configuration for PDM
```cpp
i2s_config.mode = I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM
i2s_config.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT
i2s_config.channel_format = I2S_CHANNEL_FMT_ALL_RIGHT
```

## Current Architecture: Dual-Core with Buffer Pool

### Design
- **Core 1 (PRO_CPU)**: Audio capture task (priority 10)
  - Reads from I2S PDM microphone
  - Writes to FreeRTOS queue
- **Core 0 (APP_CPU)**: Audio processing & streaming + Web UI
  - Reads from FreeRTOS queue
  - Applies gain, high-pass filter
  - Sends RTP packets to RTSP client
  - Handles web interface

### Buffer Pool Implementation
```
Startup: Allocate 8 buffers → all in freeQueue

Core 1 Flow:
1. Get buffer from freeQueue
2. Fill with I2S audio data (16-bit samples)
3. Send to fullQueue
4. Repeat

Core 0 Flow:
1. Get buffer from fullQueue (50ms timeout)
2. Process audio (gain, HPF, clipping detection)
3. Send RTP packet (with 100ms write timeout)
4. Return buffer to freeQueue
5. Repeat
```

### Why FreeRTOS Queues?
Initial attempt used a custom lock-free ring buffer, but encountered **cache coherency issues** between cores. FreeRTOS queues properly handle inter-core synchronization on ESP32.

## Recent Development History

### Session 1: Initial PDM Configuration
- Adapted code from I2S microphone to PDM microphone
- Changed from 32-bit to 16-bit samples
- Fixed "squirrel/chipmunk" audio issue (buffer size mismatch)
- Audio worked but was choppy

### Session 2: First Dual-Core Attempt
- Implemented lock-free ring buffer for inter-core communication
- **Issue**: Cache coherency problems between cores
  - Core 1 saw buffer as full
  - Core 0 saw buffer as empty simultaneously
  - Audio was heard but system unstable

### Session 3: FreeRTOS Queue Migration
- Replaced ring buffer with FreeRTOS queues
- Implemented proper buffer pool (8 buffers)
- Added queue-based ownership transfer
- **Issue**: Buffers not being returned properly (memory leaks)

### Session 4: Buffer Management Fixes
- Fixed queue cleanup (use `vQueueDelete()` instead of `xQueueReset()`)
- Added proper start/stop cycle for multiple streaming sessions
- Added write timeout (100ms → 30ms) to prevent Core 0 blocking
- Reduced debug message spam

### Session 5: Final Production Release ✅
**Major Discovery**: i2sShiftBits saved to flash was causing all audio to be zeros
- **Symptom**: Audio captured successfully but all processed samples were `[0, 0, 0]`
- **Debug Process**:
  - Disabled HPF → still zeros
  - Added step-by-step debug → found `captureData[0]=180` became `sample=0.0`
  - Root cause: `i2sShiftBits=11` (from old flash settings) → `180 >> 11 = 0`
- **Fixes Applied**:
  1. Hardcoded i2sShiftBits to 0 (ignores flash)
  2. Made Web UI read-only for i2sShiftBits
  3. Removed API endpoint for changing i2sShiftBits
  4. Optimized timeouts (30ms write, 30ms queue)
  5. Cleaned up debug output
  6. Updated branding to "M5Stack Atom Echo"

**Result**: Production-ready system with clean audio, stable streaming, responsive Web UI

## ✅ System Status: PRODUCTION READY

### Audio Streaming: Fully Functional ✅
- ✅ Constant audio without dropouts
- ✅ Clean, non-choppy playback
- ✅ Good audio quality with proper gain/filtering
- ✅ Queue health: `free: 6-7, full: 0-1` (optimal)
- ✅ Web UI responsive during streaming
- ✅ Handles client connects/disconnects gracefully
- ✅ Stable long-term operation

### Critical Fix: i2sShiftBits (RESOLVED)
**Issue**: Old I2S settings (i2sShiftBits=11) were saved to flash from earlier debugging
**Impact**: All audio samples shifted to zeros → no audio (e.g., 180 >> 11 = 0)
**Root Cause**: Code allowed users to change i2sShiftBits via Web UI, wrong value got saved
**Final Solution**:
- **Hardcoded to 0** in code (always loads as 0, ignores flash)
- **Web UI**: Changed to read-only display showing "0 bits (fixed for PDM)"
- **API endpoint**: Removed handler for changing i2sShiftBits
- **Result**: Users cannot break audio by changing this setting

**CRITICAL FOR PDM**: `i2sShiftBits` MUST ALWAYS be 0!

### Web UI Improvements (Latest)
- **Branding**: Updated to "M5Stack Atom Echo" throughout
- **i2sShiftBits**: Read-only display, cannot be modified
- **Debug output**: Cleaned up, minimal logging (only errors/important events)
- **Help tooltips**: Comprehensive explanations already present
- **Title**: "M5Stack Atom Echo - RTSP Microphone"

### Performance Optimizations
- **Write timeout**: 30ms (prevents WiFi blocking, drops frames if client slow)
- **Queue timeout**: 30ms (balances audio responsiveness with web UI)
- **Audio task priority**: 10 (elevated but allows web UI to run)
- **Buffer pool**: 8 buffers × 1024 samples (16KB total, optimal)

## Audio Processing Pipeline

```
I2S PDM Mic (Core 1)
    ↓
16-bit samples captured
    ↓
fullQueue (FreeRTOS)
    ↓
Core 0 Processing:
- Read from queue
- Apply i2sShiftBits (default: 0 for PDM)
- High-pass filter (Butterworth 2nd order, 500Hz default)
- Apply gain (default: 1.2×)
- Clipping detection
- Write to output buffer
    ↓
RTP Packet Formation:
- Byte swap for network order (big-endian L16)
- Add RTP header (sequence, timestamp, SSRC)
- Add RTSP interleaved header
    ↓
WiFi Client (with 100ms timeout)
```

## Verified Working Configuration
```cpp
Sample Rate:      16000 Hz (optimal for PDM on Atom Echo)
Gain:            1.0× - 1.2× (adjust for your environment)
Buffer Size:     1024 samples (64ms latency, good balance)
I2S Shift:       0 bits ⚠️ CRITICAL: Must be 0 for PDM!
High-Pass:       ON, 500Hz cutoff (reduces wind/traffic noise)
CPU Frequency:   240 MHz
WiFi TX Power:   19.5 dBm

Queue timeout:   30ms (audio responsiveness vs web UI balance)
Write timeout:   30ms (prevents WiFi blocking)
Audio priority:  10 (elevated but allows web UI to run)
Buffer pool:     8 buffers (optimal for smooth streaming)
```

## LED Status Indicators
- 🟡 **Yellow**: System starting up
- 🟢 **Green**: Ready - RTSP server running, waiting for connection
- 🟣 **Purple**: Actively streaming audio to RTSP client
- 🔴 **Red**: Thermal protection activated

## Debug Output Guide

### Healthy Operation
```
[Core1] Captured 100 buffers, free: 6-7, full: 1-2
[Core0] Streamed 100 buffers, buf=0x3ffc48bc, samples: 1024, first 3 values: [-147, -153, -160], free: 6-7, full: 0-1
[Core0] Returned 100 buffers to free queue
```

### Problem Indicators
```
[Core1] No free buffers! Count: 1000      // Core 0 not returning buffers fast enough
[Core0] Queue empty 100 times             // Core 1 not capturing fast enough
[Core0] Write timeout - client too slow   // WiFi client can't keep up
```

## Lessons Learned

### Critical for PDM Microphones
1. **i2sShiftBits MUST be 0** - This is non-negotiable for PDM
2. **Don't allow user configuration** of hardware-specific settings
3. **Hardcode defaults** instead of loading from flash for critical settings
4. **Debug at every step**: raw samples → processed samples → RTP packets

### Dual-Core ESP32 Best Practices
1. **Use FreeRTOS queues** not custom ring buffers (cache coherency!)
2. **Buffer pool pattern** works well for producer/consumer
3. **Balance timeouts**: Too short starves web UI, too long causes dropouts
4. **Priority 10** is good for audio capture (not too aggressive)
5. **30ms write timeout** prevents WiFi blocking without excessive frame drops

### Debugging Methodology
When audio isn't working:
1. Check raw I2S samples first (are they real values?)
2. Check processed samples (did processing zero them out?)
3. Check RTP packets (are they being sent?)
4. Add step-by-step debug output in the processing pipeline
5. Don't assume defaults - check what's actually in flash/preferences

## Optional Future Enhancements

### Web UI
1. **"Recommended for BirdNET-Go" badges** next to optimal settings
2. **Queue health display** showing buffer pool status in real-time
3. **Preset buttons**: "BirdNET-Go Defaults", "Indoor", "Outdoor"
4. **Simplified mode** with just 3-4 key settings for beginners

### Features
1. **mDNS discovery** for easier setup
2. **Multi-client support** (currently single RTSP client)
3. **Audio statistics** page (peak levels over time, clipping events, etc.)
4. **Remote logging** for debugging deployed units

### Performance
1. **Adaptive buffering** based on WiFi quality
2. **Quality metrics** (jitter, packet loss, etc.)
3. **Auto-tuning** of gain based on ambient noise levels

**Note**: Current system is fully functional - these are nice-to-haves, not requirements!

## Important Notes

### PDM vs I2S Differences
- **PDM**: Outputs 16-bit samples directly, no bit shifting needed (i2sShiftBits = 0)
- **I2S**: Typically needs bit shifting (i2sShiftBits = 11-13)
- PDM is simpler but requires ESP32 hardware PDM support

### Audio Task Lifecycle
- Task only runs during active streaming (RTSP PLAY → TEARDOWN)
- Automatically starts on PLAY command
- Automatically stops on TEARDOWN or disconnect
- Fresh buffers allocated each streaming session

### Web UI Considerations
- Runs on Core 0 along with audio streaming
- 50ms queue timeout in streamAudio allows web UI to run
- If web UI becomes unresponsive, may need to move to separate task or adjust priorities

## Build & Deploy
```bash
# Build
pio run

# Upload
pio run --target upload

# Monitor
pio device monitor -b 115200
```

## Testing Audio
```bash
# VLC
vlc rtsp://[device-ip]:8554/audio

# ffplay
ffplay -rtsp_transport tcp rtsp://[device-ip]:8554/audio

# BirdNET-Go
# Set audio source to: rtsp://[device-ip]:8554/audio
```

## Quick Reference for Future Sessions

### If Audio Stops Working
1. **First check**: Serial monitor for `i2sShiftBits` value - MUST be 0
2. **Web UI**: Verify "I2S Shift" shows "0 bits (fixed for PDM)"
3. **Debug mode**: Uncomment debug lines in streamAudio to see sample values
4. **Reset settings**: Use "Defaults" button in Web UI if values are corrupted

### If System is Unstable
1. **Check queue health**: Should show `free: 6-7, full: 0-1`
2. **WiFi signal**: RSSI should be > -70 dBm
3. **Free heap**: Should stay above ~80KB
4. **Write timeouts**: Look for "Write timeout" messages (means client is slow)

### Key Files
- **Main code**: `src/esp32_rtsp_mic_birdnetgo.ino`
- **Web UI**: `src/WebUI.cpp` (long string literals, use sed for edits)
- **Configuration**: Saved to flash in `audioPrefs` namespace
- **This document**: `CLAUDE.md`

### Architecture Quick Summary
```
┌─────────────────────────────────────────┐
│ Core 1 (PRO_CPU) - Priority 10          │
│ ┌────────────────────────────┐          │
│ │ audioCaptureTask()         │          │
│ │ - i2s_read() 16-bit PDM    │          │
│ │ - Write to fullQueue       │          │
│ └────────────────────────────┘          │
└─────────────────────────────────────────┘
              ↓ FreeRTOS Queue
┌─────────────────────────────────────────┐
│ Core 0 (APP_CPU)                        │
│ ┌────────────────────────────┐          │
│ │ loop() / streamAudio()     │          │
│ │ - Read from fullQueue      │          │
│ │ - Process (gain, HPF)      │          │
│ │ - Send RTP packet          │          │
│ │ - Return to freeQueue      │          │
│ └────────────────────────────┘          │
│ ┌────────────────────────────┐          │
│ │ webui_handleClient()       │          │
│ │ - HTTP server for config   │          │
│ └────────────────────────────┘          │
└─────────────────────────────────────────┘
```

### Common Issues & Solutions
| Issue | Cause | Solution |
|-------|-------|----------|
| No audio, samples are zeros | i2sShiftBits ≠ 0 | Hardcoded to 0 in latest version |
| Choppy audio | Buffer starvation | Check queue health, reduce write timeout |
| Web UI unresponsive | Queue timeout too short | Current 30ms is balanced |
| Client disconnects | Write timeout / slow WiFi | 30ms timeout will drop frames vs blocking |
| Cache coherency errors | Custom ring buffer | Use FreeRTOS queues (current version) |

### Production Checklist
- ✅ i2sShiftBits hardcoded to 0
- ✅ Sample rate: 16000 Hz
- ✅ Buffer size: 1024 samples
- ✅ High-pass filter: ON, 500Hz
- ✅ Gain: 1.0-1.2× (adjust per environment)
- ✅ CPU: 240 MHz
- ✅ Queue timeout: 30ms
- ✅ Write timeout: 30ms
- ✅ Debug output: Minimal (production mode)

## Dependencies
```ini
lib_deps =
    tzapu/WiFiManager @ ^2.0.17
    m5stack/M5Atom @ ^0.1.3
    fastled/FastLED @ ^3.10.3
```

## Known Working Configuration (from earlier sessions)
When we temporarily disabled dual-core for debugging:
- Direct I2S read on Core 0
- Audio was "squirrely" but audible
- Fixed by using correct buffer size (16-bit read into 16-bit buffer)

This proves the RTP sending and basic audio pipeline works. The issue with current dual-core implementation is likely subtle timing or data flow problem.
