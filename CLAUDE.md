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
- Added write timeout (100ms) to prevent Core 0 blocking on slow clients
- Reduced debug message spam

## Current Issues

### ❌ No Audio Output
**Status**: Audio is being captured and processed, but client doesn't hear it

**Evidence**:
- Raw captured samples show real values: `[-147, -153, -160]`
- I2S capture on Core 1 is working
- Buffers flow through queue successfully
- RTP packets appear to be sent

**Needs Investigation**:
- Check processed audio values before RTP send
- Verify RTP packet format compatibility
- Test with different RTSP clients (VLC, ffplay, BirdNET-Go)
- Check if sample rate/format matches RTSP DESCRIBE

### ⚠️ Buffer Starvation (Improved but not eliminated)
**Status**: Core 1 runs out of free buffers when client is slow

**Recent Fixes**:
- Added 100ms timeout to `writeAll()` to prevent Core 0 blocking
- Will drop frames if client can't keep up (better than deadlock)

**Remaining Issues**:
- Still gets "No free buffers" messages (reduced spam: only log every 1000)
- Need to monitor if timeout fix fully resolves this

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

## Default Settings
```cpp
Sample Rate:      16000 Hz (optimal for PDM on Atom Echo)
Gain:            1.2×
Buffer Size:     1024 samples (64ms latency)
I2S Shift:       0 bits (PDM doesn't need shifting like I2S)
High-Pass:       ON, 500Hz cutoff
CPU Frequency:   240 MHz
WiFi TX Power:   19.5 dBm
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

## Next Steps

1. **Fix No Audio Issue** (PRIORITY)
   - Add debug output for processed audio samples
   - Verify RTP packet contents
   - Test with different RTSP clients
   - Check RTSP DESCRIBE response matches actual format

2. **Optimize Buffer Flow**
   - Monitor if write timeout fully resolves buffer starvation
   - Consider increasing buffer pool size if needed
   - Add metrics for queue depth over time

3. **Performance Tuning**
   - Test different buffer sizes (512, 1024, 2048)
   - Optimize high-pass filter performance
   - Consider adjustable Core 1 task priority

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
