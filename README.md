# ESP32 RTSP Microphone for BirdNET-Go (M5Stack Atom Echo)

A high-quality RTSP audio streaming server for ESP32, specifically configured for the **M5Stack Atom Echo** with its built-in SPM1423 PDM microphone. Stream live audio to BirdNET-Go or any RTSP-compatible client.

## Hardware Requirements

### M5Stack Atom Echo
- **Board**: M5Stack Atom Echo (ESP32-PICO-D4)
- **Microphone**: SPM1423 PDM MEMS microphone (built-in)
- **Speaker**: Built-in I2S speaker (not used in this application)
- **Button**: Single programmable button (built-in)
- **LED**: RGB LED (built-in, WS2812C)

### Technical Specifications
- **Processor**: ESP32-PICO-D4 @ 240MHz Dual Core (configurable 80-240MHz)
- **WiFi**: 2.4GHz 802.11 b/g/n
- **Audio Format**: 16-bit PCM, 16kHz default (configurable 8-48kHz)
- **Streaming**: RTSP over TCP/IP on port 8554

## Pin Configuration

The following I2S pins are used for the Atom Echo's PDM microphone:

```cpp
I2S_BCLK_PIN    = 19  // Bit Clock
I2S_LRCLK_PIN   = 33  // Left-Right Clock / Word Select
I2S_DATA_IN_PIN = 23  // Microphone Data Input (PDM)
I2S_DATA_OUT_PIN = 22 // Speaker Data Output (not used)
```

## LED Status Indicators

The built-in RGB LED provides visual feedback:

| Color | Meaning |
|-------|---------|
| 🟡 **Yellow** | System starting up |
| 🟢 **Green** | Ready - RTSP server running, waiting for connection |
| 🟣 **Purple** | Actively streaming audio to RTSP client |
| 🔴 **Red** | Thermal protection activated - system latched |

## Features

### Core Features
- **RTSP Streaming**: Industry-standard RTSP/RTP streaming on port 8554
- **Web Interface**: Full-featured web UI for configuration and monitoring
- **PDM Support**: Native support for PDM MEMS microphones
- **Auto Recovery**: Automatic recovery from network/streaming issues
- **OTA Updates**: Over-the-air firmware updates
- **Persistent Settings**: Configuration saved to flash memory

### Audio Processing
- **High-Pass Filter**: Configurable 2nd-order Butterworth filter (default 500Hz) to remove low-frequency rumble
- **Digital Gain**: Software gain control (0.1× to 100×)
- **I2S Bit Shifting**: Fine-tune audio levels (0-24 bits)
- **Clipping Detection**: Real-time audio level monitoring and clipping alerts
- **Multiple Sample Rates**: 8kHz to 96kHz (16kHz recommended for Atom Echo)

### Performance & Reliability
- **Configurable CPU Frequency**: 80MHz, 120MHz, 160MHz, or 240MHz (default 240MHz)
- **WiFi Power Control**: Adjustable TX power to reduce RF noise
- **Buffer Profiles**: Multiple latency/stability profiles (256 to 8192 samples)
- **Thermal Protection**: Automatic shutdown on overheating (configurable 30-95°C)
- **Scheduled Resets**: Optional periodic reboots for long-term stability
- **Memory Monitoring**: Heap tracking and diagnostics

## Getting Started

### 1. Hardware Setup
1. Connect M5Stack Atom Echo via USB-C
2. No additional wiring required - microphone is built-in

### 2. Flash Firmware
Using PlatformIO:
```bash
pio run --target upload
```

Using PlatformIO IDE:
- Open project in VSCode/PlatformIO
- Click "Upload" button

### 3. Initial Configuration

#### WiFi Setup
On first boot, the device creates a WiFi access point:
- **SSID**: `ESP32-RTSP-Mic-AP`
- **Portal**: Opens automatically when you connect
- Select your WiFi network and enter credentials
- Device will reboot and connect to your network

#### Finding Your Device
Check the Serial Monitor (115200 baud) for:
```
WiFi connected: 192.168.1.XXX
RTSP URL: rtsp://192.168.1.XXX:8554/audio
Web UI: http://192.168.1.XXX/
```

The LED will turn **green** when ready.

## Usage

### RTSP Stream
Connect your client to:
```
rtsp://[device-ip]:8554/audio
```

**Audio Format:**
- Codec: L16 (16-bit PCM)
- Sample Rate: 16000 Hz (default, configurable)
- Channels: Mono
- Transport: RTP over RTSP/TCP

### Web Interface

Access the web UI at `http://[device-ip]/` for:

#### Status Monitoring
- IP address and WiFi signal strength
- WiFi TX power
- Free heap memory
- System uptime
- RTSP connection status
- Real-time packet rate
- Signal level and clipping detection

#### Audio Configuration
- **Sample Rate**: 8000-96000 Hz (16000 recommended)
- **Gain**: 0.1× to 100× amplification
- **Buffer Size**: 256-8192 samples (latency vs stability)
- **High-Pass Filter**: Enable/disable, cutoff 10-10000 Hz
- **I2S Shift**: 0-24 bits for level adjustment

#### Advanced Settings
- **Auto Recovery**: Enable/disable automatic restart on issues
- **Threshold Mode**: Auto-calculated or manual packet rate threshold
- **WiFi TX Power**: -1.0 to 19.5 dBm
- **CPU Frequency**: 80, 120, 160, or 240 MHz (240 MHz recommended for best performance)
- **Thermal Protection**: Enable/disable, shutdown limit 30-95°C
- **Scheduled Reset**: Optional periodic reboots

## Audio Tuning Guide

### Getting Good Audio Levels

1. **Check Signal Level** in Web UI
   - Target: 60-80% (about -4 to -2 dBFS)
   - If clipping: Increase I2S Shift or reduce Gain
   - If too quiet: Decrease I2S Shift or increase Gain

2. **Recommended Starting Settings** (Atom Echo)
   - Sample Rate: 16000 Hz
   - Gain: 1.2×
   - I2S Shift: 11 bits
   - High-Pass: ON, 500 Hz
   - Buffer: 1024 samples

3. **Reduce Background Noise**
   - Enable High-Pass Filter (500-600 Hz)
   - Reduce WiFi TX power if near microphone
   - Use lower CPU frequency if possible

### Buffer Size Profiles

| Size | Latency | Stability | CPU | Use Case |
|------|---------|-----------|-----|----------|
| 256 | 16ms | Low | High | Ultra-low latency, may drop packets |
| 512 | 32ms | Medium | Medium | Balanced for good WiFi |
| 1024 | 64ms | High | Low | **Recommended** - stable streaming |
| 2048+ | 128ms+ | Very High | Very Low | Poor WiFi, maximum stability |

## BirdNET-Go Integration

Configure BirdNET-Go to use your RTSP stream:

1. Set audio source to: `rtsp://[device-ip]:8554/audio`
2. Recommended settings:
   - Sample rate: 16000 Hz
   - Gain: Start with 1.2×, adjust based on detection performance
   - Enable high-pass filter to reduce wind/traffic noise

## Troubleshooting

### LED is Yellow (Stuck in Startup)
- Check Serial Monitor for errors
- WiFi credentials may be incorrect
- Reset WiFi: Connect to `ESP32-RTSP-Mic-AP` and reconfigure

### LED is Red (Thermal Protection)
- Device overheated and shut down streaming
- Allow to cool down
- Clear thermal latch via Web UI
- Consider lowering CPU frequency or improving ventilation

### No Audio / Low Volume
1. Check signal level in Web UI
2. Increase Gain (try 2.0-5.0×)
3. Decrease I2S Shift (try 10 or 9)
4. Disable High-Pass filter temporarily to test

### Audio Clipping / Distortion
1. Decrease Gain
2. Increase I2S Shift (try 12 or 13)
3. Check "Signal Level" - should be 60-80%

### Stream Drops / Connection Issues
1. Check WiFi signal strength (RSSI should be > -70 dBm)
2. Increase buffer size to 2048 or 4096
3. Enable Auto Recovery
4. Reduce WiFi TX power if causing interference
5. Try lower sample rate (8000 or 12000 Hz)

### Web UI Shows "Waiting..." for Client
- Normal when no RTSP client is connected
- Try connecting with VLC or your RTSP client

## Technical Details

### PDM Microphone Configuration

The M5Stack Atom Echo uses a PDM (Pulse Density Modulation) MEMS microphone, which requires specific I2S settings:

```cpp
i2s_config.mode = I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM
i2s_config.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT
i2s_config.channel_format = I2S_CHANNEL_FMT_ALL_RIGHT
```

### Memory Requirements
- Minimum free heap: ~80KB for stable operation
- Buffer allocation: Based on configured buffer size
- Web UI overhead: ~20KB when serving pages

### Network Requirements
- Stable WiFi connection
- Minimum bandwidth: ~256 Kbps for 16kHz stream
- Latency: <100ms recommended for real-time use

## Development

### Building from Source
```bash
# Clone repository
git clone [repository-url]
cd m5stack-atomecho

# Install dependencies
pio lib install

# Build
pio run

# Upload
pio run --target upload

# Monitor
pio device monitor
```

### Dependencies
- `tzapu/WiFiManager @ ^2.0.17` - WiFi configuration portal
- `m5stack/M5Atom @ ^0.1.3` - M5Stack Atom library
- `fastled/FastLED @ ^3.10.3` - LED control

### Default Credentials
- **OTA Password**: Not set (define `OTA_PASSWORD` to enable)

## Project Origin

This project is adapted from an ESP32 RTSP microphone implementation originally designed for M5Stack STAMP S3 with ICS-43434 I2S microphone. It has been modified for the M5Stack Atom Echo's PDM microphone and includes the following changes:

- PDM microphone support (I2S_MODE_PDM)
- Updated pin configuration for Atom Echo
- 16-bit sample format (vs 32-bit I2S)
- Optimized default settings for PDM operation
- M5Atom library integration for button and LED
- Visual feedback via RGB LED

## License

[Add your license information here]

## Contributing

Contributions are welcome! Please feel free to submit issues or pull requests.

## Support

For issues specific to:
- **M5Stack Atom Echo hardware**: [M5Stack Documentation](https://docs.m5stack.com/en/core/atom_echo)
- **BirdNET-Go**: [BirdNET-Go Repository](https://github.com/tphakala/birdnet-go)
- **This firmware**: [Open an issue](https://github.com/your-repo/issues)

## Acknowledgments

- Original RTSP microphone implementation
- M5Stack for the Atom Echo hardware
- BirdNET-Go community
