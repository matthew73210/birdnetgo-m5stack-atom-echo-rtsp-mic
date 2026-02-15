#include <WiFi.h>
#include <WiFiManager.h>
#include "driver/i2s.h"
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <math.h>
#include <M5Atom.h>
#include "WebUI.h"

// ================== DUAL-CORE AUDIO ARCHITECTURE ==================
// FreeRTOS queue with buffer pool for inter-core communication
// Core 1: I2S capture (producer) -> Queue -> Core 0: Processing + Streaming (consumer)
// PDM microphone outputs 16-bit samples directly

#define AUDIO_QUEUE_LENGTH 8  // Number of audio buffers in pool

struct AudioBuffer {
    int16_t* data;
    uint16_t sampleCount;
};

// Buffer pool - pre-allocated buffers passed between cores
AudioBuffer audioBufferPool[AUDIO_QUEUE_LENGTH];
QueueHandle_t audioFreeQueue = NULL;  // Buffers available for capture
QueueHandle_t audioFullQueue = NULL;  // Buffers ready for processing
TaskHandle_t audioCaptureTaskHandle = NULL;
volatile bool audioTaskRunning = false;
uint16_t audioBufferSize = 0;  // Samples per buffer

// ================== SETTINGS (ESP32 RTSP Mic for BirdNET-Go) ==================
#define FW_VERSION "2.0.0-dual-core"
// Expose FW version as a global C string for WebUI/API
const char* FW_VERSION_STR = FW_VERSION;

// OTA password (optional):
// - For production, set a strong password to protect OTA updates.
// - You can leave it undefined to disable password protection.
// - Example placeholder (edit as needed):
// #define OTA_PASSWORD "1234"  // Optional: change or leave undefined

// -- DEFAULT PARAMETERS (configurable via Web UI / API)
#define DEFAULT_SAMPLE_RATE 16000  // M5Stack Atom Echo: 16kHz for PDM microphone
#define DEFAULT_GAIN_FACTOR 1.2f
#define DEFAULT_BUFFER_SIZE 1024   // Stable streaming profile by default
#define DEFAULT_WIFI_TX_DBM 19.5f  // Default WiFi TX power in dBm
// High-pass filter defaults (to remove low-frequency rumble)
#define DEFAULT_HPF_ENABLED true
#define DEFAULT_HPF_CUTOFF_HZ 500

// Thermal protection defaults
#define DEFAULT_OVERHEAT_PROTECTION true
#define DEFAULT_OVERHEAT_LIMIT_C 80
#define OVERHEAT_MIN_LIMIT_C 30
#define OVERHEAT_MAX_LIMIT_C 95
#define OVERHEAT_LIMIT_STEP_C 5

// -- Pins (M5Stack Atom Echo with SPM1423 PDM Microphone)
#define I2S_BCLK_PIN    19  // Bit Clock
#define I2S_LRCLK_PIN   33  // Word Select / Left-Right Clock
#define I2S_DATA_IN_PIN 23  // Microphone Data In
#define I2S_DATA_OUT_PIN 22 // Speaker Data Out (not used for mic-only)

// -- Servers
WiFiServer rtspServer(8554);
WiFiClient rtspClient;

// -- RTSP Streaming
String rtspSessionId = "";
volatile bool isStreaming = false;
uint16_t rtpSequence = 0;
uint32_t rtpTimestamp = 0;
uint32_t rtpSSRC = 0x43215678;
unsigned long lastRTSPActivity = 0;

// -- Buffers
uint8_t rtspParseBuffer[1024];
int rtspParseBufferPos = 0;
//
int16_t* i2s_16bit_buffer = nullptr;  // 16-bit samples for PDM microphone

// -- Global state
unsigned long audioPacketsSent = 0;
unsigned long lastStatsReset = 0;
bool rtspServerEnabled = true;

// -- Audio parameters (runtime configurable)
uint32_t currentSampleRate = DEFAULT_SAMPLE_RATE;
float currentGainFactor = DEFAULT_GAIN_FACTOR;
uint16_t currentBufferSize = DEFAULT_BUFFER_SIZE;
// PDM microphones (like SPM1423 on Atom Echo) output decimated samples
// The hardware PDM decimation produces samples already close to correct range
// Typical range: 0-3 for PDM vs 11-13 for I2S microphones
uint8_t i2sShiftBits = 0;  // Default for PDM microphone on M5Stack Atom Echo

// -- Audio metering / clipping diagnostics
uint16_t lastPeakAbs16 = 0;       // last block peak absolute value (0..32767)
uint32_t audioClipCount = 0;      // total blocks where clipping occurred
bool audioClippedLastBlock = false; // clipping occurred in last processed block
uint16_t peakHoldAbs16 = 0;       // peak hold (recent window)
unsigned long peakHoldUntilMs = 0; // when to clear hold

// -- High-pass filter (biquad) to cut low-frequency rumble
struct Biquad {
    float b0{1.0f}, b1{0.0f}, b2{0.0f}, a1{0.0f}, a2{0.0f};
    float x1{0.0f}, x2{0.0f}, y1{0.0f}, y2{0.0f};
    inline float process(float x) {
        float y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
        x2 = x1; x1 = x; y2 = y1; y1 = y;
        return y;
    }
    inline void reset() { x1 = x2 = y1 = y2 = 0.0f; }
};
bool highpassEnabled = DEFAULT_HPF_ENABLED;
uint16_t highpassCutoffHz = DEFAULT_HPF_CUTOFF_HZ;
Biquad hpf;
uint32_t hpfConfigSampleRate = 0;
uint16_t hpfConfigCutoff = 0;

// -- Preferences for persistent settings
Preferences audioPrefs;

// -- Diagnostics, auto-recovery and temperature monitoring
unsigned long lastMemoryCheck = 0;
unsigned long lastPerformanceCheck = 0;
unsigned long lastWiFiCheck = 0;
unsigned long lastTempCheck = 0;
uint32_t minFreeHeap = 0xFFFFFFFF;
uint32_t maxPacketRate = 0;
uint32_t minPacketRate = 0xFFFFFFFF;
bool autoRecoveryEnabled = true;
bool autoThresholdEnabled = true; // auto compute minAcceptableRate from sample rate and buffer size
// Deferred reboot scheduling (to restart safely outside HTTP context)
volatile bool scheduledFactoryReset = false;
volatile unsigned long scheduledRebootAt = 0;
unsigned long bootTime = 0;
unsigned long lastI2SReset = 0;
float maxTemperature = 0.0f;
float lastTemperatureC = 0.0f;
bool lastTemperatureValid = false;
bool overheatProtectionEnabled = DEFAULT_OVERHEAT_PROTECTION;
float overheatShutdownC = (float)DEFAULT_OVERHEAT_LIMIT_C;
bool overheatLockoutActive = false;
float overheatTripTemp = 0.0f;
unsigned long overheatTriggeredAt = 0;
String overheatLastReason = "";
String overheatLastTimestamp = "";
bool overheatSensorFault = false;
bool overheatLatched = false;

// -- Scheduled reset
bool scheduledResetEnabled = false;
uint32_t resetIntervalHours = 24; // Default 24 hours

// -- Configurable thresholds
uint32_t minAcceptableRate = 50;        // Minimum acceptable packet rate (restart below this)
uint32_t performanceCheckInterval = 15; // Check interval in minutes
uint8_t cpuFrequencyMhz = 240;          // CPU frequency (default 240 MHz for ESP32-PICO-D4)

// -- WiFi TX power (configurable)
float wifiTxPowerDbm = DEFAULT_WIFI_TX_DBM;
wifi_power_t currentWifiPowerLevel = WIFI_POWER_19_5dBm;

// -- RTSP connect/PLAY statistics
unsigned long lastRtspClientConnectMs = 0;
unsigned long lastRtspPlayMs = 0;
uint32_t rtspConnectCount = 0;
uint32_t rtspPlayCount = 0;

// ===============================================

// Helper: convert WiFi power enum to dBm (for logs)
float wifiPowerLevelToDbm(wifi_power_t lvl) {
    switch (lvl) {
        case WIFI_POWER_19_5dBm:    return 19.5f;
        case WIFI_POWER_19dBm:      return 19.0f;
        case WIFI_POWER_18_5dBm:    return 18.5f;
        case WIFI_POWER_17dBm:      return 17.0f;
        case WIFI_POWER_15dBm:      return 15.0f;
        case WIFI_POWER_13dBm:      return 13.0f;
        case WIFI_POWER_11dBm:      return 11.0f;
        case WIFI_POWER_8_5dBm:     return 8.5f;
        case WIFI_POWER_7dBm:       return 7.0f;
        case WIFI_POWER_5dBm:       return 5.0f;
        case WIFI_POWER_2dBm:       return 2.0f;
        case WIFI_POWER_MINUS_1dBm: return -1.0f;
        default:                    return 19.5f;
    }
}

// Helper: pick the highest power level not exceeding requested dBm
static wifi_power_t pickWifiPowerLevel(float dbm) {
    if (dbm <= -1.0f) return WIFI_POWER_MINUS_1dBm;
    if (dbm <= 2.0f)  return WIFI_POWER_2dBm;
    if (dbm <= 5.0f)  return WIFI_POWER_5dBm;
    if (dbm <= 7.0f)  return WIFI_POWER_7dBm;
    if (dbm <= 8.5f)  return WIFI_POWER_8_5dBm;
    if (dbm <= 11.0f) return WIFI_POWER_11dBm;
    if (dbm <= 13.0f) return WIFI_POWER_13dBm;
    if (dbm <= 15.0f) return WIFI_POWER_15dBm;
    if (dbm <= 17.0f) return WIFI_POWER_17dBm;
    if (dbm <= 18.5f) return WIFI_POWER_18_5dBm;
    if (dbm <= 19.0f) return WIFI_POWER_19dBm;
    return WIFI_POWER_19_5dBm;
}

// Apply WiFi TX power
// Logs only when changed; can be muted with log=false
void applyWifiTxPower(bool log = true) {
    wifi_power_t desired = pickWifiPowerLevel(wifiTxPowerDbm);
    if (desired != currentWifiPowerLevel) {
        WiFi.setTxPower(desired);
        currentWifiPowerLevel = desired;
        if (log) {
            simplePrintln("WiFi TX power set to " + String(wifiPowerLevelToDbm(currentWifiPowerLevel), 1) + " dBm");
        }
    }
}

// Recompute HPF coefficients (2nd-order Butterworth high-pass)
void updateHighpassCoeffs() {
    if (!highpassEnabled) {
        hpf.reset();
        hpfConfigSampleRate = currentSampleRate;
        hpfConfigCutoff = highpassCutoffHz;
        return;
    }
    float fs = (float)currentSampleRate;
    float fc = (float)highpassCutoffHz;
    if (fc < 10.0f) fc = 10.0f;
    if (fc > fs * 0.45f) fc = fs * 0.45f; // keep reasonable

    const float pi = 3.14159265358979323846f;
    float w0 = 2.0f * pi * (fc / fs);
    float cosw0 = cosf(w0);
    float sinw0 = sinf(w0);
    float Q = 0.70710678f; // Butterworth-like
    float alpha = sinw0 / (2.0f * Q);

    float b0 =  (1.0f + cosw0) * 0.5f;
    float b1 = -(1.0f + cosw0);
    float b2 =  (1.0f + cosw0) * 0.5f;
    float a0 =  1.0f + alpha;
    float a1 = -2.0f * cosw0;
    float a2 =  1.0f - alpha;

    hpf.b0 = b0 / a0;
    hpf.b1 = b1 / a0;
    hpf.b2 = b2 / a0;
    hpf.a1 = a1 / a0;
    hpf.a2 = a2 / a0;
    hpf.reset();

    hpfConfigSampleRate = currentSampleRate;
    hpfConfigCutoff = (uint16_t)fc;
}

// Uptime -> "Xd Yh Zm Ts"
String formatUptime(unsigned long seconds) {
    unsigned long days = seconds / 86400;
    seconds %= 86400;
    unsigned long hours = seconds / 3600;
    seconds %= 3600;
    unsigned long minutes = seconds / 60;
    seconds %= 60;

    String result = "";
    if (days > 0) result += String(days) + "d ";
    if (hours > 0 || days > 0) result += String(hours) + "h ";
    if (minutes > 0 || hours > 0 || days > 0) result += String(minutes) + "m ";
    result += String(seconds) + "s";
    return result;
}

// Format "X ago" for events based on millis()
String formatSince(unsigned long eventMs) {
    if (eventMs == 0) return String("never");
    unsigned long seconds = (millis() - eventMs) / 1000;
    return formatUptime(seconds) + " ago";
}

static bool isTemperatureValid(float temp) {
    if (isnan(temp) || isinf(temp)) return false;
    if (temp < -20.0f || temp > 130.0f) return false;
    return true;
}

// Format current local time, fallback to uptime when no RTC/NTP time available
static void persistOverheatNote() {
    audioPrefs.begin("audio", false);
    audioPrefs.putString("ohReason", overheatLastReason);
    audioPrefs.putString("ohStamp", overheatLastTimestamp);
    audioPrefs.putFloat("ohTripC", overheatTripTemp);
    audioPrefs.putBool("ohLatched", overheatLatched);
    audioPrefs.end();
}

void recordOverheatTrip(float temp) {
    unsigned long uptimeSeconds = (millis() - bootTime) / 1000;
    overheatTripTemp = temp;
    overheatTriggeredAt = millis();
    overheatLastTimestamp = formatUptime(uptimeSeconds);
    overheatLastReason = String("Thermal shutdown: ") + String(temp, 1) + " C reached (limit " +
                         String(overheatShutdownC, 1) + " C). Stream disabled; acknowledge in UI.";
    overheatLatched = true;
    simplePrintln("THERMAL PROTECTION: " + overheatLastReason);
    simplePrintln("TIP: Improve cooling or lower WiFi TX power/CPU MHz if overheating persists.");
    persistOverheatNote();
}

// Temperature monitoring + thermal protection
void checkTemperature() {
    float temp = temperatureRead(); // ESP32 internal sensor (approximate)
    bool tempValid = isTemperatureValid(temp);
    if (!tempValid) {
        lastTemperatureValid = false;
        if (!overheatSensorFault) {
            overheatSensorFault = true;
            overheatLastReason = "Thermal protection disabled: temperature sensor unavailable.";
            overheatLastTimestamp = "";
            overheatTripTemp = 0.0f;
            overheatTriggeredAt = 0;
            persistOverheatNote();
            simplePrintln("WARNING: Temperature sensor unavailable. Thermal protection paused.");
        }
        return;
    }

    lastTemperatureC = temp;
    lastTemperatureValid = true;

    if (overheatSensorFault) {
        overheatSensorFault = false;
        overheatLastReason = "Thermal protection restored: temperature sensor reading valid.";
        overheatLastTimestamp = formatUptime((millis() - bootTime) / 1000);
        persistOverheatNote();
        simplePrintln("Temperature sensor restored. Thermal protection active again.");
    }

    if (temp > maxTemperature) {
        maxTemperature = temp;
    }

    bool protectionActive = overheatProtectionEnabled && !overheatSensorFault;
    if (protectionActive) {
        if (!overheatLockoutActive && temp >= overheatShutdownC) {
            overheatLockoutActive = true;
            recordOverheatTrip(temp);
            // Disable streaming until user restarts manually
            if (rtspClient && rtspClient.connected()) {
                rtspClient.stop();
            }
            if (isStreaming) {
                isStreaming = false;
            }
            rtspServerEnabled = false;
            rtspServer.stop();
        } else if (overheatLockoutActive && temp <= (overheatShutdownC - OVERHEAT_LIMIT_STEP_C)) {
            // Allow re-arming after we cool down by at least one step
            overheatLockoutActive = false;
        }
    } else {
        overheatLockoutActive = false;
    }

    // Only warn occasionally on high temperature; no periodic logging
    static unsigned long lastTempWarn = 0;
    float warnThreshold = max(overheatShutdownC - 5.0f, (float)OVERHEAT_MIN_LIMIT_C);
    if (temp > warnThreshold && (millis() - lastTempWarn) > 600000UL) { // 10 min cooldown
        simplePrintln("WARNING: High temperature detected (" + String(temp, 1) + " C). Approaching shutdown limit.");
        lastTempWarn = millis();
    }
}

// Performance diagnostics
void checkPerformance() {
    uint32_t currentHeap = ESP.getFreeHeap();
    if (currentHeap < minFreeHeap) {
        minFreeHeap = currentHeap;
    }

    if (isStreaming && (millis() - lastStatsReset) > 30000) {
        uint32_t runtime = millis() - lastStatsReset;
        uint32_t currentRate = (audioPacketsSent * 1000) / runtime;

        if (currentRate > maxPacketRate) maxPacketRate = currentRate;
        if (currentRate < minPacketRate) minPacketRate = currentRate;

        if (currentRate < minAcceptableRate) {
            simplePrintln("PERFORMANCE DEGRADATION DETECTED!");
            simplePrintln("Rate " + String(currentRate) + " < minimum " + String(minAcceptableRate) + " pkt/s");

            if (autoRecoveryEnabled) {
                simplePrintln("AUTO-RECOVERY: Restarting I2S...");
                restartI2S();
                audioPacketsSent = 0;
                lastStatsReset = millis();
                lastI2SReset = millis();
            }
        }
    }
}

// WiFi health check
void checkWiFiHealth() {
    if (WiFi.status() != WL_CONNECTED) {
        simplePrintln("WiFi disconnected! Reconnecting...");
        WiFi.reconnect();
    }

    // Re-apply TX power WITHOUT logging (prevent periodic log spam)
    applyWifiTxPower(false);

    int32_t rssi = WiFi.RSSI();
    if (rssi < -85) {
        simplePrintln("WARNING: Weak WiFi signal: " + String(rssi) + " dBm");
    }
}

// Scheduled reset
void checkScheduledReset() {
    if (!scheduledResetEnabled) return;

    unsigned long uptimeHours = (millis() - bootTime) / 3600000;
    if (uptimeHours >= resetIntervalHours) {
        simplePrintln("SCHEDULED RESET: " + String(resetIntervalHours) + " hours reached");
        delay(1000);
        ESP.restart();
    }
}

// Load settings from flash
void loadAudioSettings() {
    audioPrefs.begin("audio", false);
    currentSampleRate = audioPrefs.getUInt("sampleRate", DEFAULT_SAMPLE_RATE);
    currentGainFactor = audioPrefs.getFloat("gainFactor", DEFAULT_GAIN_FACTOR);
    currentBufferSize = audioPrefs.getUShort("bufferSize", DEFAULT_BUFFER_SIZE);
    // (1) respect compile-time default 12 on first boot
    i2sShiftBits = audioPrefs.getUChar("shiftBits", i2sShiftBits);
    autoRecoveryEnabled = audioPrefs.getBool("autoRecovery", true);
    scheduledResetEnabled = audioPrefs.getBool("schedReset", false);
    resetIntervalHours = audioPrefs.getUInt("resetHours", 24);
    minAcceptableRate = audioPrefs.getUInt("minRate", 50);
    performanceCheckInterval = audioPrefs.getUInt("checkInterval", 15);
    autoThresholdEnabled = audioPrefs.getBool("thrAuto", true);
    cpuFrequencyMhz = audioPrefs.getUChar("cpuFreq", 240);
    wifiTxPowerDbm = audioPrefs.getFloat("wifiTxDbm", DEFAULT_WIFI_TX_DBM);
    highpassEnabled = audioPrefs.getBool("hpEnable", DEFAULT_HPF_ENABLED);
    highpassCutoffHz = (uint16_t)audioPrefs.getUInt("hpCutoff", DEFAULT_HPF_CUTOFF_HZ);
    overheatProtectionEnabled = audioPrefs.getBool("ohEnable", DEFAULT_OVERHEAT_PROTECTION);
    uint32_t ohLimit = audioPrefs.getUInt("ohThresh", DEFAULT_OVERHEAT_LIMIT_C);
    if (ohLimit < OVERHEAT_MIN_LIMIT_C) ohLimit = OVERHEAT_MIN_LIMIT_C;
    if (ohLimit > OVERHEAT_MAX_LIMIT_C) ohLimit = OVERHEAT_MAX_LIMIT_C;
    ohLimit = OVERHEAT_MIN_LIMIT_C + ((ohLimit - OVERHEAT_MIN_LIMIT_C) / OVERHEAT_LIMIT_STEP_C) * OVERHEAT_LIMIT_STEP_C;
    overheatShutdownC = (float)ohLimit;
    overheatLastReason = audioPrefs.getString("ohReason", "");
    overheatLastTimestamp = audioPrefs.getString("ohStamp", "");
    overheatTripTemp = audioPrefs.getFloat("ohTripC", 0.0f);
    overheatLatched = audioPrefs.getBool("ohLatched", false);
    audioPrefs.end();

    if (autoThresholdEnabled) {
        minAcceptableRate = computeRecommendedMinRate();
    }
    if (overheatLatched) {
        rtspServerEnabled = false;
    }
    // Log the configured TX dBm (not the current enum), snapped for clarity
    float txShown = wifiPowerLevelToDbm(pickWifiPowerLevel(wifiTxPowerDbm));
    simplePrintln("Loaded settings: Rate=" + String(currentSampleRate) +
                  ", Gain=" + String(currentGainFactor, 1) +
                  ", Buffer=" + String(currentBufferSize) +
                  ", WiFiTX=" + String(txShown, 1) + "dBm" +
                  ", shiftBits=" + String(i2sShiftBits) +
                  ", HPF=" + String(highpassEnabled?"on":"off") +
                  ", HPFcut=" + String(highpassCutoffHz) + "Hz");
}

// Save settings to flash
void saveAudioSettings() {
    audioPrefs.begin("audio", false);
    audioPrefs.putUInt("sampleRate", currentSampleRate);
    audioPrefs.putFloat("gainFactor", currentGainFactor);
    audioPrefs.putUShort("bufferSize", currentBufferSize);
    audioPrefs.putUChar("shiftBits", i2sShiftBits);
    audioPrefs.putBool("autoRecovery", autoRecoveryEnabled);
    audioPrefs.putBool("schedReset", scheduledResetEnabled);
    audioPrefs.putUInt("resetHours", resetIntervalHours);
    audioPrefs.putUInt("minRate", minAcceptableRate);
    audioPrefs.putUInt("checkInterval", performanceCheckInterval);
    audioPrefs.putBool("thrAuto", autoThresholdEnabled);
    audioPrefs.putUChar("cpuFreq", cpuFrequencyMhz);
    audioPrefs.putFloat("wifiTxDbm", wifiTxPowerDbm);
    audioPrefs.putBool("hpEnable", highpassEnabled);
    audioPrefs.putUInt("hpCutoff", (uint32_t)highpassCutoffHz);
    audioPrefs.putBool("ohEnable", overheatProtectionEnabled);
    uint32_t ohLimit = (uint32_t)(overheatShutdownC + 0.5f);
    if (ohLimit < OVERHEAT_MIN_LIMIT_C) ohLimit = OVERHEAT_MIN_LIMIT_C;
    if (ohLimit > OVERHEAT_MAX_LIMIT_C) ohLimit = OVERHEAT_MAX_LIMIT_C;
    audioPrefs.putUInt("ohThresh", ohLimit);
    audioPrefs.putString("ohReason", overheatLastReason);
    audioPrefs.putString("ohStamp", overheatLastTimestamp);
    audioPrefs.putFloat("ohTripC", overheatTripTemp);
    audioPrefs.putBool("ohLatched", overheatLatched);
    audioPrefs.end();

    simplePrintln("Settings saved to flash");
}

// Schedule a safe reboot (optionally with factory reset) after delayMs
void scheduleReboot(bool factoryReset, uint32_t delayMs) {
    scheduledFactoryReset = factoryReset;
    scheduledRebootAt = millis() + delayMs;
}

// Compute recommended minimum packet-rate threshold based on current sample rate and buffer size
uint32_t computeRecommendedMinRate() {
    uint32_t buf = max((uint16_t)1, currentBufferSize);
    float expectedPktPerSec = (float)currentSampleRate / (float)buf;
    uint32_t rec = (uint32_t)(expectedPktPerSec * 0.7f + 0.5f); // 70% safety margin
    if (rec < 5) rec = 5;
    return rec;
}

// Restore application settings to safe defaults and persist
void resetToDefaultSettings() {
    simplePrintln("FACTORY RESET: Restoring default settings...");

    // Clear persisted settings in our namespace
    audioPrefs.begin("audio", false);
    audioPrefs.clear();
    audioPrefs.end();

    // Reset runtime variables to defaults
    currentSampleRate = DEFAULT_SAMPLE_RATE;
    currentGainFactor = DEFAULT_GAIN_FACTOR;
    currentBufferSize = DEFAULT_BUFFER_SIZE;
    i2sShiftBits = 0;  // Default for PDM microphone on M5Stack Atom Echo

    autoRecoveryEnabled = true;
    autoThresholdEnabled = true;
    scheduledResetEnabled = false;
    resetIntervalHours = 24;
    minAcceptableRate = computeRecommendedMinRate();
    performanceCheckInterval = 15;
    cpuFrequencyMhz = 240;
    wifiTxPowerDbm = DEFAULT_WIFI_TX_DBM;
    highpassEnabled = DEFAULT_HPF_ENABLED;
    highpassCutoffHz = DEFAULT_HPF_CUTOFF_HZ;
    overheatProtectionEnabled = DEFAULT_OVERHEAT_PROTECTION;
    overheatShutdownC = (float)DEFAULT_OVERHEAT_LIMIT_C;
    overheatLockoutActive = false;
    overheatTripTemp = 0.0f;
    overheatTriggeredAt = 0;
    overheatLastReason = "";
    overheatLastTimestamp = "";
    overheatSensorFault = false;
    overheatLatched = false;
    lastTemperatureC = 0.0f;
    lastTemperatureValid = false;

    isStreaming = false;

    saveAudioSettings();

    simplePrintln("Defaults applied. Device will reboot.");
}

// Restart I2S with new parameters
void restartI2S() {
    simplePrintln("Restarting I2S with new parameters...");
    isStreaming = false;

    // Stop audio capture task on Core 1
    stopAudioCaptureTask();

    // Free old buffers
    if (i2s_16bit_buffer) { free(i2s_16bit_buffer); i2s_16bit_buffer = nullptr; }

    // Allocate new buffers with new size (16-bit for PDM)
    i2s_16bit_buffer = (int16_t*)malloc(currentBufferSize * sizeof(int16_t));
    if (!i2s_16bit_buffer) {
        simplePrintln("FATAL: Memory allocation failed after parameter change!");
        ESP.restart();
    }

    // Restart I2S driver
    setup_i2s_driver();

    // Restart audio capture task on Core 1
    startAudioCaptureTask();

    // Refresh HPF with current parameters
    updateHighpassCoeffs();
    maxPacketRate = 0;
    minPacketRate = 0xFFFFFFFF;
    simplePrintln("I2S and audio task restarted successfully");
}

// Minimal print helpers: Serial + buffered for Web UI
void simplePrint(String message) {
    Serial.print(message);
}

void simplePrintln(String message) {
    Serial.println(message);
    webui_pushLog(message);
}

// ================== CORE 1: AUDIO CAPTURE TASK ==================
// This task runs on Core 1 with high priority and ONLY captures audio
// It uses buffer pool to pass data to Core 0
void audioCaptureTask(void* parameter) {
    simplePrintln("Audio capture task started on Core 1");
    audioTaskRunning = true;

    size_t bytesRead = 0;
    uint32_t consecutiveErrors = 0;
    const uint32_t MAX_ERRORS = 10;
    uint32_t captureCount = 0;
    uint32_t noBufferCount = 0;

    while (audioTaskRunning) {
        // Get a free buffer from the pool
        AudioBuffer* audioBuffer = NULL;
        if (xQueueReceive(audioFreeQueue, &audioBuffer, 0) != pdTRUE) {
            // No free buffers - Core 0 isn't keeping up
            noBufferCount++;
            if (noBufferCount == 1 || noBufferCount % 1000 == 0) {
                Serial.printf("[Core1] No free buffers! Count: %d\n", noBufferCount);
            }
            vTaskDelay(pdMS_TO_TICKS(1));  // Brief delay before retry
            continue;
        }

        // Read from I2S directly into the buffer
        esp_err_t result = i2s_read(I2S_NUM_0, audioBuffer->data,
                                    audioBufferSize * sizeof(int16_t),
                                    &bytesRead, portMAX_DELAY);

        if (result == ESP_OK && bytesRead > 0) {
            consecutiveErrors = 0;
            audioBuffer->sampleCount = bytesRead / sizeof(int16_t);
            captureCount++;

            // Send filled buffer to Core 0
            xQueueSend(audioFullQueue, &audioBuffer, portMAX_DELAY);

            // Debug: Log every 100 captures
            if (captureCount % 100 == 0) {
                Serial.printf("[Core1] Captured %d buffers, free: %d, full: %d\n",
                    captureCount, uxQueueMessagesWaiting(audioFreeQueue),
                    uxQueueMessagesWaiting(audioFullQueue));
            }
        } else {
            // I2S error - return buffer to free queue
            xQueueSend(audioFreeQueue, &audioBuffer, 0);

            consecutiveErrors++;
            Serial.printf("[Core1] I2S read error: %d (consecutive: %d)\n", result, consecutiveErrors);
            if (consecutiveErrors >= MAX_ERRORS) {
                simplePrintln("Audio capture: too many I2S errors, restarting task");
                vTaskDelay(pdMS_TO_TICKS(100));
                consecutiveErrors = 0;
            }
        }

        taskYIELD();
    }

    simplePrintln("Audio capture task stopped");
    vTaskDelete(NULL);
}

// Start audio capture task on Core 1
void startAudioCaptureTask() {
    if (audioCaptureTaskHandle != NULL) {
        simplePrintln("Audio task already running");
        return;
    }

    // Set buffer size for the capture task
    audioBufferSize = currentBufferSize;

    // Create queues if they don't exist
    if (audioFreeQueue == NULL) {
        audioFreeQueue = xQueueCreate(AUDIO_QUEUE_LENGTH, sizeof(AudioBuffer*));
        if (audioFreeQueue == NULL) {
            simplePrintln("FATAL: Failed to create free queue!");
            return;
        }
    }

    if (audioFullQueue == NULL) {
        audioFullQueue = xQueueCreate(AUDIO_QUEUE_LENGTH, sizeof(AudioBuffer*));
        if (audioFullQueue == NULL) {
            simplePrintln("FATAL: Failed to create full queue!");
            return;
        }
    }

    // Allocate buffer pool
    for (int i = 0; i < AUDIO_QUEUE_LENGTH; i++) {
        audioBufferPool[i].data = (int16_t*)malloc(audioBufferSize * sizeof(int16_t));
        if (!audioBufferPool[i].data) {
            simplePrintln("FATAL: Failed to allocate buffer pool!");
            return;
        }
        audioBufferPool[i].sampleCount = 0;

        // Add to free queue
        AudioBuffer* bufPtr = &audioBufferPool[i];
        xQueueSend(audioFreeQueue, &bufPtr, 0);
    }

    simplePrintln("Buffer pool created (8 buffers)");

    // Create task pinned to Core 1 with elevated priority
    // Priority 10 = above normal (1) but below critical system tasks
    BaseType_t result = xTaskCreatePinnedToCore(
        audioCaptureTask,           // Task function
        "AudioCapture",             // Name
        4096,                       // Stack size (bytes)
        NULL,                       // Parameters
        10,                         // Priority (elevated but not aggressive)
        &audioCaptureTaskHandle,    // Task handle
        1                           // Core 1 (PRO_CPU)
    );

    if (result != pdPASS) {
        simplePrintln("FATAL: Failed to create audio capture task!");
    } else {
        simplePrintln("Audio capture task created on Core 1 (priority 10)");
    }
}

// Stop audio capture task
void stopAudioCaptureTask() {
    if (audioCaptureTaskHandle != NULL) {
        audioTaskRunning = false;
        vTaskDelay(pdMS_TO_TICKS(100));  // Give task time to clean up
        audioCaptureTaskHandle = NULL;

        // Delete the queues
        if (audioFreeQueue != NULL) {
            vQueueDelete(audioFreeQueue);
            audioFreeQueue = NULL;
        }
        if (audioFullQueue != NULL) {
            vQueueDelete(audioFullQueue);
            audioFullQueue = NULL;
        }

        // Free buffer pool
        for (int i = 0; i < AUDIO_QUEUE_LENGTH; i++) {
            if (audioBufferPool[i].data) {
                free(audioBufferPool[i].data);
                audioBufferPool[i].data = NULL;
            }
        }

        simplePrintln("Audio capture task stopped, buffer pool freed");
    }
}

// OTA setup
void setupOTA() {
    ArduinoOTA.setHostname("ESP32-RTSP-Mic");
#ifdef OTA_PASSWORD
    ArduinoOTA.setPassword(OTA_PASSWORD);
#endif
    ArduinoOTA.begin();
}

// I2S setup for M5Stack Atom Echo (PDM microphone mode)
void setup_i2s_driver() {
    i2s_driver_uninstall(I2S_NUM_0);

    // DMA buffer configuration - smaller for lower latency with 16kHz
    uint16_t dma_buf_len = 60;  // Match M5Atom Echo example

    i2s_config_t i2s_config = {
        // PDM mode required for SPM1423 microphone on Atom Echo
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),
        .sample_rate = currentSampleRate,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,  // Match original demo exactly
        .channel_format = I2S_CHANNEL_FMT_ALL_RIGHT,    // ALL_RIGHT like original demo
#if ESP_IDF_VERSION > ESP_IDF_VERSION_VAL(4, 1, 0)
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
#else
        .communication_format = I2S_COMM_FORMAT_I2S,
#endif
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 6,     // Match M5Atom Echo example
        .dma_buf_len = dma_buf_len,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };

    i2s_pin_config_t pin_config = {
#if (ESP_IDF_VERSION > ESP_IDF_VERSION_VAL(4, 3, 0))
        .mck_io_num = I2S_PIN_NO_CHANGE,
#endif
        .bck_io_num = I2S_BCLK_PIN,
        .ws_io_num = I2S_LRCLK_PIN,
        .data_out_num = I2S_DATA_OUT_PIN,
        .data_in_num = I2S_DATA_IN_PIN
    };

    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &pin_config);
    i2s_set_clk(I2S_NUM_0, currentSampleRate, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);

    simplePrintln("I2S ready (PDM mode): " + String(currentSampleRate) + "Hz, gain " +
                  String(currentGainFactor, 1) + ", buffer " + String(currentBufferSize) +
                  ", shiftBits " + String(i2sShiftBits));
}

static bool writeAll(WiFiClient &client, const uint8_t* data, size_t len) {
    size_t off = 0;
    unsigned long startTime = millis();
    const unsigned long WRITE_TIMEOUT_MS = 30;  // 30ms timeout - fast failover

    while (off < len) {
        // Check timeout to prevent blocking forever
        if (millis() - startTime > WRITE_TIMEOUT_MS) {
            Serial.println("[Core0] Write timeout - dropping frame");
            return false;
        }

        int w = client.write(data + off, len - off);
        if (w <= 0) return false;
        off += (size_t)w;
    }
    return true;
}

void sendRTPPacket(WiFiClient &client, int16_t* audioData, int numSamples) {
    if (!client.connected()) return;

    const uint16_t payloadSize = (uint16_t)(numSamples * (int)sizeof(int16_t));
    const uint16_t packetSize = (uint16_t)(12 + payloadSize);

    // RTSP interleaved header: '$' 0x24, channel 0, length
    uint8_t inter[4];
    inter[0] = 0x24;
    inter[1] = 0x00;
    inter[2] = (uint8_t)((packetSize >> 8) & 0xFF);
    inter[3] = (uint8_t)(packetSize & 0xFF);

    // RTP header (12 bytes)
    uint8_t header[12];
    header[0] = 0x80;      // V=2, P=0, X=0, CC=0
    header[1] = 96;        // M=0, PT=96 (dynamic)
    // (3) safe byte-wise filling (no unaligned writes)
    header[2] = (uint8_t)((rtpSequence >> 8) & 0xFF);
    header[3] = (uint8_t)(rtpSequence & 0xFF);
    header[4] = (uint8_t)((rtpTimestamp >> 24) & 0xFF);
    header[5] = (uint8_t)((rtpTimestamp >> 16) & 0xFF);
    header[6] = (uint8_t)((rtpTimestamp >> 8) & 0xFF);
    header[7] = (uint8_t)(rtpTimestamp & 0xFF);
    header[8]  = (uint8_t)((rtpSSRC >> 24) & 0xFF);
    header[9]  = (uint8_t)((rtpSSRC >> 16) & 0xFF);
    header[10] = (uint8_t)((rtpSSRC >> 8) & 0xFF);
    header[11] = (uint8_t)(rtpSSRC & 0xFF);

    // Host->network: per-sample byte-swap (16bit PCM L16 big-endian)
    for (int i = 0; i < numSamples; ++i) {
        uint16_t s = (uint16_t)audioData[i];
        s = (uint16_t)((s << 8) | (s >> 8)); // htons without dependency
        audioData[i] = (int16_t)s;
    }

    if (!writeAll(client, inter, sizeof(inter)) ||
        !writeAll(client, header, sizeof(header)) ||
        !writeAll(client, (uint8_t*)audioData, payloadSize)) {
        isStreaming = false;
        return;
    }

    rtpSequence++;
    rtpTimestamp += (uint32_t)numSamples;
    audioPacketsSent++;
}

// Audio streaming - TEMPORARILY READING DIRECTLY FROM I2S FOR PDM DEBUGGING
void streamAudio(WiFiClient &client) {
    if (!isStreaming || !client.connected()) return;

    static uint32_t streamCount = 0;
    static uint32_t emptyCount = 0;
    static uint32_t returnCount = 0;

    // Receive from full queue (Core 1 → Core 0 transfer)
    // 30ms timeout balances responsiveness with web UI performance
    AudioBuffer* audioBuffer = NULL;
    if (xQueueReceive(audioFullQueue, &audioBuffer, pdMS_TO_TICKS(30)) != pdTRUE) {
        // No data available - Core 1 hasn't captured anything yet
        // The 30ms timeout already gave other tasks time to run
        emptyCount++;
        if (emptyCount % 100 == 0) {
            Serial.printf("[Core0] Queue empty %d times\n", emptyCount);
        }
        return;
    }

    uint16_t samplesRead = audioBuffer->sampleCount;
    int16_t* captureData = audioBuffer->data;

    streamCount++;

    // Debug: Monitor queue health
    if (streamCount % 100 == 0) {
        Serial.printf("[Core0] Streamed %d, free: %d, full: %d, peak: %d\n",
            streamCount, uxQueueMessagesWaiting(audioFreeQueue),
            uxQueueMessagesWaiting(audioFullQueue), lastPeakAbs16);
    }

    // If HPF params changed dynamically, recompute
    if (highpassEnabled && (hpfConfigSampleRate != currentSampleRate || hpfConfigCutoff != highpassCutoffHz)) {
        updateHighpassCoeffs();
    }

    bool clipped = false;
    float peakAbs = 0.0f;

    // Process audio - read from captureData, write to i2s_16bit_buffer
    for (int i = 0; i < samplesRead; i++) {
        // PDM 16-bit mode: samples are already in the right range, just apply shift if needed
        float sample = (float)(captureData[i] >> i2sShiftBits);

        if (highpassEnabled) sample = hpf.process(sample);

        float amplified = sample * currentGainFactor;
        float aabs = fabsf(amplified);
        if (aabs > peakAbs) peakAbs = aabs;
        if (aabs > 32767.0f) clipped = true;

        // Clamp to 16-bit range
        if (amplified > 32767.0f) amplified = 32767.0f;
        if (amplified < -32768.0f) amplified = -32768.0f;
        i2s_16bit_buffer[i] = (int16_t)amplified;
    }

    // Update metering after processing the block
    if (peakAbs > 32767.0f) peakAbs = 32767.0f;
    lastPeakAbs16 = (uint16_t)peakAbs;
    audioClippedLastBlock = clipped;
    if (clipped) audioClipCount++;

    // Update peak hold for a short window (~3 s) to match UI polling cadence
    if (lastPeakAbs16 > peakHoldAbs16) {
        peakHoldAbs16 = lastPeakAbs16;
        peakHoldUntilMs = millis() + 3000UL;
    } else if (peakHoldAbs16 > 0 && millis() > peakHoldUntilMs) {
        peakHoldAbs16 = 0;
    }

    sendRTPPacket(client, i2s_16bit_buffer, samplesRead);

    // Return buffer to free queue for reuse by Core 1
    if (xQueueSend(audioFreeQueue, &audioBuffer, portMAX_DELAY) == pdTRUE) {
        returnCount++;
        if (returnCount % 100 == 0) {
            Serial.printf("[Core0] Returned %d buffers to free queue\n", returnCount);
        }
    } else {
        Serial.printf("[Core0] CRITICAL: Failed to return buffer %p to free queue!\n", audioBuffer);
    }
}

// RTSP handling
void handleRTSPCommand(WiFiClient &client, String request) {
    String cseq = "1";
    int cseqPos = request.indexOf("CSeq: ");
    if (cseqPos >= 0) {
        cseq = request.substring(cseqPos + 6, request.indexOf("\r", cseqPos));
        cseq.trim();
    }

    lastRTSPActivity = millis();

    if (request.startsWith("OPTIONS")) {
        client.print("RTSP/1.0 200 OK\r\n");
        client.print("CSeq: " + cseq + "\r\n");
        client.print("Public: OPTIONS, DESCRIBE, SETUP, PLAY, TEARDOWN, GET_PARAMETER\r\n\r\n");

    } else if (request.startsWith("DESCRIBE")) {
        String ip = WiFi.localIP().toString();
        String sdp = "v=0\r\n";
        sdp += "o=- 0 0 IN IP4 " + ip + "\r\n";
        sdp += "s=ESP32 RTSP Mic (" + String(currentSampleRate) + "Hz, 16-bit PCM)\r\n";
        // better compatibility: include actual IP
        sdp += "c=IN IP4 " + ip + "\r\n";
        sdp += "t=0 0\r\n";
        sdp += "m=audio 0 RTP/AVP 96\r\n";
        sdp += "a=rtpmap:96 L16/" + String(currentSampleRate) + "/1\r\n";
        sdp += "a=control:track1\r\n";

        client.print("RTSP/1.0 200 OK\r\n");
        client.print("CSeq: " + cseq + "\r\n");
        client.print("Content-Type: application/sdp\r\n");
        client.print("Content-Base: rtsp://" + ip + ":8554/audio/\r\n");
        client.print("Content-Length: " + String(sdp.length()) + "\r\n\r\n");
        client.print(sdp);

    } else if (request.startsWith("SETUP")) {
        rtspSessionId = String(random(100000000, 999999999));
        client.print("RTSP/1.0 200 OK\r\n");
        client.print("CSeq: " + cseq + "\r\n");
        client.print("Session: " + rtspSessionId + "\r\n");
        client.print("Transport: RTP/AVP/TCP;unicast;interleaved=0-1\r\n\r\n");

    } else if (request.startsWith("PLAY")) {
        client.print("RTSP/1.0 200 OK\r\n");
        client.print("CSeq: " + cseq + "\r\n");
        client.print("Session: " + rtspSessionId + "\r\n");
        client.print("Range: npt=0.000-\r\n\r\n");

        isStreaming = true;
        rtpSequence = 0;
        rtpTimestamp = 0;
        audioPacketsSent = 0;
        lastStatsReset = millis();
        lastRtspPlayMs = millis();
        rtspPlayCount++;

        // Start audio capture task when streaming begins
        startAudioCaptureTask();

        // Set LED to purple when streaming
        M5.dis.drawpix(0, CRGB(128, 0, 128));
        simplePrintln("STREAMING STARTED");

    } else if (request.startsWith("TEARDOWN")) {
        client.print("RTSP/1.0 200 OK\r\n");
        client.print("CSeq: " + cseq + "\r\n");
        client.print("Session: " + rtspSessionId + "\r\n\r\n");
        isStreaming = false;

        // Stop audio capture task when streaming ends
        stopAudioCaptureTask();

        // Set LED back to green when streaming stops
        M5.dis.drawpix(0, CRGB(0, 128, 0));
        simplePrintln("STREAMING STOPPED");
    } else if (request.startsWith("GET_PARAMETER")) {
        // Many RTSP clients send GET_PARAMETER as keep-alive.
        client.print("RTSP/1.0 200 OK\r\n");
        client.print("CSeq: " + cseq + "\r\n\r\n");
    }
}

// RTSP processing
void processRTSP(WiFiClient &client) {
    if (!client.connected()) return;

    if (client.available()) {
        int available = client.available();

        if (rtspParseBufferPos + available >= (int)sizeof(rtspParseBuffer)) {
            available = sizeof(rtspParseBuffer) - rtspParseBufferPos - 1;
            if (available <= 0) {
                simplePrintln("RTSP buffer overflow - resetting");
                rtspParseBufferPos = 0;
                return;
            }
        }

        client.read(rtspParseBuffer + rtspParseBufferPos, available);
        rtspParseBufferPos += available;

        char* endOfHeader = strstr((char*)rtspParseBuffer, "\r\n\r\n");
        if (endOfHeader != nullptr) {
            *endOfHeader = '\0';
            String request = String((char*)rtspParseBuffer);

            handleRTSPCommand(client, request);

            int headerLen = (endOfHeader - (char*)rtspParseBuffer) + 4;
            memmove(rtspParseBuffer, rtspParseBuffer + headerLen, rtspParseBufferPos - headerLen);
            rtspParseBufferPos -= headerLen;
        }
    }
}


// Web UI is a separate module (WebUI.*)

void setup() {
    // Initialize M5Atom (for button and LED, serial enabled)
    M5.begin(true, false, true);

    Serial.begin(115200);
    delay(500);
    Serial.println("\n\n=== ESP32 RTSP Mic Starting ===");
    Serial.println("Board: M5Stack Atom Echo");

    // Set LED to indicate startup
    M5.dis.drawpix(0, CRGB(128, 128, 0));  // Yellow for startup

    // (4) seed for random(): combination of time and unique MAC
    randomSeed((uint32_t)micros() ^ (uint32_t)(ESP.getEfuseMac() & 0xFFFFFFFF));

    bootTime = millis(); // Store boot time
    rtpSSRC = (uint32_t)random(1, 0x7FFFFFFF);
    Serial.println("Random seed initialized");

    // Enable external antenna (for XIAO ESP32-C6).
    // NOTE: Commented out for M5Stack STAMP S3 - no external antenna control needed
    // pinMode(3, OUTPUT);
    // digitalWrite(3, LOW);
    // Serial.println("RF switch control enabled (GPIO3 LOW)");
    // pinMode(14, OUTPUT);
    // digitalWrite(14, HIGH);
    // Serial.println("External antenna selected (GPIO14 HIGH)");

    // Load settings from flash
    Serial.println("Loading settings...");
    loadAudioSettings();
    Serial.println("Settings loaded");

    // Allocate buffers with current size (16-bit for PDM)
    Serial.println("Allocating buffers...");
    i2s_16bit_buffer = (int16_t*)malloc(currentBufferSize * sizeof(int16_t));
    if (!i2s_16bit_buffer) {
        simplePrintln("FATAL: Memory allocation failed!");
        ESP.restart();
    }
    Serial.println("Buffers allocated");

    // WiFi optimization for stable streaming
    Serial.println("Initializing WiFi...");
    WiFi.setSleep(false);

    WiFiManager wm;
    wm.setConnectTimeout(60);
    wm.setConfigPortalTimeout(180);
    if (!wm.autoConnect("ESP32-RTSP-Mic-AP")) {
        simplePrintln("WiFi failed, restarting...");
        ESP.restart();
    }

    simplePrintln("WiFi connected: " + WiFi.localIP().toString());

    // Apply configured WiFi TX power after connect (logs once on change)
    applyWifiTxPower(true);

    Serial.println("Setting up OTA...");
    setupOTA();
    Serial.println("OTA ready");

    Serial.println("Setting up I2S driver...");
    setup_i2s_driver();
    Serial.println("I2S driver ready");

    // Audio queue and capture task will be created when RTSP streaming begins (on PLAY command)
    Serial.println("Dual-core audio ready (queue and task will start on RTSP PLAY)");

    Serial.println("Updating highpass coefficients...");
    updateHighpassCoeffs();
    Serial.println("Highpass coefficients updated");

    if (!overheatLatched) {
        rtspServer.begin();
        rtspServer.setNoDelay(true);
        rtspServerEnabled = true;
    } else {
        rtspServerEnabled = false;
        rtspServer.stop();
    }
    // Web UI
    webui_begin();

    lastStatsReset = millis();
    lastRTSPActivity = millis();
    lastMemoryCheck = millis();
    lastPerformanceCheck = millis();
    lastWiFiCheck = millis();
    minFreeHeap = ESP.getFreeHeap();
    float initialTemp = temperatureRead();
    if (isTemperatureValid(initialTemp)) {
        maxTemperature = initialTemp;
        lastTemperatureC = initialTemp;
        lastTemperatureValid = true;
        overheatSensorFault = false;
    } else {
        maxTemperature = 0.0f;
        lastTemperatureC = 0.0f;
        lastTemperatureValid = false;
        overheatSensorFault = true;
        overheatLastReason = "Thermal protection disabled: temperature sensor unavailable.";
        overheatLastTimestamp = "";
        overheatTripTemp = 0.0f;
        overheatTriggeredAt = 0;
        persistOverheatNote();
        simplePrintln("WARNING: Temperature sensor unavailable at startup. Thermal protection paused.");
    }

    setCpuFrequencyMhz(cpuFrequencyMhz);
    simplePrintln("CPU frequency set to " + String(cpuFrequencyMhz) + " MHz for optimal thermal/performance balance");

    if (!overheatLatched) {
        simplePrintln("RTSP server ready on port 8554");
        simplePrintln("RTSP URL: rtsp://" + WiFi.localIP().toString() + ":8554/audio");
        // Set LED to green when ready
        M5.dis.drawpix(0, CRGB(0, 128, 0));
    } else {
        simplePrintln("RTSP server paused due to thermal latch. Clear via Web UI before resuming streaming.");
        // Set LED to red when latched
        M5.dis.drawpix(0, CRGB(128, 0, 0));
    }
    simplePrintln("Web UI: http://" + WiFi.localIP().toString() + "/");
}

void loop() {
    // Update M5Atom (for button and LED handling)
    M5.update();

    ArduinoOTA.handle();

    webui_handleClient();

    if (millis() - lastTempCheck > 60000) { // 1 min
        checkTemperature();
        lastTempCheck = millis();
    }

    if (millis() - lastMemoryCheck > 30000) { // 30 s
        uint32_t currentHeap = ESP.getFreeHeap();
        if (currentHeap < minFreeHeap) minFreeHeap = currentHeap;
        lastMemoryCheck = millis();
    }

    if (millis() - lastPerformanceCheck > (performanceCheckInterval * 60000UL)) {
        checkPerformance();
        lastPerformanceCheck = millis();
    }

    if (millis() - lastWiFiCheck > 30000) { // 30 s
        checkWiFiHealth(); // without TX power log spam
        lastWiFiCheck = millis();
    }

    checkScheduledReset();

    // RTSP client management
    if (rtspServerEnabled) {
        if (rtspClient && !rtspClient.connected()) {
            rtspClient.stop();
            if (isStreaming) {
                isStreaming = false;
                // Stop audio capture task when client disconnects
                stopAudioCaptureTask();
            }
            // Set LED back to green when client disconnects
            M5.dis.drawpix(0, CRGB(0, 128, 0));
            simplePrintln("RTSP client disconnected");
        }

        // Timeout for RTSP clients (30 seconds of inactivity)
        if (rtspClient && rtspClient.connected() && !isStreaming) {
            if (millis() - lastRTSPActivity > 30000) {
                rtspClient.stop();
                simplePrintln("RTSP client timeout - disconnected");
            }
        }

        if (!rtspClient || !rtspClient.connected()) {
            // Ensure audio task is stopped before accepting new client
            if (isStreaming) {
                isStreaming = false;
                stopAudioCaptureTask();
                M5.dis.drawpix(0, CRGB(0, 128, 0));  // Green - ready
            }

            WiFiClient newClient = rtspServer.available();
            if (newClient) {
                rtspClient = newClient;
                rtspClient.setNoDelay(true);
                rtspParseBufferPos = 0;
                lastRTSPActivity = millis();
                lastRtspClientConnectMs = millis();
                rtspConnectCount++;
                simplePrintln("New RTSP client connected from: " + rtspClient.remoteIP().toString());
            }
        }

        if (rtspClient && rtspClient.connected()) {
            if (rtspClient.available()) {
                lastRTSPActivity = millis();
            }
            processRTSP(rtspClient);
            if (isStreaming) {
                streamAudio(rtspClient);
            }
        }
    } else {
        if (rtspClient && rtspClient.connected()) {
            rtspClient.stop();
            isStreaming = false;
            // Set LED to red if latched, otherwise green
            if (overheatLatched) {
                M5.dis.drawpix(0, CRGB(128, 0, 0));
            } else {
                M5.dis.drawpix(0, CRGB(0, 128, 0));
            }
        }
    }
    // Handle deferred reboot/reset safely here
    if (scheduledRebootAt != 0 && millis() >= scheduledRebootAt) {
        if (scheduledFactoryReset) {
            resetToDefaultSettings();
        }
        delay(50);
        ESP.restart();
    }
}
