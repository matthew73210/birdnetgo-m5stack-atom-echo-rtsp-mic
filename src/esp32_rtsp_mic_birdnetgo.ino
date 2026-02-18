#include <WiFi.h>
#include <WiFiManager.h>
#include "driver/i2s.h"
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <math.h>
#include <M5Atom.h>
#include "WebUI.h"

// ================== DUAL-CORE AUDIO ARCHITECTURE ==================
// Core 1: Complete audio pipeline (I2S → process → RTP → WiFi)
// Core 0: Web UI, diagnostics, RTSP protocol, client management
// PDM microphone outputs 16-bit samples directly

// Pointer handoff: Core 0 sets on PLAY, Core 1 uses for streaming, clears on failure
WiFiClient* volatile streamClient = NULL;
TaskHandle_t audioCaptureTaskHandle = NULL;
volatile bool audioTaskRunning = false;

// ================== SETTINGS (ESP32 RTSP Mic for BirdNET-Go) ==================
#define FW_VERSION "2.1.0"
// Expose FW version as a global C string for WebUI/API
const char* FW_VERSION_STR = FW_VERSION;

// OTA password (optional):
// - For production, set a strong password to protect OTA updates.
// - You can leave it undefined to disable password protection.
// - Example placeholder (edit as needed):
// #define OTA_PASSWORD "1234"  // Optional: change or leave undefined

// -- DEFAULT PARAMETERS (configurable via Web UI / API)
#define DEFAULT_SAMPLE_RATE 16000  // M5Stack Atom Echo: 16kHz for PDM microphone
#define DEFAULT_GAIN_FACTOR 3.0f
#define DEFAULT_BUFFER_SIZE 1024   // 64ms @ 16kHz - good balance for BirdNET-Go
#define DEFAULT_WIFI_TX_DBM 19.5f  // Default WiFi TX power in dBm
// High-pass filter defaults (to remove low-frequency rumble)
#define DEFAULT_HPF_ENABLED true
#define DEFAULT_HPF_CUTOFF_HZ 300

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
// Note: Audio buffers now allocated by Core 1 task

// -- Global state
unsigned long audioPacketsSent = 0;
unsigned long audioPacketsDropped = 0;  // Track dropped frames
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
            // Signal Core 1 to stop, then disable streaming
            isStreaming = false;
            streamClient = NULL;
            stopAudioCaptureTask();
            if (rtspClient && rtspClient.connected()) {
                rtspClient.stop();
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
    // i2sShiftBits is ALWAYS 0 for PDM microphones - not configurable
    i2sShiftBits = 0;
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
    bool wasStreaming = isStreaming;

    // Signal Core 1 to stop before touching anything
    isStreaming = false;
    streamClient = NULL;

    // Stop audio pipeline task on Core 1
    stopAudioCaptureTask();

    // Restart I2S driver
    setup_i2s_driver();

    // Refresh HPF with current parameters
    updateHighpassCoeffs();
    maxPacketRate = 0;
    minPacketRate = 0xFFFFFFFF;

    // If we were streaming, restart the pipeline with the existing client
    if (wasStreaming && rtspClient && rtspClient.connected()) {
        streamClient = &rtspClient;
        isStreaming = true;
        startAudioCaptureTask();
        simplePrintln("I2S restarted, streaming resumed");
    } else {
        simplePrintln("I2S restarted");
    }
}

// Minimal print helpers: Serial + buffered for Web UI
void simplePrint(String message) {
    Serial.print(message);
}

void simplePrintln(String message) {
    Serial.println(message);
    webui_pushLog(message);
}

// ================== CORE 1: FULL AUDIO PIPELINE ==================
// This task runs on Core 1 and handles the complete audio pipeline:
// I2S capture → processing → RTP packet formation → WiFi transmission
// Uses streamClient pointer (set by Core 0 on PLAY, cleared here on failure)
void audioCaptureTask(void* parameter) {
    simplePrintln("[Core1] Audio pipeline task started");
    audioTaskRunning = true;

    size_t bytesRead = 0;
    uint32_t consecutiveErrors = 0;
    const uint32_t MAX_ERRORS = 10;
    uint32_t packetCount = 0;

    // Allocate audio buffer for this task
    int16_t* captureBuffer = (int16_t*)malloc(currentBufferSize * sizeof(int16_t));
    int16_t* outputBuffer = (int16_t*)malloc(currentBufferSize * sizeof(int16_t));

    if (!captureBuffer || !outputBuffer) {
        simplePrintln("[Core1] FATAL: Failed to allocate audio buffers!");
        audioTaskRunning = false;
        vTaskDelete(NULL);
        return;
    }

    // High-pass filter state (local to this core)
    Biquad localHpf = hpf;  // Copy initial coefficients
    uint32_t localHpfConfigSampleRate = hpfConfigSampleRate;
    uint16_t localHpfConfigCutoff = hpfConfigCutoff;

    uint32_t i2sErrors = 0;
    unsigned long lastStatsLog = millis();

    while (audioTaskRunning) {
        // Only process audio when streaming and we have a client
        if (!isStreaming || !streamClient) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // Local copy of client pointer (set once by Core 0, stable during streaming)
        WiFiClient* client = streamClient;

        // Periodic health stats (every 30 seconds)
        if (millis() - lastStatsLog > 30000) {
            Serial.printf("[Core1] Stats: Sent=%u, I2S_errs=%u, Clipping=%lu\n",
                         packetCount, i2sErrors, audioClipCount);
            lastStatsLog = millis();
        }

        // Read from I2S
        esp_err_t result = i2s_read(I2S_NUM_0, captureBuffer,
                                    currentBufferSize * sizeof(int16_t),
                                    &bytesRead, portMAX_DELAY);

        if (result != ESP_OK || bytesRead == 0) {
            consecutiveErrors++;
            i2sErrors++;
            if (consecutiveErrors >= MAX_ERRORS) {
                simplePrintln("[Core1] Too many I2S errors, pausing");
                vTaskDelay(pdMS_TO_TICKS(100));
                consecutiveErrors = 0;
            }
            continue;
        }

        consecutiveErrors = 0;
        uint16_t samplesRead = bytesRead / sizeof(int16_t);

        // Update HPF coefficients if they changed
        if (highpassEnabled && (localHpfConfigSampleRate != currentSampleRate ||
                                localHpfConfigCutoff != highpassCutoffHz)) {
            localHpf = hpf;  // Copy updated coefficients from main
            localHpfConfigSampleRate = currentSampleRate;
            localHpfConfigCutoff = highpassCutoffHz;
        }

        // Process audio: gain, HPF, clipping detection
        bool clipped = false;
        float peakAbs = 0.0f;

        for (int i = 0; i < samplesRead; i++) {
            float sample = (float)(captureBuffer[i] >> i2sShiftBits);

            if (highpassEnabled) {
                sample = localHpf.process(sample);
            }

            float amplified = sample * currentGainFactor;
            float aabs = fabsf(amplified);
            if (aabs > peakAbs) peakAbs = aabs;
            if (aabs > 32767.0f) clipped = true;

            if (amplified > 32767.0f) amplified = 32767.0f;
            if (amplified < -32768.0f) amplified = -32768.0f;
            outputBuffer[i] = (int16_t)amplified;
        }

        // Update metering
        if (peakAbs > 32767.0f) peakAbs = 32767.0f;
        lastPeakAbs16 = (uint16_t)peakAbs;
        audioClippedLastBlock = clipped;
        if (clipped) {
            audioClipCount++;
            static unsigned long lastClipLog = 0;
            if (millis() - lastClipLog > 5000) {
                Serial.printf("[Core1] Clipping! Peak: %u, count: %lu\n",
                             lastPeakAbs16, audioClipCount);
                lastClipLog = millis();
            }
        }

        if (lastPeakAbs16 > peakHoldAbs16) {
            peakHoldAbs16 = lastPeakAbs16;
            peakHoldUntilMs = millis() + 3000UL;
        } else if (peakHoldAbs16 > 0 && millis() > peakHoldUntilMs) {
            peakHoldAbs16 = 0;
        }

        // Send RTP packet directly via streamClient (no mutex needed)
        sendRTPPacket(*client, outputBuffer, samplesRead);
        packetCount++;

        taskYIELD();
    }

    free(captureBuffer);
    free(outputBuffer);
    simplePrintln("[Core1] Audio pipeline task stopped");
    vTaskDelete(NULL);
}

// Start audio pipeline task on Core 1
void startAudioCaptureTask() {
    if (audioCaptureTaskHandle != NULL) {
        simplePrintln("[Core1] Audio task already running");
        return;
    }

    BaseType_t result = xTaskCreatePinnedToCore(
        audioCaptureTask,           // Task function
        "AudioPipeline",            // Name
        8192,                       // Stack size
        NULL,                       // Parameters
        10,                         // Priority (elevated)
        &audioCaptureTaskHandle,    // Task handle
        1                           // Core 1 (PRO_CPU)
    );

    if (result != pdPASS) {
        simplePrintln("[Core1] FATAL: Failed to create audio pipeline task!");
    }
}

// Stop audio pipeline task
void stopAudioCaptureTask() {
    if (audioCaptureTaskHandle != NULL) {
        audioTaskRunning = false;
        vTaskDelay(pdMS_TO_TICKS(100));  // Give task time to clean up
        audioCaptureTaskHandle = NULL;
        // Task prints its own "stopped" message — don't duplicate here
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
    const unsigned long WRITE_TIMEOUT_MS = 50;  // 50ms timeout - balance between responsiveness and stability

    while (off < len) {
        // Check timeout to prevent blocking Core 1
        if (millis() - startTime > WRITE_TIMEOUT_MS) {
            // Drop frame if WiFi is too slow (normal with poor signal)
            return false;
        }

        int w = client.write(data + off, len - off);
        if (w <= 0) return false;
        off += (size_t)w;
    }
    return true;
}

void sendRTPPacket(WiFiClient &client, int16_t* audioData, int numSamples) {
    if (!client.connected()) {
        // Client disconnected — signal Core 0
        isStreaming = false;
        streamClient = NULL;
        return;
    }

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
        s = (uint16_t)((s << 8) | (s >> 8));
        audioData[i] = (int16_t)s;
    }

    bool success = writeAll(client, inter, sizeof(inter)) &&
                   writeAll(client, header, sizeof(header)) &&
                   writeAll(client, (uint8_t*)audioData, payloadSize);

    if (success) {
        rtpSequence++;
        rtpTimestamp += (uint32_t)numSamples;
        audioPacketsSent++;
    } else {
        // Write failed — client disconnected or too slow
        audioPacketsDropped++;
        // Signal Core 0 to clean up
        isStreaming = false;
        streamClient = NULL;
    }
}

// ================== CORE 0: NO AUDIO STREAMING ==================
// Audio streaming is now handled entirely on Core 1
// Core 0 only manages client connections and RTSP protocol
// (streamAudio function removed - handled by Core 1 audioCaptureTask)

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
        // Send PLAY response FIRST (still on Core 0, Core 1 not started yet)
        client.print("RTSP/1.0 200 OK\r\n");
        client.print("CSeq: " + cseq + "\r\n");
        client.print("Session: " + rtspSessionId + "\r\n");
        client.print("Range: npt=0.000-\r\n\r\n");

        rtpSequence = 0;
        rtpTimestamp = 0;
        audioPacketsSent = 0;
        audioPacketsDropped = 0;
        lastStatsReset = millis();
        lastRtspPlayMs = millis();
        rtspPlayCount++;

        // Hand off client to Core 1 via pointer
        streamClient = &rtspClient;
        isStreaming = true;

        // Start audio capture task
        startAudioCaptureTask();

        M5.dis.drawpix(0, CRGB(128, 0, 128));
        simplePrintln("STREAMING STARTED");

    } else if (request.startsWith("TEARDOWN")) {
        client.print("RTSP/1.0 200 OK\r\n");
        client.print("CSeq: " + cseq + "\r\n");
        client.print("Session: " + rtspSessionId + "\r\n\r\n");

        // Signal Core 1 to stop, then clean up
        isStreaming = false;
        streamClient = NULL;
        stopAudioCaptureTask();

        M5.dis.drawpix(0, CRGB(0, 128, 0));
        simplePrintln("STREAMING STOPPED");
    } else if (request.startsWith("GET_PARAMETER")) {
        // Many RTSP clients send GET_PARAMETER as keep-alive.
        client.print("RTSP/1.0 200 OK\r\n");
        client.print("CSeq: " + cseq + "\r\n\r\n");
    }
}

// RTSP processing (runs on Core 0, only called when !isStreaming — no contention)
void processRTSP(WiFiClient &client) {
    if (!client.connected()) return;

    if (client.available()) {
        int available = client.available();
        int spaceLeft = sizeof(rtspParseBuffer) - rtspParseBufferPos - 1;

        if (available > spaceLeft) {
            available = spaceLeft;
        }

        if (available <= 0) {
            static unsigned long lastOverflowWarning = 0;
            if (millis() - lastOverflowWarning > 5000) {
                simplePrintln("RTSP buffer full - resetting");
                lastOverflowWarning = millis();
            }
            rtspParseBufferPos = 0;
            return;
        }

        client.read(rtspParseBuffer + rtspParseBufferPos, available);
        rtspParseBufferPos += available;
        rtspParseBuffer[rtspParseBufferPos] = '\0';

        char* endOfHeader = strstr((char*)rtspParseBuffer, "\r\n\r\n");
        if (endOfHeader != nullptr) {
            *endOfHeader = '\0';
            String request = String((char*)rtspParseBuffer);

            handleRTSPCommand(client, request);

            int headerLen = (endOfHeader - (char*)rtspParseBuffer) + 4;
            int remaining = rtspParseBufferPos - headerLen;
            if (remaining > 0) {
                memmove(rtspParseBuffer, rtspParseBuffer + headerLen, remaining);
            }
            rtspParseBufferPos = remaining;
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

    // Note: Audio buffers now allocated by Core 1 task (not in main)
    Serial.println("Audio buffers will be allocated by Core 1 pipeline task");

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

    // Audio pipeline task will be created when RTSP streaming begins (on PLAY command)
    Serial.println("Dual-core audio ready (Core 1 pipeline will start on RTSP PLAY)");

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

    // RTSP client management (Core 0) — clear phase separation, no mutex
    if (rtspServerEnabled) {
        // Phase: detect disconnect (Core 1 cleared streamClient)
        if (isStreaming && streamClient == NULL) {
            // Core 1 detected write failure — clean up
            isStreaming = false;
            stopAudioCaptureTask();
            rtspClient.stop();
            M5.dis.drawpix(0, CRGB(0, 128, 0));
            simplePrintln("RTSP client disconnected");
        }

        // Phase: accept new client (only when not streaming)
        if (!isStreaming) {
            if (!rtspClient || !rtspClient.connected()) {
                WiFiClient newClient = rtspServer.available();
                if (newClient) {
                    rtspClient = newClient;
                    rtspClient.setNoDelay(true);
                    rtspParseBufferPos = 0;
                    lastRTSPActivity = millis();
                    lastRtspClientConnectMs = millis();
                    rtspConnectCount++;
                    simplePrintln("New RTSP client connected");
                }
            }

            // Phase: RTSP negotiation (only when not streaming)
            if (rtspClient && rtspClient.connected()) {
                processRTSP(rtspClient);
            }
        }
    } else {
        // RTSP server disabled (overheat lockout)
        if (isStreaming) {
            isStreaming = false;
            streamClient = NULL;
            stopAudioCaptureTask();
        }
        if (rtspClient) {
            rtspClient.stop();
        }
        if (overheatLatched) {
            M5.dis.drawpix(0, CRGB(128, 0, 0));
        } else {
            M5.dis.drawpix(0, CRGB(0, 128, 0));
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
