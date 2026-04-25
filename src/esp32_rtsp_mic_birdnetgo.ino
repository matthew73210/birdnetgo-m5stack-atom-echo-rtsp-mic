#include <WiFi.h>
#include <WiFiManager.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <esp_err.h>
#include "driver/i2s_common.h"
#include "driver/i2s_pdm.h"
#include <Preferences.h>
#include <math.h>
#include <time.h>
#include <errno.h>
#include <lwip/sockets.h>
#include <esp32-hal-rgb-led.h>
#include "PdmProbeResult.h"
#include "WebUI.h"

// ================== DUAL-CORE AUDIO ARCHITECTURE ==================
// Core 1: Complete audio pipeline (I2S → process → RTP → WiFi)
// Core 0: Web UI, diagnostics, RTSP protocol, client management
// PDM microphone outputs 16-bit samples directly

enum RtspTransportMode : uint8_t {
    RTSP_TRANSPORT_TCP_INTERLEAVED = 0,
    RTSP_TRANSPORT_UDP_UNICAST = 1,
};

struct RtspSession {
    WiFiClient client;
    bool occupied = false;
    volatile bool streaming = false;
    bool setupDone = false;
    String sessionId = "";
    String remoteAddr = "";
    RtspTransportMode transport = RTSP_TRANSPORT_TCP_INTERLEAVED;
    uint16_t rtpSequence = 0;
    uint32_t rtpTimestamp = 0;
    uint32_t rtpSSRC = 0;
    unsigned long connectedAt = 0;
    unsigned long lastActivity = 0;
    unsigned long playStartedAt = 0;
    uint8_t parseBuffer[1024] = {0};
    int parseBufferPos = 0;
    char streamingRtspBuf[512] = {0};
    size_t streamingRtspBufPos = 0;
    uint16_t streamingInterleavedDiscard = 0;
    uint32_t consecutiveWriteFailures = 0;
    unsigned long firstWriteFailureAt = 0;
};

static const uint8_t MAX_RTSP_CLIENTS = 2;

// Compatibility/debug pointer: when non-null it points at the first Core-1-owned
// streaming socket. Multi-client state lives in rtspSessions.
WiFiClient* volatile streamClient = NULL;
TaskHandle_t audioCaptureTaskHandle = NULL;
volatile bool audioTaskRunning = false;

// Cross-core synchronization primitives
portMUX_TYPE logMux = portMUX_INITIALIZER_UNLOCKED;  // spinlock for log ring buffer
portMUX_TYPE hpfMux = portMUX_INITIALIZER_UNLOCKED;  // protects HPF coefficient handoff
portMUX_TYPE diagMux = portMUX_INITIALIZER_UNLOCKED; // protects live diagnostics snapshots
volatile bool stopStreamRequested = false;   // Core 0 asks Core 1 to stop
volatile bool streamCleanupDone = false;     // Core 1 confirms cleanup complete
SemaphoreHandle_t taskExitSemaphore = NULL;  // confirmed task exit
volatile bool core1OwnsLED = false;          // LED ownership flag

// ================== SETTINGS (ESP32 RTSP Mic for BirdNET-Go) ==================
#define FW_VERSION "4.1.0"
// Expose FW version as a global C string for WebUI/API
const char* FW_VERSION_STR = FW_VERSION;
static const uint16_t OTA_PORT = 3232;
static const char SETTINGS_NAMESPACE[] = "audioPrefs";
static const char LEGACY_SETTINGS_NAMESPACE[] = "audio";
static const char LOG_TIMEZONE_POSIX[] = "UTC0";
static const char LOG_TIMEZONE_LABEL[] = "UTC";

// -- DEFAULT PARAMETERS (configurable via Web UI / API)
#define DEFAULT_SAMPLE_RATE 48000  // Unit Mini PDM / BirdNET-Go preferred rate
#define DEFAULT_GAIN_FACTOR 1.0f
#define DEFAULT_BUFFER_SIZE 1024   // 64ms @ 16kHz - lower default latency while keeping WiFi headroom
#define DEFAULT_WIFI_TX_DBM 19.5f  // Default WiFi TX power in dBm
#define DEFAULT_NETWORK_HOSTNAME "atoms3mic"
static const uint32_t SUPPORTED_PDM_SAMPLE_RATES[] = {16000, 24000, 32000, 48000};
// High-pass filter defaults. V4's settled PDM feed carries low-frequency/DC
// wander, so keep HPF on and let AGC/noise suppression stay opt-in.
#define DEFAULT_HPF_ENABLED true
#define DEFAULT_HPF_CUTOFF_HZ 180

// Thermal protection defaults
#define DEFAULT_OVERHEAT_PROTECTION true
#define DEFAULT_OVERHEAT_LIMIT_C 80
#define OVERHEAT_MIN_LIMIT_C 30
#define OVERHEAT_MAX_LIMIT_C 95
#define OVERHEAT_LIMIT_STEP_C 5

// -- Pins (AtomS3 Lite + Unit Mini PDM)
// M5's reference wiring for the PDM Unit is CLK=G1 and DATA=G2.
#define I2S_CLK_PIN      1  // PDM CLK (G1)
#define I2S_DATA_IN_PIN  2  // PDM DATA (G2)
#define WS2812_LED_PIN  35  // AtomS3 Lite built-in RGB LED

struct CRGB {
    uint8_t r;
    uint8_t g;
    uint8_t b;

    constexpr CRGB(uint8_t red = 0, uint8_t green = 0, uint8_t blue = 0)
        : r(red), g(green), b(blue) {}
};

static bool statusLedNeedsRefresh = true;
static CRGB lastStatusLed = CRGB(0, 0, 0);
static uint8_t statusLedBrightness = 32;

static uint8_t scaleLedChannel(uint8_t channel) {
    return (uint8_t)(((uint16_t)channel * (uint16_t)statusLedBrightness + 127U) / 255U);
}

inline void setStatusLed(const CRGB& color) {
    if (!statusLedNeedsRefresh &&
        lastStatusLed.r == color.r &&
        lastStatusLed.g == color.g &&
        lastStatusLed.b == color.b) {
        return;
    }
    rgbLedWriteOrdered(WS2812_LED_PIN, LED_COLOR_ORDER_GRB,
                       scaleLedChannel(color.r),
                       scaleLedChannel(color.g),
                       scaleLedChannel(color.b));
    lastStatusLed = color;
    statusLedNeedsRefresh = false;
}

// -- Servers
WiFiServer rtspServer(8554);
String networkHostname = DEFAULT_NETWORK_HOSTNAME;
bool otaPreviousRtspEnabled = true;

// -- RTSP Streaming
volatile bool isStreaming = false;
unsigned long lastRTSPActivity = 0;
static const bool RTSP_ENABLE_UDP_UNICAST = false;
static RtspSession rtspSessions[MAX_RTSP_CLIENTS];

static int parseRtspCSeqValue(const char* request);
static uint16_t parseRtspContentLengthValue(const char* request);
static bool writeRtspSimpleResponse(RtspSession &session, int cseqVal, const char* extraHeaders = NULL);
static bool handleStreamingRtspCommand(RtspSession &session, const char* request);
static void pollStreamingRtspCommands(RtspSession &session);
static void closeRtspSession(RtspSession &session, bool stopClient);
static void updateStreamingStateFromSessions();
static uint8_t countRtspClients();
static uint8_t countStreamingRtspClients();
static RtspSession* firstStreamingSession();
static void clearAudioDiagnostics();
static void sendRTPPacket(RtspSession &session, int16_t* audioData, int numSamples, uint8_t activeClientCount);
static void sendRTPPacketsToActiveSessions(int16_t* audioData, int numSamples);
static esp_err_t i2sMicRead(void* dest, size_t size, size_t* bytesRead, uint32_t timeoutMs);
// Note: Audio buffers now allocated by Core 1 task

// -- Global state
unsigned long audioPacketsSent = 0;
unsigned long audioPacketsDropped = 0;  // Track dropped frames
unsigned long audioBlocksSent = 0;      // Stream cadence counter, independent of client fan-out
unsigned long lastStatsReset = 0;
bool rtspServerEnabled = true;

bool isSupportedPdmSampleRate(uint32_t rate) {
    for (size_t i = 0; i < (sizeof(SUPPORTED_PDM_SAMPLE_RATES) / sizeof(SUPPORTED_PDM_SAMPLE_RATES[0])); ++i) {
        if (SUPPORTED_PDM_SAMPLE_RATES[i] == rate) return true;
    }
    return false;
}

static uint32_t sanitizePdmSampleRate(uint32_t rate) {
    return isSupportedPdmSampleRate(rate) ? rate : (uint32_t)DEFAULT_SAMPLE_RATE;
}

// -- Audio parameters (runtime configurable)
uint32_t currentSampleRate = DEFAULT_SAMPLE_RATE;
float currentGainFactor = DEFAULT_GAIN_FACTOR;
uint16_t currentBufferSize = DEFAULT_BUFFER_SIZE;
// PDM microphones output decimated samples directly
// The hardware PDM decimation produces samples already close to correct range
// Typical range: 0-3 for PDM vs 11-13 for I2S microphones
uint8_t i2sShiftBits = 0;  // Fixed for Unit Mini PDM capture

// -- Audio metering / clipping diagnostics
uint16_t lastPeakAbs16 = 0;       // last block peak absolute value (0..32767)
uint32_t audioClipCount = 0;      // total blocks where clipping occurred
bool audioClippedLastBlock = false; // clipping occurred in last processed block
uint16_t peakHoldAbs16 = 0;       // peak hold (recent window)
unsigned long peakHoldUntilMs = 0; // when to clear hold

// -- I2S raw capture diagnostics (helps verify PDM communication/wiring)
volatile uint32_t i2sReadOkCount = 0;
volatile uint32_t i2sReadErrCount = 0;
volatile uint32_t i2sReadZeroCount = 0;
volatile uint16_t i2sLastSamplesRead = 0;
volatile int16_t i2sLastRawMin = 0;
volatile int16_t i2sLastRawMax = 0;
volatile uint16_t i2sLastRawPeakAbs = 0;
volatile uint16_t i2sLastRawRms = 0;
volatile uint16_t i2sLastRawZeroPct = 0;
volatile int16_t i2sLastRawMean = 0;
volatile uint16_t i2sLastRawSpan = 0;
volatile bool i2sLikelyUnsignedPcm = false;
volatile bool i2sUsingModernPdmDriver = false;
volatile bool i2sPdmUseLeftSlot = false;
volatile bool i2sPdmClkInverted = false;
volatile bool i2sPdmProbeFoundSigned = false;
volatile bool i2sDriverOk = false;
volatile int32_t i2sLastError = ESP_OK;
static i2s_chan_handle_t i2sRxChannel = NULL;
volatile uint32_t audioFallbackBlockCount = 0;
volatile uint32_t i2sLastGapMs = 0;
volatile float audioPipelineLoadPct = 0.0f;

// -- Lightweight spectrum diagnostics for Web UI waterfall
portMUX_TYPE fftMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint8_t fftBins[WEBUI_FFT_BINS] = {0};
volatile uint32_t fftFrameSeq = 0;

// -- Browser WebAudio diagnostics stream (latest processed PCM block)
portMUX_TYPE webAudioMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t webAudioFrameSeq = 0;
volatile uint16_t webAudioFrameSamples[WEBUI_AUDIO_RING_LEN] = {0};
volatile uint32_t webAudioSampleRate[WEBUI_AUDIO_RING_LEN] = {0};
volatile uint32_t webAudioRingSeq[WEBUI_AUDIO_RING_LEN] = {0};
int16_t webAudioFrame[WEBUI_AUDIO_RING_LEN][WEBUI_AUDIO_MAX_SAMPLES] = {{0}};

// -- Lightweight telemetry history for Web UI graphs
portMUX_TYPE telemetryMux = portMUX_INITIALIZER_UNLOCKED;
static const uint16_t TELEMETRY_HISTORY_LEN = 120;
volatile uint32_t telemetryHistorySeq = 0;
uint8_t telemetryCpuLoadPct[TELEMETRY_HISTORY_LEN] = {0};
int16_t telemetryTempDeciC[TELEMETRY_HISTORY_LEN] = {0};
uint16_t telemetryHistoryHead = 0;
uint16_t telemetryHistoryCount = 0;

// -- LED mode: 0=off, 1=static, 2=level
uint8_t ledMode = 1;  // Default: static purple during streaming

// -- Automatic Gain Control (AGC)
bool agcEnabled = false;
volatile float agcMultiplier = 1.0f;   // Current AGC multiplier (read by WebUI)
const float AGC_TARGET_RMS = 0.055f;   // Moderate target RMS (~-25 dBFS) so AGC does not lift mic hiss too hard
const float AGC_MIN_MULT = 0.1f;
const float AGC_MAX_MULT = 3.5f;       // Keep AGC from driving persistent background noise too hard
const float AGC_ATTACK_RATE = 0.05f;   // Fast attack (gain reduction) per buffer
const float AGC_RELEASE_RATE = 0.001f; // Slow release (gain increase) per buffer

// -- Adaptive background-noise filter (auto gate/bed suppression)
// Tuned to avoid the old "tap tap tap" artifact caused by per-sample gain pumping.
volatile bool noiseFilterEnabled = false;
volatile float noiseFloorDbfs = -90.0f;
volatile float noiseGateDbfs = -90.0f;
volatile float noiseReductionDb = 0.0f;
const float NOISE_FLOOR_INIT = 300.0f;
const float NOISE_FLOOR_FAST = 0.015f;
const float NOISE_FLOOR_SLOW = 0.0008f;
const float NOISE_GATE_RATIO = 2.6f;
const float NOISE_GATE_MARGIN = 220.0f;
const float NOISE_GATE_MIN = 220.0f;
const float NOISE_GATE_MAX = 5200.0f;
const float NOISE_GATE_CLOSE_RATIO = 0.72f;
const float NOISE_FILTER_MIN_GAIN = 0.70f;
const float NOISE_FILTER_ATTACK = 0.006f;
const float NOISE_FILTER_RELEASE = 0.0008f;
const float NOISE_FILTER_REOPEN = 0.0035f;
const float NOISE_ENV_ATTACK = 0.010f;
const float NOISE_ENV_RELEASE = 0.0012f;
const uint16_t NOISE_GATE_HOLD_MS = 120;
const uint16_t PDM_UNSIGNED_MAX_CODE = 4095;
const uint16_t PDM_UNSIGNED_MIN_SPAN = 64;
const int16_t PDM_UNSIGNED_EDGE_MARGIN = 24;
const float PDM_UNSIGNED_MIN_MEAN = 256.0f;
const float PDM_UNSIGNED_MIN_MEAN_TO_SPAN = 4.0f;
const float PDM_UNSIGNED_LOW_CENTER_MAX = 1536.0f;
const float PDM_UNSIGNED_LOW_NORMALIZE_GAIN = 32.0f;
const float PDM_UNSIGNED_HIGH_NORMALIZE_GAIN = 16.0f;
const float PDM_UNSIGNED_DC_TRACK_SECONDS = 1.25f;
const uint8_t PDM_UNSIGNED_CONFIDENCE_MAX = 12;
const uint8_t PDM_UNSIGNED_CONFIDENCE_ON = 6;
const uint8_t PDM_UNSIGNED_CONFIDENCE_OFF = 2;
const uint16_t PDM_PROBE_SETTLE_MS = 650;
const uint8_t PDM_PROBE_MEASURE_READS = 5;
const char* DEVICE_TITLE = "M5 Atom RTSP Microphone";
const char* DEVICE_INPUT_PROFILE = "PDM input profile (AtomS3 Lite + Unit Mini PDM tuned)";
const char* FILTER_CHAIN_BASE = "PDM PCM -> stable DC normalize when needed -> 2nd-order Butterworth high-pass -> optional noise bed suppressor -> manual gain -> limiter -> optional AGC";

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
unsigned long lastTelemetrySampleMs = 0;
uint32_t minFreeHeap = 0xFFFFFFFF;
uint32_t maxPacketRate = 0;
uint32_t minPacketRate = 0xFFFFFFFF;
bool autoRecoveryEnabled = false;
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
uint8_t cpuFrequencyMhz = 160;          // CPU frequency (default 160 MHz — sufficient for this workload)

// -- WiFi TX power (configurable)
float wifiTxPowerDbm = DEFAULT_WIFI_TX_DBM;
wifi_power_t currentWifiPowerLevel = WIFI_POWER_19_5dBm;
bool wifiTxPowerApplied = false;

// -- RTSP connect/PLAY statistics
unsigned long lastRtspClientConnectMs = 0;
unsigned long lastRtspPlayMs = 0;
uint32_t rtspConnectCount = 0;
uint32_t rtspPlayCount = 0;
uint32_t rtspRejectedClientCount = 0;

// ===============================================

static const char* rtspTransportModeName(RtspTransportMode mode) {
    switch (mode) {
        case RTSP_TRANSPORT_UDP_UNICAST: return "udp";
        case RTSP_TRANSPORT_TCP_INTERLEAVED:
        default: return "tcp";
    }
}

const char* currentRtspTransportName() {
    RtspSession* session = firstStreamingSession();
    return rtspTransportModeName(session ? session->transport : RTSP_TRANSPORT_TCP_INTERLEAVED);
}

static uint8_t countRtspClients() {
    uint8_t count = 0;
    for (uint8_t i = 0; i < MAX_RTSP_CLIENTS; ++i) {
        if (rtspSessions[i].occupied) count++;
    }
    return count;
}

static uint8_t countStreamingRtspClients() {
    uint8_t count = 0;
    for (uint8_t i = 0; i < MAX_RTSP_CLIENTS; ++i) {
        if (rtspSessions[i].occupied && rtspSessions[i].streaming) count++;
    }
    return count;
}

static RtspSession* firstStreamingSession() {
    for (uint8_t i = 0; i < MAX_RTSP_CLIENTS; ++i) {
        if (rtspSessions[i].occupied && rtspSessions[i].streaming) return &rtspSessions[i];
    }
    return NULL;
}

static int findFreeRtspSession() {
    for (uint8_t i = 0; i < MAX_RTSP_CLIENTS; ++i) {
        if (!rtspSessions[i].occupied) return i;
    }
    return -1;
}

static void updateStreamingStateFromSessions() {
    RtspSession* first = firstStreamingSession();
    streamClient = first ? &first->client : NULL;
    isStreaming = (first != NULL);
    core1OwnsLED = isStreaming;
}

static void closeRtspSession(RtspSession &session, bool stopClient) {
    if (stopClient && session.client) {
        session.client.stop();
    }
    session.occupied = false;
    session.streaming = false;
    session.setupDone = false;
    session.sessionId = "";
    session.remoteAddr = "";
    session.transport = RTSP_TRANSPORT_TCP_INTERLEAVED;
    session.rtpSequence = 0;
    session.rtpTimestamp = 0;
    session.rtpSSRC = 0;
    session.connectedAt = 0;
    session.lastActivity = 0;
    session.playStartedAt = 0;
    session.parseBufferPos = 0;
    session.streamingRtspBufPos = 0;
    session.streamingInterleavedDiscard = 0;
    session.consecutiveWriteFailures = 0;
    session.firstWriteFailureAt = 0;
    updateStreamingStateFromSessions();
}

uint8_t getConnectedRtspClientCount() {
    return countRtspClients();
}

uint8_t getActiveRtspClientCount() {
    return countStreamingRtspClients();
}

uint8_t getMaxRtspClientCount() {
    return MAX_RTSP_CLIENTS;
}

String getRtspClientSummary() {
    String out;
    for (uint8_t i = 0; i < MAX_RTSP_CLIENTS; ++i) {
        RtspSession &session = rtspSessions[i];
        if (!session.occupied) continue;
        if (out.length() > 0) out += ", ";
        out += session.remoteAddr.length() ? session.remoteAddr : String("client");
        out += session.streaming ? " (play)" : " (setup)";
    }
    return out;
}

static String extractRtspHeader(const String &request, const char* headerName) {
    String needle = String("\r\n") + headerName;
    int start = request.indexOf(needle);
    if (start < 0 && request.startsWith(String(headerName))) {
        start = 0;
    } else if (start >= 0) {
        start += 2;
    }
    if (start < 0) return String("");
    int colon = request.indexOf(':', start);
    if (colon < 0) return String("");
    int end = request.indexOf("\r\n", colon);
    if (end < 0) end = request.length();
    String value = request.substring(colon + 1, end);
    value.trim();
    return value;
}

static bool rtspMethodIs(const char* request, const char* method) {
    if (!request || !method) return false;
    size_t len = strlen(method);
    return strncmp(request, method, len) == 0 && (request[len] == ' ' || request[len] == '\t');
}

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
    if (!wifiTxPowerApplied || desired != currentWifiPowerLevel) {
        WiFi.setTxPower(desired);
        currentWifiPowerLevel = desired;
        wifiTxPowerApplied = true;
        if (log) {
            simplePrintln("WiFi TX power set to " + String(wifiPowerLevelToDbm(currentWifiPowerLevel), 1) + " dBm");
        }
    }
}

String describeHardwareProfile() {
    return String(DEVICE_TITLE) + " / " + DEVICE_INPUT_PROFILE;
}

String describeFilterChain() {
    String chain = FILTER_CHAIN_BASE;
    chain += highpassEnabled
        ? (String(" (HPF ON @ ") + String(highpassCutoffHz) + " Hz)")
        : String(" (HPF bypassed)");
    chain += noiseFilterEnabled
        ? String(", noise suppressor active")
        : String(", noise suppressor bypassed");
    chain += agcEnabled
        ? String(", AGC active")
        : String(", AGC bypassed");
    return chain;
}

static uint16_t audioDeClickSamples(uint16_t samples) {
    if (samples == 0) return 0;
    uint32_t ramp = currentSampleRate / 250U; // 4 ms edge smoothing
    if (ramp < 16U) ramp = 16U;
    if (ramp > 192U) ramp = 192U;
    if (ramp > samples) ramp = samples;
    return (uint16_t)ramp;
}

static void fillConcealmentBlock(int16_t* buffer, uint16_t samples, int16_t lastSample) {
    if (!buffer || samples == 0) return;
    uint16_t rampSamples = audioDeClickSamples(samples);
    uint32_t rampDen = (uint32_t)rampSamples * (uint32_t)rampSamples;
    if (rampDen == 0) rampDen = 1;

    for (uint16_t i = 0; i < samples; ++i) {
        if (i < rampSamples) {
            uint32_t remain = (uint32_t)(rampSamples - i);
            int64_t shaped = (int64_t)lastSample * (int64_t)remain * (int64_t)remain;
            buffer[i] = (int16_t)(shaped / (int64_t)rampDen);
        } else {
            buffer[i] = 0;
        }
    }
}

static void blendBlockStartFrom(int16_t* buffer, uint16_t samples, int16_t previousSample) {
    if (!buffer || samples == 0) return;
    uint16_t rampSamples = audioDeClickSamples(samples);
    if (rampSamples <= 1) return;

    for (uint16_t i = 0; i < rampSamples; ++i) {
        int32_t target = buffer[i];
        int32_t alpha = (int32_t)(i + 1);
        int32_t blended = ((int32_t)previousSample * ((int32_t)rampSamples - alpha) +
                           target * alpha) / (int32_t)rampSamples;
        buffer[i] = (int16_t)blended;
    }
}

static void publishWebAudioFrame(const int16_t* samples, uint16_t count) {
    if (!samples || count == 0) return;

    uint16_t publishSamples = count;
    if (publishSamples > WEBUI_AUDIO_MAX_SAMPLES) publishSamples = WEBUI_AUDIO_MAX_SAMPLES;

    portENTER_CRITICAL(&webAudioMux);
    uint32_t nextWebAudioSeq = webAudioFrameSeq + 1U;
    uint8_t webAudioSlot = (uint8_t)((nextWebAudioSeq - 1U) % WEBUI_AUDIO_RING_LEN);
    memcpy(webAudioFrame[webAudioSlot], samples, publishSamples * sizeof(int16_t));
    webAudioFrameSamples[webAudioSlot] = publishSamples;
    webAudioSampleRate[webAudioSlot] = currentSampleRate;
    webAudioRingSeq[webAudioSlot] = nextWebAudioSeq;
    webAudioFrameSeq = nextWebAudioSeq;
    portEXIT_CRITICAL(&webAudioMux);
}

static void clearAudioDiagnostics() {
    portENTER_CRITICAL(&webAudioMux);
    webAudioFrameSeq = 0;
    memset((void*)webAudioFrameSamples, 0, sizeof(webAudioFrameSamples));
    memset((void*)webAudioSampleRate, 0, sizeof(webAudioSampleRate));
    memset((void*)webAudioRingSeq, 0, sizeof(webAudioRingSeq));
    memset(webAudioFrame, 0, sizeof(webAudioFrame));
    portEXIT_CRITICAL(&webAudioMux);

    portENTER_CRITICAL(&fftMux);
    memset((void*)fftBins, 0, sizeof(fftBins));
    fftFrameSeq = 0;
    portEXIT_CRITICAL(&fftMux);
    portENTER_CRITICAL(&diagMux);
    audioClipCount = 0;
    audioFallbackBlockCount = 0;
    lastPeakAbs16 = 0;
    peakHoldAbs16 = 0;
    peakHoldUntilMs = 0;
    audioClippedLastBlock = false;
    i2sLastSamplesRead = 0;
    i2sLastRawMin = 0;
    i2sLastRawMax = 0;
    i2sLastRawPeakAbs = 0;
    i2sLastRawRms = 0;
    i2sLastRawZeroPct = 0;
    i2sLastRawMean = 0;
    i2sLastRawSpan = 0;
    i2sLikelyUnsignedPcm = false;
    audioPipelineLoadPct = 0.0f;
    agcMultiplier = 1.0f;
    noiseFloorDbfs = -90.0f;
    noiseGateDbfs = -90.0f;
    noiseReductionDb = 0.0f;
    i2sLastGapMs = 0;
    portEXIT_CRITICAL(&diagMux);
}

void recordTelemetrySample(float cpuLoadPct, float tempC, bool tempValid) {
    if (cpuLoadPct < 0.0f) cpuLoadPct = 0.0f;
    if (cpuLoadPct > 100.0f) cpuLoadPct = 100.0f;
    int16_t tempDeci = tempValid ? (int16_t)lroundf(tempC * 10.0f) : INT16_MIN;

    portENTER_CRITICAL(&telemetryMux);
    telemetryCpuLoadPct[telemetryHistoryHead] = (uint8_t)lroundf(cpuLoadPct);
    telemetryTempDeciC[telemetryHistoryHead] = tempDeci;
    telemetryHistoryHead = (telemetryHistoryHead + 1) % TELEMETRY_HISTORY_LEN;
    if (telemetryHistoryCount < TELEMETRY_HISTORY_LEN) telemetryHistoryCount++;
    telemetryHistorySeq = telemetryHistorySeq + 1U;
    portEXIT_CRITICAL(&telemetryMux);
}

// Recompute HPF coefficients (2nd-order Butterworth high-pass)
void updateHighpassCoeffs() {
    bool enabled = highpassEnabled;
    uint32_t sampleRate = currentSampleRate;
    uint16_t cutoffHz = highpassCutoffHz;
    Biquad nextHpf;

    if (enabled) {
        float fs = (float)sampleRate;
        float fc = (float)cutoffHz;
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

        nextHpf.b0 = b0 / a0;
        nextHpf.b1 = b1 / a0;
        nextHpf.b2 = b2 / a0;
        nextHpf.a1 = a1 / a0;
        nextHpf.a2 = a2 / a0;
        nextHpf.reset();
    }

    portENTER_CRITICAL(&hpfMux);
    hpf = nextHpf;
    hpfConfigSampleRate = sampleRate;
    hpfConfigCutoff = cutoffHz;
    portEXIT_CRITICAL(&hpfMux);
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
    audioPrefs.begin(SETTINGS_NAMESPACE, false);
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
            // Request Core 1 to stop streaming safely
            if (isStreaming) {
                requestStreamStop("overheat");
            }
            stopAudioCaptureTask();
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
        // Cooldown: skip performance check for 2 minutes after last I2S reset
        if (lastI2SReset > 0 && (millis() - lastI2SReset) < 120000) {
            return;
        }

        uint32_t runtime = millis() - lastStatsReset;
        uint32_t currentRate = (audioBlocksSent * 1000) / runtime;

        if (currentRate > maxPacketRate) maxPacketRate = currentRate;
        if (currentRate < minPacketRate) minPacketRate = currentRate;

        static uint8_t consecutiveLowCount = 0;
        if (currentRate < minAcceptableRate) {
            consecutiveLowCount++;
            simplePrintln("Low packet rate: " + String(currentRate) + " < " + String(minAcceptableRate) + " pkt/s (" + String(consecutiveLowCount) + "/3)");

            if (consecutiveLowCount >= 3 && autoRecoveryEnabled) {
                simplePrintln("AUTO-RECOVERY: 3 consecutive failures, restarting I2S...");
                consecutiveLowCount = 0;
                restartI2S();
                audioPacketsSent = 0;
                audioBlocksSent = 0;
                lastStatsReset = millis();
                lastI2SReset = millis();
            }
        } else {
            consecutiveLowCount = 0;
        }
    }
}

// WiFi health check
void checkWiFiHealth() {
    static bool prevConnected = false;
    static IPAddress prevIp(0, 0, 0, 0);

    bool connected = (WiFi.status() == WL_CONNECTED);
    if (!connected) {
        // Stop streaming before reconnecting — no point sending RTP into a dead radio
        if (isStreaming) {
            requestStreamStop("WiFi disconnect");
            stopAudioCaptureTask();
        }
        simplePrintln("WiFi disconnected! Reconnecting...");
        WiFi.reconnect();
    } else {
        IPAddress ip = WiFi.localIP();
        // On reconnect/IP change, make sure RTSP server socket is listening on new interface.
        if (!prevConnected || ip != prevIp) {
            simplePrintln("WiFi up: IP=" + ip.toString() + " GW=" + WiFi.gatewayIP().toString());
            if (rtspServerEnabled) {
                rtspServer.stop();
                delay(20);
                rtspServer.begin();
                rtspServer.setNoDelay(true);
                simplePrintln("RTSP server rebound on :8554");
            }
        }
        prevIp = ip;
    }

    prevConnected = connected;

    // Re-apply TX power WITHOUT logging (prevent periodic log spam)
    applyWifiTxPower(false);

    if (connected) {
        int32_t rssi = WiFi.RSSI();
        if (rssi < -85) {
            simplePrintln("WARNING: Weak WiFi signal: " + String(rssi) + " dBm");
        }
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

static bool settingsNamespaceHasData(const char* ns) {
    Preferences prefs;
    if (!prefs.begin(ns, true)) return false;
    bool hasData =
        prefs.isKey("sampleRate") ||
        prefs.isKey("gainFactor") ||
        prefs.isKey("bufferSize") ||
        prefs.isKey("wifiTxDbm") ||
        prefs.isKey("ohEnable");
    prefs.end();
    return hasData;
}

static void migrateLegacySettingsIfNeeded() {
    if (settingsNamespaceHasData(SETTINGS_NAMESPACE) || !settingsNamespaceHasData(LEGACY_SETTINGS_NAMESPACE)) {
        return;
    }

    Preferences legacyPrefs;
    Preferences newPrefs;
    if (!legacyPrefs.begin(LEGACY_SETTINGS_NAMESPACE, true)) return;
    if (!newPrefs.begin(SETTINGS_NAMESPACE, false)) {
        legacyPrefs.end();
        return;
    }

    if (legacyPrefs.isKey("sampleRate")) newPrefs.putUInt("sampleRate", legacyPrefs.getUInt("sampleRate"));
    if (legacyPrefs.isKey("gainFactor")) newPrefs.putFloat("gainFactor", legacyPrefs.getFloat("gainFactor"));
    if (legacyPrefs.isKey("bufferSize")) newPrefs.putUShort("bufferSize", legacyPrefs.getUShort("bufferSize"));
    if (legacyPrefs.isKey("shiftBits")) newPrefs.putUChar("shiftBits", legacyPrefs.getUChar("shiftBits"));
    if (legacyPrefs.isKey("autoRecovery")) newPrefs.putBool("autoRecovery", legacyPrefs.getBool("autoRecovery"));
    if (legacyPrefs.isKey("schedReset")) newPrefs.putBool("schedReset", legacyPrefs.getBool("schedReset"));
    if (legacyPrefs.isKey("resetHours")) newPrefs.putUInt("resetHours", legacyPrefs.getUInt("resetHours"));
    if (legacyPrefs.isKey("minRate")) newPrefs.putUInt("minRate", legacyPrefs.getUInt("minRate"));
    if (legacyPrefs.isKey("checkInterval")) newPrefs.putUInt("checkInterval", legacyPrefs.getUInt("checkInterval"));
    if (legacyPrefs.isKey("thrAuto")) newPrefs.putBool("thrAuto", legacyPrefs.getBool("thrAuto"));
    if (legacyPrefs.isKey("cpuFreq")) newPrefs.putUChar("cpuFreq", legacyPrefs.getUChar("cpuFreq"));
    if (legacyPrefs.isKey("wifiTxDbm")) newPrefs.putFloat("wifiTxDbm", legacyPrefs.getFloat("wifiTxDbm"));
    if (legacyPrefs.isKey("hpEnable")) newPrefs.putBool("hpEnable", legacyPrefs.getBool("hpEnable"));
    if (legacyPrefs.isKey("hpCutoff")) newPrefs.putUInt("hpCutoff", legacyPrefs.getUInt("hpCutoff"));
    if (legacyPrefs.isKey("agcEnable")) newPrefs.putBool("agcEnable", legacyPrefs.getBool("agcEnable"));
    if (legacyPrefs.isKey("noiseFilter")) newPrefs.putBool("noiseFilter", legacyPrefs.getBool("noiseFilter"));
    if (legacyPrefs.isKey("ledMode")) newPrefs.putUChar("ledMode", legacyPrefs.getUChar("ledMode"));
    if (legacyPrefs.isKey("ohEnable")) newPrefs.putBool("ohEnable", legacyPrefs.getBool("ohEnable"));
    if (legacyPrefs.isKey("ohThresh")) newPrefs.putUInt("ohThresh", legacyPrefs.getUInt("ohThresh"));
    if (legacyPrefs.isKey("ohReason")) newPrefs.putString("ohReason", legacyPrefs.getString("ohReason"));
    if (legacyPrefs.isKey("ohStamp")) newPrefs.putString("ohStamp", legacyPrefs.getString("ohStamp"));
    if (legacyPrefs.isKey("ohTripC")) newPrefs.putFloat("ohTripC", legacyPrefs.getFloat("ohTripC"));
    if (legacyPrefs.isKey("ohLatched")) newPrefs.putBool("ohLatched", legacyPrefs.getBool("ohLatched"));

    newPrefs.end();
    legacyPrefs.end();
    Serial.println("[Settings] Migrated Preferences namespace from 'audio' to 'audioPrefs'");
}

static float sanitizeBoundedFloat(float value, float fallback, float minValue, float maxValue) {
    if (!isfinite(value)) return fallback;
    if (value < minValue || value > maxValue) return fallback;
    return value;
}

static uint32_t sanitizeBoundedUInt(uint32_t value, uint32_t fallback, uint32_t minValue, uint32_t maxValue) {
    if (value < minValue || value > maxValue) return fallback;
    return value;
}

static uint16_t sanitizeBufferSize(uint16_t value) {
    return (value >= 256U && value <= 8192U) ? value : (uint16_t)DEFAULT_BUFFER_SIZE;
}

static uint8_t sanitizeCpuFrequencySetting(uint8_t value) {
    switch (value) {
        case 80:
        case 120:
        case 160:
        case 240:
            return value;
        default:
            return 160;
    }
}

static float sanitizeWifiTxPowerSetting(float dbm) {
    dbm = sanitizeBoundedFloat(dbm, DEFAULT_WIFI_TX_DBM, -1.0f, 19.5f);
    return wifiPowerLevelToDbm(pickWifiPowerLevel(dbm));
}

uint16_t maxHighpassCutoffForSampleRate(uint32_t sampleRate) {
    uint32_t maxCutoff = (sampleRate * 45UL) / 100UL;
    if (maxCutoff < 10UL) maxCutoff = 10UL;
    if (maxCutoff > 10000UL) maxCutoff = 10000UL;
    return (uint16_t)maxCutoff;
}

uint16_t sanitizeHighpassCutoffSetting(uint16_t value, uint32_t sampleRate) {
    uint16_t maxCutoff = maxHighpassCutoffForSampleRate(sampleRate);
    if (value < 10U || value > maxCutoff) {
        uint16_t fallback = DEFAULT_HPF_CUTOFF_HZ;
        return (fallback <= maxCutoff) ? fallback : (uint16_t)maxCutoff;
    }
    return value;
}

static uint8_t sanitizeLedModeSetting(uint8_t value) {
    return (value <= 2U) ? value : 1U;
}

// Load settings from flash
void loadAudioSettings() {
    migrateLegacySettingsIfNeeded();
    audioPrefs.begin(SETTINGS_NAMESPACE, false);
    bool hasSampleRatePref = audioPrefs.isKey("sampleRate");
    bool hasGainPref = audioPrefs.isKey("gainFactor");
    bool hasBufferPref = audioPrefs.isKey("bufferSize");
    bool hasHpCutoffPref = audioPrefs.isKey("hpCutoff");
    uint32_t storedSampleRate = audioPrefs.getUInt("sampleRate", DEFAULT_SAMPLE_RATE);
    currentSampleRate = sanitizePdmSampleRate(storedSampleRate);
    currentGainFactor = sanitizeBoundedFloat(
        audioPrefs.getFloat("gainFactor", DEFAULT_GAIN_FACTOR),
        DEFAULT_GAIN_FACTOR,
        0.1f,
        100.0f
    );
    currentBufferSize = sanitizeBufferSize(audioPrefs.getUShort("bufferSize", DEFAULT_BUFFER_SIZE));
    // i2sShiftBits is ALWAYS 0 for PDM microphones - not configurable
    i2sShiftBits = 0;
    autoRecoveryEnabled = audioPrefs.getBool("autoRecovery", false);
    scheduledResetEnabled = audioPrefs.getBool("schedReset", false);
    resetIntervalHours = sanitizeBoundedUInt(audioPrefs.getUInt("resetHours", 24), 24, 1, 168);
    minAcceptableRate = sanitizeBoundedUInt(audioPrefs.getUInt("minRate", 50), 50, 5, 200);
    performanceCheckInterval = sanitizeBoundedUInt(audioPrefs.getUInt("checkInterval", 15), 15, 1, 60);
    autoThresholdEnabled = audioPrefs.getBool("thrAuto", true);
    cpuFrequencyMhz = sanitizeCpuFrequencySetting(audioPrefs.getUChar("cpuFreq", 160));
    wifiTxPowerDbm = sanitizeWifiTxPowerSetting(audioPrefs.getFloat("wifiTxDbm", DEFAULT_WIFI_TX_DBM));
    highpassEnabled = audioPrefs.getBool("hpEnable", DEFAULT_HPF_ENABLED);
    highpassCutoffHz = sanitizeHighpassCutoffSetting(
        (uint16_t)audioPrefs.getUInt("hpCutoff", DEFAULT_HPF_CUTOFF_HZ),
        currentSampleRate
    );
    agcEnabled = audioPrefs.getBool("agcEnable", false);
    noiseFilterEnabled = audioPrefs.getBool("noiseFilter", false);
    if (!hasSampleRatePref) currentSampleRate = DEFAULT_SAMPLE_RATE;
    if (!hasGainPref) currentGainFactor = DEFAULT_GAIN_FACTOR;
    if (!hasBufferPref) currentBufferSize = DEFAULT_BUFFER_SIZE;
    if (!hasHpCutoffPref) highpassCutoffHz = DEFAULT_HPF_CUTOFF_HZ;
    ledMode = sanitizeLedModeSetting(audioPrefs.getUChar("ledMode", 1));
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

    if (hasSampleRatePref && currentSampleRate != storedSampleRate) {
        simplePrintln("Unsupported stored sample rate " + String(storedSampleRate) +
                      " Hz; using " + String(currentSampleRate) + " Hz");
    }

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
                  " (chunk " + String(effectiveAudioChunkSize()) + ")" +
                  ", WiFiTX=" + String(txShown, 1) + "dBm" +
                  ", shiftBits=" + String(i2sShiftBits) +
                  ", HPF=" + String(highpassEnabled?"on":"off") +
                  ", HPFcut=" + String(highpassCutoffHz) + "Hz");
}

// Save settings to flash
void saveAudioSettings() {
    audioPrefs.begin(SETTINGS_NAMESPACE, false);
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
    audioPrefs.putBool("agcEnable", agcEnabled);
    audioPrefs.putBool("noiseFilter", noiseFilterEnabled);
    audioPrefs.putUChar("ledMode", ledMode);
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

// Keep live streaming cadence bounded even if the configured buffer is large.
// Larger UI buffer values are still accepted, but the RTP pipeline is chunked to
// at most 1024 samples so playback does not turn into long bursty writes.
uint16_t effectiveAudioChunkSize() {
    uint16_t chunk = currentBufferSize;
    if (chunk < 256) chunk = 256;
    if (chunk > 1024) chunk = 1024;
    return chunk;
}

static uint16_t computeI2sDmaBufferLen() {
    // Fixed 60-sample DMA blocks are fine at 16 kHz, but at 48 kHz they only provide
    // ~7.5 ms of total capture slack and force a lot of interrupt churn. That makes
    // the stream sound choppy even when average CPU usage still looks low.
    uint16_t dmaBufLen = (uint16_t)(effectiveAudioChunkSize() / 4U);

    if (currentSampleRate >= 48000) {
        if (dmaBufLen < 256) dmaBufLen = 256;
    } else if (currentSampleRate >= 32000) {
        if (dmaBufLen < 192) dmaBufLen = 192;
    } else if (dmaBufLen < 128) {
        dmaBufLen = 128;
    }

    if (dmaBufLen > 512) dmaBufLen = 512;
    return dmaBufLen;
}

static uint8_t computeI2sDmaBufferCount() {
    return 8;
}

static uint16_t computeI2sReadBufferSamples() {
    // Read close to the RTP chunk size so Core 1 can forward audio immediately
    // instead of waiting for several packets of captured PCM and then flushing
    // them in a burst. Keep a small floor at higher sample rates so tiny UI
    // buffers do not create excessive I2S/read wakeups.
    uint32_t readSamples = effectiveAudioChunkSize();
    if (currentSampleRate >= 48000) {
        if (readSamples < 512) readSamples = 512;
    } else if (currentSampleRate >= 32000) {
        if (readSamples < 384) readSamples = 384;
    } else if (readSamples < 256) {
        readSamples = 256;
    }
    if (readSamples > WEBUI_AUDIO_MAX_SAMPLES) readSamples = WEBUI_AUDIO_MAX_SAMPLES;
    return (uint16_t)readSamples;
}

// Compute recommended minimum packet-rate threshold based on the effective stream chunk size
uint32_t computeRecommendedMinRate() {
    uint32_t buf = max((uint16_t)1, effectiveAudioChunkSize());
    float expectedPktPerSec = (float)currentSampleRate / (float)buf;
    uint32_t rec = (uint32_t)(expectedPktPerSec * 0.5f + 0.5f); // 50% safety margin
    if (rec < 5) rec = 5;
    return rec;
}

// Restore application settings to safe defaults and persist
void resetToDefaultSettings() {
    simplePrintln("FACTORY RESET: Restoring default settings...");

    // Clear persisted settings in both the current and legacy namespaces.
    audioPrefs.begin(SETTINGS_NAMESPACE, false);
    audioPrefs.clear();
    audioPrefs.end();

    Preferences legacyPrefs;
    if (legacyPrefs.begin(LEGACY_SETTINGS_NAMESPACE, false)) {
        legacyPrefs.clear();
        legacyPrefs.end();
    }

    // Reset runtime variables to defaults
    currentSampleRate = DEFAULT_SAMPLE_RATE;
    currentGainFactor = DEFAULT_GAIN_FACTOR;
    currentBufferSize = DEFAULT_BUFFER_SIZE;
    i2sShiftBits = 0;  // Fixed for Unit Mini PDM capture

    autoRecoveryEnabled = false;
    autoThresholdEnabled = true;
    scheduledResetEnabled = false;
    resetIntervalHours = 24;
    minAcceptableRate = computeRecommendedMinRate();
    performanceCheckInterval = 15;
    cpuFrequencyMhz = 160;
    wifiTxPowerDbm = DEFAULT_WIFI_TX_DBM;
    highpassEnabled = DEFAULT_HPF_ENABLED;
    highpassCutoffHz = DEFAULT_HPF_CUTOFF_HZ;
    agcEnabled = false;
    agcMultiplier = 1.0f;
    noiseFilterEnabled = false;
    noiseFloorDbfs = -90.0f;
    noiseGateDbfs = -90.0f;
    noiseReductionDb = 0.0f;
    ledMode = 1;
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

    // Request Core 1 to stop streaming safely
    if (isStreaming) {
        requestStreamStop("I2S restart");
    }

    // Stop audio pipeline task on Core 1
    stopAudioCaptureTask();

    // Restart I2S driver
    setup_i2s_driver();

    // Refresh HPF with current parameters
    updateHighpassCoeffs();
    maxPacketRate = 0;
    minPacketRate = 0xFFFFFFFF;

    if (!i2sDriverOk) {
        simplePrintln("I2S restart failed; audio pipeline remains stopped");
        return;
    }

    updateStreamingStateFromSessions();
    if (!startAudioCaptureTask()) {
        simplePrintln("I2S restarted but audio task could not start");
        return;
    }
    if (wasStreaming) {
        simplePrintln("I2S restarted; RTSP clients should reconnect");
    } else {
        simplePrintln("I2S restarted, diagnostics resumed");
    }
}

// Minimal print helpers: Serial + buffered for Web UI
// Timestamp prefix for log messages (NTP time if available, uptime fallback)
static String logTimestamp() {
    time_t now;
    time(&now);
    if (now > 100000) {
        struct tm ti;
        localtime_r(&now, &ti);
        char buf[24];
        strftime(buf, sizeof(buf), "[%H:%M:%S] ", &ti);
        return String(buf);
    }
    // Fallback: uptime
    unsigned long s = (millis() - bootTime) / 1000;
    unsigned long h = s / 3600; s %= 3600;
    unsigned long m = s / 60; s %= 60;
    char buf[16];
    snprintf(buf, sizeof(buf), "[%02lu:%02lu:%02lu] ", h, m, s);
    return String(buf);
}

void simplePrint(String message) {
    Serial.print(logTimestamp() + message);
}

void simplePrintln(String message) {
    String stamped = logTimestamp() + message;
    Serial.println(stamped);
    webui_pushLog(stamped);
}

static void resumeRtspServerAfterOtaFailure() {
    if (overheatLatched) {
        rtspServerEnabled = false;
        rtspServer.stop();
        return;
    }
    if (!rtspServerEnabled) return;
    rtspServer.stop();
    delay(20);
    rtspServer.begin();
    rtspServer.setNoDelay(true);
}

static void setupArduinoOta() {
    ArduinoOTA.setHostname(networkHostname.c_str());
    ArduinoOTA.setPort(OTA_PORT);

    ArduinoOTA.onStart([]() {
        const char* updateType = (ArduinoOTA.getCommand() == U_FLASH) ? "firmware" : "filesystem";
        simplePrintln(String("OTA start: ") + updateType + " update requested");
        otaPreviousRtspEnabled = rtspServerEnabled;
        if (isStreaming) {
            requestStreamStop("OTA update");
        }
        stopAudioCaptureTask();
        rtspServer.stop();
        rtspServerEnabled = false;
        core1OwnsLED = false;
        setStatusLed(CRGB(128, 0, 128));
    });

    ArduinoOTA.onEnd([]() {
        simplePrintln("OTA update complete, rebooting");
        setStatusLed(CRGB(0, 128, 0));
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        static uint8_t lastPercent = 255;
        uint8_t percent = total ? (progress * 100U) / total : 0;
        if (percent != lastPercent && (percent == 0 || percent == 100 || (percent % 10) == 0)) {
            simplePrintln("OTA progress: " + String(percent) + "%");
            lastPercent = percent;
        }
    });

    ArduinoOTA.onError([](ota_error_t error) {
        String reason = "unknown";
        switch (error) {
            case OTA_AUTH_ERROR:    reason = "auth failed"; break;
            case OTA_BEGIN_ERROR:   reason = "begin failed"; break;
            case OTA_CONNECT_ERROR: reason = "connect failed"; break;
            case OTA_RECEIVE_ERROR: reason = "receive failed"; break;
            case OTA_END_ERROR:     reason = "end failed"; break;
            default: break;
        }
        simplePrintln("OTA error: " + reason);
        if (i2sDriverOk) {
            if (!startAudioCaptureTask()) {
                simplePrintln("OTA recovery could not restart audio task");
            }
        }
        rtspServerEnabled = otaPreviousRtspEnabled;
        resumeRtspServerAfterOtaFailure();
        if (ledMode > 0) setStatusLed(CRGB(0, 0, 128));
        else setStatusLed(CRGB(0, 0, 0));
    });

    ArduinoOTA.begin();
    simplePrintln("OTA ready: pio run -e m5stack-atoms3-lite-ota -t upload");
    simplePrintln("OTA target: " + networkHostname + ".local:" + String(OTA_PORT));
}

static void updateFftFromBlock(const int16_t* samples, uint16_t count) {
    const int N = WEBUI_FFT_SIZE;
    const int BINS = WEBUI_FFT_BINS;
    if (!samples || count < N) return;

    static float real[WEBUI_FFT_SIZE];
    static float imag[WEBUI_FFT_SIZE];
    uint8_t nextBins[WEBUI_FFT_BINS];

    float mean = 0.0f;
    for (int i = 0; i < N; i++) mean += (float)samples[i];
    mean /= (float)N;

    for (int i = 0; i < N; i++) {
        float window = 0.5f - 0.5f * cosf((2.0f * PI * (float)i) / (float)(N - 1));
        real[i] = ((float)samples[i] - mean) * window;
        imag[i] = 0.0f;
    }

    for (int i = 1, j = 0; i < N; i++) {
        int bit = N >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) {
            float tr = real[i];
            float ti = imag[i];
            real[i] = real[j];
            imag[i] = imag[j];
            real[j] = tr;
            imag[j] = ti;
        }
    }

    for (int len = 2; len <= N; len <<= 1) {
        float angle = -2.0f * PI / (float)len;
        float wLenR = cosf(angle);
        float wLenI = sinf(angle);
        for (int i = 0; i < N; i += len) {
            float wr = 1.0f;
            float wi = 0.0f;
            for (int j = 0; j < len / 2; j++) {
                int even = i + j;
                int odd = even + len / 2;
                float ur = real[even];
                float ui = imag[even];
                float vr = real[odd] * wr - imag[odd] * wi;
                float vi = real[odd] * wi + imag[odd] * wr;
                real[even] = ur + vr;
                imag[even] = ui + vi;
                real[odd] = ur - vr;
                imag[odd] = ui - vi;

                float nextWr = wr * wLenR - wi * wLenI;
                wi = wr * wLenI + wi * wLenR;
                wr = nextWr;
            }
        }
    }

    for (int k = 0; k < BINS; k++) {
        int bin = k + 1; // skip DC; the input mean is already removed.
        float mag = sqrtf(real[bin] * real[bin] + imag[bin] * imag[bin]);
        float db = 20.0f * log10f((mag / 128.0f) + 1.0f);
        float norm = (db - 12.0f) / 64.0f;
        if (norm < 0.0f) norm = 0.0f;
        if (norm > 1.0f) norm = 1.0f;
        nextBins[k] = (uint8_t)(norm * 255.0f);
    }

    portENTER_CRITICAL(&fftMux);
    for (int k = 0; k < BINS; k++) {
        uint8_t previous = fftBins[k];
        fftBins[k] = (uint8_t)(((uint16_t)previous * 2U + nextBins[k]) / 3U);
    }
    fftFrameSeq = fftFrameSeq + 1U;
    portEXIT_CRITICAL(&fftMux);
}

// ================== CORE 1: FULL AUDIO PIPELINE ==================
// I2S capture → process (HPF, gain, AGC) → RTP → WiFi
// Uses streamClient pointer (set by Core 0 on PLAY, cleared here on failure)
// IMPORTANT: No String allocation or simplePrintln on this core (heap contention)
void audioCaptureTask(void* parameter) {
    Serial.println("[Core1] Audio pipeline task started");
    audioTaskRunning = true;

    size_t bytesRead = 0;
    uint32_t consecutiveErrors = 0;
    const uint32_t MAX_ERRORS = 10;
    uint32_t packetCount = 0;

    const uint16_t chunkSamples = effectiveAudioChunkSize();
    const uint16_t readBufferSamples = computeI2sReadBufferSamples();
    int16_t* captureBuffer = (int16_t*)malloc(readBufferSamples * sizeof(int16_t));
    int16_t* outputBuffer = (int16_t*)malloc(readBufferSamples * sizeof(int16_t));

    if (!captureBuffer || !outputBuffer) {
        Serial.println("[Core1] FATAL: Failed to allocate audio buffers!");
        if (captureBuffer) free(captureBuffer);
        if (outputBuffer) free(outputBuffer);
        audioTaskRunning = false;
        audioCaptureTaskHandle = NULL;
        if (taskExitSemaphore != NULL) {
            xSemaphoreGive(taskExitSemaphore);
        }
        vTaskDelete(NULL);
        return;
    }

    // High-pass filter state (local to this core)
    Biquad localHpf = hpf;
    uint32_t localHpfConfigSampleRate = hpfConfigSampleRate;
    uint16_t localHpfConfigCutoff = hpfConfigCutoff;
    bool localHighpassEnabled = highpassEnabled;

    // AGC, limiter, and noise-filter state (local)
    float localAgcMult = 1.0f;
    float localLimiterGain = 1.0f;
    float localNoiseFloor = NOISE_FLOOR_INIT;
    float localNoiseGate = max(NOISE_GATE_MIN, localNoiseFloor * NOISE_GATE_RATIO + NOISE_GATE_MARGIN);
    float localNoiseGain = 1.0f;
    float localNoiseEnv = 0.0f;
    uint32_t localNoiseGateHoldSamples = 0;
    float localNoiseReduction = 0.0f;
    const float LIMITER_TARGET_PEAK = 26000.0f;  // ~79% FS to keep headroom and reduce harsh clipping
    const float LIMITER_RELEASE = 0.02f;         // Slow recovery keeps ambience stable instead of pumping
    const float LIMITER_MIN_GAIN = 0.20f;

    uint32_t i2sErrors = 0;
    unsigned long lastStatsLog = millis();
    unsigned long lastLedUpdate = 0;
    unsigned long lastGoodCaptureMs = millis();
    int16_t lastOutputSample = 0;
    bool lastBlockWasConcealment = false;
    bool localUnsignedPdm = false;
    bool localPdmDcInitialized = false;
    uint8_t localUnsignedPdmConfidence = 0;
    float localPdmDc = 0.0f;
    const uint32_t readTimeoutMs = max(20UL, min(200UL,
        ((unsigned long)readBufferSamples * 1000UL) / max((uint32_t)1, currentSampleRate) + 10UL));

    while (audioTaskRunning) {
        // Check if Core 0 requested us to stop streaming
        if (stopStreamRequested) {
            for (uint8_t i = 0; i < MAX_RTSP_CLIENTS; ++i) {
                if (rtspSessions[i].occupied && rtspSessions[i].streaming) {
                    closeRtspSession(rtspSessions[i], true);
                }
            }
            updateStreamingStateFromSessions();
            streamCleanupDone = true;
            __asm__ __volatile__("memw" ::: "memory");
            // Wait for Core 0 to clear stopStreamRequested
            while (stopStreamRequested && audioTaskRunning) {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            continue;
        }

        // Keep I2S capture running even when no RTSP client is active,
        // so /api/audio_status can show live raw-mic diagnostics.
        bool streamActive = (countStreamingRtspClients() > 0);

        // Periodic stats (every 30s) — Serial.printf only, no heap alloc
        if (millis() - lastStatsLog > 30000) {
            Serial.printf("[Core1] Sent=%u I2Serr=%u Clip=%lu AGC=%.2f RawPeak=%u RawRMS=%u RawMin=%d RawMax=%d Z=%u%%\n",
                         packetCount, i2sErrors, audioClipCount, localAgcMult,
                         i2sLastRawPeakAbs, i2sLastRawRms, i2sLastRawMin, i2sLastRawMax, i2sLastRawZeroPct);
            lastStatsLog = millis();
        }

        // Read from I2S with a cadence-aware timeout so concealment can kick in
        esp_err_t result = i2sMicRead(captureBuffer,
                                      readBufferSamples * sizeof(int16_t),
                                      &bytesRead, readTimeoutMs);
        unsigned long processStartUs = micros();

        if (result != ESP_OK || bytesRead == 0) {
            if (result != ESP_OK) {
                consecutiveErrors++;
                i2sErrors++;
                i2sReadErrCount = i2sReadErrCount + 1U;
                if (consecutiveErrors >= MAX_ERRORS) {
                    Serial.println("[Core1] Too many I2S errors, pausing");
                    vTaskDelay(pdMS_TO_TICKS(100));
                    consecutiveErrors = 0;
                }
            } else {
                i2sReadZeroCount = i2sReadZeroCount + 1U;
            }

            fillConcealmentBlock(outputBuffer, chunkSamples, lastOutputSample);
            lastOutputSample = outputBuffer[chunkSamples - 1];
            lastBlockWasConcealment = true;
            portENTER_CRITICAL(&diagMux);
            audioFallbackBlockCount = audioFallbackBlockCount + 1U;
            i2sLastGapMs = millis() - lastGoodCaptureMs;
            i2sLastSamplesRead = 0;
            lastPeakAbs16 = (uint16_t)abs((int)outputBuffer[0]);
            audioClippedLastBlock = false;
            portEXIT_CRITICAL(&diagMux);
            publishWebAudioFrame(outputBuffer, chunkSamples);

            if (streamActive) {
                sendRTPPacketsToActiveSessions(outputBuffer, chunkSamples);
                audioBlocksSent++;
                packetCount++;

                float blockMs = (1000.0f * (float)chunkSamples) / max((float)currentSampleRate, 1.0f);
                float workMs = (float)(micros() - processStartUs) / 1000.0f;
                float instLoad = (blockMs > 0.0f) ? (workMs * 100.0f / blockMs) : 0.0f;
                if (instLoad > 100.0f) instLoad = 100.0f;
                portENTER_CRITICAL(&diagMux);
                audioPipelineLoadPct += (instLoad - audioPipelineLoadPct) * 0.20f;
                portEXIT_CRITICAL(&diagMux);
            }
            continue;
        }

        consecutiveErrors = 0;
        i2sReadOkCount = i2sReadOkCount + 1U;
        lastGoodCaptureMs = millis();
        uint16_t samplesRead = bytesRead / sizeof(int16_t);
        portENTER_CRITICAL(&diagMux);
        i2sLastGapMs = 0;
        i2sLastSamplesRead = samplesRead;
        portEXIT_CRITICAL(&diagMux);

        // Update HPF coefficients if changed
        if (localHighpassEnabled != highpassEnabled ||
            localHpfConfigSampleRate != hpfConfigSampleRate ||
            localHpfConfigCutoff != hpfConfigCutoff) {
            portENTER_CRITICAL(&hpfMux);
            localHpf = hpf;
            localHpfConfigSampleRate = hpfConfigSampleRate;
            localHpfConfigCutoff = hpfConfigCutoff;
            localHighpassEnabled = highpassEnabled;
            portEXIT_CRITICAL(&hpfMux);
        }

        // Determine effective gain (manual * AGC if enabled)
        float effectiveGain = currentGainFactor;
        if (agcEnabled) {
            effectiveGain *= localAgcMult;
        }
        if (localLimiterGain < 1.0f) {
            localLimiterGain += (1.0f - localLimiterGain) * LIMITER_RELEASE;
            if (localLimiterGain > 1.0f) localLimiterGain = 1.0f;
        }

        // Process audio: HPF, adaptive noise filter, gain, limiter, clipping detection, RMS for AGC
        bool clipped = false;
        float peakAbs = 0.0f;
        float sumSquares = 0.0f;

        // Raw I2S diagnostics before DSP (used to detect dead/flat/stuck mic input)
        int16_t rawMin = INT16_MAX;
        int16_t rawMax = INT16_MIN;
        uint16_t rawPeakAbs = 0;
        uint32_t rawZeroCount = 0;
        float rawSum = 0.0f;
        float rawSumSquares = 0.0f;

        // First pass: collect raw stats for diagnostics and format detection.
        for (int i = 0; i < samplesRead; i++) {
            int16_t raw = captureBuffer[i];
            if (raw < rawMin) rawMin = raw;
            if (raw > rawMax) rawMax = raw;
            uint16_t rawAbs = (raw < 0) ? (uint16_t)(-raw) : (uint16_t)raw;
            if (rawAbs > rawPeakAbs) rawPeakAbs = rawAbs;
            if (raw == 0) rawZeroCount++;
            rawSum += (float)raw;
            rawSumSquares += (float)raw * (float)raw;
        }

        float rawMean = (samplesRead > 0) ? (rawSum / (float)samplesRead) : 0.0f;
        int32_t rawSpan = (rawMax >= rawMin) ? ((int32_t)rawMax - (int32_t)rawMin) : 0;
        bool unsignedCandidate = (samplesRead > 0 &&
                                  rawMin >= 0 &&
                                  rawMax <= (int16_t)PDM_UNSIGNED_MAX_CODE &&
                                  rawSpan >= (int32_t)PDM_UNSIGNED_MIN_SPAN &&
                                  rawMin >= PDM_UNSIGNED_EDGE_MARGIN &&
                                  rawMean >= PDM_UNSIGNED_MIN_MEAN &&
                                  rawMean >= ((float)rawSpan * PDM_UNSIGNED_MIN_MEAN_TO_SPAN));
        if (unsignedCandidate) {
            if (localUnsignedPdmConfidence < PDM_UNSIGNED_CONFIDENCE_MAX) {
                localUnsignedPdmConfidence++;
            }
        } else if (localUnsignedPdmConfidence > 0) {
            localUnsignedPdmConfidence--;
        }

        if (!localUnsignedPdm && localUnsignedPdmConfidence >= PDM_UNSIGNED_CONFIDENCE_ON) {
            localUnsignedPdm = true;
            localPdmDc = rawMean;
            localPdmDcInitialized = true;
            localHpf.reset();
        } else if (localUnsignedPdm && localUnsignedPdmConfidence <= PDM_UNSIGNED_CONFIDENCE_OFF) {
            localUnsignedPdm = false;
            localPdmDcInitialized = false;
            localHpf.reset();
        }
        if (localUnsignedPdm && !localPdmDcInitialized) {
            localPdmDc = rawMean;
            localPdmDcInitialized = true;
        }
        portENTER_CRITICAL(&diagMux);
        i2sLikelyUnsignedPcm = localUnsignedPdm;
        portEXIT_CRITICAL(&diagMux);

        float pdmDcAlpha = 0.0f;
        if (localUnsignedPdm) {
            float dcDenom = (float)currentSampleRate * PDM_UNSIGNED_DC_TRACK_SECONDS;
            if (dcDenom < 1.0f) dcDenom = 1.0f;
            pdmDcAlpha = 1.0f / dcDenom;
        }

        // Second pass: DSP and output generation.
        for (int i = 0; i < samplesRead; i++) {
            int16_t raw = captureBuffer[i];

            float sample;
            if (localUnsignedPdm) {
                // Keep unsigned-PDM scaling fixed per center band so weak all-positive
                // blocks do not get over-amplified into gritty/raspy audio.
                float unsignedNormalizeGain =
                    (localPdmDc < PDM_UNSIGNED_LOW_CENTER_MAX)
                        ? PDM_UNSIGNED_LOW_NORMALIZE_GAIN
                        : PDM_UNSIGNED_HIGH_NORMALIZE_GAIN;
                localPdmDc += ((float)raw - localPdmDc) * pdmDcAlpha;
                sample = ((float)raw - localPdmDc) * unsignedNormalizeGain;
            } else {
                sample = (float)(raw >> i2sShiftBits);
            }

            if (localHighpassEnabled) {
                sample = localHpf.process(sample);
            }

            float sampleAbs = fabsf(sample);
            if (noiseFilterEnabled) {
                float prevNoiseEnv = localNoiseEnv;
                float envSlew = (sampleAbs > localNoiseEnv) ? NOISE_ENV_ATTACK : NOISE_ENV_RELEASE;
                localNoiseEnv += (sampleAbs - localNoiseEnv) * envSlew;

                float floorFollow = (localNoiseEnv <= localNoiseGate) ? NOISE_FLOOR_FAST : NOISE_FLOOR_SLOW;
                localNoiseFloor += (localNoiseEnv - localNoiseFloor) * floorFollow;
                if (localNoiseFloor < 0.0f) localNoiseFloor = 0.0f;
                localNoiseGate = localNoiseFloor * NOISE_GATE_RATIO + NOISE_GATE_MARGIN;
                if (localNoiseGate < NOISE_GATE_MIN) localNoiseGate = NOISE_GATE_MIN;
                if (localNoiseGate > NOISE_GATE_MAX) localNoiseGate = NOISE_GATE_MAX;

                uint32_t holdSamples = ((uint32_t)currentSampleRate * NOISE_GATE_HOLD_MS) / 1000UL;
                if (holdSamples < 1) holdSamples = 1;

                if (localNoiseEnv >= localNoiseGate) {
                    localNoiseGateHoldSamples = holdSamples;
                } else if (localNoiseGateHoldSamples > 0) {
                    localNoiseGateHoldSamples--;
                }

                float desiredNoiseGain = 1.0f;
                float closeThreshold = localNoiseGate * NOISE_GATE_CLOSE_RATIO;
                if (localNoiseGateHoldSamples == 0 && localNoiseGate > 1.0f && localNoiseEnv < closeThreshold) {
                    float normalized = localNoiseEnv / closeThreshold;
                    if (normalized < 0.0f) normalized = 0.0f;
                    if (normalized > 1.0f) normalized = 1.0f;
                    desiredNoiseGain = NOISE_FILTER_MIN_GAIN + (1.0f - NOISE_FILTER_MIN_GAIN) * normalized * normalized;
                }

                bool onsetDetected = (sampleAbs > prevNoiseEnv) &&
                                    ((localNoiseGateHoldSamples > 0) || (localNoiseEnv >= closeThreshold));
                float noiseSlew = (desiredNoiseGain < localNoiseGain) ? NOISE_FILTER_ATTACK : NOISE_FILTER_RELEASE;
                if (desiredNoiseGain >= 0.999f && localNoiseGain < 0.999f && onsetDetected) {
                    noiseSlew = NOISE_FILTER_REOPEN;
                }
                localNoiseGain += (desiredNoiseGain - localNoiseGain) * noiseSlew;
                if (localNoiseGain < NOISE_FILTER_MIN_GAIN) localNoiseGain = NOISE_FILTER_MIN_GAIN;
                if (localNoiseGain > 1.0f) localNoiseGain = 1.0f;
                sample *= localNoiseGain;
                localNoiseReduction = 20.0f * log10f(max(localNoiseGain, 0.0001f));
            } else {
                localNoiseGain = 1.0f;
                localNoiseEnv = 0.0f;
                localNoiseGateHoldSamples = 0;
                localNoiseReduction = 0.0f;
            }

            float amplified = sample * effectiveGain * localLimiterGain;
            float aabs = fabsf(amplified);
            if (aabs > LIMITER_TARGET_PEAK && aabs > 1.0f) {
                float neededGain = LIMITER_TARGET_PEAK / aabs;
                localLimiterGain *= neededGain;
                if (localLimiterGain < LIMITER_MIN_GAIN) localLimiterGain = LIMITER_MIN_GAIN;
                amplified = sample * effectiveGain * localLimiterGain;
                aabs = fabsf(amplified);
            }
            if (aabs > peakAbs) peakAbs = aabs;
            if (aabs > 32767.0f) clipped = true;
            sumSquares += amplified * amplified;

            if (amplified > 32767.0f) amplified = 32767.0f;
            if (amplified < -32768.0f) amplified = -32768.0f;
            outputBuffer[i] = (int16_t)amplified;
        }
        if (lastBlockWasConcealment) {
            blendBlockStartFrom(outputBuffer, samplesRead, lastOutputSample);
            lastBlockWasConcealment = false;
        }
        if (samplesRead > 0) {
            lastOutputSample = outputBuffer[samplesRead - 1];
        }

        // Publish latest processed frame for browser WebAudio endpoint
        publishWebAudioFrame(outputBuffer, samplesRead);

        // Update spectrum diagnostics from processed output (throttled)
        static uint8_t fftDecimator = 0;
        fftDecimator++;
        if ((fftDecimator & 0x03) == 0 && samplesRead >= WEBUI_FFT_SIZE) {
            updateFftFromBlock(outputBuffer, samplesRead);
        }

        // Publish raw capture diagnostics for UI/API
        portENTER_CRITICAL(&diagMux);
        i2sLastRawMin = rawMin;
        i2sLastRawMax = rawMax;
        i2sLastRawPeakAbs = rawPeakAbs;
        i2sLastRawSpan = (rawSpan > 0) ? (uint16_t)rawSpan : 0;
        if (rawMean > 32767.0f) rawMean = 32767.0f;
        if (rawMean < -32768.0f) rawMean = -32768.0f;
        i2sLastRawMean = (int16_t)rawMean;
        if (samplesRead > 0) {
            float rawRms = sqrtf(rawSumSquares / (float)samplesRead);
            if (rawRms > 65535.0f) rawRms = 65535.0f;
            i2sLastRawRms = (uint16_t)rawRms;
            i2sLastRawZeroPct = (uint16_t)((rawZeroCount * 100UL) / samplesRead);
        } else {
            i2sLastRawRms = 0;
            i2sLastRawZeroPct = 100;
        }
        portEXIT_CRITICAL(&diagMux);

        // AGC: adjust multiplier based on RMS
        if (agcEnabled && samplesRead > 0) {
            float rms = sqrtf(sumSquares / (float)samplesRead) / 32767.0f;
            if (rms > 0.001f) {  // Don't adjust on silence
                float ratio = AGC_TARGET_RMS / rms;
                if (ratio < 1.0f) {
                    // Signal too loud — fast attack (reduce gain quickly)
                    localAgcMult += (ratio - 1.0f) * AGC_ATTACK_RATE * localAgcMult;
                } else {
                    // Signal too quiet — slow release (increase gain slowly)
                    localAgcMult += (ratio - 1.0f) * AGC_RELEASE_RATE * localAgcMult;
                }
                if (localAgcMult < AGC_MIN_MULT) localAgcMult = AGC_MIN_MULT;
                if (localAgcMult > AGC_MAX_MULT) localAgcMult = AGC_MAX_MULT;
            }
            portENTER_CRITICAL(&diagMux);
            agcMultiplier = localAgcMult;  // Publish for WebUI
            portEXIT_CRITICAL(&diagMux);
        }

        portENTER_CRITICAL(&diagMux);
        if (noiseFilterEnabled) {
            float floorNorm = max(localNoiseFloor, 1.0f) / 32767.0f;
            float gateNorm = max(localNoiseGate, 1.0f) / 32767.0f;
            noiseFloorDbfs = 20.0f * log10f(floorNorm);
            noiseGateDbfs = 20.0f * log10f(gateNorm);
            noiseReductionDb = localNoiseReduction;
        } else {
            noiseFloorDbfs = -90.0f;
            noiseGateDbfs = -90.0f;
            noiseReductionDb = 0.0f;
        }
        portEXIT_CRITICAL(&diagMux);

        // Update metering
        if (peakAbs > 32767.0f) peakAbs = 32767.0f;
        bool logClip = false;
        uint16_t clipPeak = 0;
        uint32_t clipCount = 0;
        portENTER_CRITICAL(&diagMux);
        lastPeakAbs16 = (uint16_t)peakAbs;
        audioClippedLastBlock = clipped;
        if (clipped) {
            audioClipCount++;
            clipPeak = lastPeakAbs16;
            clipCount = audioClipCount;
            logClip = true;
        }

        if (lastPeakAbs16 > peakHoldAbs16) {
            peakHoldAbs16 = lastPeakAbs16;
            peakHoldUntilMs = millis() + 3000UL;
        } else if (peakHoldAbs16 > 0 && millis() > peakHoldUntilMs) {
            peakHoldAbs16 = 0;
        }
        portEXIT_CRITICAL(&diagMux);
        if (logClip) {
            static unsigned long lastClipLog = 0;
            unsigned long nowMs = millis();
            if (nowMs - lastClipLog > 5000) {
                Serial.printf("[Core1] Clipping! Peak=%u count=%lu\n",
                             clipPeak, clipCount);
                lastClipLog = nowMs;
            }
        }

        // LED update (throttled to ~10 Hz) only while actively streaming.
        if (streamActive && millis() - lastLedUpdate > 100) {
            if (ledMode == 2) {
                // Level mode: color-coded audio level
                float pct = peakAbs / 32767.0f;
                if (clipped) {
                    setStatusLed(CRGB(255, 0, 0));        // Red: clipping
                } else if (pct > 0.7f) {
                    setStatusLed(CRGB(255, 165, 0));      // Orange: hot signal
                } else if (pct > 0.3f) {
                    setStatusLed(CRGB(0, 255, 0));        // Bright green: good level
                } else if (pct > 0.05f) {
                    setStatusLed(CRGB(0, 64, 0));         // Dim green: low signal
                } else {
                    setStatusLed(CRGB(32, 0, 32));        // Dim purple: very quiet
                }
            } else if (ledMode == 1) {
                // Static mode: solid green during streaming
                setStatusLed(CRGB(0, 128, 0));
            } else {
                // Off mode: LED dark during streaming
                setStatusLed(CRGB(0, 0, 0));
            }
            lastLedUpdate = millis();
        }

        if (streamActive) {
            // Send RTP packet
            sendRTPPacketsToActiveSessions(outputBuffer, samplesRead);
            audioBlocksSent++;
            packetCount++;
        }

        float blockMs = (1000.0f * (float)max((uint16_t)1, samplesRead)) / max((float)currentSampleRate, 1.0f);
        float workMs = (float)(micros() - processStartUs) / 1000.0f;
        float instLoad = (blockMs > 0.0f) ? (workMs * 100.0f / blockMs) : 0.0f;
        if (instLoad > 100.0f) instLoad = 100.0f;
        portENTER_CRITICAL(&diagMux);
        audioPipelineLoadPct += (instLoad - audioPipelineLoadPct) * 0.15f;
        portEXIT_CRITICAL(&diagMux);

        // Process incoming RTSP commands during streaming (~every 50ms)
        // Keeps all socket I/O on Core 1 during streaming
        static unsigned long lastRtspCheck = 0;
        if (streamActive && (millis() - lastRtspCheck > 50)) {
            lastRtspCheck = millis();
            for (uint8_t i = 0; i < MAX_RTSP_CLIENTS; ++i) {
                if (rtspSessions[i].occupied && rtspSessions[i].streaming) {
                    pollStreamingRtspCommands(rtspSessions[i]);
                }
            }
        }

        taskYIELD();
    }

    free(captureBuffer);
    free(outputBuffer);
    core1OwnsLED = false;
    audioTaskRunning = false;
    audioCaptureTaskHandle = NULL;
    Serial.println("[Core1] Audio pipeline task stopped");
    xSemaphoreGive(taskExitSemaphore);
    vTaskDelete(NULL);
}

// Start audio pipeline task on Core 1 (or reuse if already running)
bool startAudioCaptureTask() {
    if (audioCaptureTaskHandle != NULL) {
        // Task already alive — it will pick up via isStreaming/streamClient
        return true;
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
        audioCaptureTaskHandle = NULL;
        audioTaskRunning = false;
        return false;
    }
    return true;
}

// Stop audio pipeline task with confirmed exit via semaphore
void stopAudioCaptureTask() {
    if (audioCaptureTaskHandle != NULL) {
        audioTaskRunning = false;
        // Wait for task to confirm exit (up to 2s)
        bool exited = (taskExitSemaphore != NULL &&
                       xSemaphoreTake(taskExitSemaphore, pdMS_TO_TICKS(2000)) == pdTRUE);
        if (!exited) {
            Serial.println("[Core0] WARNING: Audio task did not exit within 2s");
            return;
        }

        audioCaptureTaskHandle = NULL;
        for (uint8_t i = 0; i < MAX_RTSP_CLIENTS; ++i) {
            if (rtspSessions[i].occupied && rtspSessions[i].streaming) {
                closeRtspSession(rtspSessions[i], true);
            }
        }
        updateStreamingStateFromSessions();
        clearAudioDiagnostics();
    }
}

// Request Core 1 to stop streaming and clean up the socket.
// Core 0 must NOT touch the WiFiClient while Core 1 owns it.
// Returns true if Core 1 confirmed cleanup, false on timeout.
bool requestStreamStop(const char* reason) {
    // Early return if not streaming
    if (!isStreaming && streamClient == NULL) return true;

    Serial.printf("[Core0] requestStreamStop: %s\n", reason);

    // Signal Core 1 to stop
    stopStreamRequested = true;
    __asm__ __volatile__("memw" ::: "memory");  // Xtensa memory barrier

    // Poll for Core 1 confirmation (up to 3s)
    unsigned long deadline = millis() + 3000;
    while (!streamCleanupDone && millis() < deadline) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (streamCleanupDone) {
        // Core 1 confirmed — safe to proceed
        isStreaming = false;
        streamClient = NULL;
        stopStreamRequested = false;
        streamCleanupDone = false;
        Serial.printf("[Core0] Stream stopped cleanly: %s\n", reason);
        return true;
    } else {
        // Timeout — stop the Core 1 task before touching leftover session sockets.
        Serial.printf("[Core0] WARNING: Stream stop timeout, stopping audio task for cleanup: %s\n", reason);
        bool hadAudioTask = (audioCaptureTaskHandle != NULL);
        stopAudioCaptureTask();
        if (audioCaptureTaskHandle == NULL) {
            isStreaming = false;
            streamClient = NULL;
            stopStreamRequested = false;
            streamCleanupDone = false;
            core1OwnsLED = false;
            if (hadAudioTask && i2sDriverOk) {
                if (!startAudioCaptureTask()) {
                    Serial.printf("[Core0] WARNING: Audio task restart failed after cleanup timeout: %s\n", reason);
                }
            }
        } else {
            Serial.printf("[Core0] WARNING: Audio task still running after forced stop attempt: %s\n", reason);
        }
        return false;
    }
}

static esp_err_t i2sMicRead(void* dest, size_t size, size_t* bytesRead, uint32_t timeoutMs) {
    if (!i2sRxChannel) {
        if (bytesRead) *bytesRead = 0;
        return ESP_ERR_INVALID_STATE;
    }
    return i2s_channel_read(i2sRxChannel, dest, size, bytesRead, timeoutMs);
}

// I2S setup for AtomS3 Lite + Unit Mini PDM using the ESP-IDF 5 PDM RX channel driver.
static void shutdownModernI2sDriver() {
    if (!i2sRxChannel) {
        return;
    }

    esp_err_t err = i2s_channel_disable(i2sRxChannel);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        simplePrintln("WARNING: I2S channel disable returned " + String(esp_err_to_name(err)));
    }

    err = i2s_del_channel(i2sRxChannel);
    if (err != ESP_OK) {
        simplePrintln("WARNING: I2S channel delete returned " + String(esp_err_to_name(err)));
    }
    i2sRxChannel = NULL;
}

static i2s_pdm_slot_mask_t pdmLine0SlotMask(bool useLeftSlot) {
#if SOC_I2S_PDM_MAX_RX_LINES > 1
    return useLeftSlot ? I2S_PDM_RX_LINE0_SLOT_LEFT : I2S_PDM_RX_LINE0_SLOT_RIGHT;
#else
    return useLeftSlot ? I2S_PDM_SLOT_LEFT : I2S_PDM_SLOT_RIGHT;
#endif
}

static bool setupModernPdmDriverCandidate(uint16_t dmaBufLen, uint8_t dmaBufCount,
                                          bool useLeftSlot, bool clkInvert,
                                          bool verboseLogs) {
    shutdownModernI2sDriver();

    i2s_chan_config_t chanConfig = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chanConfig.dma_desc_num = dmaBufCount;
    chanConfig.dma_frame_num = dmaBufLen;

    esp_err_t err = i2s_new_channel(&chanConfig, NULL, &i2sRxChannel);
    if (err != ESP_OK) {
        i2sLastError = err;
        if (verboseLogs) {
            simplePrintln("ERROR: I2S new_channel failed: " + String(esp_err_to_name(err)));
        }
        i2sRxChannel = NULL;
        return false;
    }

    i2s_pdm_rx_config_t pdmConfig = {
        .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(currentSampleRate),
        .slot_cfg = I2S_PDM_RX_SLOT_PCM_FMT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {}
    };
    pdmConfig.slot_cfg.slot_mask = pdmLine0SlotMask(useLeftSlot);
#if SOC_I2S_SUPPORTS_PDM_RX_HP_FILTER
    // Keep the driver capture path honest; firmware DSP owns DC removal / HPF.
    pdmConfig.slot_cfg.hp_en = false;
    pdmConfig.slot_cfg.hp_cut_off_freq_hz = 35.5f;
    pdmConfig.slot_cfg.amplify_num = 1;
#endif
    pdmConfig.gpio_cfg.clk = (gpio_num_t)I2S_CLK_PIN;
#if SOC_I2S_PDM_MAX_RX_LINES > 1
    for (uint8_t i = 0; i < SOC_I2S_PDM_MAX_RX_LINES; ++i) {
        pdmConfig.gpio_cfg.dins[i] = I2S_GPIO_UNUSED;
    }
    pdmConfig.gpio_cfg.dins[0] = (gpio_num_t)I2S_DATA_IN_PIN;
#else
    pdmConfig.gpio_cfg.din = (gpio_num_t)I2S_DATA_IN_PIN;
#endif
    pdmConfig.gpio_cfg.invert_flags.clk_inv = clkInvert ? 1 : 0;

    err = i2s_channel_init_pdm_rx_mode(i2sRxChannel, &pdmConfig);
    if (err != ESP_OK) {
        i2sLastError = err;
        if (verboseLogs) {
            simplePrintln("ERROR: I2S PDM RX init failed: " + String(esp_err_to_name(err)));
        }
        shutdownModernI2sDriver();
        return false;
    }

    err = i2s_channel_enable(i2sRxChannel);
    if (err != ESP_OK) {
        i2sLastError = err;
        if (verboseLogs) {
            simplePrintln("ERROR: I2S channel enable failed: " + String(esp_err_to_name(err)));
        }
        shutdownModernI2sDriver();
        return false;
    }

    i2sLastError = ESP_OK;
    return true;
}

static bool isBetterPdmCandidate(const PdmProbeResult& candidate, const PdmProbeResult& best) {
    if (!candidate.valid) return false;
    if (!best.valid) return true;
    if (candidate.spansZero != best.spansZero) return candidate.spansZero;

    float candidateDcRatio = fabsf((float)candidate.rawMean) / max(1.0f, (float)candidate.rawSpan);
    float bestDcRatio = fabsf((float)best.rawMean) / max(1.0f, (float)best.rawSpan);
    if (fabsf(candidateDcRatio - bestDcRatio) > 0.20f) return candidateDcRatio < bestDcRatio;
    if (candidate.rawSpan != best.rawSpan) return candidate.rawSpan > best.rawSpan;
    if (candidate.rawPeakAbs != best.rawPeakAbs) return candidate.rawPeakAbs > best.rawPeakAbs;
    if (candidate.useLeftSlot != best.useLeftSlot) return candidate.useLeftSlot;
    if (candidate.clkInvert != best.clkInvert) return !candidate.clkInvert;
    return false;
}

static PdmProbeResult probeModernPdmCandidate(uint16_t dmaBufLen, uint8_t dmaBufCount,
                                              bool useLeftSlot, bool clkInvert,
                                              int16_t* probeBuffer, uint16_t probeSamples) {
    PdmProbeResult result;
    result.useLeftSlot = useLeftSlot;
    result.clkInvert = clkInvert;

    if (!probeBuffer || probeSamples == 0) {
        return result;
    }
    if (!setupModernPdmDriverCandidate(dmaBufLen, dmaBufCount, useLeftSlot, clkInvert, false)) {
        simplePrintln("Modern PDM probe " + String(useLeftSlot ? "left" : "right") +
                      (clkInvert ? " / clk inverted" : " / normal clock") +
                      " setup failed: " + String(esp_err_to_name((esp_err_t)i2sLastError)));
        return result;
    }

    size_t bytesRead = 0;
    unsigned long settleUntil = millis() + PDM_PROBE_SETTLE_MS;
    do {
        (void)i2sMicRead(probeBuffer, probeSamples * sizeof(int16_t), &bytesRead, 80);
    } while (millis() < settleUntil);

    int16_t rawMin = 32767;
    int16_t rawMax = -32768;
    uint32_t rawPeak = 0;
    int64_t rawSum = 0;
    uint32_t totalSamples = 0;

    for (uint8_t pass = 0; pass < PDM_PROBE_MEASURE_READS; ++pass) {
        esp_err_t err = i2sMicRead(probeBuffer, probeSamples * sizeof(int16_t), &bytesRead, 80);
        if (err != ESP_OK || bytesRead == 0) {
            continue;
        }

        uint16_t samples = (uint16_t)(bytesRead / sizeof(int16_t));
        for (uint16_t i = 0; i < samples; ++i) {
            int16_t raw = probeBuffer[i];
            if (raw < rawMin) rawMin = raw;
            if (raw > rawMax) rawMax = raw;
            uint32_t absRaw = (uint32_t)abs((int)raw);
            if (absRaw > rawPeak) rawPeak = absRaw;
            rawSum += raw;
        }
        totalSamples += samples;
    }

    if (totalSamples == 0) {
        simplePrintln("Modern PDM probe " + String(useLeftSlot ? "left" : "right") +
                      (clkInvert ? " / clk inverted" : " / normal clock") +
                      " yielded no audio blocks");
        return result;
    }

    result.valid = true;
    result.rawMin = rawMin;
    result.rawMax = rawMax;
    result.rawPeakAbs = (uint16_t)min<uint32_t>(rawPeak, 32767U);
    result.rawMean = (int16_t)lroundf((float)rawSum / (float)totalSamples);
    result.rawSpan = (rawMax >= rawMin) ? (uint16_t)((int32_t)rawMax - (int32_t)rawMin) : 0;
    result.spansZero = (rawMin < -32 && rawMax > 32);
    simplePrintln("Modern PDM probe " + String(useLeftSlot ? "left" : "right") +
                  (clkInvert ? " / clk inverted" : " / normal clock") +
                  ": mean " + String((int32_t)result.rawMean) +
                  ", span " + String((uint32_t)result.rawSpan) +
                  ", peak " + String((uint32_t)result.rawPeakAbs) +
                  (result.spansZero ? ", signed" : ", offset"));
    return result;
}

static bool setupModernPdmDriver(uint16_t dmaBufLen, uint8_t dmaBufCount) {
    uint16_t probeSamples = computeI2sReadBufferSamples();
    if (probeSamples < 64) {
        probeSamples = 64;
    }

    int16_t* probeBuffer = (int16_t*)malloc(probeSamples * sizeof(int16_t));
    if (!probeBuffer) {
        simplePrintln("Modern PDM probe could not allocate scratch buffer; using right-slot normal-clock fallback");
        if (!setupModernPdmDriverCandidate(dmaBufLen, dmaBufCount, false, false, true)) {
            return false;
        }
        i2sPdmUseLeftSlot = false;
        i2sPdmClkInverted = false;
        i2sPdmProbeFoundSigned = false;
    } else {
        PdmProbeResult best;
        const bool slotOptions[] = {true, false};
        const bool clockOptions[] = {false, true};
        for (uint8_t slot = 0; slot < 2; ++slot) {
            for (uint8_t clock = 0; clock < 2; ++clock) {
                PdmProbeResult probe = probeModernPdmCandidate(dmaBufLen, dmaBufCount,
                                                               slotOptions[slot],
                                                               clockOptions[clock],
                                                               probeBuffer,
                                                               probeSamples);
                if (isBetterPdmCandidate(probe, best)) {
                    best = probe;
                }
            }
        }
        free(probeBuffer);

        bool useLeftSlot = best.valid ? best.useLeftSlot : false;
        bool clkInvert = best.valid ? best.clkInvert : false;
        if (!best.valid) {
            simplePrintln("Modern PDM probe did not find a clear candidate; using right-slot normal-clock fallback");
        }
        if (!setupModernPdmDriverCandidate(dmaBufLen, dmaBufCount, useLeftSlot, clkInvert, true)) {
            return false;
        }
        i2sPdmUseLeftSlot = useLeftSlot;
        i2sPdmClkInverted = clkInvert;
        i2sPdmProbeFoundSigned = best.valid && best.spansZero;
    }

    i2sUsingModernPdmDriver = true;
    i2sDriverOk = true;
    i2sLastError = ESP_OK;
    simplePrintln("I2S ready (modern ESP-IDF PDM RX): " + String(currentSampleRate) + "Hz, gain " +
                  String(currentGainFactor, 1) + ", buffer " + String(currentBufferSize) +
                  " (chunk " + String(effectiveAudioChunkSize()) + ")" +
                  ", dma " + String(dmaBufCount) + "x" + String(dmaBufLen) +
                  ", read " + String(computeI2sReadBufferSamples()) +
                  ", slot " + String(i2sPdmUseLeftSlot ? "left" : "right") +
                  (i2sPdmClkInverted ? ", clk inverted" : ", normal clock") +
                  ", driver HPF off" +
                  (i2sPdmProbeFoundSigned ? ", signed PCM" : ", unsigned fallback armed") +
                  ", shiftBits " + String(i2sShiftBits));
    return true;
}

void setup_i2s_driver() {
    i2sDriverOk = false;
    i2sLastError = ESP_OK;
    i2sLikelyUnsignedPcm = false;
    i2sUsingModernPdmDriver = false;
    i2sPdmUseLeftSlot = false;
    i2sPdmClkInverted = false;
    i2sPdmProbeFoundSigned = false;

    const uint16_t dma_buf_len = computeI2sDmaBufferLen();
    const uint8_t dma_buf_count = computeI2sDmaBufferCount();
    shutdownModernI2sDriver();

    (void)setupModernPdmDriver(dma_buf_len, dma_buf_count);
}

static bool writeAll(WiFiClient &client, const uint8_t* data, size_t len, unsigned long timeoutMs) {
    if (!client.connected()) return false;
    int fd = client.fd();
    if (fd < 0) return false;

    size_t off = 0;
    unsigned long deadline = millis() + timeoutMs;

    while (off < len) {
        unsigned long now = millis();
        if ((long)(deadline - now) <= 0) {
            return false;
        }

        fd_set writeSet;
        FD_ZERO(&writeSet);
        FD_SET(fd, &writeSet);

        unsigned long remainingMs = deadline - now;
        unsigned long waitMs = min(remainingMs, 5UL);
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = (suseconds_t)(waitMs * 1000UL);

        int ready = select(fd + 1, NULL, &writeSet, NULL, &tv);
        if (ready < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        if (ready == 0 || !FD_ISSET(fd, &writeSet)) {
            continue;
        }

        ssize_t sent = send(fd, data + off, len - off, MSG_DONTWAIT);
        if (sent > 0) {
            off += (size_t)sent;
            continue;
        }
        if (sent < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
            continue;
        }
        if (sent < 0) {
            client.stop();
        }
        return false;
    }
    return true;
}

static int parseRtspCSeqValue(const char* request) {
    if (!request) return 1;
    const char* cseqStr = strstr(request, "CSeq:");
    if (!cseqStr) cseqStr = strstr(request, "Cseq:");
    if (!cseqStr) cseqStr = strstr(request, "cseq:");
    if (!cseqStr) return 1;
    cseqStr += 5;
    while (*cseqStr == ' ' || *cseqStr == '\t') cseqStr++;
    uint32_t cseqVal = 0;
    bool hasDigit = false;
    while (*cseqStr >= '0' && *cseqStr <= '9') {
        hasDigit = true;
        uint32_t digit = (uint32_t)(*cseqStr - '0');
        if (cseqVal > (UINT32_MAX - digit) / 10U) return 1;
        cseqVal = (cseqVal * 10U) + digit;
        cseqStr++;
    }
    while (*cseqStr == ' ' || *cseqStr == '\t') cseqStr++;
    if (!hasDigit || (*cseqStr != '\r' && *cseqStr != '\n' && *cseqStr != '\0')) return 1;
    return (cseqVal > 0 && cseqVal <= INT_MAX) ? (int)cseqVal : 1;
}

static uint16_t parseRtspContentLengthValue(const char* request) {
    if (!request) return 0;
    const char* lenStr = strstr(request, "Content-Length:");
    if (!lenStr) lenStr = strstr(request, "Content-length:");
    if (!lenStr) lenStr = strstr(request, "content-length:");
    if (!lenStr) return 0;
    lenStr += 15;
    while (*lenStr == ' ' || *lenStr == '\t') lenStr++;
    uint32_t len = 0;
    bool hasDigit = false;
    while (*lenStr >= '0' && *lenStr <= '9') {
        hasDigit = true;
        uint32_t digit = (uint32_t)(*lenStr - '0');
        if (len > (UINT32_MAX - digit) / 10U) return 0;
        len = (len * 10U) + digit;
        lenStr++;
    }
    while (*lenStr == ' ' || *lenStr == '\t') lenStr++;
    if (!hasDigit || (*lenStr != '\r' && *lenStr != '\n' && *lenStr != '\0')) return 0;
    if (len > 2048U) return 2048U;
    return (uint16_t)len;
}

static bool writeRtspSimpleResponse(RtspSession &session, int cseqVal, const char* extraHeaders) {
    char resp[256];
    int rlen = snprintf(resp, sizeof(resp),
                        "RTSP/1.0 200 OK\r\n"
                        "CSeq: %d\r\n"
                        "Session: %s\r\n"
                        "Connection: keep-alive\r\n"
                        "%s"
                        "\r\n",
                        cseqVal,
                        session.sessionId.length() ? session.sessionId.c_str() : "1",
                        extraHeaders ? extraHeaders : "");
    if (rlen <= 0) return false;
    if (rlen >= (int)sizeof(resp)) rlen = (int)sizeof(resp) - 1;
    return writeAll(session.client, (const uint8_t*)resp, (size_t)rlen, 80);
}

static bool handleStreamingRtspCommand(RtspSession &session, const char* request) {
    if (!request || request[0] == '\0') return true;

    // Some desktop clients send OPTIONS or SET_PARAMETER as keepalive during
    // PLAY. A missed response can make them abandon a stream that is still
    // carrying RTP, which presents as "spectrogram starts, then goes silent".
    int cseqVal = parseRtspCSeqValue(request);
    if (strstr(request, "TEARDOWN") != NULL) {
        writeRtspSimpleResponse(session, cseqVal);
        closeRtspSession(session, true);
        Serial.println("[Core1] TEARDOWN received, stream stopped");
        return false;
    }

    if (strstr(request, "GET_PARAMETER") != NULL) {
        writeRtspSimpleResponse(session, cseqVal);
        lastRTSPActivity = millis();
        session.lastActivity = millis();
    } else if (strstr(request, "OPTIONS") != NULL) {
        writeRtspSimpleResponse(session, cseqVal,
            "Public: OPTIONS, DESCRIBE, SETUP, PLAY, TEARDOWN, GET_PARAMETER, SET_PARAMETER\r\n");
        lastRTSPActivity = millis();
        session.lastActivity = millis();
    } else if (strstr(request, "SET_PARAMETER") != NULL) {
        writeRtspSimpleResponse(session, cseqVal);
        lastRTSPActivity = millis();
        session.lastActivity = millis();
    }

    return true;
}

static void pollStreamingRtspCommands(RtspSession &session) {
    WiFiClient &client = session.client;
    char* rtspBuf = session.streamingRtspBuf;
    size_t rtspBufSize = sizeof(session.streamingRtspBuf);
    size_t &rtspBufPos = session.streamingRtspBufPos;
    uint16_t &interleavedDiscardRemaining = session.streamingInterleavedDiscard;
    if (!rtspBuf || rtspBufSize < 8) return;

    uint8_t discardBuf[64];
    while (client.connected() && client.available() > 0) {
        if (interleavedDiscardRemaining > 0) {
            int toRead = min((int)sizeof(discardBuf), min(client.available(), (int)interleavedDiscardRemaining));
            int n = client.read(discardBuf, toRead);
            if (n <= 0) return;
            interleavedDiscardRemaining -= (uint16_t)n;
            continue;
        }

        if (rtspBufPos == 0 && client.peek() == '$') {
            if (client.available() < 4) return;
            uint8_t hdr[4];
            int n = client.read(hdr, sizeof(hdr));
            if (n != (int)sizeof(hdr)) return;
            interleavedDiscardRemaining = ((uint16_t)hdr[2] << 8) | (uint16_t)hdr[3];
            continue;
        }

        int c = client.read();
        if (c < 0) return;

        if (rtspBufPos >= rtspBufSize - 1) {
            rtspBufPos = 0;
            rtspBuf[0] = '\0';
            Serial.println("[Core1] Streaming RTSP buffer overflow, reset parser");
            return;
        }

        rtspBuf[rtspBufPos++] = (char)c;
        rtspBuf[rtspBufPos] = '\0';

        if (rtspBufPos >= 4 &&
            rtspBuf[rtspBufPos - 4] == '\r' &&
            rtspBuf[rtspBufPos - 3] == '\n' &&
            rtspBuf[rtspBufPos - 2] == '\r' &&
            rtspBuf[rtspBufPos - 1] == '\n') {
            rtspBuf[rtspBufPos - 4] = '\0';
            uint16_t bodyBytes = parseRtspContentLengthValue(rtspBuf);
            bool keepOpen = handleStreamingRtspCommand(session, rtspBuf);
            rtspBufPos = 0;
            rtspBuf[0] = '\0';
            if (bodyBytes > 0) {
                interleavedDiscardRemaining = bodyBytes;
            }
            if (!keepOpen) return;
        }
    }
}

static const uint32_t MAX_WRITE_FAILURES = 48;
static const uint32_t MAX_WRITE_FAILURE_MS = 1200;

static void sendRTPPacket(RtspSession &session, int16_t* audioData, int numSamples, uint8_t activeClientCount) {
    WiFiClient &client = session.client;
    if (!client.connected()) {
        // Client disconnected — Core 1 owns the socket, close it
        closeRtspSession(session, true);
        return;
    }

    const int maxSamplesPerPacket = (int)effectiveAudioChunkSize();
    int offsetSamples = 0;
    while (offsetSamples < numSamples) {
        const int packetSamples = min(maxSamplesPerPacket, numSamples - offsetSamples);
        const uint16_t payloadSize = (uint16_t)(packetSamples * (int)sizeof(int16_t));
        const uint16_t packetSize = (uint16_t)(12 + payloadSize);
        const unsigned long blockMs = max(1UL, (unsigned long)(((uint32_t)packetSamples * 1000UL) / max((uint32_t)1, currentSampleRate)));
        const unsigned long packetWindowMs = max(6UL, min(18UL, blockMs / max((uint8_t)1, activeClientCount)));
        uint8_t packetBuffer[4 + 12 + (1024 * sizeof(int16_t))];

        // RTSP interleaved header + RTP header + network-order PCM payload.
        packetBuffer[0] = 0x24;
        packetBuffer[1] = 0x00;
        packetBuffer[2] = (uint8_t)((packetSize >> 8) & 0xFF);
        packetBuffer[3] = (uint8_t)(packetSize & 0xFF);
        uint8_t* rtp = packetBuffer + 4;
        rtp[0] = 0x80;      // V=2, P=0, X=0, CC=0
        rtp[1] = 96;        // M=0, PT=96 (dynamic)
        rtp[2] = (uint8_t)((session.rtpSequence >> 8) & 0xFF);
        rtp[3] = (uint8_t)(session.rtpSequence & 0xFF);
        rtp[4] = (uint8_t)((session.rtpTimestamp >> 24) & 0xFF);
        rtp[5] = (uint8_t)((session.rtpTimestamp >> 16) & 0xFF);
        rtp[6] = (uint8_t)((session.rtpTimestamp >> 8) & 0xFF);
        rtp[7] = (uint8_t)(session.rtpTimestamp & 0xFF);
        rtp[8]  = (uint8_t)((session.rtpSSRC >> 24) & 0xFF);
        rtp[9]  = (uint8_t)((session.rtpSSRC >> 16) & 0xFF);
        rtp[10] = (uint8_t)((session.rtpSSRC >> 8) & 0xFF);
        rtp[11] = (uint8_t)(session.rtpSSRC & 0xFF);

        const int16_t* packetAudio = audioData + offsetSamples;
        for (int i = 0; i < packetSamples; ++i) {
            uint16_t s = (uint16_t)packetAudio[i];
            rtp[12 + (i * 2)] = (uint8_t)((s >> 8) & 0xFF);
            rtp[12 + (i * 2) + 1] = (uint8_t)(s & 0xFF);
        }

        bool success = false;
        if (session.transport == RTSP_TRANSPORT_TCP_INTERLEAVED) {
            success = writeAll(client, packetBuffer, (size_t)packetSize + 4U, packetWindowMs);
        }

        if (success) {
            session.rtpSequence++;
            session.rtpTimestamp += (uint32_t)packetSamples;
            audioPacketsSent++;
            session.consecutiveWriteFailures = 0;  // Reset on success
            session.firstWriteFailureAt = 0;
            offsetSamples += packetSamples;
        } else {
            audioPacketsDropped++;
            session.consecutiveWriteFailures++;
            if (session.firstWriteFailureAt == 0) {
                session.firstWriteFailureAt = millis();
            }

            if (session.consecutiveWriteFailures >= MAX_WRITE_FAILURES ||
                (millis() - session.firstWriteFailureAt) >= MAX_WRITE_FAILURE_MS) {
                // Sustained failure — Core 1 owns the socket, close it
                Serial.printf("[Core1] write failures (%u over %lu ms), disconnecting %s\n",
                              session.consecutiveWriteFailures,
                              millis() - session.firstWriteFailureAt,
                              session.remoteAddr.c_str());
                closeRtspSession(session, true);
            }
            return;
        }
    }
}

static void sendRTPPacketsToActiveSessions(int16_t* audioData, int numSamples) {
    uint8_t activeClientCount = countStreamingRtspClients();
    for (uint8_t i = 0; i < MAX_RTSP_CLIENTS; ++i) {
        if (rtspSessions[i].occupied && rtspSessions[i].streaming) {
            sendRTPPacket(rtspSessions[i], audioData, numSamples, activeClientCount);
        }
    }
}

// ================== CORE 0: NO AUDIO STREAMING ==================
// Audio streaming is now handled entirely on Core 1
// Core 0 only manages client connections and RTSP protocol
// (streamAudio function removed - handled by Core 1 audioCaptureTask)

// RTSP handling
void handleRTSPCommand(RtspSession &session, const String &request) {
    WiFiClient &client = session.client;
    int cseqVal = parseRtspCSeqValue(request.c_str());
    String cseq = String(cseqVal);

    lastRTSPActivity = millis();
    session.lastActivity = millis();

    if (rtspMethodIs(request.c_str(), "OPTIONS")) {
        client.print("RTSP/1.0 200 OK\r\n");
        client.print("CSeq: " + cseq + "\r\n");
        client.print("Public: OPTIONS, DESCRIBE, SETUP, PLAY, TEARDOWN, GET_PARAMETER, SET_PARAMETER\r\n\r\n");

    } else if (rtspMethodIs(request.c_str(), "DESCRIBE")) {
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

    } else if (rtspMethodIs(request.c_str(), "SETUP")) {
        if (session.sessionId.length() == 0) {
            session.sessionId = String(random(100000000, 999999999));
        }
        if (session.rtpSSRC == 0) {
            session.rtpSSRC = (uint32_t)random(1, 0x7FFFFFFF);
        }

        String transport = extractRtspHeader(request, "Transport");
        String transportLower = transport;
        transportLower.toLowerCase();
        bool wantsTcp = transportLower.indexOf("rtp/avp/tcp") >= 0 || transportLower.indexOf("interleaved=") >= 0;
        bool wantsUdp = transportLower.indexOf("rtp/avp") >= 0 && transportLower.indexOf("client_port=") >= 0 && transportLower.indexOf("rtp/avp/tcp") < 0;

        session.transport = RTSP_TRANSPORT_TCP_INTERLEAVED;

        if (wantsUdp) {
            if (!RTSP_ENABLE_UDP_UNICAST) {
                client.print("RTSP/1.0 461 Unsupported Transport\r\n");
                client.print("CSeq: " + cseq + "\r\n");
                client.print("Connection: keep-alive\r\n\r\n");
                simplePrintln("RTSP SETUP rejected UDP transport; waiting for TCP interleaved retry");
                return;
            }
        } else if (!wantsTcp && transport.length() > 0) {
            client.print("RTSP/1.0 461 Unsupported Transport\r\n");
            client.print("CSeq: " + cseq + "\r\n\r\n");
            return;
        }

        client.print("RTSP/1.0 200 OK\r\n");
        client.print("CSeq: " + cseq + "\r\n");
        // Large timeout reduces ffmpeg keepalive frequency (keepalive sent at timeout/2)
        client.print("Session: " + session.sessionId + ";timeout=86400\r\n");
        client.print("Transport: RTP/AVP/TCP;unicast;interleaved=0-1;mode=\"PLAY\";ssrc=" +
                     String(session.rtpSSRC, HEX) + "\r\n\r\n");
        session.setupDone = true;
        simplePrintln("RTSP SETUP using TCP interleaved transport for " + session.remoteAddr);

    } else if (rtspMethodIs(request.c_str(), "PLAY")) {
        if (!session.setupDone) {
            client.print("RTSP/1.0 455 Method Not Valid in This State\r\n");
            client.print("CSeq: " + cseq + "\r\n");
            client.print("Connection: keep-alive\r\n\r\n");
            simplePrintln("PLAY rejected: SETUP required before PLAY for " + session.remoteAddr);
            return;
        }

        if (!i2sDriverOk) {
            client.print("RTSP/1.0 503 Service Unavailable\r\n");
            client.print("CSeq: " + cseq + "\r\n");
            client.print("Connection: close\r\n\r\n");
            simplePrintln("PLAY rejected: I2S driver is not ready");
            delay(5);
            client.stop();
            return;
        }

        bool hadActiveStreams = (countStreamingRtspClients() > 0);
        session.rtpSequence = 0;
        session.rtpTimestamp = 0;
        session.consecutiveWriteFailures = 0;
        session.firstWriteFailureAt = 0;
        if (!hadActiveStreams) {
            audioPacketsSent = 0;
            audioPacketsDropped = 0;
            audioBlocksSent = 0;
            lastStatsReset = millis();
        }
        if (audioCaptureTaskHandle == NULL && !startAudioCaptureTask()) {
            client.print("RTSP/1.0 503 Service Unavailable\r\n");
            client.print("CSeq: " + cseq + "\r\n");
            client.print("Connection: close\r\n\r\n");
            simplePrintln("PLAY rejected: audio pipeline task could not start");
            delay(5);
            closeRtspSession(session, true);
            return;
        }
        lastRtspPlayMs = millis();
        rtspPlayCount++;

        // Send PLAY response only after the Core 1 audio task is available.
        String playUrl = "rtsp://" + WiFi.localIP().toString() + ":8554/audio/track1";
        client.print("RTSP/1.0 200 OK\r\n");
        client.print("CSeq: " + cseq + "\r\n");
        client.print("Session: " + session.sessionId + "\r\n");
        client.print("Range: npt=0.000-\r\n");
        client.print("RTP-Info: url=" + playUrl + ";seq=0;rtptime=0\r\n\r\n");

        // Initialize stream stop flags before handing off
        stopStreamRequested = false;
        streamCleanupDone = false;
        core1OwnsLED = true;
        __asm__ __volatile__("memw" ::: "memory");

        session.streaming = true;
        session.playStartedAt = millis();
        updateStreamingStateFromSessions();

        // Core 1 now owns LED during streaming
        simplePrintln("STREAMING STARTED for " + session.remoteAddr + " via " +
                      String(rtspTransportModeName(session.transport)));

    } else if (rtspMethodIs(request.c_str(), "TEARDOWN")) {
        client.print("RTSP/1.0 200 OK\r\n");
        client.print("CSeq: " + cseq + "\r\n");
        client.print("Session: " + session.sessionId + "\r\n\r\n");
        closeRtspSession(session, true);

        if (!core1OwnsLED) {
            if (ledMode > 0) setStatusLed(CRGB(0, 0, 128));
            else setStatusLed(CRGB(0, 0, 0));
        }
        simplePrintln("STREAMING STOPPED");
    } else if (rtspMethodIs(request.c_str(), "GET_PARAMETER")) {
        // Many RTSP clients send GET_PARAMETER as keep-alive.
        client.print("RTSP/1.0 200 OK\r\n");
        client.print("CSeq: " + cseq + "\r\n\r\n");
    } else if (rtspMethodIs(request.c_str(), "SET_PARAMETER")) {
        client.print("RTSP/1.0 200 OK\r\n");
        client.print("CSeq: " + cseq + "\r\n\r\n");
    } else {
        client.print("RTSP/1.0 405 Method Not Allowed\r\n");
        client.print("CSeq: " + cseq + "\r\n");
        client.print("Public: OPTIONS, DESCRIBE, SETUP, PLAY, TEARDOWN, GET_PARAMETER, SET_PARAMETER\r\n\r\n");
    }
}

// RTSP processing for sessions that are still owned by Core 0.
void processRTSP(RtspSession &session) {
    WiFiClient &client = session.client;
    if (!client.connected()) return;

    if (client.available()) {
        int available = client.available();
        int spaceLeft = (int)sizeof(session.parseBuffer) - session.parseBufferPos - 1;

        if (available > spaceLeft) {
            available = spaceLeft;
        }

        if (available <= 0) {
            static unsigned long lastOverflowWarning = 0;
            if (millis() - lastOverflowWarning > 5000) {
                simplePrintln("RTSP buffer full - closing " + session.remoteAddr);
                lastOverflowWarning = millis();
            }
            client.print("RTSP/1.0 413 Request Entity Too Large\r\nConnection: close\r\n\r\n");
            closeRtspSession(session, true);
            return;
        }

        int readBytes = client.read(session.parseBuffer + session.parseBufferPos, available);
        if (readBytes <= 0) {
            return;
        }
        session.parseBufferPos += readBytes;
        session.parseBuffer[session.parseBufferPos] = '\0';

        char* endOfHeader = strstr((char*)session.parseBuffer, "\r\n\r\n");
        while (endOfHeader != nullptr) {
            int headerLen = (endOfHeader - (char*)session.parseBuffer) + 4;
            uint16_t bodyBytes = parseRtspContentLengthValue((char*)session.parseBuffer);
            int totalMessageLen = headerLen + (int)bodyBytes;
            if (totalMessageLen >= (int)sizeof(session.parseBuffer)) {
                simplePrintln("RTSP request too large - closing " + session.remoteAddr);
                client.print("RTSP/1.0 413 Request Entity Too Large\r\nConnection: close\r\n\r\n");
                closeRtspSession(session, true);
                return;
            }
            if (session.parseBufferPos < totalMessageLen) {
                return;
            }

            *endOfHeader = '\0';
            String request = String((char*)session.parseBuffer);

            handleRTSPCommand(session, request);
            if (!session.occupied || session.streaming) {
                return;
            }

            int remaining = session.parseBufferPos - totalMessageLen;
            if (remaining > 0) {
                memmove(session.parseBuffer, session.parseBuffer + totalMessageLen, remaining);
            }
            session.parseBufferPos = max(0, remaining);
            session.parseBuffer[session.parseBufferPos] = '\0';
            endOfHeader = strstr((char*)session.parseBuffer, "\r\n\r\n");
        }
    }
}

static void rejectBusyRtspClient(WiFiClient &client) {
    if (!client) return;
    client.setNoDelay(true);

    char req[256];
    int n = 0;
    unsigned long deadline = millis() + 120UL;
    while (client.connected() && client.available() == 0 && millis() < deadline) {
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    if (client.available() > 0) {
        int avail = client.available();
        if (avail > (int)sizeof(req) - 1) avail = sizeof(req) - 1;
        n = client.read((uint8_t*)req, avail);
    }
    if (n < 0) n = 0;
    req[n] = '\0';

    char cseq[16] = "1";
    const char* cseqStart = strstr(req, "CSeq:");
    if (cseqStart) {
        cseqStart += 5;
        while (*cseqStart == ' ' || *cseqStart == '\t') cseqStart++;
        size_t i = 0;
        while (i < sizeof(cseq) - 1 && cseqStart[i] >= '0' && cseqStart[i] <= '9') {
            cseq[i] = cseqStart[i];
            i++;
        }
        cseq[i] = '\0';
        if (i == 0) strcpy(cseq, "1");
    }

    char resp[160];
    int len = snprintf(resp, sizeof(resp),
                       "RTSP/1.0 453 Not Enough Bandwidth\r\n"
                       "CSeq: %s\r\n"
                       "Connection: close\r\n\r\n",
                       cseq);
    if (len > 0) {
        client.write((const uint8_t*)resp, (size_t)len);
    }
    delay(5);
    client.stop();
    rtspRejectedClientCount++;
    simplePrintln("Rejected extra RTSP client while stream is busy");
}


// Web UI is a separate module (WebUI.*)

void setup() {
    setStatusLed(CRGB(0, 0, 0));

    Serial.begin(115200);
    delay(500);
    Serial.println("\n\n=== ESP32 RTSP Mic Starting ===");
    Serial.println("Board: M5Stack AtomS3 Lite");

    // Create task exit semaphore for confirmed Core 1 task shutdown
    taskExitSemaphore = xSemaphoreCreateBinary();

    // Set LED to indicate startup
    setStatusLed(CRGB(128, 128, 0));  // Yellow for startup

    // (4) seed for random(): combination of time and unique MAC
    randomSeed((uint32_t)micros() ^ (uint32_t)(ESP.getEfuseMac() & 0xFFFFFFFF));
    for (uint16_t i = 0; i < TELEMETRY_HISTORY_LEN; ++i) {
        telemetryTempDeciC[i] = INT16_MIN;
    }

    bootTime = millis(); // Store boot time
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

    WiFi.setHostname(DEFAULT_NETWORK_HOSTNAME);

    WiFiManager wm;
    wm.setConnectTimeout(60);
    wm.setConfigPortalTimeout(180);
    if (!wm.autoConnect("ESP32-RTSP-Mic-AP")) {
        simplePrintln("WiFi failed, restarting...");
        ESP.restart();
    }

    simplePrintln("WiFi connected: " + WiFi.localIP().toString());

    // NTP time sync (UTC by default so timestamps stay correct across regions and DST changes)
    configTzTime(LOG_TIMEZONE_POSIX, "pool.ntp.org");
    Serial.print("Waiting for NTP time sync...");
    time_t now = 0;
    for (int i = 0; i < 20 && now < 100000; i++) {
        delay(250);
        time(&now);
    }
    if (now > 100000) {
        struct tm ti;
        localtime_r(&now, &ti);
        char buf[32];
        strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &ti);
        Serial.printf(" OK: %s %s\n", buf, LOG_TIMEZONE_LABEL);
    } else {
        Serial.println(" failed (will use uptime)");
    }

    // Apply configured WiFi TX power after connect (logs once on change)
    applyWifiTxPower(true);

    const char* wifiHostname = WiFi.getHostname();
    if (wifiHostname && wifiHostname[0] != '\0') {
        networkHostname = String(wifiHostname);
    }

    // mDNS: allows rtsp://<hostname>.local:8554/audio
    if (MDNS.begin(networkHostname.c_str())) {
        MDNS.addService("rtsp", "tcp", 8554);
        MDNS.addService("http", "tcp", 80);
        MDNS.addService("arduino", "tcp", OTA_PORT);
        simplePrintln("mDNS: " + networkHostname + ".local");
    }

    setupArduinoOta();

    Serial.println("Setting up I2S driver...");
    setup_i2s_driver();
    if (i2sDriverOk) {
        Serial.println("I2S driver ready");
        Serial.println("Updating highpass coefficients...");
        updateHighpassCoeffs();
        Serial.println("Highpass coefficients updated");
    } else {
        Serial.printf("I2S driver setup failed, audio pipeline not started (err=%ld)\n", (long)i2sLastError);
    }

    if (!overheatLatched) {
        rtspServer.begin();
        rtspServer.setNoDelay(true);
        rtspServerEnabled = true;
    } else {
        rtspServerEnabled = false;
        rtspServer.stop();
    }
    if (i2sDriverOk && rtspServerEnabled) {
        if (startAudioCaptureTask()) {
            Serial.println("Dual-core audio ready (Core 1 pipeline running for live diagnostics)");
        } else {
            Serial.println("Audio task start failed; RTSP server left online for diagnostics only");
        }
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
        simplePrintln("RTSP URL: rtsp://" + networkHostname + ".local:8554/audio");
        // Set LED to blue when ready (green reserved for level indicator)
        if (ledMode > 0) setStatusLed(CRGB(0, 0, 128));
        else setStatusLed(CRGB(0, 0, 0));
    } else {
        simplePrintln("RTSP server paused due to thermal latch. Clear via Web UI before resuming streaming.");
        // Set LED to red when latched
        setStatusLed(CRGB(128, 0, 0));
    }
    simplePrintln("Web UI: http://" + WiFi.localIP().toString() + "/");
}

void loop() {
    webui_handleClient();
    ArduinoOTA.handle();

    if (millis() - lastTempCheck > 5000) { // 5 s
        checkTemperature();
        lastTempCheck = millis();
    }

    if (millis() - lastTelemetrySampleMs > 3000) {
        recordTelemetrySample(audioPipelineLoadPct, lastTemperatureC, lastTemperatureValid);
        lastTelemetrySampleMs = millis();
    }

    // Heap monitoring (every 10 minutes — useful for detecting leaks in long deployments)
    if (millis() - lastMemoryCheck > 600000) { // 10 min
        uint32_t currentHeap = ESP.getFreeHeap();
        if (currentHeap < minFreeHeap) minFreeHeap = currentHeap;
        Serial.printf("[Heap] Current: %u KB, Min: %u KB\n", currentHeap / 1024, minFreeHeap / 1024);
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

    // RTSP client management (Core 0) — clear phase separation
    static bool wasStreaming = false;
    if (rtspServerEnabled) {
        // Phase: detect disconnect (Core 1 cleared isStreaming after self-disconnect)
        if (wasStreaming && !isStreaming) {
            // Core 1 already closed the socket — just update LED and log
            if (!core1OwnsLED) {
                if (ledMode > 0) setStatusLed(CRGB(0, 0, 128));
                else setStatusLed(CRGB(0, 0, 0));
            }
            unsigned long sessionSec = (millis() - lastRtspPlayMs) / 1000;
            simplePrintln("RTSP client disconnected (session: " + String(sessionSec) + "s, dropped: " +
                         String(audioPacketsDropped) + ", RSSI: " + String(WiFi.RSSI()) + " dBm)");
        }
        wasStreaming = isStreaming;

        // Disconnect idle clients that connected but never entered PLAY.
        for (uint8_t i = 0; i < MAX_RTSP_CLIENTS; ++i) {
            RtspSession &session = rtspSessions[i];
            if (session.occupied && !session.streaming) {
                if (!session.client || !session.client.connected()) {
                    closeRtspSession(session, false);
                } else if (millis() - session.lastActivity > 60000) {
                    simplePrintln("RTSP idle timeout — disconnecting " + session.remoteAddr);
                    closeRtspSession(session, true);
                }
            }
        }

        // Phase: accept new RTSP clients while preserving Core 1 ownership of
        // already-playing sockets.
        WiFiClient newClient = rtspServer.accept();
        if (newClient) {
            int slot = findFreeRtspSession();
            if (slot < 0) {
                rejectBusyRtspClient(newClient);
            } else {
                RtspSession &session = rtspSessions[slot];
                session.client = newClient;
                session.client.setNoDelay(true);
                session.occupied = true;
                session.streaming = false;
                session.setupDone = false;
                session.sessionId = "";
                session.remoteAddr = session.client.remoteIP().toString();
                session.transport = RTSP_TRANSPORT_TCP_INTERLEAVED;
                session.rtpSequence = 0;
                session.rtpTimestamp = 0;
                session.rtpSSRC = (uint32_t)random(1, 0x7FFFFFFF);
                session.connectedAt = millis();
                session.lastActivity = millis();
                session.playStartedAt = 0;
                session.parseBufferPos = 0;
                session.streamingRtspBufPos = 0;
                session.streamingInterleavedDiscard = 0;
                session.consecutiveWriteFailures = 0;
                session.firstWriteFailureAt = 0;
                lastRTSPActivity = millis();
                lastRtspClientConnectMs = millis();
                rtspConnectCount++;
                simplePrintln("New RTSP client connected: " + session.remoteAddr);
            }
        }

        // Phase: RTSP negotiation for sockets not yet handed to Core 1.
        for (uint8_t i = 0; i < MAX_RTSP_CLIENTS; ++i) {
            RtspSession &session = rtspSessions[i];
            if (session.occupied && !session.streaming) {
                processRTSP(session);
            }
        }
    } else {
        // RTSP server disabled (overheat lockout)
        if (isStreaming) {
            requestStreamStop("server disabled");
        }
        if (audioTaskRunning) {
            stopAudioCaptureTask();
        }
        for (uint8_t i = 0; i < MAX_RTSP_CLIENTS; ++i) {
            if (rtspSessions[i].occupied && !rtspSessions[i].streaming) {
                closeRtspSession(rtspSessions[i], true);
            }
        }
        if (!core1OwnsLED) {
            if (overheatLatched) {
                setStatusLed(CRGB(128, 0, 0));
            } else {
                if (ledMode > 0) setStatusLed(CRGB(0, 0, 128));
                else setStatusLed(CRGB(0, 0, 0));
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
