#include <Arduino.h>
#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <WiFi.h>
#include <WebServer.h>
#include "WebUIAssets.h"
#include "WebUI.h"

// External variables and functions from main (.ino) – ESP32 RTSP Mic for BirdNET-Go
extern WiFiServer rtspServer;
extern volatile bool isStreaming;
extern unsigned long lastStatsReset;
extern unsigned long lastRtspPlayMs;
extern uint32_t rtspPlayCount;
extern const char* currentRtspTransportName();
extern uint8_t getConnectedRtspClientCount();
extern uint8_t getActiveRtspClientCount();
extern uint8_t getMaxRtspClientCount();
extern String getRtspClientSummary();
extern unsigned long lastRtspClientConnectMs;
extern unsigned long bootTime;
extern unsigned long lastRTSPActivity;
extern unsigned long lastWiFiCheck;
extern unsigned long lastTempCheck;
extern uint32_t minFreeHeap;
extern float maxTemperature;
extern bool rtspServerEnabled;
extern unsigned long audioPacketsSent;
extern unsigned long audioPacketsDropped;
extern unsigned long audioBlocksSent;
extern uint32_t rtspRejectedClientCount;
extern uint32_t currentSampleRate;
extern bool isSupportedPdmSampleRate(uint32_t rate);
extern float currentGainFactor;
extern uint16_t currentBufferSize;
extern uint8_t i2sShiftBits;
extern bool highpassEnabled;
extern uint16_t highpassCutoffHz;
extern uint32_t minAcceptableRate;
extern uint32_t performanceCheckInterval;
extern bool autoRecoveryEnabled;
extern uint8_t cpuFrequencyMhz;
extern wifi_power_t currentWifiPowerLevel;
extern void resetToDefaultSettings();
extern bool autoThresholdEnabled;
extern uint32_t computeRecommendedMinRate();
extern uint16_t effectiveAudioChunkSize();
extern bool scheduledResetEnabled;
extern uint32_t resetIntervalHours;
extern void scheduleReboot(bool factoryReset, uint32_t delayMs);
extern uint16_t lastPeakAbs16;
extern uint32_t audioClipCount;
extern bool audioClippedLastBlock;
extern uint16_t peakHoldAbs16;
extern bool overheatProtectionEnabled;
extern float overheatShutdownC;
extern bool overheatLockoutActive;
extern float overheatTripTemp;
extern unsigned long overheatTriggeredAt;
extern String overheatLastReason;
extern String overheatLastTimestamp;
extern bool overheatSensorFault;
extern float lastTemperatureC;
extern bool lastTemperatureValid;
extern bool overheatLatched;
extern bool agcEnabled;
extern volatile float agcMultiplier;
extern volatile bool noiseFilterEnabled;
extern volatile float noiseFloorDbfs;
extern volatile float noiseGateDbfs;
extern volatile float noiseReductionDb;
extern portMUX_TYPE diagMux;
extern uint8_t ledMode;
extern volatile uint32_t i2sReadOkCount;
extern volatile uint32_t i2sReadErrCount;
extern volatile uint32_t i2sReadZeroCount;
extern volatile uint16_t i2sLastSamplesRead;
extern volatile int16_t i2sLastRawMin;
extern volatile int16_t i2sLastRawMax;
extern volatile uint16_t i2sLastRawPeakAbs;
extern volatile uint16_t i2sLastRawRms;
extern volatile uint16_t i2sLastRawZeroPct;
extern volatile int16_t i2sLastRawMean;
extern volatile uint16_t i2sLastRawSpan;
extern volatile bool i2sLikelyUnsignedPcm;
extern volatile bool i2sUsingModernPdmDriver;
extern volatile bool i2sPdmUseLeftSlot;
extern volatile bool i2sPdmClkInverted;
extern volatile bool i2sPdmProbeFoundSigned;
extern volatile bool i2sDriverOk;
extern volatile int32_t i2sLastError;
extern volatile uint32_t audioFallbackBlockCount;
extern volatile uint32_t i2sLastGapMs;
extern volatile float audioPipelineLoadPct;
extern volatile bool audioTaskRunning;
extern bool startAudioCaptureTask();
extern void stopAudioCaptureTask();
extern portMUX_TYPE fftMux;
extern volatile uint8_t fftBins[WEBUI_FFT_BINS];
extern volatile uint32_t fftFrameSeq;
extern portMUX_TYPE webAudioMux;
extern volatile uint32_t webAudioFrameSeq;
extern volatile uint16_t webAudioFrameSamples[WEBUI_AUDIO_RING_LEN];
extern volatile uint32_t webAudioSampleRate[WEBUI_AUDIO_RING_LEN];
extern volatile uint32_t webAudioRingSeq[WEBUI_AUDIO_RING_LEN];
extern int16_t webAudioFrame[WEBUI_AUDIO_RING_LEN][WEBUI_AUDIO_MAX_SAMPLES];
extern portMUX_TYPE telemetryMux;
extern volatile uint32_t telemetryHistorySeq;
extern uint8_t telemetryCpuLoadPct[];
extern int16_t telemetryTempDeciC[];
extern uint16_t telemetryHistoryHead;
extern uint16_t telemetryHistoryCount;
static const uint16_t TELEMETRY_HISTORY_LEN = 120;

struct WebAudioSnapshot {
    uint32_t seq;
    uint16_t samples;
    uint32_t rate;
    uint8_t frames;
    uint32_t dropped;
    int16_t frame[WEBUI_AUDIO_MAX_SAMPLES];
};
static WebAudioSnapshot webAudioHttpSnapshot;

// Local helper: snap requested Wi‑Fi TX power (dBm) to nearest supported step
static float snapWifiTxDbm(float dbm) {
    static const float steps[] = {-1.0f, 2.0f, 5.0f, 7.0f, 8.5f, 11.0f, 13.0f, 15.0f, 17.0f, 18.5f, 19.0f, 19.5f};
    float best = steps[0];
    float bestd = fabsf(dbm - steps[0]);
    for (size_t i=1;i<sizeof(steps)/sizeof(steps[0]);++i){
        float d = fabsf(dbm - steps[i]);
        if (d < bestd){ bestd = d; best = steps[i]; }
    }
    return best;
}

static const uint32_t OH_MIN = 30;
static const uint32_t OH_MAX = 95;
static const uint32_t OH_STEP = 5;

// Helper functions in main
extern float wifiPowerLevelToDbm(wifi_power_t lvl);
extern String formatUptime(unsigned long seconds);
extern String formatSince(unsigned long eventMs);
extern void restartI2S();
extern void saveAudioSettings();
extern void applyWifiTxPower(bool log);
extern const char* FW_VERSION_STR;
extern String describeHardwareProfile();
extern String describeFilterChain();
extern uint16_t maxHighpassCutoffForSampleRate(uint32_t sampleRate);
extern uint16_t sanitizeHighpassCutoffSetting(uint16_t value, uint32_t sampleRate);

// Web server and in-memory log ring buffer
static WebServer web(80);
static const size_t LOG_CAP = 80;
static const size_t LOG_LINE_MAX = 256;
static char logBuffer[LOG_CAP][LOG_LINE_MAX] = {{0}};
static size_t logHead = 0;
static size_t logCount = 0;
static const char* STATE_CHANGE_HEADER = "X-Requested-With";
static const char* STATE_CHANGE_HEADER_VALUE = "birdnetgo-webui";
static const size_t SETTING_KEY_MAX = 24;
static const size_t SETTING_VALUE_MAX = 32;

extern portMUX_TYPE logMux;

void webui_pushLog(const String &line) {
    char copy[LOG_LINE_MAX];
    memset(copy, 0, sizeof(copy));
    line.toCharArray(copy, sizeof(copy));

    portENTER_CRITICAL(&logMux);
    memcpy(logBuffer[logHead], copy, sizeof(logBuffer[logHead]));
    logHead = (logHead + 1) % LOG_CAP;
    if (logCount < LOG_CAP) logCount++;
    portEXIT_CRITICAL(&logMux);
}

static String jsonEscape(const String &s) {
    String o; o.reserve(s.length()+8);
    for (size_t i=0;i<s.length();++i){char c=s[i]; if(c=='"'||c=='\\'){o+='\\';o+=c;} else if(c=='\n'){o+="\\n";} else {o+=c;}}
    return o;
}

static String profileName(uint16_t buf) {
    // Server-side fallback (English). UI localizes on client by buffer size.
    if (buf <= 256) return F("Ultra-Low Latency (Higher CPU, May have dropouts)");
    if (buf <= 512) return F("Balanced (Moderate CPU, Good stability)");
    if (buf <= 1024) return F("Stable Streaming (Lower CPU, Excellent stability)");
    return F("High Stability (Lowest CPU, Maximum stability)");
}

static void apiSendJSON(const String &json) {
    web.sendHeader("Cache-Control", "no-cache");
    web.send(200, "application/json", json);
}

static void apiSendError(const String &error, int code = 400) {
    web.sendHeader("Cache-Control", "no-cache");
    web.send(code, "application/json", "{\"ok\":false,\"error\":\"" + jsonEscape(error) + "\"}");
}

static bool webAudioPreviewAvailable() {
    return rtspServerEnabled && audioTaskRunning;
}

static bool requireTrustedStateChangeRequest() {
    if (web.header(STATE_CHANGE_HEADER) != STATE_CHANGE_HEADER_VALUE) {
        apiSendError("invalid_request_origin", 403);
        return false;
    }
    return true;
}

static bool parseStrictFloat(const String &value, float &out) {
    if (value.length() == 0 || value.length() > SETTING_VALUE_MAX) return false;
    char buf[SETTING_VALUE_MAX + 1];
    value.toCharArray(buf, sizeof(buf));
    char* end = NULL;
    errno = 0;
    float parsed = strtof(buf, &end);
    if (end == buf || errno == ERANGE || !isfinite(parsed)) return false;
    while (*end == ' ' || *end == '\t') end++;
    if (*end != '\0') return false;
    out = parsed;
    return true;
}

static bool parseStrictUInt(const String &value, uint32_t &out) {
    if (value.length() == 0 || value.length() > SETTING_VALUE_MAX) return false;
    char buf[SETTING_VALUE_MAX + 1];
    value.toCharArray(buf, sizeof(buf));
    char* cursor = buf;
    while (*cursor == ' ' || *cursor == '\t') cursor++;
    if (*cursor == '-' || *cursor == '+') return false;
    char* end = NULL;
    errno = 0;
    unsigned long parsed = strtoul(cursor, &end, 10);
    if (end == cursor || errno == ERANGE || parsed > UINT32_MAX) return false;
    while (*end == ' ' || *end == '\t') end++;
    if (*end != '\0') return false;
    out = (uint32_t)parsed;
    return true;
}

static bool parseSinceArg(uint32_t &since) {
    since = 0;
    if (!web.hasArg("since")) return true;
    String value = web.arg("since");
    if (!parseStrictUInt(value, since)) {
        apiSendError("invalid_since");
        return false;
    }
    return true;
}

// HTML UI
static String htmlStreamer() {
    String h;
    h += F(
        "<!doctype html><html><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>Web Streamer</title>"
        "<style>body{font-family:system-ui,Segoe UI,Roboto,Arial,sans-serif;background:#101113;color:#f2f4f3;margin:0;padding:18px;line-height:1.35}"
        ".card{max-width:980px;margin:0 auto;background:#17191c;border:1px solid #30363d;border-radius:8px;padding:16px;box-shadow:0 8px 24px rgba(0,0,0,.22)}"
        "button{font:inherit;padding:8px 12px;border-radius:8px;border:1px solid #30363d;background:#20242a;color:#f2f4f3;cursor:pointer}"
        "button:hover{border-color:#00a878}.mono{font-family:ui-monospace,Consolas,Menlo,monospace}.muted{color:#a9b0ac}"
        ".meter{height:10px;border:1px solid #30363d;border-radius:8px;background:#0b0c0e;overflow:hidden;margin:10px 0}"
        ".fill{height:100%;width:0;background:linear-gradient(90deg,#00a878,#6ede8a)}.row{display:flex;gap:10px;align-items:center;flex-wrap:wrap}"
        "</style></head><body><div class='card'><h2>Browser Web Streamer</h2>"
        "<p class='muted'>This player uses a binary PCM endpoint with a short jitter buffer so browser playback stays smooth.</p>"
        "<div class='row'><button id='start'>Start audio</button><button id='stop'>Stop</button><span id='st' class='muted'>Idle</span></div>"
        "<div class='meter'><div id='m' class='fill'></div></div>"
        "<p class='muted'>Fallback RTSP URL: <span id='rtsp' class='mono'></span></p>"
        "<script>const rtsp='rtsp://'+location.hostname+':8554/audio';document.getElementById('rtsp').textContent=rtsp;"
        "let ctx=null,running=false,lastSeq=0,nextT=0,timer=null,inFlight=false,lastPcm=0,haveLastPcm=false;"
        "const MIN_AHEAD=0.18,TARGET_AHEAD=0.45,MAX_AHEAD=1.2;"
        "function setStatus(t){document.getElementById('st').textContent=t;}"
        "function setMeter(p){document.getElementById('m').style.width=Math.max(0,Math.min(100,p))+'%';}"
        "function schedulePull(ms){if(timer)clearTimeout(timer);timer=setTimeout(pull,ms);}"
        "function playFrame(rate,samples,dropped){if(!ctx)ctx=new (window.AudioContext||window.webkitAudioContext)();"
        "const n=samples.length;if(!n)return;const underrun=nextT<ctx.currentTime+MIN_AHEAD;const b=ctx.createBuffer(1,n,rate);const d=b.getChannelData(0);let peak=0;"
        "const ramp=(haveLastPcm&&(underrun||dropped>0))?Math.min(n,Math.max(8,Math.floor(rate*0.004))):0;"
        "for(let i=0;i<n;i++){let s=samples[i];if(i<ramp){s=((lastPcm*(ramp-i))+(s*(i+1)))/(ramp+1);}const v=Math.max(-1,Math.min(1,s/32768));d[i]=v;const a=Math.abs(v);if(a>peak)peak=a;}"
        "lastPcm=samples[n-1];haveLastPcm=true;if(underrun)nextT=ctx.currentTime+MIN_AHEAD;if(nextT>ctx.currentTime+MAX_AHEAD)nextT=ctx.currentTime+TARGET_AHEAD;const src=ctx.createBufferSource();src.buffer=b;src.connect(ctx.destination);src.start(nextT);nextT+=n/rate;setMeter(peak*100);}"
        "async function pull(){if(!running||inFlight)return;inFlight=true;try{const r=await fetch('/api/web_audio_pcm?since='+lastSeq,{cache:'no-store'});"
        "if(r.status===204){inFlight=false;schedulePull(5);return;}if(!r.ok)throw new Error('HTTP '+r.status);"
        "const seq=Number(r.headers.get('X-WebAudio-Seq')||'0');const rate=Number(r.headers.get('X-WebAudio-Rate')||'16000');"
        "const samples=Number(r.headers.get('X-WebAudio-Samples')||'0');const frames=Number(r.headers.get('X-WebAudio-Frames')||'1');const dropped=Number(r.headers.get('X-WebAudio-Dropped')||'0');const buf=await r.arrayBuffer();let pcm=new Int16Array(buf);"
        "if(samples>0&&samples<pcm.length)pcm=pcm.subarray(0,samples);if(!pcm.length){inFlight=false;schedulePull(5);return;}"
        "lastSeq=seq||lastSeq;playFrame(rate,pcm,dropped);const ahead=Math.max(0,nextT-ctx.currentTime);setStatus('Playing • '+rate+' Hz • buffer '+Math.round(ahead*1000)+' ms • '+frames+' frame'+(frames===1?'':'s')+(dropped?(' • skipped '+dropped):''));"
        "inFlight=false;schedulePull(ahead<TARGET_AHEAD?0:25);}"
        "catch(e){inFlight=false;setStatus('Waiting for audio… '+e.message);schedulePull(150);}}"
        "document.getElementById('start').onclick=async()=>{running=true;lastSeq=0;nextT=0;haveLastPcm=false;lastPcm=0;if(!ctx)ctx=new (window.AudioContext||window.webkitAudioContext)();if(ctx.state==='suspended')await ctx.resume();setStatus('Starting…');pull();};"
        "document.getElementById('stop').onclick=()=>{running=false;inFlight=false;lastSeq=0;nextT=0;haveLastPcm=false;lastPcm=0;if(timer)clearTimeout(timer);timer=null;setStatus('Stopped');setMeter(0);};"
        "</script></div></body></html>");
    return h;
}

static void httpStreamer() { web.send(200, "text/html; charset=utf-8", htmlStreamer()); }

static bool getWebAudioSnapshot(uint32_t since, WebAudioSnapshot& snapshot) {
    bool ok = false;
    snapshot.seq = 0;
    snapshot.samples = 0;
    snapshot.rate = 0;
    snapshot.frames = 0;
    snapshot.dropped = 0;

    portENTER_CRITICAL(&webAudioMux);
    uint32_t newestSeq = webAudioFrameSeq;
    if (newestSeq != 0) {
        uint32_t oldestSeq = (newestSeq > WEBUI_AUDIO_RING_LEN) ? (newestSeq - WEBUI_AUDIO_RING_LEN + 1U) : 1U;
        // On first browser playback request, return the oldest retained frames
        // so the AudioContext can start with a useful jitter cushion.
        uint32_t targetSeq = (since == 0) ? oldestSeq : (since + 1U);
        if (targetSeq < oldestSeq) {
            snapshot.dropped = oldestSeq - targetSeq;
            targetSeq = oldestSeq;
        }
        while (targetSeq <= newestSeq && snapshot.samples < WEBUI_AUDIO_MAX_SAMPLES) {
            uint8_t slot = (uint8_t)((targetSeq - 1U) % WEBUI_AUDIO_RING_LEN);
            if (webAudioRingSeq[slot] == targetSeq) {
                uint16_t frameSamples = webAudioFrameSamples[slot];
                uint32_t frameRate = webAudioSampleRate[slot];
                if (frameSamples == 0) break;
                if (snapshot.frames > 0 && frameRate != snapshot.rate) break;
                if (frameSamples > (WEBUI_AUDIO_MAX_SAMPLES - snapshot.samples)) break;
                memcpy(snapshot.frame + snapshot.samples, webAudioFrame[slot], frameSamples * sizeof(int16_t));
                snapshot.samples += frameSamples;
                snapshot.rate = frameRate;
                snapshot.seq = targetSeq;
                snapshot.frames++;
                ok = true;
                targetSeq++;
            } else {
                break;
            }
        }
    }
    portEXIT_CRITICAL(&webAudioMux);

    return ok;
}

static void httpWebAudio() {
    if (!webAudioPreviewAvailable()) {
        apiSendError("preview_unavailable", 503);
        return;
    }
    uint32_t since = 0;
    if (!parseSinceArg(since)) return;

    WebAudioSnapshot& snapshot = webAudioHttpSnapshot;
    if (!getWebAudioSnapshot(since, snapshot)) {
        web.send(204, "text/plain", "");
        return;
    }

    String json;
    json.reserve(64 + (snapshot.samples * 8));
    json += "{\"seq\":" + String(snapshot.seq) + ",\"rate\":" + String(snapshot.rate) + ",\"samples\":[";
    for (uint16_t i = 0; i < snapshot.samples; i++) {
        if (i) json += ',';
        json += String(snapshot.frame[i]);
    }
    json += "]}";
    apiSendJSON(json);
}

static void httpWebAudioPcm() {
    if (!webAudioPreviewAvailable()) {
        apiSendError("preview_unavailable", 503);
        return;
    }
    uint32_t since = 0;
    if (!parseSinceArg(since)) return;

    WebAudioSnapshot& snapshot = webAudioHttpSnapshot;
    if (!getWebAudioSnapshot(since, snapshot)) {
        web.send(204, "text/plain", "");
        return;
    }

    size_t payloadBytes = (size_t)snapshot.samples * sizeof(int16_t);
    web.sendHeader("Cache-Control", "no-store");
    web.sendHeader("X-WebAudio-Seq", String(snapshot.seq));
    web.sendHeader("X-WebAudio-Rate", String(snapshot.rate));
    web.sendHeader("X-WebAudio-Samples", String(snapshot.samples));
    web.sendHeader("X-WebAudio-Frames", String(snapshot.frames));
    web.sendHeader("X-WebAudio-Dropped", String(snapshot.dropped));
    web.setContentLength(payloadBytes);
    web.send(200, "application/octet-stream", "");
    if (payloadBytes > 0) {
        web.client().write((const uint8_t*)snapshot.frame, payloadBytes);
    }
}

static void sendWebAudioPayloadHeaders(const WebAudioSnapshot& snapshot, const String& contentType, size_t payloadBytes, uint32_t sampleRate) {
    web.sendHeader("Cache-Control", "no-store");
    web.sendHeader("X-WebAudio-Seq", String(snapshot.seq));
    web.sendHeader("X-WebAudio-Rate", String(sampleRate));
    web.sendHeader("X-WebAudio-Samples", String(snapshot.samples));
    web.sendHeader("X-WebAudio-Frames", String(snapshot.frames));
    web.sendHeader("X-WebAudio-Dropped", String(snapshot.dropped));
    web.setContentLength(payloadBytes);
    web.send(200, contentType, "");
}

static void putLe16(uint8_t* p, uint16_t v) {
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
}

static void putLe32(uint8_t* p, uint32_t v) {
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
}

static void putBe16(uint8_t* p, uint16_t v) {
    p[0] = (uint8_t)((v >> 8) & 0xFF);
    p[1] = (uint8_t)(v & 0xFF);
}

static void putBe32(uint8_t* p, uint32_t v) {
    p[0] = (uint8_t)((v >> 24) & 0xFF);
    p[1] = (uint8_t)((v >> 16) & 0xFF);
    p[2] = (uint8_t)((v >> 8) & 0xFF);
    p[3] = (uint8_t)(v & 0xFF);
}

static void writeBigEndianPcmPayload(const int16_t* samples, uint16_t sampleCount) {
    if (!samples || sampleCount == 0) return;

    uint8_t chunk[256];
    uint16_t offset = 0;
    while (offset < sampleCount) {
        uint16_t chunkSamples = (uint16_t)min((uint16_t)(sizeof(chunk) / sizeof(int16_t)), (uint16_t)(sampleCount - offset));
        for (uint16_t i = 0; i < chunkSamples; ++i) {
            uint16_t value = (uint16_t)samples[offset + i];
            chunk[i * 2] = (uint8_t)((value >> 8) & 0xFF);
            chunk[i * 2 + 1] = (uint8_t)(value & 0xFF);
        }
        web.client().write(chunk, (size_t)chunkSamples * sizeof(int16_t));
        offset += chunkSamples;
    }
}

static void copyAiffSampleRate80(uint8_t* dest, uint32_t sampleRate) {
    static const uint8_t sr16000[10] = {0x40, 0x0C, 0xFA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    static const uint8_t sr24000[10] = {0x40, 0x0D, 0xBB, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    static const uint8_t sr32000[10] = {0x40, 0x0D, 0xFA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    static const uint8_t sr48000[10] = {0x40, 0x0E, 0xBB, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    const uint8_t* encoded = sr16000;
    switch (sampleRate) {
        case 24000: encoded = sr24000; break;
        case 32000: encoded = sr32000; break;
        case 48000: encoded = sr48000; break;
        case 16000:
        default:
            encoded = sr16000;
            break;
    }
    memcpy(dest, encoded, 10);
}

static void httpWebAudioL16() {
    if (!webAudioPreviewAvailable()) {
        apiSendError("preview_unavailable", 503);
        return;
    }
    uint32_t since = 0;
    if (!parseSinceArg(since)) return;

    WebAudioSnapshot& snapshot = webAudioHttpSnapshot;
    if (!getWebAudioSnapshot(since, snapshot)) {
        web.send(204, "text/plain", "");
        return;
    }

    const uint32_t sampleRate = snapshot.rate ? snapshot.rate : 16000;
    const size_t payloadBytes = (size_t)snapshot.samples * sizeof(int16_t);
    sendWebAudioPayloadHeaders(
        snapshot,
        String("audio/L16;rate=") + String(sampleRate) + ";channels=1",
        payloadBytes,
        sampleRate
    );
    if (payloadBytes > 0) {
        writeBigEndianPcmPayload(snapshot.frame, snapshot.samples);
    }
}

static void httpWebAudioWav() {
    if (!webAudioPreviewAvailable()) {
        apiSendError("preview_unavailable", 503);
        return;
    }
    uint32_t since = 0;
    if (!parseSinceArg(since)) return;

    WebAudioSnapshot& snapshot = webAudioHttpSnapshot;
    if (!getWebAudioSnapshot(since, snapshot)) {
        web.send(204, "text/plain", "");
        return;
    }

    const uint16_t channels = 1;
    const uint16_t bitsPerSample = 16;
    const uint16_t blockAlign = channels * (bitsPerSample / 8);
    const uint32_t sampleRate = snapshot.rate ? snapshot.rate : 16000;
    const uint32_t dataBytes = (uint32_t)snapshot.samples * blockAlign;
    const uint32_t byteRate = sampleRate * blockAlign;
    uint8_t header[44] = {
        'R','I','F','F',0,0,0,0,'W','A','V','E','f','m','t',' ',
        16,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,2,0,16,0,'d','a','t','a',0,0,0,0
    };

    putLe32(header + 4, 36U + dataBytes);
    putLe32(header + 24, sampleRate);
    putLe32(header + 28, byteRate);
    putLe16(header + 32, blockAlign);
    putLe16(header + 34, bitsPerSample);
    putLe32(header + 40, dataBytes);

    sendWebAudioPayloadHeaders(snapshot, "audio/wav", sizeof(header) + dataBytes, sampleRate);
    web.client().write(header, sizeof(header));
    if (dataBytes > 0) {
        web.client().write((const uint8_t*)snapshot.frame, dataBytes);
    }
}

static void httpWebAudioAiff() {
    if (!webAudioPreviewAvailable()) {
        apiSendError("preview_unavailable", 503);
        return;
    }
    uint32_t since = 0;
    if (!parseSinceArg(since)) return;

    WebAudioSnapshot& snapshot = webAudioHttpSnapshot;
    if (!getWebAudioSnapshot(since, snapshot)) {
        web.send(204, "text/plain", "");
        return;
    }

    const uint32_t sampleRate = snapshot.rate ? snapshot.rate : 16000;
    const uint32_t dataBytes = (uint32_t)snapshot.samples * sizeof(int16_t);
    uint8_t header[54] = {
        'F','O','R','M',0,0,0,0,'A','I','F','F',
        'C','O','M','M',0,0,0,18,0,1,0,0,0,0,0,16,0,0,0,0,0,0,0,0,0,0,
        'S','S','N','D',0,0,0,0,0,0,0,0,0,0,0,0
    };

    putBe32(header + 4, 46U + dataBytes);
    putBe32(header + 22, (uint32_t)snapshot.samples);
    copyAiffSampleRate80(header + 28, sampleRate);
    putBe32(header + 42, 8U + dataBytes);

    sendWebAudioPayloadHeaders(snapshot, "audio/aiff", sizeof(header) + dataBytes, sampleRate);
    web.client().write(header, sizeof(header));
    if (dataBytes > 0) {
        writeBigEndianPcmPayload(snapshot.frame, snapshot.samples);
    }
}

// HTTP handlery
static void httpIndex() {
    web.sendHeader("Cache-Control", "no-store");
    web.send_P(200, PSTR("text/html; charset=utf-8"), WEBUI_INDEX_HTML);
}

static void httpStatus() {
    unsigned long uptimeSeconds = (millis() - bootTime) / 1000;
    String uptimeStr = formatUptime(uptimeSeconds);
    unsigned long runtime = millis() - lastStatsReset;
    uint32_t currentRate = (isStreaming && runtime > 1000) ? (audioBlocksSent * 1000) / runtime : 0;
    String json = "{";
    json.reserve(900);
    json += "\"fw_version\":\"" + String(FW_VERSION_STR) + "\",";
    json += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
    json += "\"gateway\":\"" + WiFi.gatewayIP().toString() + "\",";
    json += "\"subnet\":\"" + WiFi.subnetMask().toString() + "\",";
    json += "\"wifi_connected\":" + String(WiFi.status()==WL_CONNECTED?"true":"false") + ",";
    json += "\"wifi_rssi\":" + String(WiFi.RSSI()) + ",";
    json += "\"wifi_tx_dbm\":" + String(wifiPowerLevelToDbm(currentWifiPowerLevel),1) + ",";
    json += "\"free_heap_kb\":" + String(ESP.getFreeHeap()/1024) + ",";
    json += "\"min_free_heap_kb\":" + String(minFreeHeap/1024) + ",";
    json += "\"uptime\":\"" + uptimeStr + "\",";
    json += "\"rtsp_server_enabled\":" + String(rtspServerEnabled?"true":"false") + ",";
    json += "\"client\":\"" + jsonEscape(getRtspClientSummary()) + "\",";
    json += "\"streaming\":" + String(isStreaming?"true":"false") + ",";
    json += "\"current_rate_pkt_s\":" + String(currentRate) + ",";
    json += "\"rtp_packets_sent\":" + String(audioPacketsSent) + ",";
    json += "\"rtp_packets_dropped\":" + String(audioPacketsDropped) + ",";
    json += "\"last_rtsp_connect\":\"" + jsonEscape(formatSince(lastRtspClientConnectMs)) + "\",";
    json += "\"last_stream_start\":\"" + jsonEscape(formatSince(lastRtspPlayMs)) + "\",";
    json += "\"rtsp_transport\":\"" + String(currentRtspTransportName()) + "\",";
    json += "\"active_rtsp_clients\":" + String(getActiveRtspClientCount()) + ",";
    json += "\"connected_rtsp_clients\":" + String(getConnectedRtspClientCount()) + ",";
    json += "\"max_rtsp_clients\":" + String(getMaxRtspClientCount()) + ",";
    json += "\"rtsp_rejected_clients\":" + String(rtspRejectedClientCount) + ",";
    json += "\"stream_formats\":\"RTSP/RTP L16, HTTP L16, AIFF PCM, WebAudio PCM, WAV PCM chunk, JSON PCM\",";
    json += "\"multi_client_policy\":\"bounded_multi_rtsp_tcp\",";
    json += "\"hardware_profile\":\"" + jsonEscape(describeHardwareProfile()) + "\"";
    json += "}";
    apiSendJSON(json);
}

static void httpAudioStatus() {
    float agcMult;
    float noiseFloor;
    float noiseGate;
    float noiseReduction;
    float pipelineLoad;
    uint32_t fallbackBlocks;
    uint32_t clipCount;
    uint32_t lastGapMs;
    bool driverOk;
    int32_t lastError;
    uint16_t peakAbs16;
    uint16_t peakHold16;
    bool clippedLastBlock;
    uint16_t lastSamplesRead;
    int16_t rawMin;
    int16_t rawMax;
    uint16_t rawPeakAbs;
    uint16_t rawRms;
    uint16_t rawZeroPct;
    int16_t rawMean;
    uint16_t rawSpan;
    bool likelyUnsignedPcm;
    bool usingModernPdmDriver;
    bool pdmUseLeftSlot;
    bool pdmClkInverted;
    bool pdmProbeFoundSigned;

    portENTER_CRITICAL(&diagMux);
    agcMult = agcMultiplier;
    noiseFloor = noiseFloorDbfs;
    noiseGate = noiseGateDbfs;
    noiseReduction = noiseReductionDb;
    pipelineLoad = audioPipelineLoadPct;
    fallbackBlocks = audioFallbackBlockCount;
    clipCount = audioClipCount;
    lastGapMs = i2sLastGapMs;
    driverOk = i2sDriverOk;
    lastError = i2sLastError;
    peakAbs16 = lastPeakAbs16;
    peakHold16 = peakHoldAbs16;
    clippedLastBlock = audioClippedLastBlock;
    lastSamplesRead = i2sLastSamplesRead;
    rawMin = i2sLastRawMin;
    rawMax = i2sLastRawMax;
    rawPeakAbs = i2sLastRawPeakAbs;
    rawRms = i2sLastRawRms;
    rawZeroPct = i2sLastRawZeroPct;
    rawMean = i2sLastRawMean;
    rawSpan = i2sLastRawSpan;
    likelyUnsignedPcm = i2sLikelyUnsignedPcm;
    usingModernPdmDriver = i2sUsingModernPdmDriver;
    pdmUseLeftSlot = i2sPdmUseLeftSlot;
    pdmClkInverted = i2sPdmClkInverted;
    pdmProbeFoundSigned = i2sPdmProbeFoundSigned;
    portEXIT_CRITICAL(&diagMux);

    uint16_t effectiveChunk = effectiveAudioChunkSize();
    float latency_ms = (float)effectiveChunk / currentSampleRate * 1000.0f;
    String json = "{";
    json.reserve(1800);
    json += "\"sample_rate\":" + String(currentSampleRate) + ",";
    json += "\"gain\":" + String(currentGainFactor,2) + ",";
    json += "\"buffer_size\":" + String(currentBufferSize) + ",";
    json += "\"i2s_shift\":" + String(i2sShiftBits) + ",";
    json += "\"latency_ms\":" + String(latency_ms,1) + ",";
    json += "\"profile\":\"" + jsonEscape(profileName(effectiveChunk)) + "\",";
    json += "\"hp_enable\":" + String(highpassEnabled?"true":"false") + ",";
    json += "\"hp_cutoff_hz\":" + String((uint32_t)highpassCutoffHz) + ",";
    json += "\"hp_cutoff_max_hz\":" + String((uint32_t)maxHighpassCutoffForSampleRate(currentSampleRate)) + ",";
    json += "\"agc_enable\":" + String(agcEnabled?"true":"false") + ",";
    json += "\"agc_multiplier\":" + String(agcMult, 2) + ",";
    json += "\"noise_filter_enable\":" + String(noiseFilterEnabled?"true":"false") + ",";
    json += "\"noise_floor_dbfs\":" + String(noiseFloor, 1) + ",";
    json += "\"noise_gate_dbfs\":" + String(noiseGate, 1) + ",";
    json += "\"noise_reduction_db\":" + String(noiseReduction, 1) + ",";
    json += "\"filter_chain\":\"" + jsonEscape(describeFilterChain()) + "\",";
    String captureMode = usingModernPdmDriver
        ? String("V4 ESP-IDF PDM RX selected ") + (pdmUseLeftSlot ? "left" : "right") +
              " slot" + (pdmClkInverted ? " with inverted clock. " : " with normal clock. ")
        : String("PDM RX driver is not active yet. ");
    String filterDetail = captureMode +
        (likelyUnsignedPcm
            ? String("Live blocks still look all-positive, so the unsigned DC tracker and fixed scaling remain active before HPF/gain.")
            : String("Signed PCM is reaching DSP directly before HPF/gain."));
    json += "\"filter_detail\":\"" + jsonEscape(filterDetail) + "\",";
    json += "\"audio_pipeline_load_pct\":" + String(pipelineLoad, 1) + ",";
    json += "\"i2s_fallback_blocks\":" + String(fallbackBlocks) + ",";
    json += "\"i2s_last_gap_ms\":" + String(lastGapMs) + ",";
    json += "\"i2s_driver_ok\":" + String(driverOk?"true":"false") + ",";
    json += "\"i2s_last_error\":" + String((int32_t)lastError) + ",";
    float effectiveGain = agcEnabled ? (currentGainFactor * agcMult) : currentGainFactor;
    json += "\"effective_gain\":" + String(effectiveGain, 2) + ",";
    // Metering/clipping
    uint16_t p = (peakHold16 > 0) ? peakHold16 : peakAbs16;
    float peak_pct = (p <= 0) ? 0.0f : (100.0f * (float)p / 32767.0f);
    float peak_dbfs = (p <= 0) ? -90.0f : (20.0f * log10f((float)p / 32767.0f));
    json += "\"peak_pct\":" + String(peak_pct,1) + ",";
    json += "\"peak_dbfs\":" + String(peak_dbfs,1) + ",";
    json += "\"clip\":" + String(clippedLastBlock?"true":"false") + ",";
    json += "\"clip_count\":" + String(clipCount) + ",";
    json += "\"led_mode\":" + String(ledMode) + ",";
    json += "\"i2s_reads_ok\":" + String(i2sReadOkCount) + ",";
    json += "\"i2s_reads_err\":" + String(i2sReadErrCount) + ",";
    json += "\"i2s_reads_zero\":" + String(i2sReadZeroCount) + ",";
    json += "\"i2s_last_samples\":" + String(lastSamplesRead) + ",";
    json += "\"i2s_raw_min\":" + String(rawMin) + ",";
    json += "\"i2s_raw_max\":" + String(rawMax) + ",";
    json += "\"i2s_raw_peak\":" + String(rawPeakAbs) + ",";
    json += "\"i2s_raw_rms\":" + String(rawRms) + ",";
    json += "\"i2s_raw_zero_pct\":" + String(rawZeroPct) + ",";
    json += "\"i2s_raw_mean\":" + String(rawMean) + ",";
    json += "\"i2s_raw_span\":" + String(rawSpan) + ",";
    json += "\"i2s_capture_driver\":\"" + String(usingModernPdmDriver ? "modern_pdm_rx" : "inactive") + "\",";
    json += "\"i2s_pdm_slot\":\"" + String(pdmUseLeftSlot ? "left" : "right") + "\",";
    json += "\"i2s_pdm_clk_inverted\":" + String(pdmClkInverted?"true":"false") + ",";
    json += "\"i2s_pdm_probe_signed\":" + String(pdmProbeFoundSigned?"true":"false") + ",";
    json += "\"i2s_unsigned_pcm\":" + String(likelyUnsignedPcm?"true":"false") + ",";
    bool likelyFlatline = !driverOk || (rawPeakAbs < 8) || ((rawMin == rawMax) && lastSamplesRead > 0) || (rawZeroPct > 98);
    json += "\"i2s_link_ok\":" + String(likelyFlatline?"false":"true") + ",";
    String i2sHint;
    if (!driverOk) {
        i2sHint = "I2S driver setup failed. Check serial logs for the failing driver step and error code.";
    } else if (likelyFlatline) {
        i2sHint = "Mic data looks flat/near-zero. Check CLK/DATA wiring, GND, and 3V3. For Unit PDM use CLK=G1 and DATA=G2.";
    } else if (likelyUnsignedPcm) {
        if (currentSampleRate >= 48000) {
            i2sHint = String("PDM path ") + (pdmUseLeftSlot ? "left" : "right") +
                      (pdmClkInverted ? " / clk inverted" : " / normal clock") +
                      " is active, but live samples still look low-amplitude and all-positive. Firmware is centering them with conservative fixed scaling; if the stream still sounds raspy, try 24000 Hz or 16000 Hz for a cleaner baseline.";
        } else {
            i2sHint = String("PDM path ") + (pdmUseLeftSlot ? "left" : "right") +
                      (pdmClkInverted ? " / clk inverted" : " / normal clock") +
                      " is active, but live samples still look low-amplitude and all-positive. Firmware is centering them with conservative fixed scaling before streaming.";
        }
    } else {
        i2sHint = String("Raw signed PDM samples are changing on the ") +
                  (pdmUseLeftSlot ? "left" : "right") +
                  (pdmClkInverted ? " / clk-inverted" : " / normal-clock") +
                  " path; I2S link appears active.";
    }
    json += "\"i2s_hint\":\"" + jsonEscape(i2sHint) + "\"";
    json += "}";
    apiSendJSON(json);
}

static void httpTelemetryHistory() {
    uint8_t cpu[TELEMETRY_HISTORY_LEN];
    int16_t temp[TELEMETRY_HISTORY_LEN];
    uint16_t count = 0;
    uint16_t head = 0;
    uint32_t seq = 0;

    portENTER_CRITICAL(&telemetryMux);
    count = telemetryHistoryCount;
    head = telemetryHistoryHead;
    seq = telemetryHistorySeq;
    memcpy(cpu, telemetryCpuLoadPct, sizeof(cpu));
    memcpy(temp, telemetryTempDeciC, sizeof(temp));
    portEXIT_CRITICAL(&telemetryMux);

    String json = "{\"seq\":" + String(seq) + ",\"sample_ms\":3000,\"cpu\":[";
    json.reserve(700);
    for (uint16_t i = 0; i < count; ++i) {
        uint16_t idx = (head + TELEMETRY_HISTORY_LEN - count + i) % TELEMETRY_HISTORY_LEN;
        if (i) json += ",";
        json += String((uint32_t)cpu[idx]);
    }
    json += "],\"temp_c\":[";
    for (uint16_t i = 0; i < count; ++i) {
        uint16_t idx = (head + TELEMETRY_HISTORY_LEN - count + i) % TELEMETRY_HISTORY_LEN;
        if (i) json += ",";
        if (temp[idx] == INT16_MIN) json += "null";
        else json += String((float)temp[idx] / 10.0f, 1);
    }
    json += "]}";
    apiSendJSON(json);
}

static void httpFft() {
    if (!webAudioPreviewAvailable()) {
        apiSendError("preview_unavailable", 503);
        return;
    }
    uint8_t bins[WEBUI_FFT_BINS];
    uint32_t seq;
    portENTER_CRITICAL(&fftMux);
    seq = fftFrameSeq;
    for (uint16_t i = 0; i < WEBUI_FFT_BINS; i++) bins[i] = fftBins[i];
    portEXIT_CRITICAL(&fftMux);

    String json = "{\"seq\":" + String(seq) + ",\"fft_size\":" + String(WEBUI_FFT_SIZE) + ",\"bins\":[";
    json.reserve(700);
    for (uint16_t i = 0; i < WEBUI_FFT_BINS; i++) {
        if (i) json += ",";
        json += String((uint32_t)bins[i]);
    }
    json += "],\"sample_rate\":" + String(currentSampleRate) + ",\"max_hz\":" + String((float)currentSampleRate * 0.5f, 1) + "}";
    apiSendJSON(json);
}

static void httpPerfStatus() {
    String json = "{";
    json.reserve(180);
    json += "\"restart_threshold_pkt_s\":" + String(minAcceptableRate) + ",";
    json += "\"check_interval_min\":" + String(performanceCheckInterval) + ",";
    json += "\"auto_recovery\":" + String(autoRecoveryEnabled?"true":"false") + ",";
    json += "\"auto_threshold\":" + String(autoThresholdEnabled?"true":"false") + ",";
    json += "\"recommended_min_rate\":" + String(computeRecommendedMinRate()) + ",";
    json += "\"scheduled_reset\":" + String(scheduledResetEnabled?"true":"false") + ",";
    json += "\"reset_hours\":" + String(resetIntervalHours) + "}";
    apiSendJSON(json);
}

static void httpStreamOptions() {
    String ip = WiFi.localIP().toString();
    String json = "{";
    json.reserve(900);
    json += "\"max_rtsp_clients\":" + String(getMaxRtspClientCount()) + ",";
    json += "\"active_rtsp_clients\":" + String(getActiveRtspClientCount()) + ",";
    json += "\"connected_rtsp_clients\":" + String(getConnectedRtspClientCount()) + ",";
    json += "\"rtsp_rejected_clients\":" + String(rtspRejectedClientCount) + ",";
    json += "\"rtsp_transport\":\"" + String(currentRtspTransportName()) + "\",";
    json += "\"multi_client_policy\":\"bounded_multi_rtsp_tcp\",";
    json += "\"note\":\"RTSP supports two bounded TCP interleaved clients. HTTP L16, AIFF, browser PCM, WAV chunks, and JSON PCM read from the diagnostics ring buffer.\",";
    json += "\"formats\":[";
    json += "{\"id\":\"rtsp_l16\",\"label\":\"RTSP/RTP L16 mono PCM\",\"url\":\"rtsp://" + ip + ":8554/audio\",\"content_type\":\"audio/L16\"},";
    json += "{\"id\":\"http_l16\",\"label\":\"HTTP L16 mono PCM chunk\",\"url\":\"http://" + ip + "/api/web_audio_l16\",\"content_type\":\"audio/L16\"},";
    json += "{\"id\":\"aiff_pcm\",\"label\":\"AIFF PCM chunk\",\"url\":\"http://" + ip + "/api/web_audio_aiff\",\"content_type\":\"audio/aiff\"},";
    json += "{\"id\":\"web_pcm\",\"label\":\"Browser PCM binary frames\",\"url\":\"http://" + ip + "/api/web_audio_pcm\",\"content_type\":\"application/octet-stream\"},";
    json += "{\"id\":\"wav_pcm\",\"label\":\"WAV PCM chunk\",\"url\":\"http://" + ip + "/api/web_audio_wav\",\"content_type\":\"audio/wav\"},";
    json += "{\"id\":\"json_pcm\",\"label\":\"JSON PCM frames\",\"url\":\"http://" + ip + "/api/web_audio\",\"content_type\":\"application/json\"}";
    json += "]}";
    apiSendJSON(json);
}

static void httpThermal() {
    String since = "";
    if (overheatTripTemp > 0.0f && overheatTriggeredAt != 0) {
        since = formatSince(overheatTriggeredAt);
    }
    bool manualRequired = overheatLatched || (!rtspServerEnabled && overheatProtectionEnabled && overheatTripTemp > 0.0f);
    String json = "{";
    json.reserve(420);
    if (lastTemperatureValid) {
        json += "\"current_c\":" + String(lastTemperatureC,1) + ",";
    } else {
        json += "\"current_c\":null,";
    }
    json += "\"current_valid\":" + String(lastTemperatureValid?"true":"false") + ",";
    json += "\"max_c\":" + String(maxTemperature,1) + ",";
    json += "\"cpu_mhz\":" + String(getCpuFrequencyMhz()) + ",";
    json += "\"protection_enabled\":" + String(overheatProtectionEnabled?"true":"false") + ",";
    json += "\"shutdown_c\":" + String(overheatShutdownC,0) + ",";
    json += "\"latched\":" + String(overheatLockoutActive?"true":"false") + ",";
    json += "\"latched_persist\":" + String(overheatLatched?"true":"false") + ",";
    json += "\"sensor_fault\":" + String(overheatSensorFault?"true":"false") + ",";
    json += "\"last_trip_c\":" + String(overheatTripTemp,1) + ",";
    json += "\"last_reason\":\"" + jsonEscape(overheatLastReason) + "\",";
    json += "\"last_trip_ts\":\"" + jsonEscape(overheatLastTimestamp) + "\",";
    json += "\"last_trip_since\":\"" + jsonEscape(since) + "\",";
    json += "\"manual_restart\":" + String(manualRequired?"true":"false");
    json += "}";
    apiSendJSON(json);
}

static bool sampleTemperatureForThermalClear(float &temp) {
    temp = temperatureRead();
    if (isnan(temp) || isinf(temp) || temp < -20.0f || temp > 130.0f) {
        lastTemperatureValid = false;
        overheatSensorFault = true;
        return false;
    }

    lastTemperatureC = temp;
    lastTemperatureValid = true;
    overheatSensorFault = false;
    if (temp > maxTemperature) {
        maxTemperature = temp;
    }
    return true;
}

static void httpThermalClear() {
    if (!requireTrustedStateChangeRequest()) return;
    if (overheatLatched) {
        if (overheatProtectionEnabled) {
            float temp = 0.0f;
            if (!sampleTemperatureForThermalClear(temp)) {
                apiSendError("thermal_sensor_unavailable", 409);
                return;
            }
            float rearmC = overheatShutdownC - (float)OH_STEP;
            if (temp > rearmC) {
                apiSendError("thermal_still_hot", 409);
                return;
            }
        }
        bool startedServer = false;
        if (!rtspServerEnabled) {
            rtspServer.begin();
            rtspServer.setNoDelay(true);
            rtspServerEnabled = true;
            startedServer = true;
        }
        if (i2sDriverOk && !audioTaskRunning) {
            if (!startAudioCaptureTask()) {
                if (startedServer) {
                    rtspServerEnabled = false;
                    rtspServer.stop();
                }
                apiSendError("audio_task_start_failed", 503);
                return;
            }
        }
        overheatLatched = false;
        overheatLockoutActive = false;
        overheatTripTemp = 0.0f;
        overheatTriggeredAt = 0;
        overheatLastReason = String("Thermal latch cleared manually.");
        overheatLastTimestamp = String("");
        saveAudioSettings();
        webui_pushLog(F("UI action: thermal_latch_clear"));
        apiSendJSON(F("{\"ok\":true}"));
    } else {
        apiSendJSON(F("{\"ok\":false}"));
    }
}

static void httpLogs() {
    String out;
    size_t head;
    size_t count;
    portENTER_CRITICAL(&logMux);
    head = logHead;
    count = logCount;
    portEXIT_CRITICAL(&logMux);

    for (size_t i=0;i<count;i++){
        size_t idx = (head + LOG_CAP - count + i) % LOG_CAP;
        char line[LOG_LINE_MAX];
        portENTER_CRITICAL(&logMux);
        memcpy(line, logBuffer[idx], sizeof(line));
        portEXIT_CRITICAL(&logMux);
        line[LOG_LINE_MAX - 1] = '\0';
        out += line; out += '\n';
    }
    web.send(200, "text/plain; charset=utf-8", out);
}

static void httpActionServerStart(){
    if (!requireTrustedStateChangeRequest()) return;
    if (overheatLatched) {
        webui_pushLog(F("Server start blocked: thermal protection latched"));
        apiSendJSON(F("{\"ok\":false,\"error\":\"thermal_latched\"}"));
        return;
    }
    if (!rtspServerEnabled) {
        rtspServerEnabled=true; rtspServer.begin(); rtspServer.setNoDelay(true);
        overheatLockoutActive = false;
    }
    if (i2sDriverOk && !audioTaskRunning) {
        if (!startAudioCaptureTask()) {
            rtspServerEnabled=false;
            rtspServer.stop();
            apiSendError("audio_task_start_failed", 503);
            return;
        }
    }
    webui_pushLog(F("UI action: server_start"));
    apiSendJSON(F("{\"ok\":true}"));
}
extern bool requestStreamStop(const char* reason);
static void httpActionServerStop(){
    if (!requireTrustedStateChangeRequest()) return;
    if (isStreaming) {
        requestStreamStop("server_stop");
    }
    if (audioTaskRunning) {
        stopAudioCaptureTask();
    }
    rtspServerEnabled=false;
    rtspServer.stop();
    webui_pushLog(F("UI action: server_stop"));
    apiSendJSON(F("{\"ok\":true}"));
}
static void httpActionResetI2S(){
    if (!requireTrustedStateChangeRequest()) return;
    webui_pushLog(F("UI action: reset_i2s"));
    restartI2S(); apiSendJSON(F("{\"ok\":true}"));
}

static void httpActionNamed() {
    if (!requireTrustedStateChangeRequest()) return;
    String name = web.hasArg("name") ? web.arg("name") : String("");
    if (name == "server_start") {
        httpActionServerStart();
    } else if (name == "server_stop") {
        httpActionServerStop();
    } else if (name == "reset_i2s") {
        httpActionResetI2S();
    } else if (name == "reboot") {
        webui_pushLog(F("UI action: reboot"));
        apiSendJSON(F("{\"ok\":true}"));
        scheduleReboot(false, 600);
    } else if (name == "factory_reset") {
        webui_pushLog(F("UI action: factory_reset"));
        apiSendJSON(F("{\"ok\":true}"));
        scheduleReboot(true, 600);
    } else {
        apiSendError("unknown_action", 404);
    }
}

static void httpSet() {
    if (!requireTrustedStateChangeRequest()) return;
    String key = web.arg("key");
    String val = web.hasArg("value") ? web.arg("value") : String("");
    if (key.length() == 0) {
        apiSendError("missing_key");
        return;
    }
    if (!web.hasArg("value")) {
        apiSendError("missing_value");
        return;
    }
    if (key.length() > SETTING_KEY_MAX || val.length() > SETTING_VALUE_MAX) {
        apiSendError("value_too_long");
        return;
    }
    if (val.length()) { webui_pushLog(String("UI set: ")+key+"="+val); }
    bool ok = false;
    String error = "invalid_value";
    if (key == "gain") { float v; if (parseStrictFloat(val, v) && v>=0.1f && v<=100.0f) { currentGainFactor=v; saveAudioSettings(); ok=true; } else error="invalid_gain"; }
    else if (key == "rate") { uint32_t v; if (parseStrictUInt(val, v) && isSupportedPdmSampleRate(v)) { currentSampleRate=v; highpassCutoffHz = sanitizeHighpassCutoffSetting(highpassCutoffHz, currentSampleRate); if (autoThresholdEnabled) { minAcceptableRate = computeRecommendedMinRate(); } saveAudioSettings(); restartI2S(); ok=true; } else error="invalid_rate"; }
    else if (key == "buffer") { uint32_t v; if (parseStrictUInt(val, v) && v>=256 && v<=8192) { currentBufferSize=(uint16_t)v; if (autoThresholdEnabled) { minAcceptableRate = computeRecommendedMinRate(); } saveAudioSettings(); restartI2S(); ok=true; } else error="invalid_buffer"; }
    // i2sShiftBits removed - fixed at 0 for PDM microphones
    else if (key == "wifi_tx") { float v; if (parseStrictFloat(val, v) && v>=-1.0f && v<=19.5f) { extern float wifiTxPowerDbm; wifiTxPowerDbm = snapWifiTxDbm(v); applyWifiTxPower(true); saveAudioSettings(); ok=true; } else error="invalid_wifi_tx"; }
    else if (key == "auto_recovery") { String v=web.arg("value"); if (v=="on"||v=="off") { autoRecoveryEnabled=(v=="on"); saveAudioSettings(); ok=true; } else error="invalid_auto_recovery"; }
    else if (key == "thr_mode") { String v=web.arg("value"); if (v=="auto") { autoThresholdEnabled=true; minAcceptableRate = computeRecommendedMinRate(); saveAudioSettings(); ok=true; } else if (v=="manual") { autoThresholdEnabled=false; saveAudioSettings(); ok=true; } else error="invalid_threshold_mode"; }
    else if (key == "min_rate") { uint32_t v; if (parseStrictUInt(val, v) && v>=5 && v<=200) { minAcceptableRate=v; saveAudioSettings(); ok=true; } else error="invalid_min_rate"; }
    else if (key == "check_interval") { uint32_t v; if (parseStrictUInt(val, v) && v>=1 && v<=60) { performanceCheckInterval=v; saveAudioSettings(); ok=true; } else error="invalid_check_interval"; }
    else if (key == "sched_reset") { String v=web.arg("value"); if (v=="on"||v=="off") { extern bool scheduledResetEnabled; scheduledResetEnabled=(v=="on"); saveAudioSettings(); ok=true; } else error="invalid_scheduled_reset"; }
    else if (key == "reset_hours") { uint32_t v; if (parseStrictUInt(val, v) && v>=1 && v<=168) { extern uint32_t resetIntervalHours; resetIntervalHours=v; saveAudioSettings(); ok=true; } else error="invalid_reset_hours"; }
    else if (key == "cpu_freq") { uint32_t v; if (parseStrictUInt(val, v) && (v==80 || v==120 || v==160 || v==240)) { cpuFrequencyMhz=(uint8_t)v; setCpuFrequencyMhz(cpuFrequencyMhz); saveAudioSettings(); ok=true; } else error="invalid_cpu_freq"; }
    else if (key == "hp_enable") { String v=web.arg("value"); if (v=="on"||v=="off") { highpassEnabled=(v=="on"); extern void updateHighpassCoeffs(); updateHighpassCoeffs(); saveAudioSettings(); ok=true; } else error="invalid_hp_enable"; }
    else if (key == "hp_cutoff") { uint32_t v; uint16_t maxCutoff = maxHighpassCutoffForSampleRate(currentSampleRate); if (parseStrictUInt(val, v) && v>=10 && v<=maxCutoff) { highpassCutoffHz=(uint16_t)v; extern void updateHighpassCoeffs(); updateHighpassCoeffs(); saveAudioSettings(); ok=true; } else error="invalid_hp_cutoff"; }
    else if (key == "agc_enable") { String v=web.arg("value"); if (v=="on"||v=="off") { agcEnabled=(v=="on"); if (!agcEnabled) agcMultiplier=1.0f; saveAudioSettings(); ok=true; } else error="invalid_agc_enable"; }
    else if (key == "noise_filter") { String v=web.arg("value"); if (v=="on"||v=="off") { noiseFilterEnabled=(v=="on"); saveAudioSettings(); ok=true; } else error="invalid_noise_filter"; }
    else if (key == "led_mode") { uint32_t v; if (parseStrictUInt(val, v) && v<=2) { ledMode=(uint8_t)v; saveAudioSettings(); ok=true; } else error="invalid_led_mode"; }
    else if (key == "oh_enable") { String v=web.arg("value"); if (v=="on"||v=="off") { overheatProtectionEnabled = (v=="on"); if (!overheatProtectionEnabled) { overheatLockoutActive = false; } saveAudioSettings(); ok=true; } else error="invalid_oh_enable"; }
    else if (key == "oh_limit") { uint32_t v; if (parseStrictUInt(val, v) && v>=OH_MIN && v<=OH_MAX) { uint32_t snapped = OH_MIN + ((v - OH_MIN)/OH_STEP)*OH_STEP; overheatShutdownC = (float)snapped; overheatLockoutActive = false; saveAudioSettings(); ok=true; } else error="invalid_oh_limit"; }
    else { error = "unknown_key"; }

    if (ok) apiSendJSON(F("{\"ok\":true}"));
    else apiSendError(error);
}

void webui_begin() {
    const char* headerKeys[] = {STATE_CHANGE_HEADER};
    web.collectHeaders(headerKeys, 1);
    web.on("/", httpIndex);
    web.on("/streamer", httpStreamer);
    web.on("/api/web_audio", httpWebAudio);
    web.on("/api/web_audio_pcm", httpWebAudioPcm);
    web.on("/api/web_audio_l16", httpWebAudioL16);
    web.on("/api/web_audio_aiff", httpWebAudioAiff);
    web.on("/api/web_audio_wav", httpWebAudioWav);
    web.on("/api/status", httpStatus);
    web.on("/api/audio_status", httpAudioStatus);
    web.on("/api/telemetry_history", httpTelemetryHistory);
    web.on("/api/fft", httpFft);
    web.on("/api/perf_status", httpPerfStatus);
    web.on("/api/stream_options", httpStreamOptions);
    web.on("/api/thermal", httpThermal);
    web.on("/api/thermal/clear", HTTP_POST, httpThermalClear);
    web.on("/api/logs", httpLogs);
    web.on("/api/action/server_start", HTTP_POST, httpActionServerStart);
    web.on("/api/action/server_stop", HTTP_POST, httpActionServerStop);
    web.on("/api/action/reset_i2s", HTTP_POST, httpActionResetI2S);
    web.on("/api/action", HTTP_POST, httpActionNamed);
    web.on("/api/action/reboot", HTTP_POST, [](){
        if (!requireTrustedStateChangeRequest()) return;
        webui_pushLog(F("UI action: reboot"));
        apiSendJSON(F("{\"ok\":true}"));
        scheduleReboot(false, 600);
    });
    web.on("/api/action/factory_reset", HTTP_POST, [](){
        if (!requireTrustedStateChangeRequest()) return;
        webui_pushLog(F("UI action: factory_reset"));
        apiSendJSON(F("{\"ok\":true}"));
        scheduleReboot(true, 600);
    });
    web.on("/api/set", HTTP_POST, httpSet);
    web.begin();
}

void webui_handleClient() {
    web.handleClient();
}
