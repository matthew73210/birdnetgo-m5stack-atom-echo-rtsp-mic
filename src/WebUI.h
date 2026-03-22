#pragma once
#include <Arduino.h>

enum : uint16_t { WEB_AUDIO_FRAME_MAX_SAMPLES = 4096 };
enum : uint8_t { WEB_AUDIO_FRAME_RING_LEN = 8 };

// Web UI (ESP32 RTSP Mic for BirdNET-Go): initialization and request handling
void webui_begin();
void webui_handleClient();

// Push a log line from main into the Web UI ring buffer
void webui_pushLog(const String &line);

// Request Core 1 to stop streaming (defined in main firmware translation unit)
bool requestStreamStop(const char* reason);
