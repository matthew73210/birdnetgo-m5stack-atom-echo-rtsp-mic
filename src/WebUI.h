#pragma once
#include <Arduino.h>

constexpr uint16_t WEBUI_AUDIO_MAX_SAMPLES = 4096;
constexpr uint8_t WEBUI_AUDIO_RING_LEN = 8;

// Web UI (ESP32 RTSP Mic for BirdNET-Go): initialization and request handling
void webui_begin();
void webui_handleClient();

// Push a log line from main into the Web UI ring buffer
void webui_pushLog(const String &line);

// Request Core 1 to stop streaming (defined in main .ino)
bool requestStreamStop(const char* reason);
