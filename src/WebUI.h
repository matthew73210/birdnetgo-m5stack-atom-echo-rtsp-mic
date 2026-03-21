#pragma once
#include <Arduino.h>

static const uint16_t WEB_AUDIO_MAX_SAMPLES = 4096;
static const uint8_t WEB_AUDIO_RING_LEN = 8;

// Web UI (ESP32 RTSP Mic for BirdNET-Go): initialization and request handling
void webui_begin();
void webui_handleClient();

// Push a log line from main into the Web UI ring buffer
void webui_pushLog(const String &line);

// Request Core 1 to stop streaming (defined in main .ino)
bool requestStreamStop(const char* reason);
