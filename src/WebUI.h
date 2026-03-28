#pragma once
#include <Arduino.h>

#if defined(BOARD_PROFILE_ATOM_ECHO) && BOARD_PROFILE_ATOM_ECHO
// Atom Echo (ESP32) has tighter DRAM; reduce browser-audio frame buffer footprint.
constexpr uint16_t WEBUI_AUDIO_MAX_SAMPLES = 1024;
#else
constexpr uint16_t WEBUI_AUDIO_MAX_SAMPLES = 4096;
#endif
constexpr uint8_t WEBUI_AUDIO_RING_LEN = 8;

// Web UI (ESP32 RTSP Mic for BirdNET-Go): initialization and request handling
void webui_begin();
void webui_handleClient();

// Push a log line from main into the Web UI ring buffer
void webui_pushLog(const String &line);

// Request Core 1 to stop streaming (defined in main .ino)
bool requestStreamStop(const char* reason);
