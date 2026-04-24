#pragma once

#include <stdint.h>

struct PdmProbeResult {
    bool valid = false;
    bool useLeftSlot = false;
    bool clkInvert = false;
    bool spansZero = false;
    int16_t rawMin = 0;
    int16_t rawMax = 0;
    int16_t rawMean = 0;
    uint16_t rawSpan = 0;
    uint16_t rawPeakAbs = 0;
};
