#pragma once

#include <Arduino.h>
#include "BoardConfig.h"

namespace BatteryMonitor {
constexpr float DIVIDER_RATIO = 2.0f;  // 100 kΩ / 100 kΩ
constexpr uint8_t SAMPLE_COUNT = 16;
constexpr uint32_t SAMPLE_DELAY_US = 250;

inline void begin() {
    pinMode(BoardConfig::VBAT_PIN, INPUT);
    analogReadResolution(12);
    analogSetPinAttenuation(BoardConfig::VBAT_PIN, ADC_11db);
    // 高インピーダンス分圧のため、最初の変換結果は捨ててADC入力を安定させる。
    (void)analogReadMilliVolts(BoardConfig::VBAT_PIN);
}

inline float readVoltage() {
    uint32_t sumMillivolts = 0;
    for (uint8_t sample = 0; sample < SAMPLE_COUNT; ++sample) {
        sumMillivolts += analogReadMilliVolts(BoardConfig::VBAT_PIN);
        delayMicroseconds(SAMPLE_DELAY_US);
    }
    const float adcVolts =
        (sumMillivolts / static_cast<float>(SAMPLE_COUNT)) / 1000.0f;
    return adcVolts * DIVIDER_RATIO;
}
}