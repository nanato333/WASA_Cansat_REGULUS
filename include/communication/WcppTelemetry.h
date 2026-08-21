#pragma once
#include <Arduino.h>
#include "CanSatData.h"

namespace WcppTelemetry {
constexpr uint8_t PACKET_ID = 2;
constexpr uint8_t COMPONENT_ID = 0;
size_t encode(const CanSatData_t &data, uint8_t *buffer, size_t capacity, bool minimal = false);
bool decodeAction(const uint8_t *buffer, size_t length, uint8_t &action);
}