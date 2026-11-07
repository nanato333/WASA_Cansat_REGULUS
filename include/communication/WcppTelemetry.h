#pragma once
#include <Arduino.h>
#include "CanSatData.h"

namespace WcppTelemetry {
constexpr uint8_t RC_DRIVE = 15;
struct Command { uint8_t action=0; bool hasGoalCoordinate=false; double goalLatitude=0, goalLongitude=0; int8_t left=0, right=0; };
constexpr uint8_t PACKET_ID = 2;
constexpr uint8_t COMPONENT_ID = 0;
size_t encode(const CanSatData_t &data, uint8_t *buffer, size_t capacity, bool minimal = false);
bool decodeCommand(const uint8_t *buffer, size_t length, Command &command);
bool decodeAction(const uint8_t *buffer, size_t length, uint8_t &action);
}
