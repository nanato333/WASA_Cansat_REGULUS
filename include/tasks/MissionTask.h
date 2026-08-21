#pragma once
#include "communication/WcppTelemetry.h"
void MissionTask(void *pvParameters);
bool submit_mission_command(uint8_t command);
bool submit_mission_command(const WcppTelemetry::Command &command);
