#pragma once

#include <Arduino.h>

bool init_sensor_bus();
bool take_sensor_bus(TickType_t timeout);
void give_sensor_bus();