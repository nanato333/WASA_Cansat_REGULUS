#pragma once

#include <Arduino.h>

namespace MissionConfig {
constexpr uint32_t TASK_PERIOD_MS = 50;

constexpr uint32_t NORMAL_IMU_PERIOD_MS = 20;
constexpr uint32_t FAST_IMU_PERIOD_MS = 5;
constexpr uint32_t NORMAL_MAG_PERIOD_MS = 50;
constexpr uint32_t FAST_MAG_PERIOD_MS = 10;
constexpr uint32_t NORMAL_BARO_PERIOD_MS = 100;
constexpr uint32_t FAST_BARO_PERIOD_MS = 25;
constexpr uint32_t NORMAL_TELEMETRY_PERIOD_MS = 500;
constexpr uint32_t FAST_TELEMETRY_PERIOD_MS = 100;
constexpr uint32_t LOW_BATTERY_TELEMETRY_PERIOD_MS = 5000;

// flight_mockは手動LAUNCHを優先し、未操作時だけ自動判定を再現する。
constexpr uint32_t MOCK_AUTO_LAUNCH_MS = 10000;
constexpr uint32_t MOCK_POWERED_ASCENT_MS = 3200;
constexpr uint32_t MOCK_PARACHUTE_DESCENT_MS = 6000;

constexpr float LAUNCH_ACCEL_MPS2 = 15.0f;
constexpr uint32_t LAUNCH_CONFIRM_MS = 300;
constexpr uint32_t MIN_LAUNCH_TO_DEPLOY_MS = 3000;
constexpr float DEPLOY_FREEFALL_ACCEL_MPS2 = 4.0f;
constexpr uint32_t DEPLOY_CONFIRM_MS = 400;
constexpr uint32_t MIN_DEPLOYED_TO_LANDED_MS = 5000;
constexpr float LANDED_MAX_VERTICAL_SPEED_MPS = 0.35f;
constexpr float LANDED_MIN_ACCEL_MPS2 = 7.0f;
constexpr float LANDED_MAX_ACCEL_MPS2 = 12.5f;
constexpr float LANDED_MAX_GYRO_DPS = 20.0f;
constexpr uint32_t LANDED_CONFIRM_MS = 3000;
constexpr uint32_t SEPARATION_HOLD_MS = 3000;
constexpr uint32_t FINISHED_HOLD_MS = 10000;
constexpr float VERTICAL_SPEED_FILTER_ALPHA = 0.15f;

constexpr float BATTERY_VALID_MIN_VOLTAGE = 2.0f;
constexpr float BATTERY_VALID_MAX_VOLTAGE = 5.0f;
constexpr float LOW_BATTERY_VOLTAGE = 3.3f;
constexpr uint32_t LOW_BATTERY_CONFIRM_MS = 5000;
constexpr float FAILSAFE_RECOVERY_VOLTAGE = 3.5f;
constexpr uint32_t FAILSAFE_RECOVERY_CONFIRM_MS = 10000;

#if defined(MISSION_GOAL_LATITUDE) && defined(MISSION_GOAL_LONGITUDE)
constexpr bool GOAL_COORDINATE_CONFIGURED = true;
constexpr double GOAL_LATITUDE = MISSION_GOAL_LATITUDE;
constexpr double GOAL_LONGITUDE = MISSION_GOAL_LONGITUDE;
#else
constexpr bool GOAL_COORDINATE_CONFIGURED = false;
constexpr double GOAL_LATITUDE = 0.0;
constexpr double GOAL_LONGITUDE = 0.0;
#endif

constexpr float GOAL_RADIUS_M = 5.0f;
constexpr uint32_t GOAL_CONFIRM_MS = 5000;
constexpr int RSSI_HOMING_THRESHOLD_DBM = -75;
constexpr uint32_t STACK_SAMPLE_INTERVAL_MS = 1000;
constexpr uint32_t STACK_CONFIRM_MS = 15000;
constexpr uint32_t STACK_ESCAPE_HOLD_MS = 5000;
constexpr float STACK_POSITION_DELTA_M = 1.0f;
constexpr float STACK_HEADING_DELTA_DEG = 5.0f;
constexpr int STACK_RSSI_DELTA_DB = 3;
}
