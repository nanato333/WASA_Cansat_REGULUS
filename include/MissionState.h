#pragma once
#include <Arduino.h>

// 数値は地上局との通信仕様。変更時は地上局も同時に更新する。
enum class MissionState : uint8_t {
    BOOT = 0, LAUNCH_STANDBY = 1, LAUNCH = 2, DEPLOYED = 3,
    LANDED = 4, RUNNING = 5, FINISHED = 6, FAILSAFE = 7,
    LOW_BATTERY = FAILSAFE  // 旧コードとの互換名
};
enum class MissionSubState : uint8_t { NONE = 0, GPS_NAVIGATION = 1, RSSI_HOMING = 2, STACK_ESCAPE = 3 };
enum class OperationMode : uint8_t { UNSELECTED = 0, DEVELOPMENT = 1, MISSION = 2 };
enum class FailsafeReason : uint8_t { NONE = 0, LOW_BATTERY = 1, UNSAFE_FLIGHT_RESTART = 2, SENSOR_FAILURE = 3 };
enum class MissionCommand : uint8_t {
    SELECT_DEVELOPMENT_MODE = 1, SELECT_MISSION_MODE = 2,
    START_LAUNCH = 3, CLEAR_FAILSAFE = 4,
    // 飛行中再起動後、着地を地上確認した場合だけ状態4から再開する。
    RESUME_LANDED = 5, SET_GOAL_COORDINATE = 6,
    CONFIRM_DEPLOYED = 7, CONFIRM_LANDED = 8
};
const char *mission_state_name(MissionState);
const char *mission_sub_state_name(MissionSubState);
const char *operation_mode_name(OperationMode);
const char *failsafe_reason_name(FailsafeReason);
bool is_valid_mission_command(uint8_t);