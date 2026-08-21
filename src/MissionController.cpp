#include "MissionController.h"

#include <Preferences.h>
#include <math.h>
#include "MissionConfig.h"

namespace {
constexpr char NVS_NAMESPACE[] = "mission";
constexpr char NVS_STATE_KEY[] = "state";
constexpr char NVS_MODE_KEY[] = "mode";
constexpr char NVS_FAILSAFE_KEY[] = "failsafe";
constexpr double EARTH_RADIUS_M = 6371000.0;

bool isSafelyRestorable(MissionState state) {
    return state == MissionState::BOOT ||
           state == MissionState::LAUNCH_STANDBY ||
           state == MissionState::LANDED ||
           state == MissionState::RUNNING ||
           state == MissionState::FINISHED ||
           state == MissionState::LOW_BATTERY;
}

void saveByte(const char *key, uint8_t value) {
    Preferences preferences;
    if (preferences.begin(NVS_NAMESPACE, false)) {
        preferences.putUChar(key, value);
        preferences.end();
    }
}

float distanceMeters(double latitudeA, double longitudeA,
                     double latitudeB, double longitudeB) {
    const double lat1 = latitudeA * DEG_TO_RAD;
    const double lat2 = latitudeB * DEG_TO_RAD;
    const double deltaLat = (latitudeB - latitudeA) * DEG_TO_RAD;
    const double deltaLon = (longitudeB - longitudeA) * DEG_TO_RAD;
    const double a = sin(deltaLat / 2.0) * sin(deltaLat / 2.0) +
                     cos(lat1) * cos(lat2) *
                     sin(deltaLon / 2.0) * sin(deltaLon / 2.0);
    return static_cast<float>(EARTH_RADIUS_M * 2.0 * atan2(sqrt(a), sqrt(1.0 - a)));
}

float headingDifference(float a, float b) {
    float difference = fabsf(a - b);
    return difference > 180.0f ? 360.0f - difference : difference;
}
}

void MissionController::begin() {
    Preferences preferences;
    uint8_t savedState = static_cast<uint8_t>(MissionState::BOOT);
    uint8_t savedMode = static_cast<uint8_t>(OperationMode::UNSELECTED);
    uint8_t savedReason = static_cast<uint8_t>(FailsafeReason::NONE);
    if (preferences.begin(NVS_NAMESPACE, true)) {
        savedState = preferences.getUChar(NVS_STATE_KEY, savedState);
        savedMode = preferences.getUChar(NVS_MODE_KEY, savedMode);
        savedReason = preferences.getUChar(NVS_FAILSAFE_KEY, savedReason);
        preferences.end();
    }

    if (savedMode <= static_cast<uint8_t>(OperationMode::MISSION)) {
        mode_ = static_cast<OperationMode>(savedMode);
    }

    const MissionState candidate = savedState <= static_cast<uint8_t>(MissionState::LOW_BATTERY)
        ? static_cast<MissionState>(savedState)
        : MissionState::BOOT;
    // 飛行中の再起動は位置や姿勢を保証できないため、回収用の低電力フェイルセーフへ入る。
    const bool safelyRestorable = isSafelyRestorable(candidate);
    state_ = safelyRestorable ? candidate : MissionState::FAILSAFE;
    if (!safelyRestorable) {
        failsafeReason_ = FailsafeReason::UNSAFE_FLIGHT_RESTART;
        saveByte(NVS_STATE_KEY, static_cast<uint8_t>(state_));
        saveByte(NVS_FAILSAFE_KEY, static_cast<uint8_t>(failsafeReason_));
    } else if (state_ == MissionState::FAILSAFE && savedReason <= static_cast<uint8_t>(FailsafeReason::SENSOR_FAILURE)) {
        failsafeReason_ = static_cast<FailsafeReason>(savedReason);
    }
    subState_ = state_ == MissionState::RUNNING
        ? MissionSubState::GPS_NAVIGATION
        : MissionSubState::NONE;
    stateStartedMs_ = millis();
    subStateStartedMs_ = stateStartedMs_;
    update_mission_status(state_, subState_, mode_);
    update_failsafe_reason(failsafeReason_);
}

bool MissionController::handleCommand(uint8_t command, uint32_t now) {
    if (!is_valid_mission_command(command)) {
        return false;
    }

    const MissionCommand missionCommand = static_cast<MissionCommand>(command);
    if (missionCommand == MissionCommand::RESUME_LANDED) {
        // センサーだけでは落下中と着地後を完全には区別できないため、
        // 飛行中再起動のFAILSAFEに限り、地上局で着地確認後に受理する。
        if (state_ != MissionState::FAILSAFE ||
            failsafeReason_ != FailsafeReason::UNSAFE_FLIGHT_RESTART ||
            mode_ != OperationMode::MISSION) {
            return false;
        }
        failsafeReason_ = FailsafeReason::NONE;
        saveByte(NVS_FAILSAFE_KEY, static_cast<uint8_t>(failsafeReason_));
        update_failsafe_reason(failsafeReason_);
        transitionTo(MissionState::LANDED, now);
        return true;
    }
    if (missionCommand == MissionCommand::CLEAR_FAILSAFE) {
        if (state_ != MissionState::FAILSAFE ||
            failsafeReason_ != FailsafeReason::LOW_BATTERY ||
            !failsafeRecoveryReady_) return false;
        mode_ = OperationMode::UNSELECTED;
        failsafeReason_ = FailsafeReason::NONE;
        saveByte(NVS_MODE_KEY, static_cast<uint8_t>(mode_));
        saveByte(NVS_FAILSAFE_KEY, static_cast<uint8_t>(failsafeReason_));
        update_failsafe_reason(failsafeReason_);
        transitionTo(MissionState::BOOT, now);
        return true;
    }
    if (missionCommand == MissionCommand::START_LAUNCH) {
        // 地上局からの打上げ指示は、ミッションモードの待機中だけ受理する。
        if (mode_ != OperationMode::MISSION ||
            state_ != MissionState::LAUNCH_STANDBY) {
            return false;
        }
        transitionTo(MissionState::LAUNCH, now);
        return true;
    }

    // 運用モードは起動状態でのみ選択できる。
    if (state_ != MissionState::BOOT) {
        return false;
    }
    mode_ = missionCommand == MissionCommand::SELECT_MISSION_MODE
        ? OperationMode::MISSION
        : OperationMode::DEVELOPMENT;
    saveByte(NVS_MODE_KEY, static_cast<uint8_t>(mode_));
    update_mission_status(state_, subState_, mode_);
    update_failsafe_reason(failsafeReason_);
    if (mode_ == OperationMode::MISSION) {
        transitionTo(MissionState::LAUNCH_STANDBY, now);
    }
    return true;
}

bool MissionController::conditionHeld(bool condition, uint32_t now, uint32_t requiredMs) {
    if (!condition) {
        conditionStartedMs_ = 0;
        return false;
    }
    if (conditionStartedMs_ == 0) {
        conditionStartedMs_ = now;
    }
    return now - conditionStartedMs_ >= requiredMs;
}

void MissionController::setSubState(MissionSubState next, uint32_t now) {
    if (next == subState_) {
        return;
    }
    subState_ = next;
    subStateStartedMs_ = now;
    update_mission_status(state_, subState_, mode_);
    update_failsafe_reason(failsafeReason_);
    Serial.printf("[Mission] substate -> %s\n", mission_sub_state_name(subState_));
}

void MissionController::enterFailsafe(FailsafeReason reason, uint32_t now) {
    failsafeReason_ = reason;
    saveByte(NVS_FAILSAFE_KEY, static_cast<uint8_t>(failsafeReason_));
    update_failsafe_reason(failsafeReason_);
    transitionTo(MissionState::FAILSAFE, now);
}
void MissionController::transitionTo(MissionState next, uint32_t now) {
    if (next == state_) {
        return;
    }
    state_ = next;
    stateStartedMs_ = now;
    conditionStartedMs_ = 0;
    subState_ = next == MissionState::RUNNING
        ? MissionSubState::GPS_NAVIGATION
        : MissionSubState::NONE;
    subStateStartedMs_ = now;
    update_mission_status(state_, subState_, mode_);
    update_failsafe_reason(failsafeReason_);
    saveByte(NVS_STATE_KEY, static_cast<uint8_t>(state_));
    Serial.printf("[Mission] transition -> %s\n", mission_state_name(state_));
}

void MissionController::updateVerticalSpeed(const CanSatData_t &data, uint32_t now) {
    if (!data.baro.is_valid) {
        altitudeInitialized_ = false;
        filteredVerticalSpeed_ = 0.0f;
        return;
    }
    if (!altitudeInitialized_) {
        altitudeInitialized_ = true;
        previousAltitude_ = data.baro.altitude;
        previousAltitudeMs_ = now;
        return;
    }
    if (now == previousAltitudeMs_) {
        return;
    }

    const float seconds = (now - previousAltitudeMs_) / 1000.0f;
    const float rawSpeed = (data.baro.altitude - previousAltitude_) / seconds;
    filteredVerticalSpeed_ += MissionConfig::VERTICAL_SPEED_FILTER_ALPHA *
                              (rawSpeed - filteredVerticalSpeed_);
    previousAltitude_ = data.baro.altitude;
    previousAltitudeMs_ = now;
}

MissionSubState MissionController::preferredRunningSubState(const CanSatData_t &data) const {
    const bool validRssi = data.sys.rssi < 0;
    if (validRssi && data.sys.rssi >= MissionConfig::RSSI_HOMING_THRESHOLD_DBM) {
        return MissionSubState::RSSI_HOMING;
    }
    return MissionSubState::GPS_NAVIGATION;
}

void MissionController::updateRunningState(const CanSatData_t &data, uint32_t now) {
    const bool positionValid = data.gnss.is_valid && data.gnss.fix;
    if (MissionConfig::GOAL_COORDINATE_CONFIGURED && positionValid) {
        const float goalDistance = distanceMeters(
            data.gnss.latitude, data.gnss.longitude,
            MissionConfig::GOAL_LATITUDE, MissionConfig::GOAL_LONGITUDE);
        if (goalDistance <= MissionConfig::GOAL_RADIUS_M) {
            if (goalStartedMs_ == 0) {
                goalStartedMs_ = now;
            }
            if (now - goalStartedMs_ >= MissionConfig::GOAL_CONFIRM_MS) {
                transitionTo(MissionState::FINISHED, now);
                return;
            }
        } else {
            goalStartedMs_ = 0;
        }
    } else {
        goalStartedMs_ = 0;
    }

    if (subState_ == MissionSubState::STACK_ESCAPE) {
        if (now - subStateStartedMs_ >= MissionConfig::STACK_ESCAPE_HOLD_MS) {
            stackStartedMs_ = 0;
            motionBaselineInitialized_ = false;
            setSubState(preferredRunningSubState(data), now);
        }
        return;
    }

    setSubState(preferredRunningSubState(data), now);
    const bool rssiValid = data.sys.rssi < 0;
    if (!positionValid || !data.mag.is_valid || !rssiValid) {
        motionBaselineInitialized_ = false;
        stackStartedMs_ = 0;
        return;
    }
    if (!motionBaselineInitialized_) {
        motionBaselineInitialized_ = true;
        motionLatitude_ = data.gnss.latitude;
        motionLongitude_ = data.gnss.longitude;
        motionHeading_ = data.mag.heading;
        motionRssi_ = data.sys.rssi;
        motionBaselineMs_ = now;
        return;
    }
    if (now - motionBaselineMs_ < MissionConfig::STACK_SAMPLE_INTERVAL_MS) {
        return;
    }

    const float positionDelta = distanceMeters(
        motionLatitude_, motionLongitude_,
        data.gnss.latitude, data.gnss.longitude);
    const float headingDelta = headingDifference(motionHeading_, data.mag.heading);
    const int rssiDelta = abs(data.sys.rssi - motionRssi_);
    const bool stationary =
        positionDelta <= MissionConfig::STACK_POSITION_DELTA_M &&
        headingDelta <= MissionConfig::STACK_HEADING_DELTA_DEG &&
        rssiDelta <= MissionConfig::STACK_RSSI_DELTA_DB;

    if (stationary) {
        if (stackStartedMs_ == 0) {
            stackStartedMs_ = now;
        }
        if (now - stackStartedMs_ >= MissionConfig::STACK_CONFIRM_MS) {
            setSubState(MissionSubState::STACK_ESCAPE, now);
        }
    } else {
        stackStartedMs_ = 0;
    }

    motionLatitude_ = data.gnss.latitude;
    motionLongitude_ = data.gnss.longitude;
    motionHeading_ = data.mag.heading;
    motionRssi_ = data.sys.rssi;
    motionBaselineMs_ = now;
}

void MissionController::update(const CanSatData_t &data, uint32_t now) {
    const bool batteryMeasured =
        isfinite(data.sys.battery_voltage) &&
        data.sys.battery_voltage >= MissionConfig::BATTERY_VALID_MIN_VOLTAGE &&
        data.sys.battery_voltage <= MissionConfig::BATTERY_VALID_MAX_VOLTAGE;
    const bool batteryLow =
        batteryMeasured && data.sys.battery_voltage <= MissionConfig::LOW_BATTERY_VOLTAGE;
    if (batteryLow) {
        if (lowBatteryStartedMs_ == 0) {
            lowBatteryStartedMs_ = now;
        }
        if (now - lowBatteryStartedMs_ >= MissionConfig::LOW_BATTERY_CONFIRM_MS) {
            enterFailsafe(FailsafeReason::LOW_BATTERY, now);
            return;
        }
    } else {
        lowBatteryStartedMs_ = 0;
    }

    if (state_ == MissionState::FAILSAFE) {
        const bool recovered = batteryMeasured &&
            data.sys.battery_voltage >= MissionConfig::FAILSAFE_RECOVERY_VOLTAGE;
        if (!recovered) {
            failsafeRecoveryStartedMs_ = 0;
            failsafeRecoveryReady_ = false;
        } else {
            if (failsafeRecoveryStartedMs_ == 0) failsafeRecoveryStartedMs_ = now;
            failsafeRecoveryReady_ = now - failsafeRecoveryStartedMs_ >= MissionConfig::FAILSAFE_RECOVERY_CONFIRM_MS;
        }
        return;
    }
    if (mode_ != OperationMode::MISSION) return;

    updateVerticalSpeed(data, now);
    const float acceleration = sqrtf(
        data.imu.ax * data.imu.ax +
        data.imu.ay * data.imu.ay +
        data.imu.az * data.imu.az);
    const float angularRate = sqrtf(
        data.imu.gx * data.imu.gx +
        data.imu.gy * data.imu.gy +
        data.imu.gz * data.imu.gz);

    switch (state_) {
        case MissionState::LAUNCH_STANDBY:
            if (conditionHeld(
                    data.imu.is_valid &&
                    acceleration >= MissionConfig::LAUNCH_ACCEL_MPS2,
                    now, MissionConfig::LAUNCH_CONFIRM_MS)) {
                transitionTo(MissionState::LAUNCH, now);
            }
            break;

        case MissionState::LAUNCH:
            if (conditionHeld(
                    data.imu.is_valid &&
                    now - stateStartedMs_ >= MissionConfig::MIN_LAUNCH_TO_DEPLOY_MS &&
                    acceleration <= MissionConfig::DEPLOY_FREEFALL_ACCEL_MPS2,
                    now, MissionConfig::DEPLOY_CONFIRM_MS)) {
                transitionTo(MissionState::DEPLOYED, now);
            }
            break;

        case MissionState::DEPLOYED:
            if (conditionHeld(
                    now - stateStartedMs_ >= MissionConfig::MIN_DEPLOYED_TO_LANDED_MS &&
                    data.baro.is_valid && data.imu.is_valid && altitudeInitialized_ &&
                    fabsf(filteredVerticalSpeed_) <= MissionConfig::LANDED_MAX_VERTICAL_SPEED_MPS &&
                    acceleration >= MissionConfig::LANDED_MIN_ACCEL_MPS2 &&
                    acceleration <= MissionConfig::LANDED_MAX_ACCEL_MPS2 &&
                    angularRate <= MissionConfig::LANDED_MAX_GYRO_DPS,
                    now, MissionConfig::LANDED_CONFIRM_MS)) {
                transitionTo(MissionState::LANDED, now);
            }
            break;

        case MissionState::LANDED:
            // 分離機構は未実装。モーターを動かさず、待機後に走行フェーズ表示へ進む。
            if (now - stateStartedMs_ >= MissionConfig::SEPARATION_HOLD_MS) {
                transitionTo(MissionState::RUNNING, now);
            }
            break;

        case MissionState::RUNNING:
            // 現段階ではサブ状態判定と通知のみで、走行出力は行わない。
            updateRunningState(data, now);
            break;

        case MissionState::FINISHED:
            if (now - stateStartedMs_ >= MissionConfig::FINISHED_HOLD_MS) {
                mode_ = OperationMode::UNSELECTED;
                saveByte(NVS_MODE_KEY, static_cast<uint8_t>(mode_));
                transitionTo(MissionState::BOOT, now);
            }
            break;

        default:
            conditionStartedMs_ = 0;
            break;
    }
}
