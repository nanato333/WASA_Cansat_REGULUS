#include <Arduino.h>
#include <math.h>
#include "tasks/MissionTask.h"
#include "tasks/TaskHealth.h"
#include "CanSatData.h"
#include "MissionConfig.h"
#include "MissionState.h"

namespace {
constexpr TickType_t PERIOD = pdMS_TO_TICKS(MissionConfig::TASK_PERIOD_MS);

// 一瞬の振動やノイズで遷移しないよう、条件が連続成立した時間を確認する。
bool conditionHeld(bool condition, uint32_t now, uint32_t requiredMs, uint32_t &startedMs) {
    if (!condition) {
        startedMs = 0;
        return false;
    }
    if (startedMs == 0) startedMs = now;
    return now - startedMs >= requiredMs;
}

// 状態変更、開始時刻の更新、共有データへの反映を一か所で行う。
void transitionTo(MissionState next, MissionState &current,
                  uint32_t now, uint32_t &stateStartedMs, uint32_t &conditionStartedMs) {
    current = next;
    stateStartedMs = now;
    conditionStartedMs = 0;
    update_mission_state(current);
    Serial.printf("[MissionTask] transition -> %s\n", mission_state_name(current));
}

#ifdef USE_MOCK_SENSORS
// センサーや無線を実機確認できない期間にUI・状態遷移を試すための時系列。
struct MockPhase {
    MissionState state;
    uint32_t durationMs;
};

constexpr MockPhase MOCK_SEQUENCE[] = {
    {MissionState::ROCKET_LOADED, 3000},
    {MissionState::LAUNCH, 2000},
    {MissionState::DEPLOYED, 2000},
    {MissionState::DESCENT, 5000},
    {MissionState::LANDED, 3000},
    {MissionState::RUNNING, 5000},
    {MissionState::STOPPED, 0}
};
#endif
}

void MissionTask(void *pvParameters) {
    (void)pvParameters;
    TickType_t lastWake = xTaskGetTickCount();
    MissionState current = MissionState::ROCKET_LOADED;
    uint32_t stateStartedMs = millis();
    uint32_t conditionStartedMs = 0;
    update_mission_state(current);

#ifdef USE_MOCK_SENSORS
// センサーや無線を実機確認できない期間にUI・状態遷移を試すための時系列。
    size_t phaseIndex = 0;
    Serial.println("[MissionTask] mock transition sequence enabled");
#elif defined(ENABLE_AUTOMATIC_FLIGHT_PHASES)
    // 実センサー値を使うのはfreertos_mission/freertos_flight環境だけ。
    bool altitudeInitialized = false;
    float previousAltitude = 0.0f;
    float filteredVerticalSpeed = 0.0f;
    uint32_t previousAltitudeMs = 0;
    Serial.println("[MissionTask] automatic flight detection enabled; RUNNING remains inhibited");
#else
    Serial.println("[MissionTask] automatic transitions disabled; holding ROCKET_LOADED");
#endif

    for (;;) {
        const uint32_t now = millis();
#ifdef USE_MOCK_SENSORS
// センサーや無線を実機確認できない期間にUI・状態遷移を試すための時系列。
        const uint32_t duration = MOCK_SEQUENCE[phaseIndex].durationMs;
        if (duration > 0 && now - stateStartedMs >= duration) {
            ++phaseIndex;
            transitionTo(MOCK_SEQUENCE[phaseIndex].state, current, now,
                         stateStartedMs, conditionStartedMs);
        }
#elif defined(ENABLE_AUTOMATIC_FLIGHT_PHASES)
    // 実センサー値を使うのはfreertos_mission/freertos_flight環境だけ。
        CanSatData_t data{};
        if (get_cansat_data(&data)) {
            const float acceleration = sqrtf(data.imu.ax * data.imu.ax +
                                             data.imu.ay * data.imu.ay +
                                             data.imu.az * data.imu.az);
            const float angularRate = sqrtf(data.imu.gx * data.imu.gx +
                                            data.imu.gy * data.imu.gy +
                                            data.imu.gz * data.imu.gz);

            // 気圧高度の差分から鉛直速度を求め、一次フィルタでノイズを抑える。
            if (data.baro.is_valid) {
                if (!altitudeInitialized) {
                    altitudeInitialized = true;
                    previousAltitude = data.baro.altitude;
                    previousAltitudeMs = now;
                } else if (now - previousAltitudeMs >= MissionConfig::TASK_PERIOD_MS) {
                    const float dt = (now - previousAltitudeMs) / 1000.0f;
                    const float rawVerticalSpeed = (data.baro.altitude - previousAltitude) / dt;
                    filteredVerticalSpeed += MissionConfig::VERTICAL_SPEED_FILTER_ALPHA *
                                             (rawVerticalSpeed - filteredVerticalSpeed);
                    previousAltitude = data.baro.altitude;
                    previousAltitudeMs = now;
                }
            } else {
                altitudeInitialized = false;
                filteredVerticalSpeed = 0.0f;
            }

            // 状態ごとに次の状態だけを評価し、逆向きの遷移は行わない。
            switch (current) {
                case MissionState::ROCKET_LOADED: {
                    const bool launchCandidate = data.imu.is_valid &&
                        acceleration >= MissionConfig::LAUNCH_ACCEL_MPS2;
                    if (conditionHeld(launchCandidate, now, MissionConfig::LAUNCH_CONFIRM_MS,
                                      conditionStartedMs)) {
                        transitionTo(MissionState::LAUNCH, current, now,
                                     stateStartedMs, conditionStartedMs);
                    }
                    break;
                }
                case MissionState::LAUNCH: {
                    const bool deploymentCandidate = data.imu.is_valid &&
                        now - stateStartedMs >= MissionConfig::MIN_LAUNCH_TO_DEPLOY_MS &&
                        acceleration <= MissionConfig::DEPLOY_FREEFALL_ACCEL_MPS2;
                    if (conditionHeld(deploymentCandidate, now, MissionConfig::DEPLOY_CONFIRM_MS,
                                      conditionStartedMs)) {
                        transitionTo(MissionState::DEPLOYED, current, now,
                                     stateStartedMs, conditionStartedMs);
                    }
                    break;
                }
                case MissionState::DEPLOYED: {
                    const bool descentCandidate = data.baro.is_valid && altitudeInitialized &&
                        filteredVerticalSpeed <= MissionConfig::DESCENT_SPEED_MPS;
                    if (conditionHeld(descentCandidate, now, MissionConfig::DESCENT_CONFIRM_MS,
                                      conditionStartedMs)) {
                        transitionTo(MissionState::DESCENT, current, now,
                                     stateStartedMs, conditionStartedMs);
                    }
                    break;
                }
                case MissionState::DESCENT: {
                    const bool landingCandidate = now - stateStartedMs >=
                            MissionConfig::MIN_DESCENT_TO_LANDED_MS &&
                        data.baro.is_valid && data.imu.is_valid && altitudeInitialized &&
                        fabsf(filteredVerticalSpeed) <= MissionConfig::LANDED_MAX_VERTICAL_SPEED_MPS &&
                        acceleration >= MissionConfig::LANDED_MIN_ACCEL_MPS2 &&
                        acceleration <= MissionConfig::LANDED_MAX_ACCEL_MPS2 &&
                        angularRate <= MissionConfig::LANDED_MAX_GYRO_DPS;
                    if (conditionHeld(landingCandidate, now, MissionConfig::LANDED_CONFIRM_MS,
                                      conditionStartedMs)) {
                        transitionTo(MissionState::LANDED, current, now,
                                     stateStartedMs, conditionStartedMs);
                        Serial.println("[MissionTask] RUNNING inhibited until motor specification is approved");
                    }
                    break;
                }
                // 安全のため、現在はLANDED以降へ自動遷移させない。
                case MissionState::LANDED:
                case MissionState::RUNNING:
                case MissionState::STOPPED:
                default:
                    conditionStartedMs = 0;
                    break;
            }
        }
#endif
        task_health_heartbeat(TaskId::MISSION);
        vTaskDelayUntil(&lastWake, PERIOD);
    }
}