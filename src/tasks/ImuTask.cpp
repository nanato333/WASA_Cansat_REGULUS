#include <Arduino.h>
#include <math.h>
#include "tasks/ImuTask.h"
#include "tasks/SensorBus.h"
#include "tasks/TaskHealth.h"
#include "CanSatData.h"
#include "MissionConfig.h"
#ifndef USE_MOCK_SENSORS
#include <Wire.h>
#include "drivers/QMI8658.h"
#endif

namespace {
// QMI8658は10 ms周期（100 Hz）で加速度・角速度を取得する。

constexpr TickType_t RETRY_PERIOD = pdMS_TO_TICKS(5000);
#ifndef USE_MOCK_SENSORS
QMI8658 imu(Wire, QMI8658::ADDRESS_LOW);
#else
MissionState mockPreviousState = MissionState::BOOT;
uint32_t mockStateStartedMs = 0;
#endif

MissionState currentMissionState() {
    CanSatData_t data{};
    get_cansat_data(&data);
    return static_cast<MissionState>(data.sys.phase);
}
}

void ImuTask(void *pvParameters) {
    (void)pvParameters;
    TickType_t lastWake = xTaskGetTickCount();
    TickType_t lastRetry = 0;
    bool initialized = false;

    for (;;) {
        const MissionState state = currentMissionState();
#ifndef USE_MOCK_SENSORS
        const TickType_t now = xTaskGetTickCount();
        // 初期化失敗後もタスクは終了せず、5秒ごとにセンサーを再検出する。
        if (!initialized && (lastRetry == 0 || now - lastRetry >= RETRY_PERIOD)) {
            lastRetry = now;
            if (take_sensor_bus(pdMS_TO_TICKS(50))) {
                initialized = imu.begin();
                give_sensor_bus();
            }
            Serial.printf("[ImuTask] init %s\n", initialized ? "OK" : "failed");
        }

        // Wireは他のI2Cセンサーと共有するため、通信中だけMutexを保持する。
        QMI8658::Data data{};
        bool valid = false;
        if (initialized && take_sensor_bus(pdMS_TO_TICKS(5))) {
            valid = imu.read(data);
            give_sensor_bus();
        }
        update_imu_data(data.ax, data.ay, data.az, data.gx, data.gy, data.gz, valid);
        if (valid) task_health_sensor_success(TaskId::IMU);
        else task_health_sensor_error(TaskId::IMU);
#else
        const float seconds = millis() / 1000.0f;
        const uint32_t now = millis();
        if (state != mockPreviousState) {
            mockPreviousState = state;
            mockStateStartedMs = now;
        }
        const uint32_t stateElapsedMs = now - mockStateStartedMs;
        float az = 9.80665f;
        if (state == MissionState::LAUNCH_STANDBY &&
            stateElapsedMs >= MissionConfig::MOCK_AUTO_LAUNCH_MS) {
            // 手動指示がない場合も、打上げ加速度の自動判定を試験する。
            az = 20.0f;
        } else if (state == MissionState::LAUNCH &&
                   stateElapsedMs >= MissionConfig::MOCK_POWERED_ASCENT_MS) {
            // 上昇後の微小重力を再現し、ロケット放出を検知させる。
            az = 1.0f;
        }
        update_imu_data(0.15f * sinf(seconds), 0.10f * cosf(seconds), az,
                        1.5f * sinf(seconds * 0.5f),
                        0.8f * cosf(seconds * 0.4f), 5.0f, true);
        task_health_sensor_success(TaskId::IMU);
#endif
        // データ取得成否とは別に、タスク自体の生存を記録する。
        task_health_heartbeat(TaskId::IMU);

        const TickType_t period = pdMS_TO_TICKS(state == MissionState::LAUNCH ? MissionConfig::FAST_IMU_PERIOD_MS : MissionConfig::NORMAL_IMU_PERIOD_MS);
        vTaskDelayUntil(&lastWake, period);
    }
}