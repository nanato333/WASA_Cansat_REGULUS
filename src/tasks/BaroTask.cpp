#include <Arduino.h>
#include <math.h>
#include "tasks/BaroTask.h"
#include "tasks/SensorBus.h"
#include "tasks/TaskHealth.h"
#include "CanSatData.h"
#include "MissionConfig.h"
#ifndef USE_MOCK_SENSORS
#include <Wire.h>
#include "drivers/HP203B.h"
#endif

namespace {
// HP203Bは100 ms周期（10 Hz）で気圧・温度・高度を更新する。

constexpr TickType_t RETRY_PERIOD = pdMS_TO_TICKS(5000);
#ifndef USE_MOCK_SENSORS
HP203B barometer(Wire);
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

void BaroTask(void *pvParameters) {
    (void)pvParameters;
    TickType_t lastWake = xTaskGetTickCount();
    TickType_t lastRetry = 0;
    bool initialized = false;

    for (;;) {
        const MissionState state = currentMissionState();
#ifndef USE_MOCK_SENSORS
        const TickType_t now = xTaskGetTickCount();
        if (!initialized && (lastRetry == 0 || now - lastRetry >= RETRY_PERIOD)) {
            lastRetry = now;
            if (take_sensor_bus(pdMS_TO_TICKS(50))) {
                initialized = barometer.begin();
                give_sensor_bus();
            }
            Serial.printf("[BaroTask] init %s\n", initialized ? "OK" : "failed");
        }

        // 変換開始と読取りの間はMutexを解放し、他のI2Cタスクを待たせない。
        bool conversionStarted = false;
        if (initialized && take_sensor_bus(pdMS_TO_TICKS(10))) {
            conversionStarted = barometer.startConversion();
            give_sensor_bus();
        }
        HP203B::Data data{};
        bool valid = false;
        if (conversionStarted) {
            vTaskDelay(pdMS_TO_TICKS(20));
            if (take_sensor_bus(pdMS_TO_TICKS(10))) {
                valid = barometer.readConversion(data);
                give_sensor_bus();
            }
        }
        update_baro_data(data.pressureHpa, data.temperatureC, data.altitudeM, valid);
        if (valid) task_health_sensor_success(TaskId::BARO);
        else task_health_sensor_error(TaskId::BARO);
#else
        const uint32_t now = millis();
        if (state != mockPreviousState) {
            mockPreviousState = state;
            mockStateStartedMs = now;
        }
        const uint32_t stateElapsedMs = now - mockStateStartedMs;
        float altitude = 20.0f;
        if (state == MissionState::LAUNCH) {
            altitude += 8.0f * stateElapsedMs / 1000.0f;
        } else if (state == MissionState::DEPLOYED) {
            const float descentSeconds = min(
                stateElapsedMs, MissionConfig::MOCK_PARACHUTE_DESCENT_MS) / 1000.0f;
            altitude = 50.0f - 5.0f * descentSeconds;
        } else if (state == MissionState::BOOT ||
                   state == MissionState::LAUNCH_STANDBY) {
            altitude += 0.15f * sinf(millis() / 2000.0f);
        }
        const float pressure = 1013.25f * powf(1.0f - altitude / 44330.0f, 5.255f);
        update_baro_data(pressure, 24.0f, altitude, true);
        task_health_sensor_success(TaskId::BARO);
#endif
        // 取得に失敗してもタスクは継続し、失敗回数だけを記録する。
        task_health_heartbeat(TaskId::BARO);

        const TickType_t period = pdMS_TO_TICKS(state == MissionState::LAUNCH ? MissionConfig::FAST_BARO_PERIOD_MS : MissionConfig::NORMAL_BARO_PERIOD_MS);
        vTaskDelayUntil(&lastWake, period);
    }
}