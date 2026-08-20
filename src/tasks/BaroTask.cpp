#include <Arduino.h>
#include <math.h>
#include "tasks/BaroTask.h"
#include "tasks/SensorBus.h"
#include "tasks/TaskHealth.h"
#include "CanSatData.h"
#ifndef USE_MOCK_SENSORS
#include <Wire.h>
#include "drivers/HP203B.h"
#endif

namespace {
// HP203Bは100 ms周期（10 Hz）で気圧・温度・高度を更新する。
constexpr TickType_t PERIOD = pdMS_TO_TICKS(100);
constexpr TickType_t RETRY_PERIOD = pdMS_TO_TICKS(5000);
#ifndef USE_MOCK_SENSORS
HP203B barometer(Wire);
#endif
}

void BaroTask(void *pvParameters) {
    (void)pvParameters;
    TickType_t lastWake = xTaskGetTickCount();
    TickType_t lastRetry = 0;
    bool initialized = false;

    for (;;) {
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
        const float altitude = 20.0f + 5.0f * sinf(millis() / 10000.0f);
        const float pressure = 1013.25f * powf(1.0f - altitude / 44330.0f, 5.255f);
        update_baro_data(pressure, 24.0f, altitude, true);
        task_health_sensor_success(TaskId::BARO);
#endif
        // 取得に失敗してもタスクは継続し、失敗回数だけを記録する。
        task_health_heartbeat(TaskId::BARO);
        vTaskDelayUntil(&lastWake, PERIOD);
    }
}