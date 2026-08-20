#include <Arduino.h>
#include <math.h>
#include "tasks/ImuTask.h"
#include "tasks/SensorBus.h"
#include "tasks/TaskHealth.h"
#include "CanSatData.h"
#ifndef USE_MOCK_SENSORS
#include <Wire.h>
#include "drivers/QMI8658.h"
#endif

namespace {
// QMI8658は10 ms周期（100 Hz）で加速度・角速度を取得する。
constexpr TickType_t PERIOD = pdMS_TO_TICKS(10);
constexpr TickType_t RETRY_PERIOD = pdMS_TO_TICKS(5000);
#ifndef USE_MOCK_SENSORS
QMI8658 imu(Wire, QMI8658::ADDRESS_HIGH);
#endif
}

void ImuTask(void *pvParameters) {
    (void)pvParameters;
    TickType_t lastWake = xTaskGetTickCount();
    TickType_t lastRetry = 0;
    bool initialized = false;

    for (;;) {
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
        update_imu_data(0.15f * sinf(seconds), 0.10f * cosf(seconds), 9.80665f,
                        1.5f * sinf(seconds * 0.5f), 0.8f * cosf(seconds * 0.4f), 5.0f, true);
        task_health_sensor_success(TaskId::IMU);
#endif
        // データ取得成否とは別に、タスク自体の生存を記録する。
        task_health_heartbeat(TaskId::IMU);
        vTaskDelayUntil(&lastWake, PERIOD);
    }
}