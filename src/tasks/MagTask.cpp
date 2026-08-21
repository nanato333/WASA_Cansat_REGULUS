#include <Arduino.h>
#include <math.h>
#include "tasks/MagTask.h"
#include "tasks/SensorBus.h"
#include "tasks/TaskHealth.h"
#include "CanSatData.h"
#include "MissionConfig.h"
#ifndef USE_MOCK_SENSORS
#include <Wire.h>
#include "drivers/MMC5603.h"
#endif

namespace {
// MMC5603は20 ms周期（50 Hz）で磁気ベクトルと方位を更新する。

constexpr TickType_t RETRY_PERIOD = pdMS_TO_TICKS(5000);
#ifndef USE_MOCK_SENSORS
MMC5603 magnetometer(Wire);
#endif
}

void MagTask(void *pvParameters) {
    (void)pvParameters;
    TickType_t lastWake = xTaskGetTickCount();
    TickType_t lastRetry = 0;
    bool initialized = false;

    for (;;) {
#ifndef USE_MOCK_SENSORS
        const TickType_t now = xTaskGetTickCount();
        // センサーが見つからなくても5秒ごとに初期化を再試行する。
        if (!initialized && (lastRetry == 0 || now - lastRetry >= RETRY_PERIOD)) {
            lastRetry = now;
            if (take_sensor_bus(pdMS_TO_TICKS(50))) {
                initialized = magnetometer.begin();
                give_sensor_bus();
            }
            Serial.printf("[MagTask] init %s\n", initialized ? "OK" : "failed");
        }

        MMC5603::MagData data{};
        bool valid = false;
        if (initialized && take_sensor_bus(pdMS_TO_TICKS(5))) {
            if (magnetometer.isMMCdataready()) valid = magnetometer.read(data);
            give_sensor_bus();
        }
        if (valid) {
            // 未校正の水平面方位。実機ではハード／ソフトアイアン補正が必要。
            float heading = atan2f(data.magY, data.magX) * 180.0f / PI;
            if (heading < 0.0f) heading += 360.0f;
            update_mag_data(data.magX, data.magY, data.magZ, heading, true);
            task_health_sensor_success(TaskId::MAG);
        } else {
            update_mag_data(0.0f, 0.0f, 0.0f, 0.0f, false);
            task_health_sensor_error(TaskId::MAG);
        }
#else
        const float heading = fmodf(millis() * 0.02f, 360.0f);
        const float radians = heading * PI / 180.0f;
        update_mag_data(35.0f * cosf(radians), 35.0f * sinf(radians), 28.0f, heading, true);
        task_health_sensor_success(TaskId::MAG);
#endif
        // タスク周期が止まっていないことをTaskHealthへ通知する。
        task_health_heartbeat(TaskId::MAG);
        const MissionState state = static_cast<MissionState>(([](){ CanSatData_t d{}; get_cansat_data(&d); return d.sys.phase; })());
        const TickType_t period = pdMS_TO_TICKS(state == MissionState::LAUNCH ? MissionConfig::FAST_MAG_PERIOD_MS : MissionConfig::NORMAL_MAG_PERIOD_MS);
        vTaskDelayUntil(&lastWake, period);
    }
}