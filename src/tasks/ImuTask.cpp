#include <Arduino.h>
#include "tasks/ImuTask.h"
#include "CanSatData.h"

static bool init_imu()
{
    // QMI8658Aの初期化処理を記述
    return true;
}

void ImuTask(void *pvParameters)
{
    if (!init_imu())
    {
        vTaskDelete(NULL);
    }

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // 100ms周期 (10Hz)

    for (;;)
    {
        float ax = 0.0f, ay = 0.0f, az = 0.0f; // 単位: m/s^2
        float gx = 0.0f, gy = 0.0f, gz = 0.0f; // 単位: dps
        bool is_valid = false;

        // --- センサ値の取得処理 ---

        // 共有構造体へ保存
        update_imu_data(ax, ay, az, gx, gy, gz, is_valid);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}