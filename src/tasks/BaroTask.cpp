#include <Arduino.h>
#include "tasks/BaroTask.h"
#include "CanSatData.h"

static bool init_baro()
{
    // HP203Bの初期化処理を記述
    return true;
}

void BaroTask(void *pvParameters)
{
    if (!init_baro())
    {
        vTaskDelete(NULL);
    }

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // 100ms周期 (10Hz)

    for (;;)
    {
        float press = 0.0f; // 単位: hPa
        float temp = 0.0f;  // 単位: ℃
        float alt = 0.0f;   // 単位: m
        bool is_valid = false;

        // --- センサ値の取得 & 高度計算処理 ---

        // 共有構造体へ保存
        update_baro_data(press, temp, alt, is_valid);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}