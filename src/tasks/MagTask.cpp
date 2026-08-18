#include <Arduino.h>
#include "tasks/MagTask.h"
#include "CanSatData.h"

static bool init_mag()
{
    // MMC5603NJの初期化処理を記述
    return true;
}

void MagTask(void *pvParameters)
{
    if (!init_mag())
    {
        vTaskDelete(NULL);
    }

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // 100ms周期 (10Hz)

    for (;;)
    {
        float mx = 0.0f, my = 0.0f, mz = 0.0f; // 単位: uT
        float heading = 0.0f;                  // 単位: deg (0〜360)
        bool is_valid = false;

        // --- センサ値の取得 & 方角計算処理 ---

        // 共有構造体へ保存
        update_mag_data(mx, my, mz, heading, is_valid);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}