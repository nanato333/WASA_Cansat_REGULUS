#include <Arduino.h>
#include "tasks/GnssTask.h"
#include "CanSatData.h"

static bool init_gnss()
{
    // 例: Serial2.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
    return true;
}

void GnssTask(void *pvParameters)
{
    if (!init_gnss())
    {
        vTaskDelete(NULL);
    }

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(200); // 200ms周期 (5Hz)

    for (;;)
    {
        double lat = 0.0, lon = 0.0;
        float alt = 0.0f;
        uint8_t sats = 0;
        bool fix = false;
        bool valid = false;

        // ここでUARTからのNMEA文のパース処理などを記述してもらう

        update_gnss_data(lat, lon, alt, sats, fix, valid);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}