#include <Arduino.h>
#include "tasks/GnssTask.h"
#include "tasks/TaskHealth.h"
#include "BoardConfig.h"
#include "CanSatData.h"
#ifndef USE_MOCK_SENSORS
#include "drivers/MaxM10M.h"
#endif

namespace {
// UART受信バッファを溢れさせないよう、10 ms周期でNMEAを処理する。
constexpr TickType_t PERIOD = pdMS_TO_TICKS(10);
#ifndef USE_MOCK_SENSORS
HardwareSerial gnssSerial(1);
MaxM10M gnss(gnssSerial);
#endif
}

void GnssTask(void *pvParameters) {
    (void)pvParameters;
#ifndef USE_MOCK_SENSORS
    gnss.begin(BoardConfig::GNSS_BAUD, BoardConfig::GNSS_RX, BoardConfig::GNSS_TX);
    Serial.println("[GnssTask] UART ready");
#endif
    TickType_t lastWake = xTaskGetTickCount();

    for (;;) {
#ifndef USE_MOCK_SENSORS
        // update()は受信済み文字列を読み、GGA/RMCが完成した時点で共有値を更新する。
        gnss.update();
        const MaxM10M::Data &data = gnss.data();
        update_gnss_data(data.latitude, data.longitude, data.altitudeM,
                         data.satellites, data.fix, data.valid);
        // Fixなしはタスク異常ではない。有効なNMEA受信時刻だけを成功として記録する。
        if (data.valid) task_health_sensor_success(TaskId::GNSS);
#else
        const double offset = (millis() / 1000UL) * 0.000001;
        update_gnss_data(35.681236 + offset, 139.767125 + offset, 25.0f, 12, true, true);
        task_health_sensor_success(TaskId::GNSS);
#endif
        // GNSS Fixの有無に関係なくUART処理タスクの生存を記録する。
        task_health_heartbeat(TaskId::GNSS);
        vTaskDelayUntil(&lastWake, PERIOD);
    }
}