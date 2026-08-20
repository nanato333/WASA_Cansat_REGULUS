#ifdef ENABLE_ESPNOW
#include <Arduino.h>
#include <WiFi.h>
#include "tasks/TelemetryTask.h"
#include "tasks/TaskHealth.h"
#include "communication/EspNowRadio.h"
#include "communication/WcppTelemetry.h"
#include "CanSatData.h"

namespace {
// 最新の共有データを100 ms周期でWCPPパケットとして送信する。
constexpr TickType_t PERIOD = pdMS_TO_TICKS(100);
constexpr uint32_t RETRY_INTERVAL_MS = 5000;
constexpr uint8_t ESPNOW_CHANNEL = 1;
EspNowRadio radio;
}

void TelemetryTask(void *pvParameters) {
    (void)pvParameters;
    TickType_t lastWake = xTaskGetTickCount();
    bool radioReady = false;
    uint32_t lastRetryMs = 0;
    uint32_t lastStatsMs = 0;

    for (;;) {
        const uint32_t now = millis();
        // 初期化に失敗しても他タスクは止めず、5秒後に無線だけ再試行する。
        if (!radioReady && (lastRetryMs == 0 || now - lastRetryMs >= RETRY_INTERVAL_MS)) {
            lastRetryMs = now;
            radioReady = radio.begin(ESPNOW_CHANNEL);
            if (radioReady) {
                Serial.printf("[TelemetryTask] ESP-NOW ready channel=%u MAC=%s\n",
                              ESPNOW_CHANNEL, WiFi.macAddress().c_str());
            } else {
                Serial.println("[TelemetryTask] ESP-NOW initialization failed; retrying in 5 s");
                task_health_sensor_error(TaskId::TELEMETRY);
            }
        }

        if (radioReady) {
            // Mutexで保護された共有データをコピーしてからパケット化する。
            CanSatData_t data{};
            uint8_t packet[250]{};
            if (get_cansat_data(&data)) {
                const size_t length = WcppTelemetry::encode(data, packet, sizeof(packet));
                if (length > 0 && radio.send(packet, length)) {
                    task_health_sensor_success(TaskId::TELEMETRY);
                } else {
                    task_health_sensor_error(TaskId::TELEMETRY);
                }
            }

            // PC操作は通信試験用として受信するが、モーターには接続しない。
            uint8_t action = 0;
            if (radio.takeAction(action)) {
                Serial.printf("[TelemetryTask] action=%u ignored; motor control is disabled\n", action);
            }

            if (now - lastStatsMs >= 5000) {
                lastStatsMs = now;
                Serial.printf("[TelemetryTask] queued=%lu rejected=%lu\n",
                              static_cast<unsigned long>(radio.sentCount()),
                              static_cast<unsigned long>(radio.sendFailCount()));
            }
        }

        // 送信成功とは別に、TelemetryTask自体が動いていることを記録する。
        task_health_heartbeat(TaskId::TELEMETRY);
        vTaskDelayUntil(&lastWake, PERIOD);
    }
}
#endif
