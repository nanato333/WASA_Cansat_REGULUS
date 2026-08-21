#ifdef ENABLE_ESPNOW
#include <Arduino.h>
#include <WiFi.h>
#include "tasks/TelemetryTask.h"
#include "tasks/MissionTask.h"
#include "tasks/TaskHealth.h"
#include "communication/EspNowRadio.h"
#include "communication/WcppTelemetry.h"
#include "MissionConfig.h"

namespace {
constexpr uint8_t ESPNOW_CHANNEL = 1;
constexpr uint32_t RETRY_MS = 5000;
constexpr uint32_t STATUS_LOG_MS = 5000;
EspNowRadio radio;
}

void TelemetryTask(void *) {
    bool ready = false;
    uint32_t lastRetryMs = 0;
    uint32_t lastSendMs = 0;
    uint32_t lastStatusLogMs = 0;

    for (;;) {
        const uint32_t now = millis();

        if (!ready &&
            (lastRetryMs == 0 || now - lastRetryMs >= RETRY_MS)) {
            lastRetryMs = now;
            ready = radio.begin(ESPNOW_CHANNEL);
            Serial.printf(
                "[Telemetry] ESP-NOW %s channel=%u MAC=%s\n",
                ready ? "ready" : "FAILED",
                static_cast<unsigned int>(ESPNOW_CHANNEL),
                WiFi.macAddress().c_str());
        }

        if (ready) {
            WcppTelemetry::Command command;
            if (radio.takeCommand(command) &&
                !submit_mission_command(command)) {
                Serial.printf("[Telemetry] rejected AC=%u\n", (unsigned)command.action);
            }

            CanSatData_t data{};
            if (get_cansat_data(&data)) {
                const MissionState state =
                    static_cast<MissionState>(data.sys.phase);
                const uint32_t period =
                    state == MissionState::LOW_BATTERY
                        ? MissionConfig::LOW_BATTERY_TELEMETRY_PERIOD_MS
                        : (state == MissionState::LAUNCH
                            ? MissionConfig::FAST_TELEMETRY_PERIOD_MS
                            : MissionConfig::NORMAL_TELEMETRY_PERIOD_MS);

                if (now - lastSendMs >= period) {
                    lastSendMs = now;
                    uint8_t packet[250]{};
                    const size_t length = WcppTelemetry::encode(
                        data, packet, sizeof(packet),
                        state == MissionState::LOW_BATTERY);
                    if (length > 0 && radio.send(packet, length)) {
                        task_health_sensor_success(TaskId::TELEMETRY);
                    } else {
                        task_health_sensor_error(TaskId::TELEMETRY);
                    }
                }
            }

            if (now - lastStatusLogMs >= STATUS_LOG_MS) {
                lastStatusLogMs = now;
                Serial.printf(
                    "[Telemetry] sent=%lu failed=%lu\n",
                    static_cast<unsigned long>(radio.sentCount()),
                    static_cast<unsigned long>(radio.sendFailCount()));
            }
        }

        task_health_heartbeat(TaskId::TELEMETRY);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
#endif