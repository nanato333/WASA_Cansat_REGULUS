#include <Arduino.h>
#include "tasks/LoggerTask.h"
#include "tasks/TaskHealth.h"
#include "CanSatData.h"
#include "MissionState.h"

namespace {
// 2秒ごとにタスクの生存、データ鮮度、失敗回数、最小スタック残量を一覧表示する。
void printTaskHealth(uint32_t now) {
    Serial.println("[TASK] name      state   age_ms sample_ms errors stack_min");
    for (uint8_t raw = 0; raw < static_cast<uint8_t>(TaskId::COUNT); ++raw) {
        const TaskId id = static_cast<TaskId>(raw);
        TaskHealthSnapshot health{};
        if (!get_task_health(id, health)) continue;
        const uint32_t age = health.lastHeartbeatMs == 0 ? UINT32_MAX : now - health.lastHeartbeatMs;
        const bool alive = health.lastHeartbeatMs != 0 && age <= task_health_stale_limit_ms(id);
        const uint32_t sampleAge = health.lastSensorSuccessMs == 0 ? UINT32_MAX : now - health.lastSensorSuccessMs;
        char ageText[12];
        char sampleText[12];
        if (age == UINT32_MAX) snprintf(ageText, sizeof(ageText), "---");
        else snprintf(ageText, sizeof(ageText), "%lu", static_cast<unsigned long>(age));
        if (sampleAge == UINT32_MAX) snprintf(sampleText, sizeof(sampleText), "---");
        else snprintf(sampleText, sizeof(sampleText), "%lu", static_cast<unsigned long>(sampleAge));
        Serial.printf("[TASK] %-9s %-7s %6s %9s %6lu %9lu\n",
                      task_health_name(id), alive ? "ALIVE" : "STALE",
                      ageText, sampleText,
                      static_cast<unsigned long>(health.errorCount),
                      static_cast<unsigned long>(health.stackHighWaterMark));
    }
}
}

// センサー値は500 ms周期で出力し、他タスクより低い優先度で動かす。
void LoggerTask(void *pvParameters) {
    (void)pvParameters;
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(500);
    uint8_t healthPrintDivider = 0;

    for (;;) {
        task_health_heartbeat(TaskId::LOGGER);
        CanSatData_t data{};
        if (get_cansat_data(&data)) {
            const MissionState state = static_cast<MissionState>(data.sys.phase);
            Serial.printf("[SYS ] t=%lu state=%s heap=%u\n",
                          static_cast<unsigned long>(data.sys.timestamp),
                          mission_state_name(state),
                          static_cast<unsigned int>(ESP.getFreeHeap()));
            Serial.printf("[IMU ] %s A=%.2f,%.2f,%.2f G=%.2f,%.2f,%.2f\n",
                          data.imu.is_valid ? "OK" : "NG", data.imu.ax, data.imu.ay, data.imu.az,
                          data.imu.gx, data.imu.gy, data.imu.gz);
            Serial.printf("[MAG ] %s M=%.2f,%.2f,%.2f heading=%.1f\n",
                          data.mag.is_valid ? "OK" : "NG", data.mag.mx, data.mag.my,
                          data.mag.mz, data.mag.heading);
            Serial.printf("[BARO] %s p=%.2f temp=%.2f alt=%.2f\n",
                          data.baro.is_valid ? "OK" : "NG", data.baro.pressure,
                          data.baro.temperature, data.baro.altitude);
            Serial.printf("[GNSS] %s fix=%d lat=%.7f lon=%.7f alt=%.1f sats=%u\n",
                          data.gnss.is_valid ? "OK" : "WAIT", data.gnss.fix,
                          data.gnss.latitude, data.gnss.longitude, data.gnss.altitude,
                          data.gnss.satellites);
        }

        if (++healthPrintDivider >= 4) {
            healthPrintDivider = 0;
            printTaskHealth(millis());
        }
        Serial.println();
        vTaskDelayUntil(&lastWake, period);
    }
}