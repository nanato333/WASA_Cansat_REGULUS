#include <Arduino.h>
#include "CanSatData.h"
#include "MissionState.h"
#include "tasks/SensorBus.h"
#include "tasks/ImuTask.h"
#include "tasks/MagTask.h"
#include "tasks/BaroTask.h"
#include "tasks/GnssTask.h"
#include "tasks/BatteryTask.h"
#include "tasks/LoggerTask.h"
#include "tasks/MissionTask.h"
#include "tasks/TaskHealth.h"
#ifdef ENABLE_ESPNOW
#include "tasks/TelemetryTask.h"
#endif

// タスク生成処理を共通化し、生成失敗を起動ログで確認できるようにする。
namespace {
bool startTask(TaskFunction_t function, const char *name, uint32_t stackBytes,
               UBaseType_t priority, BaseType_t core) {
    const BaseType_t result = xTaskCreatePinnedToCore(
        function, name, stackBytes, nullptr, priority, nullptr, core);
    Serial.printf("[BOOT] %-10s %s core=%ld priority=%u stack=%lu\n",
                  name, result == pdPASS ? "started" : "FAILED",
                  static_cast<long>(core), static_cast<unsigned int>(priority),
                  static_cast<unsigned long>(stackBytes));
    return result == pdPASS;
}
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.printf("\n=== REGULUS FreeRTOS (%s sensors) ===\n",
#ifdef USE_MOCK_SENSORS
                  "mock"
#else
                  "real"
#endif
    );

    // タスクを起動する前に、共有データ・監視情報・I2C排他制御を準備する。
    init_cansat_data();
    CanSatData_t bootData{};
    if (get_cansat_data(&bootData)) {
        Serial.printf("[BOOT] reset_reason=%u\n",
                      static_cast<unsigned int>(bootData.sys.reset_reason));
    }
    init_task_health();
    update_mission_state(MissionState::BOOT);

    if (!init_sensor_bus()) {
        Serial.println("[BOOT] I2C bus initialization FAILED");
    }

    // センサー取得はCore 1、管理・通信・ログはCore 0へ割り当てる。
    // 優先度は周期の短いIMU/GNSSを高くし、ログを最も低くしている。
    bool allStarted = true;
    allStarted &= startTask(ImuTask, "ImuTask", 3072, 3, 1);
    allStarted &= startTask(GnssTask, "GnssTask", 4096, 3, 1);
    allStarted &= startTask(MagTask, "MagTask", 3072, 2, 1);
    allStarted &= startTask(BaroTask, "BaroTask", 3072, 2, 1);
    allStarted &= startTask(BatteryTask, "BatteryTask", 2048, 2, 0);
    allStarted &= startTask(MissionTask, "MissionTask", 3072, 2, 0);
#ifdef ENABLE_ESPNOW
    allStarted &= startTask(TelemetryTask, "TelemetryTask", 6144, 2, 0);
#endif
    allStarted &= startTask(LoggerTask, "LoggerTask", 4096, 1, 0);

    Serial.printf("[BOOT] task startup %s; mission state=%s\n\n",
                  allStarted ? "complete" : "INCOMPLETE",
                  mission_state_name(MissionState::BOOT));
}

// ArduinoのloopTaskは処理を持たず、実処理は上で生成した各タスクが担当する。
void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}