#include "tasks/TaskHealth.h"

namespace {
// Core 0/1の両方から短時間アクセスするため、軽量なクリティカルセクションで保護する。
portMUX_TYPE healthMux = portMUX_INITIALIZER_UNLOCKED;
TaskHealthSnapshot health[static_cast<size_t>(TaskId::COUNT)]{};
uint32_t lastStackCheckMs[static_cast<size_t>(TaskId::COUNT)]{};

bool validId(TaskId id) {
    return static_cast<size_t>(id) < static_cast<size_t>(TaskId::COUNT);
}
}

void init_task_health() {
    portENTER_CRITICAL(&healthMux);
    memset(health, 0, sizeof(health));
    memset(lastStackCheckMs, 0, sizeof(lastStackCheckMs));
    portEXIT_CRITICAL(&healthMux);
}

// 各周期で生存時刻を更新する。負荷を抑えるためスタック計測は1秒ごとに行う。
void task_health_heartbeat(TaskId id) {
    if (!validId(id)) return;
    const size_t index = static_cast<size_t>(id);
    const uint32_t now = millis();
    bool checkStack = false;
    portENTER_CRITICAL(&healthMux);
    health[index].lastHeartbeatMs = now;
    health[index].heartbeatCount++;
    if (lastStackCheckMs[index] == 0 || now - lastStackCheckMs[index] >= 1000) {
        lastStackCheckMs[index] = now;
        checkStack = true;
    }
    portEXIT_CRITICAL(&healthMux);

    if (checkStack) {
        const uint32_t stackMark = uxTaskGetStackHighWaterMark(nullptr);
        portENTER_CRITICAL(&healthMux);
        health[index].stackHighWaterMark = stackMark;
        portEXIT_CRITICAL(&healthMux);
    }
}

// センサー取得またはテレメトリ送信が成功した時刻を保存する。
void task_health_sensor_success(TaskId id) {
    if (!validId(id)) return;
    const uint32_t now = millis();
    portENTER_CRITICAL(&healthMux);
    health[static_cast<size_t>(id)].lastSensorSuccessMs = now;
    portEXIT_CRITICAL(&healthMux);
}

void task_health_sensor_error(TaskId id) {
    if (!validId(id)) return;
    portENTER_CRITICAL(&healthMux);
    health[static_cast<size_t>(id)].errorCount++;
    portEXIT_CRITICAL(&healthMux);
}

bool get_task_health(TaskId id, TaskHealthSnapshot &snapshot) {
    if (!validId(id)) return false;
    portENTER_CRITICAL(&healthMux);
    snapshot = health[static_cast<size_t>(id)];
    portEXIT_CRITICAL(&healthMux);
    return true;
}

const char *task_health_name(TaskId id) {
    switch (id) {
        case TaskId::IMU: return "IMU";
        case TaskId::MAG: return "MAG";
        case TaskId::BARO: return "BARO";
        case TaskId::GNSS: return "GNSS";
        case TaskId::BATTERY: return "BATTERY";
        case TaskId::MISSION: return "MISSION";
        case TaskId::TELEMETRY: return "TELEMETRY";
        case TaskId::LOGGER: return "LOGGER";
        default: return "UNKNOWN";
    }
}

// 通常周期より十分長い猶予を持たせ、これを超えた場合にSTALEと表示する。
uint32_t task_health_stale_limit_ms(TaskId id) {
    switch (id) {
        case TaskId::IMU: return 100;
        case TaskId::MAG: return 200;
        case TaskId::BARO: return 500;
        case TaskId::GNSS: return 500;
        case TaskId::BATTERY: return 2500;
        case TaskId::MISSION: return 500;
        case TaskId::TELEMETRY: return 500;
        case TaskId::LOGGER: return 1500;
        default: return 0;
    }
}