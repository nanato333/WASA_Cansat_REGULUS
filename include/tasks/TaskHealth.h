#pragma once

#include <Arduino.h>

// 監視対象のタスクID。COUNTは配列サイズ計算専用。
enum class TaskId : uint8_t {
    IMU = 0,
    MAG,
    BARO,
    GNSS,
    MISSION,
    TELEMETRY,
    LOGGER,
    COUNT
};

// LoggerTaskが表示する、各タスクの監視情報のスナップショット。
struct TaskHealthSnapshot {
    uint32_t lastHeartbeatMs;       // 最後にタスクのループが動いた時刻
    uint32_t lastSensorSuccessMs;   // 最後に有効なデータを取得・送信した時刻
    uint32_t heartbeatCount;        // タスクループ実行回数
    uint32_t errorCount;            // センサー取得・送信失敗の累積回数
    uint32_t stackHighWaterMark;    // 起動後に確認された最小スタック残量
};

void init_task_health();
void task_health_heartbeat(TaskId id);
void task_health_sensor_success(TaskId id);
void task_health_sensor_error(TaskId id);
bool get_task_health(TaskId id, TaskHealthSnapshot &snapshot);
const char *task_health_name(TaskId id);
uint32_t task_health_stale_limit_ms(TaskId id);