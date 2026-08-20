#include "tasks/SensorBus.h"

#include <Wire.h>
#include "BoardConfig.h"

namespace {
// QMI8658、MMC5603、HP203Bが同じWireを同時操作しないためのMutex。
SemaphoreHandle_t i2cMutex = nullptr;
}

// タスク生成前に一度だけ呼び、MutexとI2Cバスを初期化する。
bool init_sensor_bus() {
    if (i2cMutex != nullptr) return true;
    i2cMutex = xSemaphoreCreateMutex();
    if (i2cMutex == nullptr) return false;
#ifndef USE_MOCK_SENSORS
    return Wire.begin(BoardConfig::I2C_SDA, BoardConfig::I2C_SCL, BoardConfig::I2C_FREQ);
#else
    return true;
#endif
}

// 各I2Cタスクは通信直前に取得し、終了後すぐgive_sensor_bus()で解放する。
bool take_sensor_bus(TickType_t timeout) {
    return i2cMutex != nullptr && xSemaphoreTake(i2cMutex, timeout) == pdTRUE;
}

void give_sensor_bus() {
    if (i2cMutex != nullptr) xSemaphoreGive(i2cMutex);
}