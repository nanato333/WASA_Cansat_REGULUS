#include <Arduino.h>
#include "tasks/BatteryTask.h"
#include "tasks/TaskHealth.h"
#include "BatteryMonitor.h"
#include "CanSatData.h"

namespace {
constexpr TickType_t BATTERY_PERIOD = pdMS_TO_TICKS(1000);
}

void BatteryTask(void *pvParameters) {
    (void)pvParameters;
    TickType_t lastWake = xTaskGetTickCount();
#ifndef USE_MOCK_SENSORS
    BatteryMonitor::begin();
#endif

    for (;;) {
#ifdef USE_MOCK_SENSORS
        const float voltage = 4.0f;
#else
        const float voltage = BatteryMonitor::readVoltage();
#endif
        if (update_battery_voltage(voltage)) {
            task_health_sensor_success(TaskId::BATTERY);
        } else {
            task_health_sensor_error(TaskId::BATTERY);
        }
        task_health_heartbeat(TaskId::BATTERY);
        vTaskDelayUntil(&lastWake, BATTERY_PERIOD);
    }
}