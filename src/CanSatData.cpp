#include "CanSatData.h"
#include <esp_system.h>

// 全タスクが参照する最新値。直接触らず、以下の更新・取得関数を経由する。
static CanSatData_t g_cansat_data;
static SemaphoreHandle_t g_data_mutex;

void init_cansat_data()
{
    if (g_data_mutex == NULL)
        g_data_mutex = xSemaphoreCreateMutex();
    memset(&g_cansat_data, 0, sizeof(CanSatData_t));
    g_cansat_data.sys.phase = static_cast<uint8_t>(MissionState::BOOT);
    g_cansat_data.sys.reset_reason = static_cast<uint8_t>(esp_reset_reason());
}

bool update_imu_data(float ax, float ay, float az, float gx, float gy, float gz, bool valid)
{
    if (g_data_mutex == NULL)
        return false;
    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        g_cansat_data.imu.ax = ax;
        g_cansat_data.imu.ay = ay;
        g_cansat_data.imu.az = az;
        g_cansat_data.imu.gx = gx;
        g_cansat_data.imu.gy = gy;
        g_cansat_data.imu.gz = gz;
        g_cansat_data.imu.is_valid = valid;
        xSemaphoreGive(g_data_mutex);
        return true;
    }
    return false;
}

bool update_mag_data(float mx, float my, float mz, float heading, bool valid)
{
    if (g_data_mutex == NULL)
        return false;
    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        g_cansat_data.mag.mx = mx;
        g_cansat_data.mag.my = my;
        g_cansat_data.mag.mz = mz;
        g_cansat_data.mag.heading = heading;
        g_cansat_data.mag.is_valid = valid;
        xSemaphoreGive(g_data_mutex);
        return true;
    }
    return false;
}

bool update_baro_data(float press, float temp, float alt, bool valid)
{
    if (g_data_mutex == NULL)
        return false;
    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        g_cansat_data.baro.pressure = press;
        g_cansat_data.baro.temperature = temp;
        g_cansat_data.baro.altitude = alt;
        g_cansat_data.baro.is_valid = valid;
        xSemaphoreGive(g_data_mutex);
        return true;
    }
    return false;
}

bool update_gnss_data(double lat, double lon, float alt, uint8_t sats, bool fix, bool valid)
{
    if (g_data_mutex == NULL)
        return false;
    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        g_cansat_data.gnss.latitude = lat;
        g_cansat_data.gnss.longitude = lon;
        g_cansat_data.gnss.altitude = alt;
        g_cansat_data.gnss.satellites = sats;
        g_cansat_data.gnss.fix = fix;
        g_cansat_data.gnss.is_valid = valid;
        xSemaphoreGive(g_data_mutex);
        return true;
    }
    return false;
}


// ミッション状態もセンサーデータと同じMutexで保護する。
bool update_mission_state(MissionState state)
{
    if (g_data_mutex == NULL)
        return false;
    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        g_cansat_data.sys.phase = static_cast<uint8_t>(state);
        xSemaphoreGive(g_data_mutex);
        return true;
    }
    return false;
}
// Mutex保持時間を短くするため、呼出側へ構造体全体をコピーしてから処理する。
bool update_navigation_status(bool configured, double latitude, double longitude, float distance_m, uint8_t motor_state, uint8_t separation_status)
{
 if(!g_data_mutex)return false; if(xSemaphoreTake(g_data_mutex,pdMS_TO_TICKS(10))!=pdTRUE)return false;
 g_cansat_data.sys.goal_configured=configured;g_cansat_data.sys.goal_latitude=latitude;g_cansat_data.sys.goal_longitude=longitude;g_cansat_data.sys.goal_distance_m=distance_m;g_cansat_data.sys.motor_state=motor_state;g_cansat_data.sys.separation_status=separation_status;
 xSemaphoreGive(g_data_mutex);return true;
}
bool get_cansat_data(CanSatData_t *out_data)
{
    if (g_data_mutex == NULL || out_data == NULL)
        return false;
    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        g_cansat_data.sys.timestamp = millis();
        *out_data = g_cansat_data;
        xSemaphoreGive(g_data_mutex);
        return true;
    }
    return false;
}
bool update_mission_status(MissionState state, MissionSubState substate, OperationMode mode)
{
    if (g_data_mutex == NULL) return false;
    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        g_cansat_data.sys.phase = static_cast<uint8_t>(state);
        g_cansat_data.sys.subphase = static_cast<uint8_t>(substate);
        g_cansat_data.sys.operation_mode = static_cast<uint8_t>(mode);
        xSemaphoreGive(g_data_mutex);
        return true;
    }
    return false;
}
bool update_system_status(float battery_voltage, int rssi)
{
    if (g_data_mutex == NULL) return false;
    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        g_cansat_data.sys.battery_voltage = battery_voltage;
        g_cansat_data.sys.rssi = rssi;
        xSemaphoreGive(g_data_mutex);
        return true;
    }
    return false;
}
bool update_battery_voltage(float battery_voltage)
{
    if (g_data_mutex == NULL) return false;
    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        g_cansat_data.sys.battery_voltage = battery_voltage;
        xSemaphoreGive(g_data_mutex);
        return true;
    }
    return false;
}
bool update_failsafe_reason(FailsafeReason reason)
{
    if (g_data_mutex == NULL) return false;
    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        g_cansat_data.sys.failsafe_reason = static_cast<uint8_t>(reason);
        xSemaphoreGive(g_data_mutex);
        return true;
    }
    return false;
}