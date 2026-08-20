#include "CanSatData.h"

// 全タスクが参照する最新値。直接触らず、以下の更新・取得関数を経由する。
static CanSatData_t g_cansat_data;
static SemaphoreHandle_t g_data_mutex;

void init_cansat_data()
{
    if (g_data_mutex == NULL)
        g_data_mutex = xSemaphoreCreateMutex();
    memset(&g_cansat_data, 0, sizeof(CanSatData_t));
    g_cansat_data.sys.phase = static_cast<uint8_t>(MissionState::ROCKET_LOADED);
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