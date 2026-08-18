#ifndef CANSAT_DATA_H
#define CANSAT_DATA_H

#include <Arduino.h>

// --- データ構造体 ---
typedef struct
{
    float ax, ay, az; // 加速度 [m/s^2]
    float gx, gy, gz; // 角速度 [dps]
    bool is_valid;    // 取得成功フラグ
} IMUData_t;

typedef struct
{
    float mx, my, mz; // 磁気 [uT]
    float heading;    // 方角 [deg] (0〜360)
    bool is_valid;    // 取得成功フラグ
} MagData_t;

typedef struct
{
    float pressure;    // 気圧 [hPa]
    float temperature; // 気温 [℃]
    float altitude;    // 高度 [m]
    bool is_valid;     // 取得成功フラグ
} BaroData_t;

typedef struct
{
    double latitude;    // 緯度 [deg]
    double longitude;   // 経度 [deg]
    float altitude;     // 海抜高度 [m]
    uint8_t satellites; // 衛星数
    bool fix;           // 3D Fix測位完了フラグ
    bool is_valid;      // UART受信正常フラグ
} GNSSData_t;

typedef struct
{
    uint8_t phase;         // ミッションフェーズ (0〜7)
    float battery_voltage; // バッテリー電圧 [V]
    int rssi;              // 電波強度 [dBm]
    uint32_t timestamp;    // 経過時間 [ms]
} SystemStatus_t;

typedef struct
{
    IMUData_t imu;
    MagData_t mag;
    BaroData_t baro;
    GNSSData_t gnss;
    SystemStatus_t sys;
} CanSatData_t;

// --- 初期化 & アクセス関数プロトタイプ ---
void init_cansat_data();

bool update_imu_data(float ax, float ay, float az, float gx, float gy, float gz, bool valid);
bool update_mag_data(float mx, float my, float mz, float heading, bool valid);
bool update_baro_data(float press, float temp, float alt, bool valid);
bool update_gnss_data(double lat, double lon, float alt, uint8_t sats, bool fix, bool valid);

bool get_cansat_data(CanSatData_t *out_data);

#endif