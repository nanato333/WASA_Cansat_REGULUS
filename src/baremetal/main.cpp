#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "BoardConfig.h"
#include "CanSatData.h"
#include "MissionController.h"
#include "MissionConfig.h"
#include "BatteryMonitor.h"
#ifdef ENABLE_ESPNOW
#include <WiFi.h>
#include "communication/EspNowRadio.h"
#include "communication/WcppTelemetry.h"
#endif

#ifndef USE_MOCK_SENSORS
#include "drivers/QMI8658.h"
#include "drivers/MMC5603.h"
#include "drivers/HP203B.h"
#include "drivers/MaxM10M.h"
#endif

namespace {



constexpr uint32_t PRINT_INTERVAL_MS = 500;
constexpr uint32_t RETRY_INTERVAL_MS = 5000;
#ifdef ENABLE_ESPNOW

#endif

struct SensorState {
    bool initialized = false;
    bool valid = false;
    uint32_t lastSuccessMs = 0;
    uint32_t errorCount = 0;
    uint32_t lastRetryMs = 0;
};

SensorState imuState, magState, baroState, gnssState;
MissionController mission;
uint32_t lastImuMs = 0, lastMagMs = 0, lastBaroMs = 0, lastBatteryMs = 0, lastPrintMs = 0;
#ifdef ENABLE_ESPNOW
uint32_t lastRadioMs = 0;
EspNowRadio radio;
bool radioReady = false;
#endif
bool baroConversionPending = false;
uint32_t baroConversionStartedMs = 0;

#ifndef USE_MOCK_SENSORS
QMI8658 imu(Wire, QMI8658::ADDRESS_LOW);
MMC5603 mag(Wire);
HP203B baro(Wire);
HardwareSerial gnssSerial(1);
MaxM10M gnss(gnssSerial);
#endif

bool due(uint32_t now, uint32_t &previous, uint32_t interval) {
    if (now - previous < interval) return false;
    previous = now;
    return true;
}

#ifndef USE_MOCK_SENSORS
void retrySensors(uint32_t now) {
    if (!imuState.initialized && now - imuState.lastRetryMs >= RETRY_INTERVAL_MS) {
        imuState.lastRetryMs = now; imuState.initialized = imu.begin();
    }
    if (!magState.initialized && now - magState.lastRetryMs >= RETRY_INTERVAL_MS) {
        magState.lastRetryMs = now; magState.initialized = mag.begin();
    }
    if (!baroState.initialized && now - baroState.lastRetryMs >= RETRY_INTERVAL_MS) {
        baroState.lastRetryMs = now; baroState.initialized = baro.begin();
    }
}

void updateImu(uint32_t now) {
    if (!due(now, lastImuMs, mission.state() == MissionState::LAUNCH ? MissionConfig::FAST_IMU_PERIOD_MS : MissionConfig::NORMAL_IMU_PERIOD_MS) || !imuState.initialized) return;
    QMI8658::Data data;
    imuState.valid = imu.read(data);
    if (imuState.valid) {
        imuState.lastSuccessMs = now;
        update_imu_data(data.ax, data.ay, data.az, data.gx, data.gy, data.gz, true);
    } else {
        ++imuState.errorCount;
        update_imu_data(0, 0, 0, 0, 0, 0, false);
    }
}

void updateMag(uint32_t now) {
    if (!due(now, lastMagMs, mission.state() == MissionState::LAUNCH ? MissionConfig::FAST_MAG_PERIOD_MS : MissionConfig::NORMAL_MAG_PERIOD_MS) || !magState.initialized) return;
    if (!mag.isMMCdataready()) return;
    MMC5603::MagData data;
    magState.valid = mag.read(data);
    if (magState.valid) {
        float heading = atan2f(data.magY, data.magX) * 180.0f / PI;
        if (heading < 0.0f) heading += 360.0f;
        magState.lastSuccessMs = now;
        update_mag_data(data.magX, data.magY, data.magZ, heading, true);
    } else {
        ++magState.errorCount;
        update_mag_data(0, 0, 0, 0, false);
    }
}

void updateBaro(uint32_t now) {
    if (!baroState.initialized) return;
    if (!baroConversionPending) {
        if (!due(now, lastBaroMs, mission.state() == MissionState::LAUNCH ? MissionConfig::FAST_BARO_PERIOD_MS : MissionConfig::NORMAL_BARO_PERIOD_MS)) return;
        baroConversionPending = baro.startConversion();
        baroConversionStartedMs = now;
        if (!baroConversionPending) ++baroState.errorCount;
        return;
    }
    if (now - baroConversionStartedMs < 20) return;
    baroConversionPending = false;
    HP203B::Data data;
    baroState.valid = baro.readConversion(data);
    if (baroState.valid) {
        baroState.lastSuccessMs = now;
        update_baro_data(data.pressureHpa, data.temperatureC, data.altitudeM, true);
    } else {
        ++baroState.errorCount;
        update_baro_data(0, 0, 0, false);
    }
}

void updateGnss(uint32_t now) {
    gnss.update();
    const MaxM10M::Data &data = gnss.data();
    gnssState.valid = data.valid;
    if (data.valid) gnssState.lastSuccessMs = now;
    update_gnss_data(data.latitude, data.longitude, data.altitudeM,
                     data.satellites, data.fix, data.valid);
}
#else
void retrySensors(uint32_t) {
    imuState.initialized = magState.initialized = true;
    baroState.initialized = gnssState.initialized = true;
}

void updateImu(uint32_t now) {
    if (!due(now, lastImuMs, mission.state() == MissionState::LAUNCH ? MissionConfig::FAST_IMU_PERIOD_MS : MissionConfig::NORMAL_IMU_PERIOD_MS)) return;
    const float t = now / 1000.0f;
    update_imu_data(0.15f*sinf(t), 0.10f*cosf(t), 9.80665f,
                    1.5f*sinf(t*0.5f), 0.8f*cosf(t*0.4f), 5.0f, true);
    imuState.valid = true; imuState.lastSuccessMs = now;
}
void updateMag(uint32_t now) {
    if (!due(now, lastMagMs, mission.state() == MissionState::LAUNCH ? MissionConfig::FAST_MAG_PERIOD_MS : MissionConfig::NORMAL_MAG_PERIOD_MS)) return;
    const float heading = fmodf(now * 0.02f, 360.0f);
    const float rad = heading * PI / 180.0f;
    update_mag_data(35.0f*cosf(rad), 35.0f*sinf(rad), 28.0f, heading, true);
    magState.valid = true; magState.lastSuccessMs = now;
}
void updateBaro(uint32_t now) {
    if (!due(now, lastBaroMs, mission.state() == MissionState::LAUNCH ? MissionConfig::FAST_BARO_PERIOD_MS : MissionConfig::NORMAL_BARO_PERIOD_MS)) return;
    const float altitude = 20.0f + 5.0f*sinf(now/10000.0f);
    const float pressure = 1013.25f*powf(1.0f-altitude/44330.0f, 5.255f);
    update_baro_data(pressure, 24.0f, altitude, true);
    baroState.valid = true; baroState.lastSuccessMs = now;
}
void updateGnss(uint32_t now) {
    const double offset = (now / 1000) * 0.000001;
    update_gnss_data(35.681236 + offset, 139.767125 + offset,
                     25.0f, 12, true, true);
    gnssState.valid = true; gnssState.lastSuccessMs = now;
}
#endif


void updateBattery(uint32_t now) {
    if (!due(now, lastBatteryMs, 1000)) return;
#ifdef USE_MOCK_SENSORS
    update_battery_voltage(4.0f);
#else
    update_battery_voltage(BatteryMonitor::readVoltage());
#endif
}
#ifdef ENABLE_ESPNOW
void updateRadio(uint32_t now) {
    if (!radioReady) return;
    CanSatData_t current{}; get_cansat_data(&current);
    MissionState state = static_cast<MissionState>(current.sys.phase);
    const uint32_t interval = state == MissionState::LOW_BATTERY ? MissionConfig::LOW_BATTERY_TELEMETRY_PERIOD_MS : (state == MissionState::LAUNCH ? MissionConfig::FAST_TELEMETRY_PERIOD_MS : MissionConfig::NORMAL_TELEMETRY_PERIOD_MS);
    if (!due(now, lastRadioMs, interval)) return;
    CanSatData_t data;
    if (!get_cansat_data(&data)) return;
    uint8_t packet[250];
    const size_t length = WcppTelemetry::encode(data, packet, sizeof(packet), state == MissionState::LOW_BATTERY);
    if (length > 0) radio.send(packet, length);
    uint8_t action = 0;
    if (radio.takeAction(action)) {
        if (!mission.handleCommand(action, now)) Serial.printf("[RADIO] rejected AC=%u\n", action);
    }
}
#endif
void printState(uint32_t now) {
    if (!due(now, lastPrintMs, PRINT_INTERVAL_MS)) return;
    CanSatData_t d;
    if (!get_cansat_data(&d)) return;
    Serial.printf("[IMU ] %s A=%.2f,%.2f,%.2f G=%.2f,%.2f,%.2f err=%lu\n",
                  d.imu.is_valid?"OK":"NG", d.imu.ax,d.imu.ay,d.imu.az,
                  d.imu.gx,d.imu.gy,d.imu.gz,(unsigned long)imuState.errorCount);
    Serial.printf("[MAG ] %s M=%.2f,%.2f,%.2f heading=%.1f err=%lu\n",
                  d.mag.is_valid?"OK":"NG",d.mag.mx,d.mag.my,d.mag.mz,d.mag.heading,
                  (unsigned long)magState.errorCount);
    Serial.printf("[BARO] %s p=%.2f temp=%.2f alt=%.2f err=%lu\n",
                  d.baro.is_valid?"OK":"NG",d.baro.pressure,d.baro.temperature,d.baro.altitude,
                  (unsigned long)baroState.errorCount);
    Serial.printf("[GNSS] %s fix=%d lat=%.7f lon=%.7f alt=%.1f sats=%u\n\n",
                  d.gnss.is_valid?"OK":"WAIT",d.gnss.fix,d.gnss.latitude,d.gnss.longitude,
                  d.gnss.altitude,d.gnss.satellites);
    Serial.printf("[VBAT] %.3f V\n\n", d.sys.battery_voltage);
}
}

void setup() {
    Serial.begin(115200); delay(1000);
    init_cansat_data();
#ifndef USE_MOCK_SENSORS
    BatteryMonitor::begin();
#endif
    mission.begin();
#ifndef USE_MOCK_SENSORS
    Wire.begin(BoardConfig::I2C_SDA, BoardConfig::I2C_SCL, BoardConfig::I2C_FREQ);
    imuState.initialized = imu.begin();
    magState.initialized = mag.begin();
    baroState.initialized = baro.begin();
    gnss.begin(BoardConfig::GNSS_BAUD, BoardConfig::GNSS_RX, BoardConfig::GNSS_TX);
    gnssState.initialized = true;
#else
    retrySensors(0);
#endif
#ifdef ENABLE_ESPNOW
    radioReady = radio.begin(1);
    Serial.printf("ESP-NOW channel 1: %s, MAC=%s\n", radioReady ? "ready" : "failed", WiFi.macAddress().c_str());
#endif
    Serial.printf("=== REGULUS superloop (%s sensors) ===\n",
#ifdef USE_MOCK_SENSORS
                  "mock"
#else
                  "real"
#endif
    );
}

void loop() {
    const uint32_t now = millis();
    retrySensors(now);
    updateGnss(now);
    updateImu(now);
    updateMag(now);
    updateBaro(now);
    updateBattery(now);
    CanSatData_t missionData{};
    if (get_cansat_data(&missionData)) mission.update(missionData, now);
#ifdef ENABLE_ESPNOW
    updateRadio(now);
#endif
    printState(now);
    yield();
}