#include "drivers/HP203B.h"
#include <math.h>

namespace {
constexpr uint8_t CMD_SOFT_RESET = 0x06;
constexpr uint8_t CMD_ADC_CVT_PT_OSR_512 = 0x4C;
constexpr uint8_t CMD_READ_PT = 0x10;
int32_t signed24(const uint8_t *p) {
    int32_t value = ((int32_t)p[0] << 16) | ((int32_t)p[1] << 8) | p[2];
    return (value & 0x800000) ? (value | (int32_t)0xFF000000) : value;
}
}

HP203B::HP203B(TwoWire &wire) : wire_(wire) {}

bool HP203B::begin() {
    if (!sendCommand(CMD_SOFT_RESET)) return false;
    delay(20);
    wire_.beginTransmission(ADDRESS);
    return wire_.endTransmission() == 0;
}

bool HP203B::startConversion() {
    return sendCommand(CMD_ADC_CVT_PT_OSR_512);
}

bool HP203B::readConversion(Data &data, float seaLevelHpa) {
    wire_.beginTransmission(ADDRESS); wire_.write(CMD_READ_PT);
    if (wire_.endTransmission(false) != 0) return false;
    if (wire_.requestFrom((uint8_t)ADDRESS, (uint8_t)6) != 6 || wire_.available() < 6) {
        while (wire_.available()) wire_.read();
        return false;
    }
    uint8_t raw[6];
    for (uint8_t i = 0; i < 6; ++i) raw[i] = wire_.read();
    const float pressure = signed24(raw) / 100.0f;
    const float temperature = signed24(raw + 3) / 100.0f;
    if (pressure <= 0.0f || seaLevelHpa <= 0.0f) return false;
    data.pressureHpa = pressure;
    data.temperatureC = temperature;
    data.altitudeM = 44330.0f * (1.0f - powf(pressure / seaLevelHpa, 0.19029495f));
    return true;
}

bool HP203B::read(Data &data, float seaLevelHpa) {
    if (!startConversion()) return false;
    delay(20);
    return readConversion(data, seaLevelHpa);
}

bool HP203B::sendCommand(uint8_t command) {
    wire_.beginTransmission(ADDRESS); wire_.write(command);
    return wire_.endTransmission() == 0;
}