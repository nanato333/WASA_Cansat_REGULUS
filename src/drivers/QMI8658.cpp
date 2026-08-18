#include "drivers/QMI8658.h"

namespace {
constexpr uint8_t REG_WHO_AM_I = 0x00;
constexpr uint8_t REG_CTRL1 = 0x02;
constexpr uint8_t REG_CTRL2 = 0x03;
constexpr uint8_t REG_CTRL3 = 0x04;
constexpr uint8_t REG_CTRL7 = 0x08;
constexpr uint8_t REG_AX_L = 0x35;
constexpr float GRAVITY = 9.80665f;
}

QMI8658::QMI8658(TwoWire &wire, uint8_t address) : wire_(wire), address_(address) {}

bool QMI8658::begin() {
    uint8_t id = 0;
    if (!readRegister(REG_WHO_AM_I, id) || id != 0x05) return false;
    if (!writeRegister(REG_CTRL1, 0x40)) return false;
    if (!writeRegister(REG_CTRL2, 0x26)) return false;
    if (!writeRegister(REG_CTRL3, 0x76)) return false;
    if (!writeRegister(REG_CTRL7, 0x03)) return false;
    delay(20);
    return true;
}

bool QMI8658::read(Data &data) {
    wire_.beginTransmission(address_);
    wire_.write(REG_AX_L);
    if (wire_.endTransmission(false) != 0) return false;
    if (wire_.requestFrom(address_, (uint8_t)12) != 12 || wire_.available() < 12) {
        while (wire_.available()) wire_.read();
        return false;
    }
    int16_t raw[6];
    for (uint8_t i = 0; i < 6; ++i) {
        const uint8_t low = wire_.read();
        const uint8_t high = wire_.read();
        raw[i] = (int16_t)((uint16_t)high << 8 | low);
    }
    constexpr float accelScale = GRAVITY / 4096.0f;
    constexpr float gyroScale = 1.0f / 16.0f;
    data = {raw[0]*accelScale, raw[1]*accelScale, raw[2]*accelScale,
            raw[3]*gyroScale, raw[4]*gyroScale, raw[5]*gyroScale};
    return true;
}

bool QMI8658::readRegister(uint8_t reg, uint8_t &value) {
    wire_.beginTransmission(address_); wire_.write(reg);
    if (wire_.endTransmission(false) != 0) return false;
    if (wire_.requestFrom(address_, (uint8_t)1) != 1 || wire_.available() < 1) return false;
    value = wire_.read(); return true;
}

bool QMI8658::writeRegister(uint8_t reg, uint8_t value) {
    wire_.beginTransmission(address_); wire_.write(reg); wire_.write(value);
    return wire_.endTransmission() == 0;
}