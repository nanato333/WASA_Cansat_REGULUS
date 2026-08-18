#include <Arduino.h>
#include "drivers/MMC5603.h"

MMC5603::MMC5603(TwoWire &wire) : wire(wire) {}

bool MMC5603::begin() {  //Wire.begin()はmainでやること
  // 1. 接続確認 (Chip IDのチェック)
  uint8_t productId = 0;
  if (!readRegister8(MMC5603_REG_ID, productId) || productId != 0x10) {
    return false;
  }

  if (!writeRegister8(MMC5603_REG_ODR, MMC5603_ODR_100HZ)) return false;
  delay(1);
  if (!writeRegister8(MMC5603_REG_CTRL1, MMC5603_CTRL1)) return false;
  delay(1);
  if (!writeRegister8(MMC5603_REG_CTRL0, MMC5603_CTRL0)) return false;
  delay(1);
  if (!writeRegister8(MMC5603_REG_CTRL2, MMC5603_CTRL2)) return false;
  delay(10);

  return true;
}

bool MMC5603::read(MagData &magData) {
  if (!isMMCdataready()) {
    return false;
  }

  wire.beginTransmission(MMC5603_I2C_ADDR);
  wire.write(MMC5603_REG_DATA);
  if (wire.endTransmission(false) != 0) {
    return false;
  }

  // X, Y, Z (各3バイト) ＝ 計9バイトを一括要求
  if (wire.requestFrom((uint8_t)MMC5603_I2C_ADDR, (uint8_t)9) != 9 || wire.available() < 9) {
    while (wire.available()) wire.read();
    return false;
  }

  uint8_t buf[9];
  for (uint8_t i = 0; i < 9; i++) {
    buf[i] = wire.read();
  }

  const uint32_t xRaw = ((uint32_t)buf[0] << 12) | ((uint32_t)buf[1] << 4) | ((uint32_t)buf[6] >> 4);
  const uint32_t yRaw = ((uint32_t)buf[2] << 12) | ((uint32_t)buf[3] << 4) | ((uint32_t)buf[7] >> 4);
  const uint32_t zRaw = ((uint32_t)buf[4] << 12) | ((uint32_t)buf[5] << 4) | ((uint32_t)buf[8] >> 4);

  // 20ビットモードの場合、生データは符号なし(0 ~ 1048575)で、中心が524288。
  // そのため、524288を引いて ±524288 の符号付き整数（中心0）に変換する
  magData.magX = ((int32_t)xRaw - 524288) * MMC5603_LSB_RESOLUTION;
  magData.magY = ((int32_t)yRaw - 524288) * MMC5603_LSB_RESOLUTION;
  magData.magZ = ((int32_t)zRaw - 524288) * MMC5603_LSB_RESOLUTION;
  return true;
}

struct MMC5603::MagData MMC5603::read() {
  MagData magData;
  read(magData);
  return magData;
}

bool MMC5603::readRegister8(uint8_t reg, uint8_t &data) {
  wire.beginTransmission(MMC5603_I2C_ADDR);
  wire.write(reg);
  if (wire.endTransmission(false) != 0) return false;
  if (wire.requestFrom((uint8_t)MMC5603_I2C_ADDR, (uint8_t)1) != 1 || wire.available() < 1) return false;
  data = wire.read();
  return true;
}

bool MMC5603::writeRegister8(uint8_t reg, uint8_t data) {
  wire.beginTransmission(MMC5603_I2C_ADDR);
  wire.write(reg);
  wire.write(data);
  return wire.endTransmission() == 0;
}

bool MMC5603::isMMCdataready() {
  uint8_t status = 0;
  return readRegister8(MMC5603_Status1, status) && (status & 0x40) != 0;
}