#pragma once
#include <Arduino.h>
#include <Wire.h>

class QMI8658 {
public:
    static constexpr uint8_t ADDRESS_LOW = 0x6A;
    static constexpr uint8_t ADDRESS_HIGH = 0x6B;
    struct Data { float ax, ay, az; float gx, gy, gz; };
    explicit QMI8658(TwoWire &wire = Wire, uint8_t address = ADDRESS_HIGH);
    bool begin();
    bool read(Data &data);
    uint8_t address() const { return address_; }
private:
    TwoWire &wire_;
    uint8_t address_;
    bool readRegister(uint8_t reg, uint8_t &value);
    bool writeRegister(uint8_t reg, uint8_t value);
};