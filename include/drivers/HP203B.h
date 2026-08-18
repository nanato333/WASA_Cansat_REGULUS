#pragma once
#include <Arduino.h>
#include <Wire.h>

class HP203B {
public:
    static constexpr uint8_t ADDRESS = 0x76;
    struct Data { float pressureHpa; float temperatureC; float altitudeM; };
    explicit HP203B(TwoWire &wire = Wire);
    bool begin();
    bool startConversion();
    bool readConversion(Data &data, float seaLevelHpa = 1013.25f);
    bool read(Data &data, float seaLevelHpa = 1013.25f);
private:
    TwoWire &wire_;
    bool sendCommand(uint8_t command);
};