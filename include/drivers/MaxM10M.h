#pragma once
#include <Arduino.h>
#include <TinyGPS++.h>

class MaxM10M {
public:
    struct Data {
        double latitude = 0.0;
        double longitude = 0.0;
        float altitudeM = 0.0f;
        uint8_t satellites = 0;
        bool fix = false;
        bool valid = false;
        uint32_t lastReceiveMs = 0;
        uint32_t charsProcessed = 0;
        uint32_t passedChecksum = 0;
        uint32_t failedChecksum = 0;
    };
    explicit MaxM10M(HardwareSerial &serial);
    void begin(uint32_t baud, int8_t rxPin, int8_t txPin);
    void update();
    const Data &data() const { return data_; }
private:
    static constexpr uint32_t DATA_TIMEOUT_MS = 2000;
    HardwareSerial &serial_;
    TinyGPSPlus gps_;
    Data data_;
};
