#pragma once
#include <Arduino.h>

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
    };
    explicit MaxM10M(HardwareSerial &serial);
    void begin(uint32_t baud, int8_t rxPin, int8_t txPin);
    void update();
    const Data &data() const { return data_; }
private:
    HardwareSerial &serial_;
    Data data_;
    char line_[128] = {};
    size_t length_ = 0;
    bool parseLine();
    static bool checksumValid(const char *line);
    static double parseCoordinate(const char *value, const char *hemisphere);
};