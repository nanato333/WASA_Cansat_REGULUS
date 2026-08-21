#include "drivers/MaxM10M.h"

MaxM10M::MaxM10M(HardwareSerial &serial) : serial_(serial) {}

void MaxM10M::begin(uint32_t baud, int8_t rxPin, int8_t txPin) {
    serial_.begin(baud, SERIAL_8N1, rxPin, txPin);
}

void MaxM10M::update() {
    while (serial_.available() > 0) {
        const char value = static_cast<char>(serial_.read());
        data_.lastReceiveMs = millis();
        gps_.encode(value);
    }

    data_.charsProcessed = gps_.charsProcessed();
    data_.passedChecksum = gps_.passedChecksum();
    data_.failedChecksum = gps_.failedChecksum();

    const uint32_t now = millis();
    data_.valid = data_.lastReceiveMs != 0 &&
                  now - data_.lastReceiveMs <= DATA_TIMEOUT_MS;

    const bool freshLocation = gps_.location.isValid() &&
                               gps_.location.age() <= DATA_TIMEOUT_MS;
    data_.fix = data_.valid && freshLocation;

    if (freshLocation) {
        data_.latitude = gps_.location.lat();
        data_.longitude = gps_.location.lng();
    }
    if (gps_.altitude.isValid() &&
        gps_.altitude.age() <= DATA_TIMEOUT_MS) {
        data_.altitudeM = static_cast<float>(gps_.altitude.meters());
    }
    if (gps_.satellites.isValid() &&
        gps_.satellites.age() <= DATA_TIMEOUT_MS) {
        data_.satellites = static_cast<uint8_t>(
            constrain(gps_.satellites.value(), 0UL, 255UL));
    } else {
        data_.satellites = 0;
    }

    if (!data_.valid) {
        data_.fix = false;
        data_.satellites = 0;
    }
}
