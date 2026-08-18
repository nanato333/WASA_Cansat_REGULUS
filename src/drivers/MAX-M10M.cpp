#include "drivers/MaxM10M.h"
#include <stdlib.h>
#include <string.h>

MaxM10M::MaxM10M(HardwareSerial &serial) : serial_(serial) {}
void MaxM10M::begin(uint32_t baud, int8_t rxPin, int8_t txPin) { serial_.begin(baud, SERIAL_8N1, rxPin, txPin); }

void MaxM10M::update() {
    while (serial_.available()) {
        const char c = (char)serial_.read();
        data_.lastReceiveMs = millis();
        if (c == '\n') {
            line_[length_] = '\0';
            parseLine();
            length_ = 0;
        } else if (c != '\r') {
            if (length_ < sizeof(line_) - 1) line_[length_++] = c;
            else length_ = 0;
        }
    }
    data_.valid = data_.lastReceiveMs != 0 && millis() - data_.lastReceiveMs < 2000;
    if (!data_.valid) data_.fix = false;
}

bool MaxM10M::parseLine() {
    if (!checksumValid(line_) || line_[0] != '$') return false;
    char copy[sizeof(line_)]; strncpy(copy, line_, sizeof(copy)); copy[sizeof(copy)-1] = '\0';
    char *asterisk = strchr(copy, '*'); if (asterisk) *asterisk = '\0';
    char *fields[20] = {}; size_t count = 0; char *save = nullptr;
    for (char *token = strtok_r(copy, ",", &save); token && count < 20; token = strtok_r(nullptr, ",", &save)) fields[count++] = token;
    if (count == 0) return false;
    const size_t typeLen = strlen(fields[0]); const char *type = typeLen >= 3 ? fields[0] + typeLen - 3 : fields[0];
    if (strcmp(type, "GGA") == 0 && count >= 10) {
        const int quality = atoi(fields[6]);
        data_.latitude = parseCoordinate(fields[2], fields[3]);
        data_.longitude = parseCoordinate(fields[4], fields[5]);
        data_.satellites = (uint8_t)constrain(atoi(fields[7]), 0, 255);
        data_.altitudeM = (float)atof(fields[9]);
        data_.fix = quality > 0;
        return true;
    }
    if (strcmp(type, "RMC") == 0 && count >= 7) {
        data_.fix = fields[2][0] == 'A';
        if (data_.fix) {
            data_.latitude = parseCoordinate(fields[3], fields[4]);
            data_.longitude = parseCoordinate(fields[5], fields[6]);
        }
        return true;
    }
    return false;
}

bool MaxM10M::checksumValid(const char *line) {
    if (!line || line[0] != '$') return false;
    const char *star = strchr(line, '*'); if (!star || strlen(star + 1) < 2) return false;
    uint8_t checksum = 0; for (const char *p = line + 1; p < star; ++p) checksum ^= (uint8_t)*p;
    return checksum == (uint8_t)strtoul(star + 1, nullptr, 16);
}

double MaxM10M::parseCoordinate(const char *value, const char *hemisphere) {
    if (!value || !*value || !hemisphere || !*hemisphere) return 0.0;
    const double raw = atof(value); const int degrees = (int)(raw / 100.0);
    double result = degrees + (raw - degrees * 100.0) / 60.0;
    if (*hemisphere == 'S' || *hemisphere == 'W') result = -result;
    return result;
}