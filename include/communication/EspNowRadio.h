#pragma once
#include <Arduino.h>

class EspNowRadio {
public:
    bool begin(uint8_t channel = 1);
    bool send(const uint8_t *data, size_t length);
    bool takeAction(uint8_t &action);
    uint32_t sentCount() const { return sentCount_; }
    uint32_t sendFailCount() const { return sendFailCount_; }
    static void receivePacket(const uint8_t *data, int length);
private:
    static volatile bool actionPending_;
    static volatile uint8_t pendingAction_;
    bool initialized_ = false;
    uint32_t sentCount_ = 0;
    uint32_t sendFailCount_ = 0;
};