#pragma once

#include <Arduino.h>

enum class MotorDriveState : uint8_t {
    STOPPED = 0,
    FORWARD = 1,
    TURN_LEFT = 2,
    TURN_RIGHT = 3
};

// TB6612FNGの出力をFreeRTOS版とbaremetal版で共通化する。
class MotorController {
public:
    void begin();
    void stop();
    void forward();
    void turnLeft();
    void turnRight();
    MotorDriveState state() const { return state_; }

private:
    void driveMotorA(bool forward);
    void driveMotorB(bool forward);
    void stopMotorA();
    void stopMotorB();
    MotorDriveState state_ = MotorDriveState::STOPPED;
};
