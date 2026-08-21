#include "MotorController.h"

#include "BoardConfig.h"

namespace {
void writeDirection(int in1, int in2, bool in1High) {
    digitalWrite(in1, in1High ? HIGH : LOW);
    digitalWrite(in2, in1High ? LOW : HIGH);
}
}

void MotorController::begin() {
    pinMode(BoardConfig::MOTOR_PWMA, OUTPUT);
    pinMode(BoardConfig::MOTOR_AIN1, OUTPUT);
    pinMode(BoardConfig::MOTOR_AIN2, OUTPUT);
    pinMode(BoardConfig::MOTOR_PWMB, OUTPUT);
    pinMode(BoardConfig::MOTOR_BIN1, OUTPUT);
    pinMode(BoardConfig::MOTOR_BIN2, OUTPUT);
    stop();
}

void MotorController::driveMotorA(bool forward) {
    const bool in1High = forward
        ? BoardConfig::MOTOR_A_FORWARD_IN1_HIGH
        : !BoardConfig::MOTOR_A_FORWARD_IN1_HIGH;
    writeDirection(BoardConfig::MOTOR_AIN1, BoardConfig::MOTOR_AIN2, in1High);
    digitalWrite(BoardConfig::MOTOR_PWMA, HIGH);
}

void MotorController::driveMotorB(bool forward) {
    const bool in1High = forward
        ? BoardConfig::MOTOR_B_FORWARD_IN1_HIGH
        : !BoardConfig::MOTOR_B_FORWARD_IN1_HIGH;
    writeDirection(BoardConfig::MOTOR_BIN1, BoardConfig::MOTOR_BIN2, in1High);
    digitalWrite(BoardConfig::MOTOR_PWMB, HIGH);
}

void MotorController::stopMotorA() {
    digitalWrite(BoardConfig::MOTOR_PWMA, LOW);
    digitalWrite(BoardConfig::MOTOR_AIN1, LOW);
    digitalWrite(BoardConfig::MOTOR_AIN2, LOW);
}

void MotorController::stopMotorB() {
    digitalWrite(BoardConfig::MOTOR_PWMB, LOW);
    digitalWrite(BoardConfig::MOTOR_BIN1, LOW);
    digitalWrite(BoardConfig::MOTOR_BIN2, LOW);
}

void MotorController::stop() {
    stopMotorA();
    stopMotorB();
    state_ = MotorDriveState::STOPPED;
}

void MotorController::forward() {
    driveMotorA(true);
    driveMotorB(true);
    state_ = MotorDriveState::FORWARD;
}

void MotorController::turnLeft() {
    if (BoardConfig::MOTOR_A_IS_RIGHT) {
        driveMotorA(true);
        stopMotorB();
    } else {
        stopMotorA();
        driveMotorB(true);
    }
    state_ = MotorDriveState::TURN_LEFT;
}

void MotorController::turnRight() {
    if (BoardConfig::MOTOR_A_IS_RIGHT) {
        stopMotorA();
        driveMotorB(true);
    } else {
        driveMotorA(true);
        stopMotorB();
    }
    state_ = MotorDriveState::TURN_RIGHT;
}
