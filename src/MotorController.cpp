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
#ifdef ENABLE_RC
    const bool invert = BoardConfig::MOTOR_A_IS_RIGHT
        ? BoardConfig::RC_RIGHT_MOTOR_INVERT : BoardConfig::RC_LEFT_MOTOR_INVERT;
    if (invert) forward = !forward;
#endif
    const bool in1High = forward
        ? BoardConfig::MOTOR_A_FORWARD_IN1_HIGH
        : !BoardConfig::MOTOR_A_FORWARD_IN1_HIGH;
    writeDirection(BoardConfig::MOTOR_AIN1, BoardConfig::MOTOR_AIN2, in1High);
    digitalWrite(BoardConfig::MOTOR_PWMA, HIGH);
}

void MotorController::driveMotorB(bool forward) {
#ifdef ENABLE_RC
    const bool invert = BoardConfig::MOTOR_A_IS_RIGHT
        ? BoardConfig::RC_LEFT_MOTOR_INVERT : BoardConfig::RC_RIGHT_MOTOR_INVERT;
    if (invert) forward = !forward;
#endif
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
#ifdef ENABLE_RC
    driveRc(0, 0);
#else
    stopMotorA();
    stopMotorB();
    state_ = MotorDriveState::STOPPED;
#endif
}

#ifdef ENABLE_RC
void MotorController::driveRc(int8_t left, int8_t right) {
    const auto write = [](int8_t value, int8_t &previous, int pwm, int in1, int in2, bool forwardHigh) {
        if (value == previous) return;
        if (value == 0 || previous == 0 || (value > 0) != (previous > 0)) {
            analogWrite(pwm, 0);
            if (value == 0) {
                digitalWrite(in1, LOW);
                digitalWrite(in2, LOW);
            } else {
                writeDirection(in1, in2, value > 0 ? forwardHigh : !forwardHigh);
            }
        }
        analogWrite(pwm, abs(value) * 17);
        previous = value;
    };
    constexpr bool aRight = BoardConfig::MOTOR_A_IS_RIGHT;
    constexpr bool aInvert = aRight ? BoardConfig::RC_RIGHT_MOTOR_INVERT : BoardConfig::RC_LEFT_MOTOR_INVERT;
    constexpr bool bInvert = aRight ? BoardConfig::RC_LEFT_MOTOR_INVERT : BoardConfig::RC_RIGHT_MOTOR_INVERT;
    write(aRight ? right : left, rcA_, BoardConfig::MOTOR_PWMA, BoardConfig::MOTOR_AIN1, BoardConfig::MOTOR_AIN2, BoardConfig::MOTOR_A_FORWARD_IN1_HIGH != aInvert);
    write(aRight ? left : right, rcB_, BoardConfig::MOTOR_PWMB, BoardConfig::MOTOR_BIN1, BoardConfig::MOTOR_BIN2, BoardConfig::MOTOR_B_FORWARD_IN1_HIGH != bInvert);
    state_ = left && right ? MotorDriveState::FORWARD : left ? MotorDriveState::TURN_RIGHT : right ? MotorDriveState::TURN_LEFT : MotorDriveState::STOPPED;
}
#endif

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
