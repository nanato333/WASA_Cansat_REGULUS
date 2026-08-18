#pragma once

#include <Arduino.h>

namespace BoardConfig
{
    // ========================================
    // I2C
    // ========================================
    constexpr int I2C_SDA = 9;
    constexpr int I2C_SCL = 10;

    constexpr uint32_t I2C_FREQ = 100000;

    // ========================================
    // GNSS UART (MAX-M10M)
    // ========================================
    constexpr int GNSS_TX = 17;
    constexpr int GNSS_RX = 18;

    // TODO: MAX-M10Mの設定に合わせて確認
    constexpr uint32_t GNSS_BAUD = 9600;

    // TB6612FNG Motor Driver
    constexpr int MOTOR_PWMA = 4;
    constexpr int MOTOR_PWMB = 16;

    constexpr int MOTOR_AIN1 = 6;
    constexpr int MOTOR_AIN2 = 5;

    constexpr int MOTOR_BIN1 = 7;
    constexpr int MOTOR_BIN2 = 15;
}