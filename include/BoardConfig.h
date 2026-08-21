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

    // 現在のmotor_testと同じ正転極性。実機の回転方向が逆なら片側だけ変更する。
    constexpr bool MOTOR_A_FORWARD_IN1_HIGH = true;
    constexpr bool MOTOR_B_FORWARD_IN1_HIGH = true;

    // M1/M2をはんだ付け後、Motor Aが右輪でなければfalseへ変更する。
    constexpr bool MOTOR_A_IS_RIGHT = true;

    // バッテリー監視用ADC。100 kΩ + 100 kΩ分圧の中点を接続する。
    constexpr int VBAT_PIN = 8;

    // 状態表示用LED
    constexpr int LED_PIN = 21;

}