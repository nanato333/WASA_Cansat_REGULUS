#include <Arduino.h>
#include "BoardConfig.h"

void stopMotors();

void setup()
{
    Serial.begin(115200);

    pinMode(BoardConfig::MOTOR_PWMA, OUTPUT);
    pinMode(BoardConfig::MOTOR_AIN1, OUTPUT);
    pinMode(BoardConfig::MOTOR_AIN2, OUTPUT);

    pinMode(BoardConfig::MOTOR_PWMB, OUTPUT);
    pinMode(BoardConfig::MOTOR_BIN1, OUTPUT);
    pinMode(BoardConfig::MOTOR_BIN2, OUTPUT);

    // PWMは使用せず常時有効
    digitalWrite(BoardConfig::MOTOR_PWMA, HIGH);
    digitalWrite(BoardConfig::MOTOR_PWMB, HIGH);

    // 起動直後にモータが回らないようにする
    stopMotors();

    Serial.println("=== REGULUS Motor Test ===");
}

void loop()
{
    // Motor A 正転
    Serial.println("Motor A: Forward");

    digitalWrite(BoardConfig::MOTOR_AIN1, HIGH);
    digitalWrite(BoardConfig::MOTOR_AIN2, LOW);

    delay(2000);

    stopMotors();
    delay(1000);

    // Motor A 逆転
    Serial.println("Motor A: Reverse");

    digitalWrite(BoardConfig::MOTOR_AIN1, LOW);
    digitalWrite(BoardConfig::MOTOR_AIN2, HIGH);

    delay(2000);

    stopMotors();
    delay(1000);

    // Motor B 正転
    Serial.println("Motor B: Forward");

    digitalWrite(BoardConfig::MOTOR_BIN1, HIGH);
    digitalWrite(BoardConfig::MOTOR_BIN2, LOW);

    delay(2000);

    stopMotors();
    delay(1000);

    // Motor B 逆転
    Serial.println("Motor B: Reverse");

    digitalWrite(BoardConfig::MOTOR_BIN1, LOW);
    digitalWrite(BoardConfig::MOTOR_BIN2, HIGH);

    delay(2000);

    stopMotors();
    delay(3000);
}

void stopMotors()
{
    digitalWrite(BoardConfig::MOTOR_AIN1, LOW);
    digitalWrite(BoardConfig::MOTOR_AIN2, LOW);

    digitalWrite(BoardConfig::MOTOR_BIN1, LOW);
    digitalWrite(BoardConfig::MOTOR_BIN2, LOW);
}