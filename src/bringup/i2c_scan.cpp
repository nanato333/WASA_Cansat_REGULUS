#include <Arduino.h>
#include <Wire.h>
#include "BoardConfig.h"

void scanI2C();

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("=== REGULUS I2C Scanner ===");

    Wire.begin(
        BoardConfig::I2C_SDA,
        BoardConfig::I2C_SCL,
        BoardConfig::I2C_FREQ);

    Serial.printf("SDA: GPIO%d\n", BoardConfig::I2C_SDA);
    Serial.printf("SCL: GPIO%d\n", BoardConfig::I2C_SCL);
}

void loop()
{
    scanI2C();
    delay(2000);
}

void scanI2C()
{
    uint8_t count = 0;

    Serial.println("Scanning...");

    for (uint8_t address = 0x08; address <= 0x77; address++)
    {
        Wire.beginTransmission(address);
        uint8_t error = Wire.endTransmission();

        if (error == 0)
        {
            Serial.printf("Found: 0x%02X\n", address);
            count++;
        }
    }

    Serial.printf("Total: %u device(s)\n\n", count);
}