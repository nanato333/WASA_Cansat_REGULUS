#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "BoardConfig.h"
#include "drivers/MMC5603.h"

MMC5603 magnetometer(Wire);

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Wire.begin(
        BoardConfig::I2C_SDA,
        BoardConfig::I2C_SCL,
        BoardConfig::I2C_FREQ);

    Serial.println("=== REGULUS MMC5603 Test ===");
    Serial.printf("I2C: SDA=GPIO%d, SCL=GPIO%d, address=0x%02X\n",
                  BoardConfig::I2C_SDA,
                  BoardConfig::I2C_SCL,
                  MMC5603_I2C_ADDR);

    if (!magnetometer.begin())
    {
        Serial.println("MMC5603 initialization failed");
        return;
    }

    Serial.println("MMC5603 initialized");
}

void loop()
{
    static uint32_t lastPrintMs = 0;
    const uint32_t now = millis();
    if (now - lastPrintMs < 100)
    {
        return;
    }
    lastPrintMs = now;

    MMC5603::MagData data;
    if (!magnetometer.read(data))
    {
        Serial.println("MMC5603 read failed or data not ready");
        return;
    }

    float heading = atan2f(data.magY, data.magX) * 180.0f / PI;
    if (heading < 0.0f)
    {
        heading += 360.0f;
    }

    Serial.printf("X=%9.3f uT, Y=%9.3f uT, Z=%9.3f uT, heading=%7.2f deg\n",
                  data.magX,
                  data.magY,
                  data.magZ,
                  heading);
}