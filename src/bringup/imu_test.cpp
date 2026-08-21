#include <Arduino.h>
#include <Wire.h>
#include "BoardConfig.h"
#include "drivers/QMI8658.h"

QMI8658 imu(Wire, QMI8658::ADDRESS_LOW); //  ADRESS_LOWに変更して機能しました
void setup()
{
    Serial.begin(115200);
    delay(1000);
    Wire.begin(BoardConfig::I2C_SDA, BoardConfig::I2C_SCL, BoardConfig::I2C_FREQ);
    Serial.println("=== REGULUS QMI8658 Test ===");
    if (!imu.begin())
        Serial.println("QMI8658 initialization failed (try address 0x6A if SA0 is low)");
    else
        Serial.printf("QMI8658 initialized at 0x%02X\n", imu.address());
}
void loop()
{
    static uint32_t previous = 0;
    const uint32_t now = millis();
    if (now - previous < 100)
        return;
    previous = now;
    QMI8658::Data d;
    if (!imu.read(d))
    {
        Serial.println("QMI8658 read failed");
        return;
    }
    Serial.printf("A[m/s2] %8.3f %8.3f %8.3f  G[dps] %8.3f %8.3f %8.3f\n", d.ax, d.ay, d.az, d.gx, d.gy, d.gz);
}