#include <Arduino.h>
#include <Wire.h>
#include "BoardConfig.h"
#include "drivers/HP203B.h"

HP203B barometer(Wire);
void setup() {
    Serial.begin(115200); delay(1000);
    Wire.begin(BoardConfig::I2C_SDA, BoardConfig::I2C_SCL, BoardConfig::I2C_FREQ);
    Serial.println("=== REGULUS HP203B Test ===");
    if (!barometer.begin()) Serial.println("HP203B initialization failed");
    else Serial.println("HP203B initialized at 0x76");
}
void loop() {
    static uint32_t previous = 0; const uint32_t now = millis();
    if (now - previous < 500) return; previous = now;
    HP203B::Data d;
    if (!barometer.read(d)) { Serial.println("HP203B read failed"); return; }
    Serial.printf("pressure=%.2f hPa temperature=%.2f C altitude=%.2f m\n", d.pressureHpa,d.temperatureC,d.altitudeM);
}