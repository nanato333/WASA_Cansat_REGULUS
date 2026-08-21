#include <Arduino.h>
#include "BoardConfig.h"
#include "drivers/MaxM10M.h"

HardwareSerial gnssSerial(1);
MaxM10M gnss(gnssSerial);
void setup() {
    Serial.begin(115200); delay(1000);
    gnss.begin(BoardConfig::GNSS_BAUD, BoardConfig::GNSS_RX, BoardConfig::GNSS_TX);
    Serial.println("=== REGULUS MAX-M10M Test ===");
    Serial.printf("UART1 RX=GPIO%d TX=GPIO%d baud=%lu\n", BoardConfig::GNSS_RX, BoardConfig::GNSS_TX, (unsigned long)BoardConfig::GNSS_BAUD);
}
void loop() {
    gnss.update();
    static uint32_t previous = 0; const uint32_t now = millis();
    if (now - previous < 500) return; previous = now;
    const MaxM10M::Data &d = gnss.data();
    Serial.printf("valid=%d fix=%d lat=%.7f lon=%.7f alt=%.1f m sats=%u age=%lu ms chars=%lu ok=%lu failed=%lu\n",
                  d.valid, d.fix, d.latitude, d.longitude, d.altitudeM, d.satellites,
                  d.lastReceiveMs ? (unsigned long)(now - d.lastReceiveMs) : 0UL,
                  (unsigned long)d.charsProcessed, (unsigned long)d.passedChecksum,
                  (unsigned long)d.failedChecksum);
}