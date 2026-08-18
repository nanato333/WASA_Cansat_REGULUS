#include <Arduino.h>
#include "BoardConfig.h"

// 解析無しでGNSSからシリアルモニタに文字列が送られてくる つまり生きてることが確認できる
HardwareSerial GNSS(1);

void setup()
{
    Serial.begin(115200);

    GNSS.begin(
        BoardConfig::GNSS_BAUD,
        SERIAL_8N1,
        BoardConfig::GNSS_RX,
        BoardConfig::GNSS_TX);

    Serial.println("=== REGULUS GNSS Test ===");
}

void loop()
{
    while (GNSS.available())
    {
        Serial.write(GNSS.read());
    }
}