#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_arduino_version.h>
#include <esp_system.h>

namespace {
constexpr uint8_t WIFI_CHANNEL = 1;
constexpr size_t MAX_ESPNOW_PAYLOAD = 250;
constexpr uint8_t STATUS_LED_PIN = RECEIVER_LED_PIN;
constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t SERIAL_TIMEOUT_MS = 100;
constexpr uint8_t DIAGNOSTIC_COMPONENT_ID = 0x7F;
constexpr uint32_t DIAGNOSTIC_PERIOD_MS = 1000;
const uint8_t BROADCAST_ADDRESS[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

struct RadioFrame {
    uint8_t length;
    uint8_t data[MAX_ESPNOW_PAYLOAD];
};

QueueHandle_t receiveQueue = nullptr;
uint8_t serialReceiveBuffer[MAX_ESPNOW_PAYLOAD] = {};
uint8_t serialReceiveLength = 0;
uint32_t lastSerialReceiveMs = 0;
uint32_t lastLedOnMs = 0;
uint32_t lastDiagnosticMs = 0;
uint8_t receiverResetReason = 0;
bool ledOn = false;

uint8_t crc8(const uint8_t *data, size_t length) {
    uint8_t crc = 0;
    for (size_t index = 0; index < length; ++index) {
        crc ^= data[index];
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x80)
                ? static_cast<uint8_t>((crc << 1) ^ 0x07)
                : static_cast<uint8_t>(crc << 1);
        }
    }
    return crc;
}

bool isValidWcpp(const uint8_t *data, size_t length) {
    return data != nullptr &&
           length >= 5 &&
           length <= MAX_ESPNOW_PAYLOAD &&
           data[0] == length &&
           crc8(data, length - 1) == data[length - 1];
}

void queueReceivedPacket(const uint8_t *data, int length) {
    if (receiveQueue == nullptr ||
        length < 0 ||
        !isValidWcpp(data, static_cast<size_t>(length)) ||
        (data[1] & 0x80) == 0) {
        // PCへ戻すのは下りテレメトリだけ。自局のアップリンク反射は破棄する。
        return;
    }

    RadioFrame frame{};
    frame.length = static_cast<uint8_t>(length);
    memcpy(frame.data, data, frame.length);
    xQueueSend(receiveQueue, &frame, 0);
}

#if defined(ESP_ARDUINO_VERSION) &&     ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
void onDataReceived(const esp_now_recv_info_t *, const uint8_t *data, int length) {
    queueReceivedPacket(data, length);
}
#else
void onDataReceived(const uint8_t *, const uint8_t *data, int length) {
    queueReceivedPacket(data, length);
}
#endif

bool initializeEspNow() {
    WiFi.mode(WIFI_STA);

    if (esp_wifi_set_promiscuous(true) != ESP_OK) {
        return false;
    }
    const esp_err_t channelResult =
        esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);
    if (channelResult != ESP_OK || esp_now_init() != ESP_OK) {
        return false;
    }

    esp_now_register_recv_cb(onDataReceived);

    esp_now_peer_info_t peer{};
    memcpy(peer.peer_addr, BROADCAST_ADDRESS, sizeof(BROADCAST_ADDRESS));
    peer.channel = WIFI_CHANNEL;
    peer.encrypt = false;
    return esp_now_is_peer_exist(BROADCAST_ADDRESS) ||
           esp_now_add_peer(&peer) == ESP_OK;
}

void sendReceiverDiagnostic(uint32_t now) {
    if (lastDiagnosticMs != 0 &&
        now - lastDiagnosticMs < DIAGNOSTIC_PERIOD_MS) {
        return;
    }
    lastDiagnosticMs = now;

    // RRをsmall uintとして持つ、受信基板専用のWCPP telemetry。
    uint8_t frame[7] = {};
    frame[0] = sizeof(frame);
    frame[1] = 0xFE;  // telemetry bit + diagnostic packet ID
    frame[2] = DIAGNOSTIC_COMPONENT_ID;
    frame[3] = 0x00;  // local unit
    const uint8_t dataType = static_cast<uint8_t>(
        0x20 | (receiverResetReason & 0x1F));
    frame[4] = static_cast<uint8_t>(((dataType & 0x07) << 5) | 18); // R
    frame[5] = static_cast<uint8_t>((((dataType >> 3) & 0x07) << 5) | 18); // R
    frame[6] = crc8(frame, sizeof(frame) - 1);
    Serial.write(frame, sizeof(frame));
}

void indicatePacket(uint32_t now) {
    digitalWrite(STATUS_LED_PIN, HIGH);
    lastLedOnMs = now;
    ledOn = true;
}

void forwardRadioToUsb(uint32_t now) {
    RadioFrame frame{};
    while (receiveQueue != nullptr &&
           xQueueReceive(receiveQueue, &frame, 0) == pdTRUE) {
        // 地上局バックエンドへはログ文字列を混ぜず、WCPPバイナリだけを送る。
        Serial.write(frame.data, frame.length);
        indicatePacket(now);
    }
}

void forwardUsbToRadio(uint32_t now) {
    if (serialReceiveLength > 0 &&
        now - lastSerialReceiveMs > SERIAL_TIMEOUT_MS) {
        serialReceiveLength = 0;
    }

    while (Serial.available() > 0) {
        lastSerialReceiveMs = now;
        const uint8_t value = static_cast<uint8_t>(Serial.read());

        if (serialReceiveLength >= sizeof(serialReceiveBuffer)) {
            serialReceiveLength = 0;
        }
        serialReceiveBuffer[serialReceiveLength++] = value;

        const uint8_t expectedLength = serialReceiveBuffer[0];
        if (expectedLength < 5 || expectedLength > MAX_ESPNOW_PAYLOAD) {
            serialReceiveLength = 0;
            continue;
        }
        if (serialReceiveLength == expectedLength) {
            if (isValidWcpp(serialReceiveBuffer, expectedLength)) {
                esp_now_send(
                    BROADCAST_ADDRESS,
                    serialReceiveBuffer,
                    expectedLength);
            }
            serialReceiveLength = 0;
        }
    }
}
}

void setup() {
    Serial.begin(SERIAL_BAUD);
    receiverResetReason = static_cast<uint8_t>(esp_reset_reason());
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);

    receiveQueue = xQueueCreate(8, sizeof(RadioFrame));
    if (receiveQueue == nullptr || !initializeEspNow()) {
        // 初期化失敗時は点灯を維持する。USBへ文字列は送らない。
        digitalWrite(STATUS_LED_PIN, HIGH);
    }
}

void loop() {
    const uint32_t now = millis();
    sendReceiverDiagnostic(now);
    forwardRadioToUsb(now);
    forwardUsbToRadio(now);

    if (ledOn && now - lastLedOnMs > 50) {
        digitalWrite(STATUS_LED_PIN, LOW);
        ledOn = false;
    }
    delay(1);
}