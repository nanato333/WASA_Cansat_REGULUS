#include "communication/EspNowRadio.h"
#include "communication/WcppTelemetry.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_arduino_version.h>

namespace { uint8_t broadcastAddress[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}; }
volatile bool EspNowRadio::actionPending_ = false;
volatile uint8_t EspNowRadio::pendingAction_ = 0;

#if defined(ESP_ARDUINO_VERSION) && ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3,0,0)
static void onReceive(const esp_now_recv_info_t *, const uint8_t *data, int len) { EspNowRadio::receivePacket(data, len); }
#else
static void onReceive(const uint8_t *, const uint8_t *data, int len) { EspNowRadio::receivePacket(data, len); }
#endif

bool EspNowRadio::begin(uint8_t channel) {
    WiFi.mode(WIFI_STA);
    if (esp_wifi_set_promiscuous(true) != ESP_OK) return false;
    const esp_err_t channelResult = esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);
    if (channelResult != ESP_OK || esp_now_init() != ESP_OK) return false;
    esp_now_register_recv_cb(onReceive);
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, broadcastAddress, 6);
    peer.channel = channel;
    peer.encrypt = false;
    return esp_now_is_peer_exist(broadcastAddress) || esp_now_add_peer(&peer) == ESP_OK;
}

bool EspNowRadio::send(const uint8_t *data, size_t length) {
    if (!data || length == 0 || length > 250) { ++sendFailCount_; return false; }
    const bool ok = esp_now_send(broadcastAddress, data, length) == ESP_OK;
    if (ok) ++sentCount_; else ++sendFailCount_;
    return ok;
}

bool EspNowRadio::takeAction(uint8_t &action) {
    if (!actionPending_) return false;
    noInterrupts(); action = pendingAction_; actionPending_ = false; interrupts();
    return true;
}

void EspNowRadio::receivePacket(const uint8_t *data, int length) {
    uint8_t action = 0;
    if (WcppTelemetry::decodeAction(data, (size_t)length, action)) {
        pendingAction_ = action;
        actionPending_ = true;
    }
}