#include "communication/EspNowRadio.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_arduino_version.h>
namespace{uint8_t broadcastAddress[6]={255,255,255,255,255,255};}
volatile bool EspNowRadio::actionPending_=false;volatile uint8_t EspNowRadio::pendingAction_=0;WcppTelemetry::Command EspNowRadio::pendingCommand_;
#if defined(ESP_ARDUINO_VERSION) && ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3,0,0)
static void onReceive(const esp_now_recv_info_t*,const uint8_t*d,int n){EspNowRadio::receivePacket(d,n);}
#else
static void onReceive(const uint8_t*,const uint8_t*d,int n){EspNowRadio::receivePacket(d,n);}
#endif
bool EspNowRadio::begin(uint8_t ch){if(initialized_)return true;WiFi.mode(WIFI_STA);if(esp_wifi_set_promiscuous(true)!=ESP_OK)return false;auto cr=esp_wifi_set_channel(ch,WIFI_SECOND_CHAN_NONE);esp_wifi_set_promiscuous(false);if(cr!=ESP_OK)return false;auto ir=esp_now_init();if(ir!=ESP_OK&&ir!=ESP_ERR_ESPNOW_EXIST)return false;esp_now_register_recv_cb(onReceive);esp_now_peer_info_t p={};memcpy(p.peer_addr,broadcastAddress,6);p.channel=ch;p.encrypt=false;initialized_=esp_now_is_peer_exist(broadcastAddress)||esp_now_add_peer(&p)==ESP_OK;return initialized_;}
bool EspNowRadio::send(const uint8_t*d,size_t n){if(!d||!n||n>250){++sendFailCount_;return false;}bool ok=esp_now_send(broadcastAddress,d,n)==ESP_OK;if(ok)++sentCount_;else++sendFailCount_;return ok;}
bool EspNowRadio::takeCommand(WcppTelemetry::Command&c){if(!actionPending_)return false;noInterrupts();c=pendingCommand_;actionPending_=false;interrupts();return true;}
bool EspNowRadio::takeAction(uint8_t&a){WcppTelemetry::Command c;if(!takeCommand(c))return false;a=c.action;return true;}
void EspNowRadio::receivePacket(const uint8_t*d,int n){WcppTelemetry::Command c;if(WcppTelemetry::decodeCommand(d,(size_t)n,c)){pendingCommand_=c;pendingAction_=c.action;actionPending_=true;}}
