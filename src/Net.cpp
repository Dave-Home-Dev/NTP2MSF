#include "Net.h"
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include "Led.h"
#include "RtcState.h"
#include "WiFi_Credentials.h"  // defines WIFI_SSID and WIFI_PASS (see include/)

namespace net {
namespace {

// A static address skips the DHCP exchange, which matters when waking every hour.
const IPAddress kAddress(192, 168, 0, 241);
const IPAddress kGateway(192, 168, 0, 1);
const IPAddress kSubnet(255, 255, 255, 0);
const IPAddress kDns(192, 168, 0, 1);

constexpr uint32_t kFastReconnectMs = 10000;

// Blink at `blinkMs` per toggle until connected; a timeout of 0 means wait forever.
bool waitForConnection(uint32_t timeoutMs, uint32_t blinkMs) {
    const uint32_t start = millis();
    uint32_t lastBlink = start;
    while (WiFi.status() != WL_CONNECTED) {
        if (timeoutMs != 0 && millis() - start >= timeoutMs) {
            return false;
        }
        if (millis() - lastBlink >= blinkMs) {
            lastBlink = millis();
            led::toggle();
        }
        yield();
    }
    return true;
}

}  // namespace

void connect() {
    WiFi.mode(WIFI_STA);
    WiFi.setPhyMode(WIFI_PHY_MODE_11N);  // some access points insist on 802.11n only
    WiFi.setAutoReconnect(true);

    if (rtc::wokeFromDeepSleep()) {
        WiFi.persistent(false);  // reuse stored credentials without rewriting flash
        WiFi.config(kAddress, kGateway, kSubnet, kDns);
        WiFi.begin();
        if (waitForConnection(kFastReconnectMs, 25)) {
            return;
        }
    }

    WiFi.persistent(true);
    WiFi.config(kAddress, kGateway, kSubnet, kDns);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    waitForConnection(0, 100);
}

}  // namespace net
