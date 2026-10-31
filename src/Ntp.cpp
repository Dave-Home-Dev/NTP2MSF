#include "Ntp.h"
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <cstring>

namespace ntp {
namespace {

constexpr char kPool[] = "uk.pool.ntp.org";
constexpr uint16_t kLocalPort = 2390;
constexpr size_t kPacketSize = 48;
constexpr uint32_t kNtpToUnixSeconds = 2208988800UL;  // 1900-01-01 to 1970-01-01
constexpr uint32_t kAttempts = 10;
constexpr uint32_t kReplyTimeoutMs = 10000;

WiFiUDP udp;

uint32_t readBigEndian32(const uint8_t* p) {
    return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | uint32_t(p[3]);
}

void sendRequest(const IPAddress& server) {
    uint8_t packet[kPacketSize] = {};
    packet[0] = 0b11100011;  // leap indicator 3, version 4, mode 3 (client)
    packet[2] = 6;           // poll interval
    packet[3] = 0xEC;        // clock precision
    memcpy(packet + 12, "1N14", 4);

    udp.beginPacket(server, 123);
    udp.write(packet, kPacketSize);
    udp.endPacket();
}

// One request/response exchange with a freshly resolved pool member.
bool exchange(Fix& fix) {
    IPAddress server;
    if (!WiFi.hostByName(kPool, server)) {
        return false;
    }

    sendRequest(server);
    const uint32_t sent = millis();

    while (millis() - sent < kReplyTimeoutMs) {
        if (udp.parsePacket() >= static_cast<int>(kPacketSize)) {
            const uint32_t received = millis();
            uint8_t packet[kPacketSize];
            udp.read(packet, kPacketSize);

            const uint32_t seconds = readBigEndian32(packet + 40);  // transmit timestamp
            const uint32_t fraction = readBigEndian32(packet + 44);
            if (packet[1] == 0 || seconds < kNtpToUnixSeconds) {
                return false;  // kiss-o'-death or unsynchronised server
            }

            // The server stamped the reply about half a round trip before it reached us.
            const uint32_t ms = static_cast<uint32_t>((uint64_t(fraction) * 1000) >> 32) + (received - sent) / 2;
            fix.utc = static_cast<time_t>(seconds - kNtpToUnixSeconds + ms / 1000);
            fix.ms = ms % 1000;
            fix.rxMillis = received;
            return true;
        }
        delay(1);
    }
    return false;
}

}  // namespace

bool fetch(Fix& fix) {
    udp.begin(kLocalPort);
    bool ok = false;
    for (uint32_t attempt = 0; attempt < kAttempts && !ok; ++attempt) {
        ok = exchange(fix);
    }
    udp.stop();
    return ok;
}

}  // namespace ntp
