// MSF time-signal emulator for an ESP8266 (LoLin/Wemos D1 Mini).
//
// Fetches the time over NTP, then transmits it as an MSF60 bit stream on a GPIO pin so a
// radio-controlled clock (here a Sony ICL-C50) can sync to it. It runs for a quarter of
// an hour just before 03:00 each night and deep-sleeps the rest of the time.
//
//   Wi-Fi + NTP  ->  align to a minute boundary  ->  timer ISR plays the frame
//                         ^                                  |
//                         +---- loop() builds the next frame +

#include <Arduino.h>
#include "Config.h"
#include "Led.h"
#include "MsfFrame.h"
#include "Net.h"
#include "Ntp.h"
#include "RtcState.h"
#include "Transmitter.h"
#include "UkTime.h"

namespace {

constexpr int32_t kMinLeadMs = 20;  // least time we need between "now" and starting the timer

time_t g_minute = 0;    // UTC of the minute marker that is about to be transmitted
time_t g_runUntil = 0;  // keep transmitting until g_minute reaches this

[[noreturn]] void deepSleepFor(time_t seconds) {
    ESP.deepSleep(static_cast<uint64_t>(seconds) * 1000000ULL, WAKE_RF_DEFAULT);
    for (;;) {
        delay(1);  // deepSleep() takes effect asynchronously
    }
}

time_t capNap(time_t seconds) {
    return seconds > static_cast<time_t>(cfg::kMaxNapSeconds) ? static_cast<time_t>(cfg::kMaxNapSeconds) : seconds;
}

// Woken early by the hourly nap timer? Go straight back to sleep until it is time.
void napIfNotDue(time_t now) {
    const time_t wake = rtc::loadWakeTime();
    if (wake == 0) {
        return;
    }
    const int64_t remaining = static_cast<int64_t>(wake) - static_cast<int64_t>(now);
    if (remaining > cfg::kMinNapSeconds && remaining <= cfg::kMaxWakeAheadSeconds) {
        deepSleepFor(capNap(static_cast<time_t>(remaining)));
    }
}

// Breathe the LED until timer1 must start, then start it so that its first tick lands
// exactly on a minute boundary. Sets g_minute and g_runUntil.
void alignAndStart(const ntp::Fix& fix) {
    const time_t minuteZero = fix.utc - fix.utc % 60;
    const uint32_t sinceMs = static_cast<uint32_t>(fix.utc % 60) * 1000u + fix.ms + (millis() - fix.rxMillis);

    // Time until the next boundary, less one sub-bit because that is how long the first tick takes.
    int32_t wait = 60000 - static_cast<int32_t>(sinceMs % 60000) - static_cast<int32_t>(cfg::kSubbitMs);
    g_minute = minuteZero + 60 * static_cast<time_t>(sinceMs / 60000 + 1);
    if (wait < kMinLeadMs) {
        wait += 60000;  // too close to make it: aim for the following minute
        g_minute += 60;
    }
    g_runUntil = g_minute + static_cast<time_t>(cfg::kRunSeconds);

    const uint32_t startAt = millis() + static_cast<uint32_t>(wait);
    led::pulse(static_cast<uint32_t>(wait) - 10, cfg::kWaitPulseStepMs, false);
    while (static_cast<int32_t>(startAt - millis()) > 0) {
        // spin for the last few milliseconds
    }
    transmitter::start();
}

// At the end of the run, deep sleep until the next wake-up time.
void sleepIfRunFinished() {
    if (!cfg::kSleepEnabled || g_minute < g_runUntil) {
        return;
    }
    const time_t wake = uk::nextLocal(g_minute, cfg::kWakeLocalSecond);
    const time_t until = wake - g_minute;
    if (until < static_cast<time_t>(cfg::kMinNapSeconds)) {
        g_runUntil += static_cast<time_t>(cfg::kMaxNapSeconds);  // wake-up is imminent: just carry on
        return;
    }
    rtc::saveWakeTime(wake);
    deepSleepFor(capNap(until));
}

}  // namespace

void setup() {
    led::begin();
    transmitter::begin();

    net::connect();  // blinks while waiting, blocks until connected
    led::set(true);

    ntp::Fix fix;
    while (!ntp::fetch(fix)) {
    }

    napIfNotDue(fix.utc);
    alignAndStart(fix);  // critical timing: nothing slow may happen after the NTP reading

    led::set(false);
}

void loop() {
    if (!transmitter::takeMinuteDue()) {
        return;
    }

    // 100 ms before the marker for g_minute. The frame sent during that minute announces
    // the next one, and we have the whole marker second to hand it over.
    sleepIfRunFinished();
    transmitter::publish(MsfFrame::encode(g_minute + 60));
    g_minute += 60;

    led::pulse(cfg::kTransmitPulseMs, cfg::kTransmitPulseStepMs, true);
}
