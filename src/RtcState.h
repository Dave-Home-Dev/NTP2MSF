#pragma once
#include <ctime>

// The one thing that must survive deep sleep: when to wake up for real.
// It lives in the ESP8266's RTC memory, protected by a magic number and a CRC.
namespace rtc {

bool wokeFromDeepSleep();

// UTC time stored by saveWakeTime(), or 0 if none (cold boot or corrupt data).
time_t loadWakeTime();

void saveWakeTime(time_t utc);

}  // namespace rtc
