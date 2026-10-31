#pragma once
#include <cstdint>
#include <ctime>

namespace ntp {

// A time reading: at local millis() == rxMillis the UTC time was utc + ms/1000.
struct Fix {
    time_t utc;         // whole seconds
    uint32_t ms;        // 0..999
    uint32_t rxMillis;  // value of millis() the reading refers to
};

// Query the UK NTP pool, retrying for up to a couple of minutes. Half the measured round
// trip is added to compensate for network delay.
bool fetch(Fix& fix);

}  // namespace ntp
