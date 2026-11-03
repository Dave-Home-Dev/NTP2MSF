#pragma once
#include <cstdint>
#include <ctime>

// Where to start the MSF timer so that its first tick lands exactly on a minute boundary.
struct Alignment {
    int32_t waitMs;      // from now until the timer must be started
    time_t firstMinute;  // UTC of the minute marker that first tick lands on
};

// `utc` and `ms` are an NTP reading (whole seconds and 0..999 ms) taken `elapsedMs` ago.
// The timer's first tick fires one sub-bit after it is started, so it is started
// `subbitMs` early. If that would leave less than `minLeadMs` to spare, the following
// minute is used instead.
Alignment alignToMinute(time_t utc, uint32_t ms, uint32_t elapsedMs, uint32_t subbitMs, int32_t minLeadMs);