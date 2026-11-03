#include "Align.h"

Alignment alignToMinute(time_t utc, uint32_t ms, uint32_t elapsedMs, uint32_t subbitMs, int32_t minLeadMs) {
    const uint32_t sinceMs = static_cast<uint32_t>(utc % 60) * 1000u + ms + elapsedMs;  // since the minute began

    Alignment a;
    a.waitMs = 60000 - static_cast<int32_t>(sinceMs % 60000) - static_cast<int32_t>(subbitMs);
    a.firstMinute = utc - utc % 60 + 60 * static_cast<time_t>(sinceMs / 60000 + 1);

    if (a.waitMs < minLeadMs) {
        a.waitMs += 60000;  // too close to make it: aim for the following minute
        a.firstMinute += 60;
    }
    return a;
}