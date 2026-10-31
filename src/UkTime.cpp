#include "UkTime.h"
#include "Config.h"

namespace uk {
namespace {

constexpr time_t kHour = 3600;
constexpr time_t kDay = 24 * kHour;

// Days since 1970-01-01 for a proleptic Gregorian date (Howard Hinnant's algorithm).
constexpr int64_t daysFromCivil(int64_t y, unsigned m, unsigned d) {
    y -= m <= 2;
    const int64_t era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = static_cast<unsigned>(y - era * 400);
    const unsigned doy = (153 * (m > 2 ? m - 3 : m + 9) + 2) / 5 + d - 1;
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + static_cast<int64_t>(doe) - 719468;
}

// 01:00 UTC on the last Sunday of a 31-day month (March or October).
time_t lastSundayUtc(int year, unsigned month) {
    const int64_t lastDay = daysFromCivil(year, month, 31);
    const int weekday = static_cast<int>((lastDay + 4) % 7);  // 1970-01-01 was a Thursday; 0 = Sunday
    return static_cast<time_t>(lastDay - weekday) * kDay + kHour;
}

}  // namespace

Time fromUtc(time_t utc) {
    struct tm parts;
    gmtime_r(&utc, &parts);
    const int year = parts.tm_year + 1900;
    const time_t start = lastSundayUtc(year, 3);
    const time_t end = lastSundayUtc(year, 10);

    const auto soon = [utc](time_t change) {
        return change > utc && change - utc <= static_cast<time_t>(cfg::kDstWarningSeconds);
    };

    Time t;
    t.bst = utc >= start && utc < end;
    t.changeSoon = soon(start) || soon(end);
    t.local = utc + (t.bst ? kHour : 0);
    return t;
}

time_t toUtc(time_t local) {
    const time_t asBst = local - kHour;
    return fromUtc(asBst).bst ? asBst : local;
}

time_t nextLocal(time_t utc, uint32_t secondOfDay) {
    const time_t local = fromUtc(utc).local;
    time_t target = local - local % kDay + static_cast<time_t>(secondOfDay);
    if (target <= local) {
        target += kDay;
    }
    return toUtc(target);
}

}  // namespace uk
