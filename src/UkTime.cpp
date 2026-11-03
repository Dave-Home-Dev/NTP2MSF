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

// Inverse of daysFromCivil(): fills year, month and day for a day count since 1970-01-01.
void civilFromDays(int64_t z, int& year, int& month, int& day) {
    z += 719468;
    const int64_t era = (z >= 0 ? z : z - 146096) / 146097;
    const unsigned doe = static_cast<unsigned>(z - era * 146097);
    const unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    const unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    const unsigned mp = (5 * doy + 2) / 153;
    day = static_cast<int>(doy - (153 * mp + 2) / 5 + 1);
    month = static_cast<int>(mp < 10 ? mp + 3 : mp - 9);
    year = static_cast<int>(yoe + era * 400) + (month <= 2 ? 1 : 0);
}

// 01:00 UTC on the last Sunday of a 31-day month (March or October).
time_t lastSundayUtc(int year, unsigned month) {
    const int64_t lastDay = daysFromCivil(year, month, 31);
    const int weekday = static_cast<int>((lastDay + 4) % 7);  // 1970-01-01 was a Thursday; 0 = Sunday
    return static_cast<time_t>(lastDay - weekday) * kDay + kHour;
}

}  // namespace

Civil civil(time_t utc) {
    int64_t days = static_cast<int64_t>(utc) / 86400;
    int64_t secs = static_cast<int64_t>(utc) % 86400;
    if (secs < 0) {  // floor division for instants before 1970
        secs += 86400;
        --days;
    }

    Civil c;
    civilFromDays(days, c.year, c.month, c.day);
    c.weekday = static_cast<int>(((days % 7) + 7 + 4) % 7);  // 1970-01-01 was a Thursday
    c.hour = static_cast<int>(secs / 3600);
    c.minute = static_cast<int>(secs % 3600 / 60);
    c.second = static_cast<int>(secs % 60);
    return c;
}

Time fromUtc(time_t utc) {
    const int year = civil(utc).year;
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