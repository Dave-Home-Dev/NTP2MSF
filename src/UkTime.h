#pragma once
#include <cstdint>
#include <ctime>

// UK civil time: GMT in winter, BST (UTC+1) from 01:00 UTC on the last Sunday of
// March until 01:00 UTC on the last Sunday of October.
namespace uk {

// Broken-down UTC calendar time. Replaces gmtime_r(), which is POSIX-only and missing
// from some Windows toolchains, so this code builds identically everywhere.
struct Civil {
    int year;     // e.g. 2026
    int month;    // 1..12
    int day;      // 1..31
    int weekday;  // 0 = Sunday
    int hour;
    int minute;
    int second;
};

Civil civil(time_t utc);

struct Time {
    time_t local;     // UTC shifted by the UK offset; civil() of it gives wall-clock fields
    bool bst;         // British Summer Time is in force
    bool changeSoon;  // a clock change is due within an hour (the MSF "summer time warning")
};

Time fromUtc(time_t utc);

// Inverse of fromUtc() for local times that are neither skipped nor repeated.
time_t toUtc(time_t local);

// The first instant strictly after `utc` at which the UK wall clock reads `secondOfDay`.
time_t nextLocal(time_t utc, uint32_t secondOfDay);

}  // namespace uk