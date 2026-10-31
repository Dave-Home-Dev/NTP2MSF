#pragma once
#include <cstdint>
#include <ctime>

// UK civil time: GMT in winter, BST (UTC+1) from 01:00 UTC on the last Sunday of
// March until 01:00 UTC on the last Sunday of October.
namespace uk {

struct Time {
    time_t local;     // UTC shifted by the UK offset; gmtime() of it gives wall-clock fields
    bool bst;         // British Summer Time is in force
    bool changeSoon;  // a clock change is due within an hour (the MSF "summer time warning")
};

Time fromUtc(time_t utc);

// Inverse of fromUtc() for local times that are neither skipped nor repeated.
time_t toUtc(time_t local);

// The first instant strictly after `utc` at which the UK wall clock reads `secondOfDay`.
time_t nextLocal(time_t utc, uint32_t secondOfDay);

}  // namespace uk
