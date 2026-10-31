#pragma once
#include <cstdint>

namespace cfg {

// --- Hardware -------------------------------------------------------------
constexpr uint8_t kMsfPin = 4;           // D2 on the D1 Mini: drives the clock's receiver input
constexpr uint8_t kLedPin = 2;           // D4 on the D1 Mini: on-board LED, active low
constexpr bool kInvertCarrier = false;   // set if the clock's input expects the opposite polarity

// --- Behaviour ------------------------------------------------------------
constexpr bool kSleepEnabled = true;                      // false = transmit forever (debugging)
constexpr uint32_t kRunSeconds = 15 * 60;                 // transmit for this long after each wake-up
constexpr uint32_t kWakeLocalSecond = 2 * 3600 + 59 * 60; // wake at 02:59 local, just before 03:00
constexpr uint32_t kMaxNapSeconds = 60 * 60;              // deep sleep in chunks of at most an hour
constexpr uint32_t kMinNapSeconds = 60;                   // not worth sleeping for less than this
constexpr uint32_t kMaxWakeAheadSeconds = 24 * 3600;      // ignore stored wake times further out

// --- MSF ------------------------------------------------------------------
constexpr uint32_t kSubbitMs = 100;              // an MSF second is ten 100 ms sub-bits
constexpr uint32_t kDstWarningSeconds = 3660;    // summer-time warning is raised 61 minutes ahead

// --- LED ------------------------------------------------------------------
constexpr uint32_t kTransmitPulseMs = 55000;     // breathing LED while transmitting
constexpr uint32_t kTransmitPulseStepMs = 75;    // ms per brightness step (3 s per breath)
constexpr uint32_t kWaitPulseStepMs = 25;        // faster breathing while waiting for the minute

}  // namespace cfg
