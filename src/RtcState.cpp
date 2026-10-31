#include "RtcState.h"
#include <Arduino.h>
#include <user_interface.h>  // system_rtc_mem_read/write, system_get_rst_info
#include <cstddef>
#include <cstdint>

namespace rtc {
namespace {

constexpr uint8_t kFirstUserBlock = 64;  // RTC memory blocks below this belong to the SDK
constexpr uint32_t kMagic = 0x4D534632;  // "MSF2": bump when the layout changes

struct Record {
    uint32_t magic;
    uint32_t wakeUtc;
    uint32_t crc;
};
static_assert(sizeof(Record) % 4 == 0, "RTC memory is read and written in 4-byte blocks");

uint32_t crc32(const void* data, size_t length) {
    const uint8_t* p = static_cast<const uint8_t*>(data);
    uint32_t crc = 0xFFFFFFFF;
    while (length--) {
        crc ^= *p++;
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
        }
    }
    return ~crc;
}

uint32_t checksum(const Record& r) {
    return crc32(&r, offsetof(Record, crc));
}

}  // namespace

bool wokeFromDeepSleep() {
    return system_get_rst_info()->reason == REASON_DEEP_SLEEP_AWAKE;
}

time_t loadWakeTime() {
    if (!wokeFromDeepSleep()) {
        return 0;
    }
    Record r;
    if (!system_rtc_mem_read(kFirstUserBlock, &r, sizeof(r)) || r.magic != kMagic || r.crc != checksum(r)) {
        return 0;
    }
    return static_cast<time_t>(r.wakeUtc);
}

void saveWakeTime(time_t utc) {
    Record r = {kMagic, static_cast<uint32_t>(utc), 0};
    r.crc = checksum(r);
    system_rtc_mem_write(kFirstUserBlock, &r, sizeof(r));
}

}  // namespace rtc
