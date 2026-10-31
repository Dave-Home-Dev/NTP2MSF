#include "Transmitter.h"
#include <Arduino.h>
#include "Config.h"

namespace transmitter {
namespace {

constexpr uint32_t kApbHz = 80'000'000;      // timer1 counts APB clock...
constexpr uint32_t kTimerDivider = 16;       // ...divided by 16 (TIM_DIV16)
constexpr uint32_t kTicksPerMs = kApbHz / kTimerDivider / 1000;
constexpr uint32_t kSubbitTicks = cfg::kSubbitMs * kTicksPerMs;  // 500 000, well inside the 23-bit load
constexpr uint32_t kPinMask = 1u << cfg::kMsfPin;

MsfFrame g_frame;                  // read by the ISR, replaced with interrupts masked
unsigned g_second = 0;             // ISR-owned position within the minute
unsigned g_tenth = 0;
volatile bool g_minuteDue = false;

[[gnu::always_inline]] inline void drive(bool carrierOn) {
    if (carrierOn != cfg::kInvertCarrier) {
        GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, kPinMask);  // high
    } else {
        GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, kPinMask);  // low
    }
}

void IRAM_ATTR onTick() {
    drive(g_frame.carrier(g_second, g_tenth));

    if (++g_tenth == 10) {
        g_tenth = 0;
        if (++g_second == 60) {
            g_second = 0;
            g_minuteDue = true;  // the marker starts after the next tick: 100 ms to build a frame
        }
    }
}

}  // namespace

void begin() {
    pinMode(cfg::kMsfPin, OUTPUT);
    drive(false);

    timer1_isr_init();
    timer1_attachInterrupt(onTick);
    timer1_write(kSubbitTicks);
}

void start() {
    g_second = 0;
    g_tenth = 0;
    g_minuteDue = true;  // the very first frame is needed immediately
    timer1_enable(TIM_DIV16, TIM_EDGE, TIM_LOOP);
}

bool takeMinuteDue() {
    if (!g_minuteDue) {
        return false;
    }
    g_minuteDue = false;
    return true;
}

void publish(const MsfFrame& frame) {
    noInterrupts();
    g_frame = frame;
    interrupts();
}

}  // namespace transmitter
