#include "Led.h"
#include <Arduino.h>
#include "Config.h"

namespace led {
namespace {

constexpr int kMaxLevel = 20;

// Perceptual brightness curve (gamma 2) mapped onto a 0..255 duty cycle.
constexpr uint8_t duty(int level) {
    return static_cast<uint8_t>(255 * level * level / (kMaxLevel * kMaxLevel));
}

bool g_lit = false;

}  // namespace

void begin() {
    pinMode(cfg::kLedPin, OUTPUT);
    set(false);
}

void set(bool lit) {
    g_lit = lit;
    digitalWrite(cfg::kLedPin, lit ? LOW : HIGH);
}

void toggle() {
    set(!g_lit);
}

void pulse(uint32_t durationMs, uint32_t stepMs, bool dim) {
    const uint32_t end = millis() + durationMs;
    uint32_t nextStep = millis() + stepMs;
    int level = 0;
    int direction = 1;
    uint16_t error = 0;  // Bresenham accumulator: spreads `d` lit ticks per 255 evenly

    while (static_cast<int32_t>(end - millis()) > 0) {
        uint8_t d = duty(level);
        if (dim) {
            d /= 2;
        }
        error += d;
        const bool lit = error >= 255;
        if (lit) {
            error -= 255;
        }
        set(lit);
        delayMicroseconds(10);  // 100 kHz tick, so about 390 Hz PWM

        if (static_cast<int32_t>(millis() - nextStep) >= 0) {
            nextStep += stepMs;
            if (level + direction < 0 || level + direction > kMaxLevel) {
                direction = -direction;
            }
            level += direction;
            yield();
        }
    }
    set(false);
}

}  // namespace led
