#pragma once
#include <cstdint>

// The D1 Mini's on-board LED (active low).
namespace led {

void begin();
void set(bool lit);
void toggle();

// Breathe the LED for `durationMs`, taking `stepMs` per brightness step (21 steps up,
// 21 down). Blocks, using software PWM, because timer1 belongs to the MSF transmitter.
// Keep `stepMs` well under a second so the watchdog is fed.
void pulse(uint32_t durationMs, uint32_t stepMs, bool dim);

}  // namespace led
