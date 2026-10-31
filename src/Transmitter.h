#pragma once
#include "MsfFrame.h"

// Plays an MsfFrame out of the MSF pin. A hardware timer ticks every 100 ms and sets the
// pin from the frame, so the waveform stays exact whatever the main loop is doing.
namespace transmitter {

// Configure the pin (carrier off) and attach the timer interrupt, without starting it.
void begin();

// Start ticking. The first tick, one sub-bit from now, is the start of a minute marker,
// so call this exactly 100 ms before a UTC minute boundary.
void start();

// True once per minute, 100 ms before each minute marker, when the next frame is needed.
// Calling it clears the flag.
bool takeMinuteDue();

// Install the frame for the minute about to begin. Safe at any time during the marker
// second and the 100 ms before it.
void publish(const MsfFrame& frame);

}  // namespace transmitter
