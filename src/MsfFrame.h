#pragma once
#include <cstdint>
#include <ctime>

// One minute of the MSF time code: sixty seconds, each carrying an A and a B bit.
//
//   second 0      : minute marker, carrier off for 500 ms
//   seconds 1..59 : carrier off 100 ms, then A (100 ms), B (100 ms), then carrier on
//
// Bits use negative polarity: carrier OFF means 1, carrier ON means 0.
class MsfFrame {
public:
    // The code sent during the minute that ends at the marker for `announcedUtc`:
    // it tells the clock what the time will be at the *next* minute marker.
    static MsfFrame encode(time_t announcedUtc);

    [[gnu::always_inline]] bool a(unsigned second) const { return a_[second]; }
    [[gnu::always_inline]] bool b(unsigned second) const { return b_[second]; }

    // Carrier level during one 100 ms sub-bit. Kept inline and branch-light because the
    // timer interrupt calls it.
    [[gnu::always_inline]] bool carrier(unsigned second, unsigned tenth) const {
        if (second == 0) {
            return tenth >= 5;
        }
        switch (tenth) {
        case 0:
            return false;
        case 1:
            return !a_[second];
        case 2:
            return !b_[second];
        default:
            return true;
        }
    }

private:
    // 60 bits in two 32-bit words, so ISR access never needs 64-bit library helpers.
    class Row {
    public:
        [[gnu::always_inline]] bool operator[](unsigned i) const { return (word_[i >> 5] >> (i & 31)) & 1u; }
        void set(unsigned i, bool value = true) {
            const uint32_t mask = 1u << (i & 31);
            if (value) {
                word_[i >> 5] |= mask;
            } else {
                word_[i >> 5] &= ~mask;
            }
        }

    private:
        uint32_t word_[2] = {0, 0};
    };

    void putBcd(unsigned first, unsigned width, unsigned value);
    bool oddParityBit(unsigned first, unsigned last) const;

    Row a_;
    Row b_;
};
