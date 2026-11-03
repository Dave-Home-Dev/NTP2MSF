#include "MsfFrame.h"
#include "UkTime.h"

// Write `value` as binary-coded decimal into A bits [first, first + width), MSB first.
// The units digit takes the last four bits; any bits left over hold the tens digit.
void MsfFrame::putBcd(unsigned first, unsigned width, unsigned value) {
    unsigned pos = first + width;  // one past the least significant bit
    for (unsigned left = width; left > 0;) {
        const unsigned n = left < 4 ? left : 4;
        const unsigned digit = value % 10;
        value /= 10;
        for (unsigned i = 0; i < n; ++i) {
            a_.set(--pos, (digit >> i) & 1u);
        }
        left -= n;
    }
}

// Parity bit that makes the number of ones in A[first..last], plus itself, odd.
bool MsfFrame::oddParityBit(unsigned first, unsigned last) const {
    unsigned ones = 0;
    for (unsigned i = first; i <= last; ++i) {
        ones += a_[i];
    }
    return (ones & 1u) == 0;
}

MsfFrame MsfFrame::encode(time_t announcedUtc) {
    const uk::Time wall = uk::fromUtc(announcedUtc);
    const uk::Civil c = uk::civil(wall.local);

    MsfFrame f;
    f.putBcd(17, 8, c.year % 100);  // year
    f.putBcd(25, 5, c.month);     // month
    f.putBcd(30, 6, c.day);        // day of month
    f.putBcd(36, 3, c.weekday);        // day of week, 0 = Sunday
    f.putBcd(39, 6, c.hour);        // hour
    f.putBcd(45, 7, c.minute);         // minute

    // Seconds 52..59 carry 01111110 on the A bits as a fixed marker.
    for (unsigned s = 53; s <= 58; ++s) {
        f.a_.set(s);
    }

    f.b_.set(53, wall.changeSoon);  // summer-time warning
    f.b_.set(54, f.oddParityBit(17, 24));
    f.b_.set(55, f.oddParityBit(25, 35));
    f.b_.set(56, f.oddParityBit(36, 38));
    f.b_.set(57, f.oddParityBit(39, 51));
    f.b_.set(58, wall.bst);         // 1 = BST, 0 = GMT

    // B bits 1..16 would carry DUT1; they are left at zero (UT1 = UTC).
    return f;
}