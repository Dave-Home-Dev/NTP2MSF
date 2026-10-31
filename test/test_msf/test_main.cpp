#include <unity.h>
#include <ctime>
#include "MsfFrame.h"
#include "UkTime.h"

// Reference instants computed independently (Python calendar.timegm).
constexpr time_t kMar29_0059_59 = 1774745999;  // 2026-03-29 00:59:59 UTC, last second of GMT
constexpr time_t kMar29_0100 = 1774746000;     // first second of BST
constexpr time_t kOct25_0059_59 = 1792889999;  // last second of BST
constexpr time_t kOct25_0100 = 1792890000;     // first second of GMT
constexpr time_t kOct30_1200 = 1793361600;
constexpr time_t kOct30_1407 = 1793369220;     // Fri 2026-10-30 14:07 GMT
constexpr time_t kJul01_1200 = 1782907200;     // 13:00 BST

void setUp() {}
void tearDown() {}

void test_bst_boundaries() {
    TEST_ASSERT_FALSE(uk::fromUtc(kMar29_0059_59).bst);
    TEST_ASSERT_TRUE(uk::fromUtc(kMar29_0100).bst);
    TEST_ASSERT_TRUE(uk::fromUtc(kOct25_0059_59).bst);
    TEST_ASSERT_FALSE(uk::fromUtc(kOct25_0100).bst);
}

void test_change_warning_window() {
    TEST_ASSERT_TRUE(uk::fromUtc(kMar29_0100 - 3660).changeSoon);
    TEST_ASSERT_FALSE(uk::fromUtc(kMar29_0100 - 3661).changeSoon);
    TEST_ASSERT_FALSE(uk::fromUtc(kMar29_0100).changeSoon);
    TEST_ASSERT_TRUE(uk::fromUtc(kOct25_0100 - 60).changeSoon);
}

void test_next_wake() {
    constexpr uint32_t k0259 = 2 * 3600 + 59 * 60;
    TEST_ASSERT_EQUAL_INT64(1793415540, uk::nextLocal(kOct30_1200, k0259));   // winter: 02:59 UTC
    TEST_ASSERT_EQUAL_INT64(1782957540, uk::nextLocal(1782948600, k0259));    // summer, 23:30 UTC (00:30 BST): 01:59 UTC next day
    TEST_ASSERT_EQUAL_INT64(1792897140, uk::nextLocal(1792843200, k0259));    // across the October change: 02:59 GMT
    TEST_ASSERT_EQUAL_INT64(1774749540, uk::nextLocal(1774699200, k0259));    // across the March change: 02:59 BST
}

// --- an independent decoder for the A bits ---------------------------------
static unsigned bcd(const MsfFrame& f, unsigned first, unsigned width) {
    unsigned units = 0, tens = 0;  // the last four bits are the units digit, the rest the tens
    for (unsigned i = 0; i < width; ++i) {
        const unsigned bit = f.a(first + i);
        const unsigned fromEnd = width - 1 - i;
        if (fromEnd < 4) units |= bit << fromEnd; else tens |= bit << (fromEnd - 4);
    }
    return tens * 10 + units;
}

static bool parityOk(const MsfFrame& f, unsigned first, unsigned last, unsigned parityBitSecond) {
    unsigned ones = f.b(parityBitSecond);
    for (unsigned i = first; i <= last; ++i) ones += f.a(i);
    return ones % 2 == 1;
}

static void checkFrame(time_t announced, unsigned yy, unsigned mon, unsigned day, unsigned dow,
                       unsigned hh, unsigned mm, bool bst) {
    const MsfFrame f = MsfFrame::encode(announced);
    TEST_ASSERT_EQUAL_UINT(yy, bcd(f, 17, 8));
    TEST_ASSERT_EQUAL_UINT(mon, bcd(f, 25, 5));
    TEST_ASSERT_EQUAL_UINT(day, bcd(f, 30, 6));
    TEST_ASSERT_EQUAL_UINT(dow, bcd(f, 36, 3));
    TEST_ASSERT_EQUAL_UINT(hh, bcd(f, 39, 6));
    TEST_ASSERT_EQUAL_UINT(mm, bcd(f, 45, 7));
    TEST_ASSERT_EQUAL(bst, f.b(58));
    TEST_ASSERT_TRUE(parityOk(f, 17, 24, 54));
    TEST_ASSERT_TRUE(parityOk(f, 25, 35, 55));
    TEST_ASSERT_TRUE(parityOk(f, 36, 38, 56));
    TEST_ASSERT_TRUE(parityOk(f, 39, 51, 57));
    // fixed 01111110 pattern on A bits 52..59, no stray bits elsewhere
    TEST_ASSERT_FALSE(f.a(52));
    for (unsigned s = 53; s <= 58; ++s) TEST_ASSERT_TRUE(f.a(s));
    TEST_ASSERT_FALSE(f.a(59));
    for (unsigned s = 1; s <= 16; ++s) { TEST_ASSERT_FALSE(f.a(s)); TEST_ASSERT_FALSE(f.b(s)); }
}

void test_frame_winter() { checkFrame(kOct30_1407, 26, 10, 30, 5, 14, 7, false); }
void test_frame_summer() { checkFrame(kJul01_1200, 26, 7, 1, 3, 13, 0, true); }

void test_carrier_waveform() {
    const MsfFrame f = MsfFrame::encode(kOct30_1407);
    for (unsigned t = 0; t < 10; ++t) TEST_ASSERT_EQUAL(t >= 5, f.carrier(0, t));  // minute marker
    // second 53: A = 1 (carrier off), B = 0 (carrier on), then on
    TEST_ASSERT_FALSE(f.carrier(53, 0));
    TEST_ASSERT_FALSE(f.carrier(53, 1));
    TEST_ASSERT_TRUE(f.carrier(53, 2));
    for (unsigned t = 3; t < 10; ++t) TEST_ASSERT_TRUE(f.carrier(53, t));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_bst_boundaries);
    RUN_TEST(test_change_warning_window);
    RUN_TEST(test_next_wake);
    RUN_TEST(test_frame_winter);
    RUN_TEST(test_frame_summer);
    RUN_TEST(test_carrier_waveform);
    return UNITY_END();
}
