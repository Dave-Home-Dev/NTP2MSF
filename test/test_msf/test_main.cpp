#include <unity.h>
#include <ctime>

#include "MsfFrame.h"
#include "UkTime.h"
#include "Align.h"

namespace BasicTests
{
    // Reference instants computed independently (Python calendar.timegm).
    constexpr time_t kMar29_0059_59 = 1774745999; // 2026-03-29 00:59:59 UTC, last second of GMT
    constexpr time_t kMar29_0100 = 1774746000;    // first second of BST
    constexpr time_t kOct25_0059_59 = 1792889999; // last second of BST
    constexpr time_t kOct25_0100 = 1792890000;    // first second of GMT
    constexpr time_t kOct30_1200 = 1793361600;
    constexpr time_t kOct30_1407 = 1793369220; // Fri 2026-10-30 14:07 GMT
    constexpr time_t kJul01_1200 = 1782907200; // 13:00 BST

    void setUp() {}
    void tearDown() {}

    void test_bst_boundaries()
    {
        TEST_ASSERT_FALSE(uk::fromUtc(kMar29_0059_59).bst);
        TEST_ASSERT_TRUE(uk::fromUtc(kMar29_0100).bst);
        TEST_ASSERT_TRUE(uk::fromUtc(kOct25_0059_59).bst);
        TEST_ASSERT_FALSE(uk::fromUtc(kOct25_0100).bst);
    }

    void test_change_warning_window()
    {
        TEST_ASSERT_TRUE(uk::fromUtc(kMar29_0100 - 3660).changeSoon);
        TEST_ASSERT_FALSE(uk::fromUtc(kMar29_0100 - 3661).changeSoon);
        TEST_ASSERT_FALSE(uk::fromUtc(kMar29_0100).changeSoon);
        TEST_ASSERT_TRUE(uk::fromUtc(kOct25_0100 - 60).changeSoon);
    }

    void test_next_wake()
    {
        constexpr uint32_t k0259 = 2 * 3600 + 59 * 60;
        TEST_ASSERT_EQUAL_INT64(1793415540, uk::nextLocal(kOct30_1200, k0259)); // winter: 02:59 UTC
        TEST_ASSERT_EQUAL_INT64(1782957540, uk::nextLocal(1782948600, k0259));  // summer, 23:30 UTC (00:30 BST): 01:59 UTC next day
        TEST_ASSERT_EQUAL_INT64(1792897140, uk::nextLocal(1792843200, k0259));  // across the October change: 02:59 GMT
        TEST_ASSERT_EQUAL_INT64(1774749540, uk::nextLocal(1774699200, k0259));  // across the March change: 02:59 BST
    }

    // --- an independent decoder for the A bits ---------------------------------
    static unsigned bcd(const MsfFrame &f, unsigned first, unsigned width)
    {
        unsigned units = 0, tens = 0; // the last four bits are the units digit, the rest the tens
        for (unsigned i = 0; i < width; ++i)
        {
            const unsigned bit = f.a(first + i);
            const unsigned fromEnd = width - 1 - i;
            if (fromEnd < 4)
                units |= bit << fromEnd;
            else
                tens |= bit << (fromEnd - 4);
        }
        return tens * 10 + units;
    }

    static bool parityOk(const MsfFrame &f, unsigned first, unsigned last, unsigned parityBitSecond)
    {
        unsigned ones = f.b(parityBitSecond);
        for (unsigned i = first; i <= last; ++i)
            ones += f.a(i);
        return ones % 2 == 1;
    }

    static void checkFrame(time_t announced, unsigned yy, unsigned mon, unsigned day, unsigned dow,
                           unsigned hh, unsigned mm, bool bst)
    {
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
        for (unsigned s = 53; s <= 58; ++s)
            TEST_ASSERT_TRUE(f.a(s));
        TEST_ASSERT_FALSE(f.a(59));
        for (unsigned s = 1; s <= 16; ++s)
        {
            TEST_ASSERT_FALSE(f.a(s));
            TEST_ASSERT_FALSE(f.b(s));
        }
    }

    void test_frame_winter() { checkFrame(kOct30_1407, 26, 10, 30, 5, 14, 7, false); }
    void test_frame_summer() { checkFrame(kJul01_1200, 26, 7, 1, 3, 13, 0, true); }

    void test_carrier_waveform()
    {
        const MsfFrame f = MsfFrame::encode(kOct30_1407);
        for (unsigned t = 0; t < 10; ++t)
            TEST_ASSERT_EQUAL(t >= 5, f.carrier(0, t)); // minute marker
        // second 53: A = 1 (carrier off), B = 0 (carrier on), then on
        TEST_ASSERT_FALSE(f.carrier(53, 0));
        TEST_ASSERT_FALSE(f.carrier(53, 1));
        TEST_ASSERT_TRUE(f.carrier(53, 2));
        for (unsigned t = 3; t < 10; ++t)
            TEST_ASSERT_TRUE(f.carrier(53, t));
    }
}

/// @brief Test alignment

namespace AlignmentTests
{
    constexpr time_t kBase = 1793361600; // 2026-10-30 12:00:00 UTC, a minute boundary
    constexpr uint32_t kSubbit = 100;
    constexpr int32_t kLead = 20;

    static Alignment align(time_t utc, uint32_t ms, uint32_t elapsed)
    {
        return alignToMinute(utc, ms, elapsed, kSubbit, kLead);
    }

    void test_mid_minute()
    {
        const Alignment a = align(kBase + 10, 0, 0);
        TEST_ASSERT_EQUAL_INT64(49900, a.waitMs);
        TEST_ASSERT_EQUAL_INT64(kBase + 60, a.firstMinute);
    }

    void test_elapsed_time_counts()
    {
        const Alignment a = align(kBase + 10, 0, 2500);
        TEST_ASSERT_EQUAL_INT64(47400, a.waitMs);
        TEST_ASSERT_EQUAL_INT64(kBase + 60, a.firstMinute);
    }

    void test_reading_just_after_boundary()
    {
        const Alignment a = align(kBase, 5, 0);
        TEST_ASSERT_EQUAL_INT64(59895, a.waitMs);
        TEST_ASSERT_EQUAL_INT64(kBase + 60, a.firstMinute);
    }

    void test_elapsed_time_crosses_boundary()
    {
        // 59.900 s plus 300 ms elapsed puts us 200 ms into the next minute
        const Alignment a = align(kBase + 59, 900, 300);
        TEST_ASSERT_EQUAL_INT64(59700, a.waitMs);
        TEST_ASSERT_EQUAL_INT64(kBase + 120, a.firstMinute);
    }

    void test_elapsed_time_spans_whole_minute()
    {
        const Alignment a = align(kBase, 0, 65000);
        TEST_ASSERT_EQUAL_INT64(54900, a.waitMs);
        TEST_ASSERT_EQUAL_INT64(kBase + 120, a.firstMinute);
    }

    void test_lead_guard_boundary()
    {
        // exactly the minimum lead is accepted...
        const Alignment ok = align(kBase + 59, 880, 0);
        TEST_ASSERT_EQUAL_INT64(20, ok.waitMs);
        TEST_ASSERT_EQUAL_INT64(kBase + 60, ok.firstMinute);

        // ...one millisecond less slips to the next minute
        const Alignment slip = align(kBase + 59, 881, 0);
        TEST_ASSERT_EQUAL_INT64(60019, slip.waitMs);
        TEST_ASSERT_EQUAL_INT64(kBase + 120, slip.firstMinute);
    }

    void test_start_always_lands_on_a_boundary()
    {
        // For every instant over two minutes: now + wait + subbit must equal the marker time.
        for (uint32_t offsetMs = 0; offsetMs < 120000; offsetMs += 7)
        {
            const time_t utc = kBase + offsetMs / 1000;
            const uint32_t ms = offsetMs % 1000;
            const Alignment a = align(utc, ms, 0);

            const int64_t nowMs = static_cast<int64_t>(utc) * 1000 + ms;
            const int64_t tickMs = nowMs + a.waitMs + kSubbit;
            TEST_ASSERT_TRUE(tickMs == static_cast<int64_t>(a.firstMinute) * 1000);
            TEST_ASSERT_TRUE(a.firstMinute % 60 == 0);
            TEST_ASSERT_TRUE(a.waitMs >= kLead);
            TEST_ASSERT_TRUE(a.waitMs < 60000 + kLead);
        }
    }
}

/// @brief Test DST transitions

namespace CivilTests
{
    static void checkCivil(time_t t, int y, int mo, int d, int wd, int h, int mi, int sec)
    {
        const uk::Civil c = uk::civil(t);
        TEST_ASSERT_EQUAL_INT(y, c.year);
        TEST_ASSERT_EQUAL_INT(mo, c.month);
        TEST_ASSERT_EQUAL_INT(d, c.day);
        TEST_ASSERT_EQUAL_INT(wd, c.weekday);
        TEST_ASSERT_EQUAL_INT(h, c.hour);
        TEST_ASSERT_EQUAL_INT(mi, c.minute);
        TEST_ASSERT_EQUAL_INT(sec, c.second);
    }

    void test_civil()
    {
        checkCivil(0, 1970, 1, 1, 4, 0, 0, 0);
        checkCivil(951827696, 2000, 2, 29, 2, 12, 34, 56);  // leap day in a century year
        checkCivil(1709251199, 2024, 2, 29, 4, 23, 59, 59); // leap day, last second
        checkCivil(1793369229, 2026, 10, 30, 5, 14, 7, 9);
        checkCivil(1798761599, 2026, 12, 31, 4, 23, 59, 59); // year end
        checkCivil(4107542400LL, 2100, 3, 1, 1, 0, 0, 0);    // 2100 is not a leap year
    }
}

int main()
{
    UNITY_BEGIN();

    RUN_TEST(BasicTests::test_bst_boundaries);
    RUN_TEST(BasicTests::test_change_warning_window);
    RUN_TEST(BasicTests::test_next_wake);
    RUN_TEST(BasicTests::test_frame_winter);
    RUN_TEST(BasicTests::test_frame_summer);
    RUN_TEST(BasicTests::test_carrier_waveform);

    RUN_TEST(AlignmentTests::test_mid_minute);
    RUN_TEST(AlignmentTests::test_elapsed_time_counts);
    RUN_TEST(AlignmentTests::test_reading_just_after_boundary);
    RUN_TEST(AlignmentTests::test_elapsed_time_crosses_boundary);
    RUN_TEST(AlignmentTests::test_elapsed_time_spans_whole_minute);
    RUN_TEST(AlignmentTests::test_lead_guard_boundary);
    RUN_TEST(AlignmentTests::test_start_always_lands_on_a_boundary);

    RUN_TEST(CivilTests::test_civil);

    return UNITY_END();
}