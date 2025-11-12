/*

From Wikipedia:

Each UTC second begins with 100 ms of 'off', preceded by at least 500 ms of
carrier. The second marker is transmitted with an accuracy better than ±1 ms
relative to Coordinated Universal Time (UTC), which is itself always within
±0.9 seconds of Universal Time (UT1) which is the mean solar time which would
actually be observed at 0° longitude.

The first second of the minute, denoted second 00, begins with a period of 500
ms with the carrier off, to serve as a minute marker.

The other 59 (or, exceptionally, 60 or 58) seconds of the minute always begin
with at least 100 ms 'off', followed by two data bits of 100 ms each, and end
with at least 700 ms of carrier.

Bit A is transmitted from 100 to 200 ms after the second
Bit B is transmitted from 200 to 300 ms after the second

Negative Polarity Bit Signalling

Carrier ON represents a bit value of 0.
Carrier OFF represents a bit value of 1.

If each second is considered as ten 100 ms pieces, the minute marker is
transmitted as 1111100000, while all other seconds are transmitted as
1AB0000000

*/

#if F_CPU == 160000000L
#warning "Running at 160 MHz"
#else
#warning "Not 160 MHz"
#endif

#include <array>
#include <cmath>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <time.h>
#include <user_interface.h> // for system_rtc_mem_read/write
#include "WiFi_Credentials.h"

// LED GPIOs
#define MSF_GPIO 4 // Labelled D2 on NodeMCU
#define LED_GPIO 2 // Labelled D4 on NodeMCU

// Debugging flags
#define INVERT_TCO 0
#define ENABLE_SLEEP_MODE 1

// Define the positions of the A and B bits in each 1 second frame
#define A 0
#define B 1

//
// Global constants
//

//
// Wi-Fi & NTP logic
//
const char *ssid = WIFI_SSID;                  // your network SSID (name)
const char *pass = WIFI_PASS;                  // your network password
const char *ntpServerName = "uk.pool.ntp.org"; // Hostname of NTP pool

//
// Optional static IP (avoids DHCP delay)
//
const IPAddress local_IP(192, 168, 0, 241);
const IPAddress gateway(192, 168, 0, 1);
const IPAddress subnet(255, 255, 255, 0);
const IPAddress dns1(192, 168, 0, 1);

//
// Numerical constants
//
constexpr uint16_t LOCALPORT = 2390;                                          // local port to listen for UDP packets
constexpr size_t NTP_PACKET_SIZE = 48;                                        // NTP time stamp is in the first 48 bytes of the message
constexpr uint32_t SEVENTY_YEARS = 2208988800UL;                              // NTP timestamp of UNIX epoch
constexpr uint32_t STW_DURATION = 3660;                                       // Duration (seconds) to show the STW flag for before DST change
constexpr uint32_t WAKE_UP_TIME = 2 * 3600 + 59 * 60;                         // Local time
constexpr uint32_t RUN_DURATION = 15 * 60;                                    // How long to run for before sleeping
constexpr uint32_t APB = 80'000'000;                                          // Timer1 runs off APB
constexpr uint32_t TIMER1_PRESCALER = 16;                                     // fixed on ESP8266
constexpr uint32_t TICKS_PER_MS = APB / TIMER1_PRESCALER / 1000;              // Duration of a ms in timer1 ticks
constexpr uint32_t SUBBIT_DURATION_MS = 100;                                  // A subbit is 100 ms
constexpr uint32_t SUBBIT_DURATION_TICKS = SUBBIT_DURATION_MS * TICKS_PER_MS; // A subbit is 5M ticks
constexpr uint32_t MAX_SLEEP_DURATION = 60 * 60;                              // 60 min

//
// Global variables
//

//
// Wi-Fi + NTP
//
IPAddress timeServerIP;             // Store IP address of the UK NTP pool
byte packetBuffer[NTP_PACKET_SIZE]; // buffer to hold incoming and outgoing packets
WiFiUDP udp;                        // A UDP instance to let us send and receive packets over UDP

//
// Time keeping
//
uint32_t lastResyncMillis; // Timestamp of when we received the NTP packet
uint32_t firstMillis;      // Timestamp of when we received the first NTP packet

//
// Data shared between ISR and main loop
//
volatile uint8_t msfBits[60][2] = {0}; // 120-bit MSF frame
volatile bool nextFrame = false;       // Indicate when to build the next MSF frame
volatile uint8_t ISR_bit = 0;
volatile uint8_t ISR_subbit = 0;

//
// EEPROM data structure
//

const uint32_t VERSION = 2; // Used to invalidate the CRC

struct __attribute__((packed))
{
    time_t nextRunTimeUtc;
    time_t lastSleepTimeUtc;
    uint32_t version = VERSION; // Used to invalidate the CRC
    bool firstHour;
    uint32_t crc;
} rtcData;

//
// Shared UTC timestamps
//
time_t utcTime = 0;
time_t nextSleepTime = 0;

//
// BST bookends
//
time_t bst_start;
time_t bst_end;

//
// ISR to toggle output pin(s)
//

void IRAM_ATTR onTimerISR()
{
    static uint8_t carrier = 1;

    if (ISR_bit == 0)
    {
        // Carrier off for 500 ms, then on for 500 ms
        carrier = (ISR_subbit < 5) ? LOW : HIGH;
    }
    else
    {
        switch (ISR_subbit)
        {
        case 0:
            // carrier off 100 ms
            carrier = LOW;
            break;
        case 1:
            // 0 = carrier on
            // 1 = carrier off
            carrier = msfBits[ISR_bit][A] ? LOW : HIGH;
            break;
        case 2:
            // 0 = carrier on
            // 1 = carrier off
            carrier = msfBits[ISR_bit][B] ? LOW : HIGH;
            break;
        default:
            // carrier on 700ms
            carrier = HIGH;
            break;
        }
    }

#if INVERT_TCO
    carrier = (carrier == HIGH) ? LOW : HIGH;
#endif

    if (carrier == HIGH)
    {
        GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, 1 << MSF_GPIO); // Set HIGH
                                                              // LEDon();
    }
    else
    {
        GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, 1 << MSF_GPIO); // Set LOW
                                                              // LEDoff();
    }

    if (++ISR_subbit == 10)
    {
        ISR_subbit = 0;
        if (++ISR_bit == 60)
        {
            ISR_bit = 0;
            // We have 100ms to calculate the next frame
            // Fortunately, it only takes ~1ms.
            nextFrame = true;
        }
    }
}

void initTimer1ISR()
{
    // 80MHz / 16 / 10 = 10Hz
    timer1_isr_init();
    timer1_attachInterrupt(onTimerISR);
    timer1_write(SUBBIT_DURATION_TICKS);

    // Prime the ISR related variables
    ISR_bit = 0;
    ISR_subbit = 0;
    nextFrame = true;
}

void activateTimer1ISR()
{
    timer1_enable(TIM_DIV16, TIM_EDGE, TIM_LOOP);
}

inline void ledOn()
{
    // Clear bit to fire up LED
    GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, 1 << LED_GPIO);
}

inline void ledOff()
{
    // Set bit to extinguish LED
    GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, 1 << LED_GPIO);
}

void setLedState(bool &state)
{
    state = !state;
    if (state)
        ledOn();
    else
        ledOff();
}

uint32_t calculateCRC32(const uint8_t *data, size_t length)
{
    uint32_t crc = 0xFFFFFFFF;
    while (length--)
    {
        uint8_t c = *data++;
        for (uint8_t i = 0; i < 8; i++)
        {
            uint32_t mix = (crc ^ c) & 0x01;
            crc >>= 1;
            if (mix)
                crc ^= 0xEDB88320;
            c >>= 1;
        }
    }
    return ~crc;
}

//
// Check that:
// 1) The system awoke from a deep sleep
// 2) That the RTC data is valid
// 3) That the first hour is finished
// 4) That the next run time is still in the future
//
bool loadRTCData()
{
    if (REASON_DEEP_SLEEP_AWAKE != system_get_rst_info()->reason)
    {
        return false;
    }

    system_rtc_mem_read(64, &rtcData, sizeof(rtcData));
    if (rtcData.crc != calculateCRC32((uint8_t *)&rtcData, sizeof(rtcData) - 4) || rtcData.version != VERSION)
    {
        // Invalid or uninitialised data — set defaults
        rtcData.firstHour = true;
        rtcData.nextRunTimeUtc = 0;
        rtcData.version = VERSION;
        saveRTCData();
        return false;
    }

    return ((rtcData.firstHour == false) && (rtcData.nextRunTimeUtc > utcTime));
}

void saveRTCData()
{
    rtcData.crc = calculateCRC32((uint8_t *)&rtcData, sizeof(rtcData) - 4);
    system_rtc_mem_write(64, &rtcData, sizeof(rtcData));
}

//
// Init WiFi
//
void initWiFi()
{
    // State variables
    uint32_t lastLedMillis = 0;
    bool led = false;

    // --- Wi-Fi initialisation ---
    WiFi.mode(WIFI_STA);                // initialise Wi-Fi stack
    WiFi.setPhyMode(WIFI_PHY_MODE_11N); // force N-only for APs that require it
    WiFi.setAutoReconnect(true);        // automatically reconnect on drops

    // --- Handle deep sleep wakeup vs normal boot ---

    if (REASON_DEEP_SLEEP_AWAKE == system_get_rst_info()->reason)
    {
        WiFi.persistent(false);
        WiFi.config(local_IP, gateway, subnet, dns1);
        WiFi.begin(); // reuse stored creds

        // --- Wait for connection with LED blink ---
        uint32_t start = millis();

        // --- Fast reset for 10s ---
        while (WiFi.status() != WL_CONNECTED && (millis() - start) < 10000)
        {
            if ((millis() - lastLedMillis) >= 25)
            { // 20Hz cycle
                lastLedMillis = millis();
                setLedState(led);
            }
            yield();
        }

        if (WiFi.status() == WL_CONNECTED)
        {
            return; // Success!
        }
    }

    // --- Fallback if fast reconnect failed ---

    WiFi.persistent(true);
    WiFi.config(local_IP, gateway, subnet, dns1);
    WiFi.begin(ssid, pass);

    while (WiFi.status() != WL_CONNECTED)
    {
        if ((millis() - lastLedMillis) >= 100)
        { // 5Hz cycle
            lastLedMillis = millis();
            setLedState(led);
        }
        yield();
    }
}

//
//  Get NTP time
//  Returns TRUE upon successfully retrieving an NTP packet
//  Will give up after 100s if NTP not available
//
bool GetNtpTime(time_t *utcTime_p, uint32_t *milliseconds)
{
    uint32_t cb = 0;

    // Retry ten times
    for (uint32_t j = 0; j < 10 && !cb; ++j)
    {
        sendNTPpacket(timeServerIP); // send an NTP packet to a time server
        // Poll buffers for response
        for (uint32_t i = 0; i < 1000 && !cb; ++i)
        {
            delay(10);
            cb = udp.parsePacket();
        };
    }

    lastResyncMillis = millis(); // Mark moment packet was delivered

    if (!cb)
    {
        return false;
    }

    udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

    // the timestamp starts at byte 40 of the received packet and is four bytes,
    uint32_t secsSince1900 = (packetBuffer[40] << 24) | (packetBuffer[41] << 16) | (packetBuffer[42] << 8) | (packetBuffer[43] << 0);

    // Calculate UTC Unix epoch time
    *utcTime_p = secsSince1900 - SEVENTY_YEARS;

    // Work out milliseconds fraction in bytes 44-47
    if (milliseconds)
    {
        uint32_t fract = (packetBuffer[44] << 24) | (packetBuffer[45] << 16) | (packetBuffer[46] << 8) | (packetBuffer[47] << 0);
        *milliseconds = fract / 4294968; // 99.99998% accurate...
    }

    return true;
}

//
//  Send NTP request and initialise time settings
//
void initialiseRtcClock()
{
    uint32_t ntpMilliSeconds(0);

    // No Wi-Fi?  Bail out now!
    if (WiFi.status() != WL_CONNECTED)
    {
        return;
    }

    timer1_disable();

    if (!GetNtpTime(&utcTime, &ntpMilliSeconds))
    {
        return;
    }

    // Calculate DST limits for current year
    struct tm *t = gmtime(&utcTime);
    uint32_t year = static_cast<uint32_t>(t->tm_year + 1900);
    bst_start = last_sunday_utc(year, 3);
    bst_end = last_sunday_utc(year, 10);

    uint32_t modulo60 = (utcTime % 60);

    utcTime -= modulo60; // Round off seconds, add 60s = time at end of current minute

    // Calculate the exact milliseconds until the start of the next subbit
    uint32_t elapsedSinceNtp = millis() - lastResyncMillis; // time since NTP packet received
    uint32_t subbitOffset = SUBBIT_DURATION_MS;             // small offset for ISR alignment
    uint32_t secondsMs = 1000 * (modulo60);                 // milliseconds into current minute

    // Total time since the last full minute
    uint32_t totalMs = secondsMs + ntpMilliSeconds + elapsedSinceNtp + subbitOffset;

    // Compute remaining time to sleep until the next minute starts
    uint32_t sleepMs = (totalMs < 60000) ? (60000 - totalMs)
                                         : (120000 - totalMs); // wrap-around for safety

    uint32_t wakeup = sleepMs + millis();

    // Set up timer 1 ISR
    initTimer1ISR();

    // If we have time, pulse the LED while waiting for the the next minute boundary
    // If <10ms remaining, don't bother pulsing
    if (sleepMs > 10)
        pulseLED(sleepMs - 10, 25, false); // 25*40 = 1000ms/cycle

    // Wait for the last few milliseconds
    delay(wakeup - millis());

    // Wake up and enable timer interrupts
    activateTimer1ISR();
}

//
// send an NTP request to the time server at the given address
//
void sendNTPpacket(IPAddress &address)
{
    // set all bytes in the buffer to 0
    memset(packetBuffer, 0, NTP_PACKET_SIZE);
    // Initialize values needed to form NTP request
    // (see URL above for details on the packets)
    packetBuffer[0] = 0b11100011; // LI, Version, Mode
    packetBuffer[1] = 0;          // Stratum, or type of clock
    packetBuffer[2] = 6;          // Polling Interval
    packetBuffer[3] = 0xEC;       // Peer Clock Precision
    // 8 bytes of zero for Root Delay & Root Dispersion
    packetBuffer[12] = 49;
    packetBuffer[13] = 0x4E;
    packetBuffer[14] = 49;
    packetBuffer[15] = 52;

    // all NTP fields have been given values, now
    // you can send a packet requesting a timestamp:
    udp.beginPacket(address, 123); // NTP requests are to port 123
    udp.write(packetBuffer, NTP_PACKET_SIZE);
    udp.endPacket();
}

//
// Build a MSF frame using the supplied UNIX epoch time
//
void buildMSFframe(time_t epochTime)
{

    noInterrupts();

    // Use 3660 because the STW flag should be set for 61 minutes before change over
    bool stw = ((bst_start > epochTime) && (bst_start - epochTime <= STW_DURATION)) || ((bst_end > epochTime) && (bst_end - epochTime <= STW_DURATION));
    bool bst = (epochTime >= bst_start) && (epochTime < bst_end);

    time_t localtime = epochTime + (bst ? 3600 : 0); // Round off seconds

    struct tm *t = gmtime(&localtime);

    // Prepare array
    for (uint32_t i = 1; i < 59; i++)
    {
        msfBits[i][A] = 0;
        msfBits[i][B] = 0;
    }

    // Encode fields with correct bit lengths
    encodeBCD(17, t->tm_year % 100, 8); // year units  0-99
    encodeBCD(25, t->tm_mon + 1, 5);    // month units 1-12
    encodeBCD(30, t->tm_mday, 6);       // day units   1-31

    encodeBCD(36, t->tm_wday, 3); // DOW, 3 bits 0=Sunday, 6=Saturday

    encodeBCD(39, t->tm_hour, 6); // hour units   0-23
    encodeBCD(45, t->tm_min, 7);  // minute units 0-59

    // msfBits[52][A] = 0;  // Minute marker
    // msfBits[52][B] = 0;  // Minute marker

    // DST flag
    msfBits[53][A] = 1;
    msfBits[53][B] = stw; // DST change warning (set 1 if 1 hour before BST/GMT change)

    // Minute marker pattern (52–57) is usually 011111, but simplified here
    msfBits[54][A] = 1;
    msfBits[54][B] = oddParity(17, 24);
    msfBits[55][A] = 1;
    msfBits[55][B] = oddParity(25, 35);
    msfBits[56][A] = 1;
    msfBits[56][B] = oddParity(36, 38);
    msfBits[57][A] = 1;
    msfBits[57][B] = oddParity(39, 51);

    // DST flag
    msfBits[58][A] = 1;
    msfBits[58][B] = bst; // DST currently in effect: 1=BST, 0=GMT

    interrupts();
}

//
// Encode value to BCD using given bit length
//
void encodeBCD(uint32_t start, uint32_t val, uint32_t bits)
{
    uint32_t position = start + bits - 1; // position of LSB
    uint32_t remaining = bits;
    while (remaining > 0)
    {
        uint32_t i_max = remaining > 4 ? 4 : remaining;
        uint8_t digit = val % 10;
        val /= 10;
        remaining -= i_max;
        for (uint32_t i = 0; i < i_max; ++i)
        {
            msfBits[position--][A] = (digit >> i) & 1;
        }
    }
}

//
// Odd parity calculation
//
uint8_t oddParity(uint32_t a, uint32_t b)
{
    uint32_t ones = 0;
    // Count number of bits set to 1
    for (uint32_t i = a; i <= b; i++)
        if (msfBits[i][A])
            ones++;
    // Calculate parity bit to make the total odd
    return 1 - (ones & 1); // make total number of 1s odd
}

//
// Deep sleep until next 3 am
// Note: UTCtime is time of the next timestamp to be sent, so is 1m ahead
//
void sleepUntil()
{
    bool bst = (utcTime >= bst_start) && (utcTime < bst_end);
    time_t utcMidnight = utcTime - (utcTime % 86400);
    time_t dstOffset = bst ? 3600 : 0;
    time_t localTime = utcTime + dstOffset;
    time_t timeOfDay = localTime % 86400;
    time_t targetToD = WAKE_UP_TIME; // 03:00:00

    // Calculate target wake up time from local time of day
    // If targetSec is < 10s in the future, add 24h
    if (targetToD < timeOfDay)
        targetToD += 24 * 3600;

    rtcData.firstHour = false;
    rtcData.nextRunTimeUtc = utcMidnight + targetToD - dstOffset;
    rtcData.lastSleepTimeUtc = 0;
    rtcData.version = VERSION;

    saveRTCData();

    // Calculate sleep duration to target time of day
    time_t sleepSec = targetToD - timeOfDay;

    if (sleepSec < 60)
    {
        nextSleepTime += MAX_SLEEP_DURATION; // Stay up another hour
        return;
    }

    // Wake every hour(ish) or less
    if (sleepSec > MAX_SLEEP_DURATION)
        sleepSec = MAX_SLEEP_DURATION;

    ESP.deepSleep((uint64_t)sleepSec * 1000000ULL, WAKE_RF_DEFAULT);
}

time_t my_timegm(struct tm *tm)
{
    static const uint32_t days_in_month[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

    uint32_t year = tm->tm_year + 1900;
    uint32_t month = tm->tm_mon;
    if (month > 11)
        return -1;

    // Days since epoch to start of this year
    time_t days = (year - 1970) * 365 + (year - 1969) / 4 // leap years since 1970
                  - (year - 1901) / 100 + (year - 1601) / 400;

    // Add days of months before this one
    for (uint32_t i = 0; i < month; i++)
    {
        days += days_in_month[i];
        if (i == 1 && ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0)))
            days++; // leap day in Feb
    }

    days += tm->tm_mday - 1;

    return days * 86400 + tm->tm_hour * 3600 + tm->tm_min * 60 + tm->tm_sec;
}

time_t last_sunday_utc(uint32_t year, uint32_t month)
{
    struct tm tm;
    tm.tm_year = year - 1900;
    tm.tm_mon = month - 1;
    tm.tm_mday = 31; // start at last possible day
    tm.tm_hour = 1;  // 01:00 UTC
    time_t t = my_timegm(&tm);

    struct tm *g = gmtime(&t);
    uint32_t wday = g->tm_wday;    // 0=Sun .. 6=Sat
    return t - (wday * 24 * 3600); // back to Sunday
}

//
// Set up NTP server and start UDP listener
//
void setup()
{
    bool ntpSuccess;

    // LED (GPIO2)
    GPF(LED_GPIO) = GPFFS(GPFFS_GPIO(LED_GPIO));          // Set mode to GPIO
    GPC(LED_GPIO) = (GPC(LED_GPIO) & (0xF << GPCI));      // SOURCE(GPIO) | DRIVER(NORMAL) | INT_TYPE(UNCHANGED) | WAKEUP_ENABLE(DISABLED)
    GPES = (1 << LED_GPIO);                               // Enable
    GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, 1 << LED_GPIO); // Turn LED off (inverted logic)

    // MSF (GPIO4)
    GPF(MSF_GPIO) = GPFFS(GPFFS_GPIO(MSF_GPIO));          // Set mode to GPIO
    GPC(MSF_GPIO) = (GPC(MSF_GPIO) & (0xF << GPCI));      // SOURCE(GPIO) | DRIVER(NORMAL) | INT_TYPE(UNCHANGED) | WAKEUP_ENABLE(DISABLED)
    GPES = (1 << MSF_GPIO);                               // Enable
    GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, 1 << MSF_GPIO); // Ensure MSF output is low

    // Block here forever until WiFi is discovered
    initWiFi();

    // Indicate Wi-Fi connected
    ledOn();

    // Start UDP listener
    udp.begin(LOCALPORT);

    // get a random server from the pool
    WiFi.hostByName(ntpServerName, timeServerIP);

    // Order matters! Call GetNtpTime first!

    // Get the current time from NTP server

    do
    {
        // GetNtpTime will attempt to retrieve an NTP packet ten time
        ntpSuccess = GetNtpTime(&utcTime, NULL);
    } while (ntpSuccess == false);

    // Now recover next run time from RTC RAM and decide whether to go back to sleep

    if (loadRTCData())
    {
        // nextRunTimeUtc should always be > utcTime because of the check inside loadRTCData()
        // Therefore there is no risk of temp being negative or hugely positive
        time_t temp = rtcData.nextRunTimeUtc - utcTime;
        if (temp <= 86400 && temp > 60)
        { // If more than a minute remaining, take a nap.
            if (temp > MAX_SLEEP_DURATION)
            {                              // If more that 60 minutes remaining
                temp = MAX_SLEEP_DURATION; // cap next nap at 60 minutes
            }

            ESP.deepSleep((uint64_t)temp * 1000000ULL, WAKE_RF_DEFAULT);
        }
    }

    // Continue to send MSF!

    nextSleepTime = utcTime - (utcTime % 60) + RUN_DURATION;

    firstMillis = millis();

    // Critical timing starts here!

    initialiseRtcClock();

    ledOff();
}
//
// End of setup()
//

//
// Parametric, constexpr LUT generator
//
template <size_t Steps>
constexpr auto makeGammaLUT(double exponent = 2.2)
{
    return [=]() constexpr
    {
        std::array<uint8_t, Steps> lut{};
        const double divider = Steps - 1;
        for (size_t i = 0; i < Steps; ++i)
        {
            // compute the curve value, scale to 0..255, round properly
            lut[i] = static_cast<uint8_t>(255 * std::pow(i / divider, exponent) + 0.5);
        }
        return lut;
    }(); // immediately invoke the lambda
}

//
// Blocking function that will pulse the LED for a given while at a given rate
// 21 brightness levels
// Full cycle will be 40 steps
//
void pulseLED(uint32_t duration_ms, uint8_t ms_step, bool dim)
{
    static const uint8_t max_idx = 20;
    static constexpr auto pwm_lut = makeGammaLUT<max_idx + 1>();

    uint8_t level = 0;
    int8_t step = 1;
    uint32_t lastmillis = millis();
    uint32_t wakeup = lastmillis + duration_ms;
    uint8_t pwm = dim ? pwm_lut[level] / 2 : pwm_lut[level];

    // Bresenham accumulator for PWM
    uint16_t acc = 0;

    // WDT will fire if we don't yeild every 1.5s or so.  Prevent WDT firing....
    if (ms_step > 75)
    {
        ms_step = 75;
    }

    while ((int32_t)(wakeup - millis()) > 0)
    {

        // PWM tick, 250kHz
        acc += pwm;
        if (acc >= 255)
        {
            acc -= 255;
            ledOn();
        }
        else
        {
            ledOff();
        }

        delayMicroseconds(10); // 10 µs tick → 100 kHz PWM clk or 392 Hz pulse rate

        // Update fade level at ms_step
        uint32_t now = millis();
        if ((uint32_t)(now - lastmillis) >= ms_step)
        {
            // reverse direction at edges, call yield for WDT
            if (level == 0)
            {
                step = 1;
                yield();
            }
            else if (level == max_idx)
            {
                step = -1;
                yield();
            }

            level += step;
            pwm = dim ? pwm_lut[level] / 2 : pwm_lut[level];
            lastmillis = now;
        }
    }
    ledOff();
}

//
// Main loop tasks
//
void loop()
{

    // Wait until ISR set the nextFrame flag at 59m 59.9s past the hour...
    if (!nextFrame)
    {
        return;
    }

    nextFrame = false;

    utcTime += 60; // Increment UTC time by one minute.

#if ENABLE_SLEEP_MODE
    // After an hour, sleep until 3 am
    if (utcTime >= nextSleepTime)
    {
        sleepUntil();
    }
#endif

    // Calcuate the next MSF frame
    buildMSFframe(utcTime + 60); // One minute ahead.

    // Up to here ~8 ms
    // We now have at least 59.9 seconds until the next update is due.

    // Pulse the LED for 55s
    pulseLED(55000, 75, true); // 75*40 = 3000ms/cycle
}
//
// End of loop()
//
