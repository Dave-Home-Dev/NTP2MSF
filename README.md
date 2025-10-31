# NTP2MSF
A tool to set a radio controlled clock

## Software and Hardware
*Arduino IDE v2.3.6
*Wemos Mini D1
*Wemos/Lolin NodeMCU 0.9 V3

## Synopsis
This software performs the following tasks:
1. At power up:
	* find the WiFi AP and camp on
	* When successful cache the WiFi credentials in the RTC RAM
	* Blink LED at 2Hz while waiting
	* Start the time alignment process
2. Or wake up from deep sleep:
	* Wake up
	* Re-establish WiFi connectivity using the cached credentials
	* Blink LED at 20Hz while waiting
	* If this fails fall back to the longer method used in 1.
	* Get time from NTP
	* If it's not yet 03.00, go to sleep for an hour, or until 02.59 if that is sooner
	* Start the time alignment process
3. Start the time alignment process
	* Obtain NTP packet
	* Decode contents
	* Compute the milliseconds until the end of the current minute
	* If there is time, show the pulsing LED at 1Hz
	* Wait for the end of the minute
	* Activate the ISR
4. Send MSF frame
	* Generate the MSF frame
	* Wait for the end of the frame
	* Meanwhile, pulse the LED slowly over 3s
5. After 15 minutes of sending the MSF frame, got to sleep for an hour, or until 3am if that is sooner.

## Issues Encountered
(Notes to self)
* ESP8266 timekeeping when in deep sleep is only about 90% accurate, despite the microsecond resolution of the system call to invoke it
* Watch Dog Timer will reset the software after <1.8s if one forgets to yield() in the code!

