# NTP2MSF
A tool to set a radio controlled clock

## What Problem Does This Solve?
Many years ago (90's) I bought an alarm clock, a Sony ICF-C50L.  One of the USPs of this device was that it is self-setting, using the MSF radio signal.  This time signal was broadcast by the National Physics Labaratory from a transmitter in Rubgy, UK.  This worked well, until the transmitter was relocated to Anthorn in 2007.

The intention of this project is to create a device that replaces the VLF receiver built into the ICF-C50L.  It will replicates the original MSF signal bitstream using NTP as a time reference.  This only needs to run for 15 minutes day.  Bitstream generation will start at 03.00, local time, as the clock itself is designed to start looking for the MSF bitstream at 03.05 everyday.  This includes the double-whammy of 03.05, localtime, being observed twice at the end of BST in October!  Since NTP is always UTC, this software will determine the limits of BST automatically and apply BST when appropriate.  It will also honour the STW flag in the MSF frame.

More info: https://en.wikipedia.org/wiki/Time_from_NPL_(MSF)

## Software and Hardware Used
	* Arduino IDE v2.3.6
    * Visual Studio Code
	* Wemos Mini D1 (Final target)
	* Wemos/Lolin NodeMCU 0.9 V3 (Testing)
    * Linux Mint

## Synopsis
This software performs the following tasks:
1. At power up:
	* find the WiFi AP and camp on
	* When successful cache the WiFi credentials in the RTC RAM
	* Blink LED at 2Hz while waiting
	* Start the time alignment process
2. Start the time alignment process
	* Obtain NTP packet
	* Decode contents
	* Compute the milliseconds until the end of the current minute
	* If there is time, show the pulsing LED at 1Hz
	* Wait for the end of the minute
	* Activate the ISR
3. Send MSF frame
	* Generate the MSF frame
	* Wait for the end of the frame
	* Meanwhile, pulse the LED slowly over 3s
4. After 15 minutes of sending the MSF frame, got to sleep for an hour, or until 3am if that is sooner.
5. Upon waking up from deep sleep:
	* Wake up
	* Re-establish WiFi connectivity using the cached credentials
	* Blink LED at 20Hz while waiting
	* If this fails fall back to the longer method used in 1.
	* Get time from NTP
	* If it's not yet 03.00, go to sleep for an hour, or until 02.59 if that is sooner
	* Start the time alignment process

## Issues Encountered
(Notes to self)
    * When in the deep-slepp state, the timekeeping of the ESP8266 is only about 90% accurate, despite the microsecond resolution of the system call to invoke it
    * The Watch Dog Timer will reset the MCU after <1.8s if one forgets to yield(), return from from loop(), regularly in the code!
    * If WiFi is not available then the code will keep trying until it is.
    * If WiFi is established, but an NTP packet can not be obtained then we will have no idea of the current time.  In this situation, go to sleep for an hour(ish) and try again.
