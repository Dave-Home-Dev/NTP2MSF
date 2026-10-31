# MSF time-signal emulator (ESP8266 / D1 Mini)

Fetches UTC over NTP and transmits it as an MSF60 bit stream so a radio-controlled clock
(Sony ICL-C50) can sync to it. Wakes at 02:59 UK time, transmits for 15 minutes, then
deep-sleeps (hourly naps) until the next night.

## Build on Linux with VS Code

1. Install VS Code and the **PlatformIO IDE** extension (the project recommends it).
2. Serial access: `sudo usermod -aG dialout $USER` (Arch: `uucp`), then log out and in.
   If the board is not detected, install PlatformIO's udev rules:
   `curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules`
   then `sudo udevadm control --reload-rules && sudo udevadm trigger`.
3. `cp include/WiFi_Credentials.h.example include/WiFi_Credentials.h` and fill it in.
4. Open this folder in VS Code. Use the PlatformIO toolbar (Build / Upload / Monitor) or:
   `pio run`, `pio run -t upload`, `pio device monitor`.
5. Host-side unit tests (no hardware): `pio test -e native`.

Deep sleep needs D0 (GPIO16) wired to RST. Pins and behaviour are in `src/Config.h`.

## Layout

| File | Responsibility |
|---|---|
| `main.cpp` | Boot flow and the per-minute loop |
| `Transmitter` | Timer1 ISR that plays the frame on the pin |
| `MsfFrame` | Pure: encodes one minute of MSF (BCD, parity, flags) |
| `UkTime` | Pure: BST rules, local/UTC conversion, next wake time |
| `Ntp`, `Net` | NTP query with round-trip compensation; Wi-Fi connect |
| `RtcState` | Wake time kept in RTC memory across deep sleep |
| `Led` | Breathing LED (software PWM) |
