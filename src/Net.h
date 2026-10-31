#pragma once

namespace net {

// Join the Wi-Fi network with a fixed IP address, blinking the LED while waiting.
// After a deep sleep the stored credentials are tried first (fast); otherwise, or if
// that fails, it connects from scratch. Blocks until connected.
void connect();

}  // namespace net
