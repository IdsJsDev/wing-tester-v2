#include "../firmware/wing_tester/reconnect_detection.h"

constexpr bool reconnect() {
  ReconnectDetection gate;
  return !gate.update(0, 3) && !gate.update(0, 3) && !gate.update(0, 3) &&
      !gate.update(1, 3) && !gate.update(1, 3) && gate.update(1, 3) &&
      !gate.update(1, 3) && !gate.update(1, 3);
}
constexpr bool noUnplug() {
  ReconnectDetection gate;
  for (int i = 0; i < 20; ++i) if (gate.update(1, 3)) return false;
  return true;
}
constexpr bool shortDrop() {
  ReconnectDetection gate;
  return !gate.update(0, 3) && !gate.update(0, 3) &&
      !gate.update(1, 3) && !gate.update(1, 3) && !gate.update(1, 3);
}
constexpr bool timeoutIsNotUnplug() {
  ReconnectDetection gate;
  for (int i = 0; i < 10; ++i) if (gate.update(2, 3)) return false;
  return !gate.update(1, 3) && !gate.update(1, 3) && !gate.update(1, 3);
}
constexpr bool confirmationsMustBeConsecutive() {
  ReconnectDetection gate;
  gate.update(0, 3);
  gate.update(0, 3);
  gate.update(2, 3);
  gate.update(0, 3);
  gate.update(1, 3);
  if (gate.armed) return false;
  gate.update(0, 3); gate.update(0, 3); gate.update(0, 3);
  gate.update(1, 3); gate.update(1, 3);
  gate.update(2, 3);
  return !gate.update(1, 3) && !gate.update(1, 3) && gate.update(1, 3);
}
static_assert(reconnect(), "Detected reconnect must restart exactly once");
static_assert(noUnplug(), "A continuously connected sensor must not restart");
static_assert(shortDrop(), "Brief loss must not arm restart");
static_assert(timeoutIsNotUnplug(), "Bus timeouts must not trigger restart");
static_assert(confirmationsMustBeConsecutive(), "Unknown bus state must break each confirmation streak");
