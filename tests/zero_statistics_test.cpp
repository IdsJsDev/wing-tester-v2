#include "../firmware/wing_tester/zero_statistics.h"

constexpr bool near(float a, float b) { return a - b < 0.01F && b - a < 0.01F; }

constexpr ZeroStatistics signal(int mode) {
  ZeroStatistics stats;
  for (int window = 0; window < 10; ++window) {
    for (int i = 0; i < 50; ++i) {
      float pressure = 100; // Постоянное смещение не считается дрейфом.
      if (mode == 1) pressure += window;         // Рост 1 Па/с.
      if (mode == 2) pressure -= window;         // Падение 1 Па/с.
      if (mode == 3) pressure += (i % 2 ? 10 : -10); // Шум: размах 20, SD >5.
      if (mode == 4) pressure += window < 5 ? window * 3 : (9 - window) * 3;
      stats.add(pressure);
    }
    stats.finishWindow(1.0F);
  }
  return stats;
}

constexpr auto stable = signal(0);
static_assert(stable.samples == 500 && stable.windows == 10, "Incorrect sample/window counts");
static_assert(near(stable.mean, 100) && stable.variance() == 0 && stable.slope() == 0,
              "Constant offset must remain stable");
constexpr auto rising = signal(1);
static_assert(near(rising.slope(), 1) && near(rising.lastMean - rising.firstMean, 9),
              "Slow positive drift must be detected");
constexpr auto falling = signal(2);
static_assert(near(falling.slope(), -1) && near(falling.maxMean - falling.minMean, 9),
              "Slow negative drift must be detected");
constexpr auto noisy = signal(3);
static_assert(near(noisy.maximum - noisy.minimum, 20) && noisy.variance() > 25 &&
              near(noisy.slope(), 0), "Noise must be distinct from drift");
constexpr auto reversing = signal(4);
static_assert(near(reversing.firstMean, reversing.lastMean) &&
              reversing.maxMean - reversing.minMean > 8,
              "Excursion returning to baseline must not hide drift");
constexpr ZeroStatistics empty;
static_assert(empty.variance() == 0 && empty.slope() == 0, "Empty statistics must be defined");
