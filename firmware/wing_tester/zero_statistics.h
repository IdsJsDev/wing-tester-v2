#pragma once
#include <stdint.h>

// В тестовой сборке C++14 эти же методы проверяются через static_assert.
#if __cplusplus >= 201402L
#define ZERO_CONSTEXPR constexpr
#else
#define ZERO_CONSTEXPR
#endif

struct ZeroStatistics {
  uint32_t samples = 0, windowSamples = 0;
  uint16_t windows = 0;
  float mean = 0, m2 = 0, minimum = 0, maximum = 0;
  float windowSum = 0, firstMean = 0, lastMean = 0;
  float minMean = 0, maxMean = 0;
  float meanX = 0, meanY = 0, xx = 0, xy = 0;

  ZERO_CONSTEXPR void add(float pressure) {
    if (samples == 0) minimum = maximum = pressure;
    if (pressure < minimum) minimum = pressure;
    if (pressure > maximum) maximum = pressure;
    ++samples;
    const float delta = pressure - mean;
    mean += delta / samples;
    m2 += delta * (pressure - mean);
    windowSum += pressure;
    ++windowSamples;
  }

  ZERO_CONSTEXPR void finishWindow(float seconds) {
    if (windowSamples == 0) return;
    const float value = windowSum / windowSamples;
    if (windows == 0) firstMean = minMean = maxMean = value;
    lastMean = value;
    if (value < minMean) minMean = value;
    if (value > maxMean) maxMean = value;
    const float x = (windows + 0.5F) * seconds;
    ++windows;
    const float dx = x - meanX, dy = value - meanY;
    meanX += dx / windows;
    meanY += dy / windows;
    xx += dx * (x - meanX);
    xy += dx * (value - meanY);
    windowSum = 0;
    windowSamples = 0;
  }

  constexpr float slope() const { return xx > 0 ? xy / xx : 0; }
  constexpr float variance() const { return samples > 1 ? m2 / (samples - 1) : 0; }
};

#undef ZERO_CONSTEXPR
