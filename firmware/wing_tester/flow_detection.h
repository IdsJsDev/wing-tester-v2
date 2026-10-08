#pragma once
#include <stdint.h>

#if __cplusplus >= 201402L
#define FLOW_CONSTEXPR constexpr
#else
#define FLOW_CONSTEXPR
#endif

struct FlowDetection {
  bool holding = false;
  bool confirmed = false;
  uint32_t sinceMs = 0;

  FLOW_CONSTEXPR bool update(bool aboveThreshold, uint32_t nowMs, uint32_t holdMs) {
    if (!aboveThreshold) {
      holding = false;
      return false;
    }
    if (!holding) {
      holding = true;
      sinceMs = nowMs;
    }
    return nowMs - sinceMs >= holdMs;
  }

  // Для подачи потока: однажды подтверждённый выдох сохраняется до нового этапа.
  FLOW_CONSTEXPR bool updateLatched(bool aboveThreshold, uint32_t nowMs, uint32_t holdMs) {
    if (!confirmed && update(aboveThreshold, nowMs, holdMs)) confirmed = true;
    return confirmed;
  }
};
#undef FLOW_CONSTEXPR
