#pragma once
#include <stdint.h>

#if __cplusplus >= 201402L
#define RECONNECT_CONSTEXPR constexpr
#else
#define RECONNECT_CONSTEXPR
#endif

struct ReconnectDetection {
  uint8_t missing = 0, present = 0;
  bool armed = false;

  // 0: NACK адреса, 1: ACK, 2: неопределённая ошибка шины/тайм-аут.
  RECONNECT_CONSTEXPR bool update(uint8_t state, uint8_t confirmations) {
    if (state == 0) {
      present = 0;
      if (missing < confirmations) ++missing;
      if (missing == confirmations) armed = true;
    } else if (state == 1) {
      missing = 0;
      if (!armed) return false;
      if (present < confirmations) ++present;
      if (present == confirmations) {
        armed = false;
        present = 0;
        return true;
      }
    } else {
      missing = present = 0; // Тайм-аут сам по себе не подтверждает отключение.
    }
    return false;
  }
};
#undef RECONNECT_CONSTEXPR
