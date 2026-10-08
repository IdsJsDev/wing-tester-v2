#include <Arduino.h>
#include <SPI.h>
#include <Servo.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <avr/pgmspace.h>
#include <math.h>
#include "zero_statistics.h"
#include "flow_detection.h"
#include "reconnect_detection.h"
#include "startup_reset.h"

// ================= НАСТРОЙКИ ПОЛЬЗОВАТЕЛЯ =================
namespace cfg {
constexpr bool kServoTestEnabled = true; // Первый этап при включении/Reset, затем Пито.
constexpr uint8_t kServoPin = 7;          // Сигнал серво, перенесён с D9.
constexpr int kServoNeutral = 92;         // Нейтраль, команда Servo.write(), градусы.
constexpr int kServoMax = 160;            // Верхний предел старого теста, градусы.
constexpr int kServoMin = 20;             // Нижний предел старого теста, градусы.
constexpr uint16_t kServoNeutralHoldMs = 500; // Пауза в нейтрали перед проходом, мс.
constexpr uint16_t kServoStepMs = 10;     // Скорость: задержка на шаг 1°, мс. Больше = медленнее.
// Примеры: 3 мс — старый быстрый проход; 10 мс — медленнее; 20 мс — ещё медленнее.
// SH1106 128x64. Рабочая конфигурация SPI-экрана.
constexpr uint8_t kDisplayCs = 9;       // CS: выбор дисплея.
constexpr uint8_t kDisplayDc = 8;       // DC: данные/команды.
constexpr uint8_t kDisplayReset = 10;   // RES: сброс дисплея.
// Аппаратный SPI классической Nano: MOSI=D11, SCK=D13.
constexpr uint8_t kDisplayContrast = 180; // Контраст, 0..255.
constexpr bool kRotate180 = false;       // true: повернуть изображение на 180°.
constexpr uint32_t kDisplayUpdateMs = 100; // Интервал обновления, мс, >0.
constexpr uint16_t kSpinnerFrameMs = 80; // Период анимации общего индикатора, мс, >0.
const uint8_t *const kUiFont = u8g2_font_9x15_t_cyrillic; // Один шрифт для всех экранов.
constexpr uint8_t kSensorAddress = 0x28; // 7-битный адрес ASPD-4525.
constexpr uint32_t kI2cClockHz = 400000; // Шина I2C 400 кГц, как в исходном тесте.
constexpr uint32_t kWireTimeoutUs = 3000; // Тайм-аут I2C, мкс; меньше периода опроса.
constexpr uint32_t kSensorStartupMs = 250; // Ожидание питания датчика, мс.
constexpr uint32_t kSamplePeriodUs = 5000; // Период опроса, мкс: 5000 = 200 Гц.
constexpr uint8_t kLinkBlocks = 5;         // Количество блоков проверки связи, >0.
constexpr uint16_t kLinkSamplesPerBlock = 20; // 5 блоков по 20 чтений = 100 всего.
constexpr bool kLinkDiagnosticMode = true; // true: все чтения; false: остановка при первой ошибке.
constexpr uint32_t kSerialBaud = 115200;   // Скорость диагностического журнала.
constexpr bool kDiagnoseI2cOnFailure = false; // Поиск адресов после сбоя: выключен для экономии Flash.
constexpr bool kDetailedSerialLog = false; // Расширенные счётчики/байты LINK; краткий журнал остаётся.
constexpr bool kProbeOnLinkFailure = false; // Дополнительная диагностика после ошибки, выключена для экономии Flash.
constexpr uint32_t kDiagnosticTimeoutUs = 20000; // Тайм-аут только медленной диагностики, мкс.
constexpr uint32_t kDiagnosticI2cHz = 10000; // Частота поиска адресов после сбоя, Гц.
// Экран и Serial обновляем между блоками, вне измеряемого опроса.
// Проверка нуля запускается автоматически после успешной связи.
constexpr uint32_t kZeroTestDurationMs = 10000; // Проверка нуля, мс, >0.
constexpr uint32_t kZeroSamplePeriodUs = 20000; // Этап нуля: 50 Гц; запас времени для SPI-экрана.
constexpr uint32_t kZeroPageBudgetUs = 10000; // Запас до опроса для одной страницы OLED, мкс.
constexpr uint32_t kZeroWindowMs = 1000; // Окно оценки среднего и дрейфа, мс.
constexpr float kSensorMinPa = -6894.76F; // Нижний предел: -1 PSI, как в старом тесте.
constexpr float kSensorMaxPa = 6894.76F;  // Верхний предел: +1 PSI; подтвердить маркировку.
constexpr float kNoiseSpanLimitPa = 30.0F; // Предварительный предел размаха, Па.
constexpr float kNoiseSdLimitPa = 5.0F;    // Предварительный предел стандартного отклонения, Па.
constexpr float kDriftRangeLimitPa = 8.0F; // Предварительный размах средних окон, Па.
constexpr float kDriftSlopeLimitPaPerSec = 0.8F; // Предварительный предел тренда, Па/с.
constexpr uint32_t kFlowWaitMs = 10000; // Время на подачу потока после подсказки, мс.
constexpr uint32_t kFlowSamplePeriodUs = 20000; // Этап потока: 50 Гц.
constexpr uint32_t kFlowHoldMs = 100; // Короткое подтверждение выдоха, мс; затем событие запоминается.
constexpr uint32_t kFlowMinimumMs = 4000; // Минимальное время экрана «Дуйте», мс.
constexpr float kFlowThresholdPa = 50.0F; // Предварительный порог реакции относительно нуля, Па.
constexpr bool kFlowAllowNegative = true; // true: реакция в обе стороны; false: только положительная.
constexpr float kAirDensityKgM3 = 1.225F; // Плотность для расчётной скорости, кг/м³.
constexpr float kSpeedZeroBandPa = 3.0F; // Зона отображения нулевой скорости, Па; не порог теста.
constexpr bool kSpeedInKmh = false; // false: м/с; true: км/ч.
constexpr uint16_t kPeakSpeedDisplayMs = 5000; // Максимальная скорость после потока, перед возвратом.
constexpr uint32_t kReturnWaitMs = 5000; // Ожидание возврата после «Не дуйте», мс.
constexpr uint32_t kReturnHoldMs = 1000; // Устойчивое нахождение возле исходного нуля, мс.
constexpr float kReturnBandPa = 5.0F; // Предварительный допуск возврата к нулю, ±Па.
constexpr bool kAutoRestartOnReconnect = true; // После итога: новый цикл при обнаруженном переподключении.
constexpr uint32_t kReconnectPollMs = 250; // Период проверки подключения после итога, мс.
constexpr uint8_t kReconnectConfirmations = 3; // Подтверждений отключения и подключения подряд.
constexpr uint32_t kReconnectI2cHz = 100000; // Проверка адреса в режиме ожидания, Гц.
// Все пороги предварительные, не заводские критерии брака.
}
// =============== КОНЕЦ НАСТРОЕК ПОЛЬЗОВАТЕЛЯ ===============

static_assert(cfg::kDisplayCs != cfg::kDisplayDc &&
              cfg::kDisplayCs != cfg::kDisplayReset &&
              cfg::kDisplayDc != cfg::kDisplayReset, "Display pins must differ");
static_assert(cfg::kDisplayUpdateMs > 0 &&
              cfg::kZeroTestDurationMs > 0 && cfg::kSpinnerFrameMs > 0,
              "Durations must be positive");
static_assert(cfg::kServoMin >= 1 && cfg::kServoMax <= 180 &&
              cfg::kServoMin < cfg::kServoNeutral && cfg::kServoNeutral < cfg::kServoMax,
              "Invalid servo range");
static_assert(cfg::kServoPin != cfg::kDisplayCs && cfg::kServoPin != cfg::kDisplayDc &&
              cfg::kServoPin != cfg::kDisplayReset && cfg::kServoPin != 11 &&
              cfg::kServoPin != 13, "Servo pin conflicts with display");
static_assert(cfg::kServoStepMs > 0, "Servo step delay must be positive");
static_assert(cfg::kLinkBlocks > 0 && cfg::kLinkSamplesPerBlock > 0,
              "Link test must contain samples");
static_assert(cfg::kSamplePeriodUs > cfg::kWireTimeoutUs && cfg::kWireTimeoutUs > 0 &&
              cfg::kSamplePeriodUs < 0x80000000UL, "Invalid sample period or timeout");
static_assert(cfg::kSensorAddress > 0 && cfg::kSensorAddress < 0x78,
              "Invalid I2C address");
static_assert(cfg::kI2cClockHz >= 5000 && cfg::kI2cClockHz <= 400000 &&
              cfg::kDiagnosticI2cHz >= 5000 && cfg::kDiagnosticI2cHz <= 400000,
              "Supported diagnostic I2C range: 5..400 kHz");
static_assert(cfg::kZeroWindowMs > 0 && cfg::kZeroTestDurationMs / cfg::kZeroWindowMs >= 3 &&
              cfg::kZeroTestDurationMs % cfg::kZeroWindowMs == 0,
              "Zero test requires at least three complete windows");
static_assert(cfg::kZeroSamplePeriodUs > cfg::kWireTimeoutUs &&
              cfg::kZeroPageBudgetUs > 0 && cfg::kZeroPageBudgetUs < cfg::kZeroSamplePeriodUs &&
              cfg::kZeroWindowMs <= 60000UL && cfg::kZeroTestDurationMs <= 600000UL &&
              cfg::kZeroSamplePeriodUs <= cfg::kZeroWindowMs * 1000UL / 2 &&
              cfg::kZeroWindowMs * 1000UL % cfg::kZeroSamplePeriodUs == 0,
              "Invalid zero sampling configuration");
static_assert(cfg::kSensorMaxPa > cfg::kSensorMinPa && cfg::kNoiseSpanLimitPa > 0 &&
              cfg::kNoiseSdLimitPa > 0 && cfg::kDriftRangeLimitPa > 0 &&
              cfg::kDriftSlopeLimitPaPerSec > 0, "Invalid pressure limits");
static_assert(cfg::kFlowWaitMs > cfg::kFlowHoldMs && cfg::kFlowHoldMs > 0 &&
              cfg::kFlowMinimumMs >= cfg::kFlowHoldMs && cfg::kFlowMinimumMs < cfg::kFlowWaitMs &&
              cfg::kFlowWaitMs <= 600000UL && cfg::kFlowSamplePeriodUs > cfg::kWireTimeoutUs &&
              cfg::kFlowSamplePeriodUs > cfg::kZeroPageBudgetUs &&
              cfg::kFlowSamplePeriodUs <= cfg::kFlowHoldMs * 1000UL / 2 &&
              cfg::kFlowThresholdPa > 0 && cfg::kAirDensityKgM3 > 0 &&
              cfg::kSpeedZeroBandPa >= 0 && cfg::kSpeedZeroBandPa < cfg::kFlowThresholdPa,
              "Invalid flow configuration");
static_assert(cfg::kReturnWaitMs > cfg::kReturnHoldMs && cfg::kReturnHoldMs > 0 &&
              cfg::kReturnWaitMs <= 600000UL && cfg::kReturnBandPa > 0 &&
              cfg::kReturnBandPa < cfg::kFlowThresholdPa &&
              cfg::kFlowSamplePeriodUs <= cfg::kReturnHoldMs * 1000UL / 2,
              "Invalid return configuration");
static_assert(cfg::kReconnectPollMs > 0 && cfg::kReconnectConfirmations > 0 &&
              cfg::kReconnectI2cHz >= 5000 && cfg::kReconnectI2cHz <= 400000,
              "Invalid reconnect configuration");

// _1_: страничный буфер вместо полного кадрового буфера.
U8G2_SH1106_128X64_NONAME_1_4W_HW_SPI display(
    cfg::kRotate180 ? U8G2_R2 : U8G2_R0,
    cfg::kDisplayCs, cfg::kDisplayDc, cfg::kDisplayReset);

bool finalFrameDrawn = false;
Servo wingServo;
enum LinkState : uint8_t { LINK_RUNNING, LINK_PASSED, LINK_FAILED };
enum LinkError : uint8_t { NO_ERROR, BUS_ERROR, TIMEOUT_ERROR, STALE_ERROR,
                          FAULT_ERROR, STATUS_ERROR, TIMING_ERROR };
LinkState linkState = LINK_RUNNING;
LinkError linkError = NO_ERROR;
LinkError firstLinkError = NO_ERROR;
uint32_t errorCounts[7] = {}; // Индекс = LinkError; отдельно от пропущенных слотов.
uint8_t firstErrorPacket[4] = {};
uint8_t firstErrorStatus = 255, firstErrorRx = 0;
uint32_t goodSamples = 0;
uint32_t attempts = 0;
uint32_t nextSampleUs = 0;
uint32_t missedSamples = 0;
uint32_t maxReadUs = 0;
uint16_t blockSamples = 0;
uint8_t completedBlocks = 0;
uint8_t lastStatus = 255;
uint8_t receivedBytes = 0;
uint16_t rawPressure = 0;
uint16_t rawTemperature = 0;
uint8_t lastPacket[4] = {}; // Последний полный пакет, в том числе с ошибочным статусом.
uint8_t diagnosticAck = 255; // 255: диагностика ещё не выполнялась.
bool zeroActive = false;
bool zeroReady = false;
uint32_t zeroStartedUs = 0, zeroLastFrameMs = 0;
uint32_t zeroMaxPageUs = 0, zeroMaxLateUs = 0, zeroMaxReadUs = 0;
uint32_t zeroFrameSeconds = 0;
uint8_t zeroFramePhase = 0;
bool zeroFrameActive = false;
ZeroStatistics zeroStats;
float zeroBaselinePa = 0;
bool flowActive = false;
bool returnActive = false, flowConfirmed = false;
bool frameFlowConfirmed = false;
uint32_t flowStartedUs = 0, flowSamples = 0;
float flowDeltaPa = 0, flowPeakPa = 0;
uint32_t flowFrameSpeed10 = 0;
FlowDetection flowDetection;
ReconnectDetection reconnectDetection;
uint32_t reconnectLastPollMs = 0;
constexpr uint32_t kLinkTotalSamples =
    static_cast<uint32_t>(cfg::kLinkBlocks) * cfg::kLinkSamplesPerBlock;

// AVR Wire.setClock() не выбирает предделитель: на 16 МГц 10000 переполняет TWBR.
// Настройка только между транзакциями. SCL=F_CPU/(16+2*TWBR*prescaler).
void setSensorClock(uint32_t frequency) {
  uint8_t prescalerBits = 0;
  uint16_t prescaler = 1;
  const uint32_t numerator = F_CPU / frequency - 16;
  while ((numerator + 2UL * prescaler - 1) / (2UL * prescaler) > 255 &&
         prescalerBits < 3) {
    ++prescalerBits;
    prescaler *= 4;
  }
  TWSR = (TWSR & ~(_BV(TWPS0) | _BV(TWPS1))) | prescalerBits;
  TWBR = (numerator + 2UL * prescaler - 1) / (2UL * prescaler);
}

// Общий вывод числовых полей уменьшает Flash без сокращения диагностики.
void __attribute__((noinline)) logNumber(const __FlashStringHelper *label, uint32_t value) {
  Serial.print(label);
  Serial.print(value);
}

// Постоянные сообщения храним во Flash; в RAM копируем только одну строку.
void drawFlash(uint8_t x, uint8_t y, const __FlashStringHelper *message) {
  char buffer[48];
  strncpy_P(buffer, reinterpret_cast<const char *>(message), sizeof(buffer) - 1);
  buffer[sizeof(buffer) - 1] = '\0';
  display.drawUTF8(x, y, buffer);
}

// Общий индикатор: кольцо из точек с вращающимся ярким хвостом.
// phase фиксируется до firstPage(), чтобы страницы одного кадра совпадали.
void drawSpinner(uint8_t cx, uint8_t cy, uint8_t phase) {
  const int8_t dx[8] = {0, 7, 10, 7, 0, -7, -10, -7};
  const int8_t dy[8] = {-10, -7, 0, 7, 10, 7, 0, -7};
  for (uint8_t i = 0; i < 8; ++i) {
    const uint8_t age = (phase + 8 - i) % 8;
    const uint8_t radius = age == 0 ? 3 : (age < 3 ? 2 : 1);
    if (age < 3) display.drawDisc(cx + dx[i], cy + dy[i], radius);
    else display.drawCircle(cx + dx[i], cy + dy[i], radius);
  }
}

// Только четыре буквы: крупный заголовок без второго большого кириллического шрифта.
const uint8_t kRejectGlyphs[4][7] PROGMEM = {
    {31, 16, 16, 30, 17, 17, 30}, // Б
    {30, 17, 17, 30, 16, 16, 16}, // Р
    {14, 17, 17, 31, 17, 17, 17}, // А
    {17, 18, 20, 24, 20, 18, 17}  // К
};

void drawRejectTitle() {
  for (uint8_t letter = 0; letter < 4; ++letter) {
    for (uint8_t row = 0; row < 7; ++row) {
      const uint8_t bits = pgm_read_byte(&kRejectGlyphs[letter][row]);
      for (uint8_t col = 0; col < 5; ++col) {
        if (bits & (16 >> col)) display.drawBox(18 + letter * 24 + col * 4,
                                               3 + row * 4, 4, 4);
      }
    }
  }
}

// Перенос причины по словам: 14 символов шрифта 9x15 помещаются в 128 пикселей.
void drawCenteredUtf8(uint8_t y, const char *message) {
  const int16_t width = display.getUTF8Width(message);
  display.drawUTF8(width < 128 ? (128 - width) / 2 : 0, y, message);
}

void drawCenteredFlash(uint8_t y, const __FlashStringHelper *message) {
  char buffer[64];
  strncpy_P(buffer, reinterpret_cast<const char *>(message), sizeof(buffer) - 1);
  buffer[sizeof(buffer) - 1] = '\0';
  drawCenteredUtf8(y, buffer);
}

void drawErrorReason(const __FlashStringHelper *reason) {
  char buffer[64];
  strncpy_P(buffer, reinterpret_cast<const char *>(reason), sizeof(buffer) - 1);
  buffer[sizeof(buffer) - 1] = '\0';
  uint8_t characters = 0, split = 0, position = 0;
  while (buffer[position] && characters < 14) {
    if (buffer[position] == ' ') split = position;
    ++position;
    while ((static_cast<uint8_t>(buffer[position]) & 0xC0) == 0x80) ++position;
    ++characters;
  }
  if (!buffer[position]) {
    drawCenteredUtf8(53, buffer);
    return;
  }
  if (!split) split = position;
  const bool space = buffer[split] == ' ';
  const char saved = buffer[split];
  buffer[split] = '\0';
  drawCenteredUtf8(46, buffer);
  buffer[split] = saved;
  drawCenteredUtf8(61, buffer + split + (space ? 1 : 0));
}

// Общий экран ошибки для текущего и будущих этапов.
void renderErrorScreen(const __FlashStringHelper *reason,
                       const __FlashStringHelper *detail = nullptr) {
  display.firstPage();
  do {
    drawRejectTitle();
    display.drawHLine(8, 32, 112);
    display.setFont(cfg::kUiFont);
    if (detail) {
      drawCenteredFlash(46, reason);
      drawCenteredFlash(61, detail);
    } else drawErrorReason(reason);
  } while (display.nextPage());
}

void renderLinkScreen() {
  if (linkState == LINK_FAILED) {
    const bool noPackets = attempts > 0 && goodSamples == 0 &&
        errorCounts[BUS_ERROR] + errorCounts[TIMEOUT_ERROR] == attempts;
    if (errorCounts[TIMING_ERROR]) {
      renderErrorScreen(F("Сбой тайминга теста"));
    } else if (noPackets) {
      renderErrorScreen(F("Нет пакетов"), F("от датчика"));
    } else {
      switch (firstLinkError) {
        case BUS_ERROR: renderErrorScreen(F("Потеря пакетов")); break;
        case TIMEOUT_ERROR: renderErrorScreen(F("Потеря пакетов"), F("Тайм-аут I2C")); break;
        case STALE_ERROR: renderErrorScreen(F("Старые данные")); break;
        case FAULT_ERROR: renderErrorScreen(F("Статус датчика: 3")); break;
        default: renderErrorScreen(F("Неверный статус")); break;
      }
    }
    return;
  }
  const uint8_t phase = (millis() / cfg::kSpinnerFrameMs) % 8;
  display.firstPage();
  do {
    display.setFont(cfg::kUiFont);
    drawFlash(0, 15, F("Проверка"));
    drawFlash(0, 32, F("связи..."));
    drawSpinner(111, 18, phase);
    drawCenteredFlash(61, F("Не дуйте"));
  } while (display.nextPage());
}

// Формат 4 байт и статусы — как в old/aspd_4525_test.ino.
bool readSensor() {
  linkError = NO_ERROR; // Ошибка прошлого чтения не должна блокировать новый пакет.
  lastStatus = 255;
  setSensorClock(cfg::kI2cClockHz); // Восстановить после возможного reset Wire при тайм-ауте.
  Wire.clearWireTimeoutFlag();
  receivedBytes = Wire.requestFrom(cfg::kSensorAddress, static_cast<uint8_t>(4));
  if (Wire.getWireTimeoutFlag()) linkError = TIMEOUT_ERROR;
  else if (receivedBytes != 4 || Wire.available() != 4) linkError = BUS_ERROR;
  if (linkError != NO_ERROR) {
    while (Wire.available()) Wire.read();
    return false;
  }
  const uint8_t a = Wire.read(), b = Wire.read(), c = Wire.read(), d = Wire.read();
  lastPacket[0] = a; lastPacket[1] = b; lastPacket[2] = c; lastPacket[3] = d;
  lastStatus = a >> 6;
  if (lastStatus == 2) linkError = STALE_ERROR;
  else if (lastStatus == 3) linkError = FAULT_ERROR;
  else if (lastStatus != 0) linkError = STATUS_ERROR;
  if (linkError != NO_ERROR) return false;
  rawPressure = (static_cast<uint16_t>(a & 0x3F) << 8) | b;
  rawTemperature = ((static_cast<uint16_t>(c) << 8) | d) >> 5;
  return true;
}

void recordLinkError() {
  ++errorCounts[linkError];
  if (firstLinkError == NO_ERROR) {
    firstLinkError = linkError;
    firstErrorStatus = lastStatus;
    firstErrorRx = receivedBytes;
    for (uint8_t i = 0; i < 4; ++i) firstErrorPacket[i] = lastPacket[i];
  }
}

// Вывод только между измеряемыми блоками либо после остановки.
void reportLink() {
  logNumber(F("LINK state="), static_cast<uint8_t>(linkState));
  logNumber(F(" error="), static_cast<uint8_t>(linkError));
  logNumber(F(" ok="), goodSamples);
  logNumber(F(" attempts="), attempts);
  logNumber(F(" missed="), missedSamples);
  logNumber(F(" bus="), errorCounts[BUS_ERROR]);
  logNumber(F(" timeout="), errorCounts[TIMEOUT_ERROR]);
  logNumber(F(" status="), lastStatus);
  logNumber(F(" rx="), receivedBytes);
  if (cfg::kDetailedSerialLog) {
  logNumber(F(" stale="), errorCounts[STALE_ERROR]);
  logNumber(F(" fault="), errorCounts[FAULT_ERROR]);
  logNumber(F(" bad_status="), errorCounts[STATUS_ERROR]);
  logNumber(F(" raw="), rawPressure);
  logNumber(F(" rawT="), rawTemperature);
  logNumber(F(" max_read_us="), maxReadUs);
  Serial.print(F(" packet="));
  if (receivedBytes == 4) {
    for (uint8_t i = 0; i < 4; ++i) {
      if (lastPacket[i] < 16) Serial.print('0');
      Serial.print(lastPacket[i], HEX); Serial.print(' ');
    }
  } else Serial.print(F("incomplete"));
  }
  Serial.println();
  if (cfg::kDetailedSerialLog && linkState == LINK_FAILED && firstLinkError != TIMING_ERROR) {
    logNumber(F("FIRST_ERROR status="), firstErrorStatus);
    logNumber(F(" rx="), firstErrorRx);
    Serial.print(F(" packet="));
    if (firstErrorRx == 4) {
      for (uint8_t i = 0; i < 4; ++i) {
        if (firstErrorPacket[i] < 16) Serial.print('0');
        Serial.print(firstErrorPacket[i], HEX); Serial.print(' ');
      }
    } else Serial.print(F("incomplete"));
    Serial.println();
  }
  Serial.flush();
}

// Только после провала: результаты не меняют оценку измеряемого теста.
void probeSensorAfterFailure() {
  setSensorClock(cfg::kI2cClockHz);
  Wire.clearWireTimeoutFlag();
  Wire.beginTransmission(cfg::kSensorAddress);
  const uint8_t ack = Wire.endTransmission();
  const bool ackTimeout = Wire.getWireTimeoutFlag();
  Wire.setWireTimeout(cfg::kDiagnosticTimeoutUs, true);
  setSensorClock(cfg::kDiagnosticI2cHz);
  Wire.clearWireTimeoutFlag();
  const uint8_t rx = Wire.requestFrom(cfg::kSensorAddress, static_cast<uint8_t>(4));
  const bool readTimeout = Wire.getWireTimeoutFlag();
  uint8_t status = 255;
  if (rx == 4 && Wire.available() == 4) status = Wire.read() >> 6;
  while (Wire.available()) Wire.read();
  logNumber(F("PROBE ack="), ack);
  logNumber(F(" ack_timeout="), ackTimeout);
  logNumber(F(" slow_rx="), rx);
  logNumber(F(" status="), status);
  Serial.print(F(" read_timeout=")); Serial.println(readTimeout);
  Serial.flush();
  Wire.setWireTimeout(cfg::kWireTimeoutUs, true);
  setSensorClock(cfg::kI2cClockHz);
  Wire.clearWireTimeoutFlag();
}

void diagnoseI2c() {
  Wire.setWireTimeout(cfg::kDiagnosticTimeoutUs, true);
  Wire.clearWireTimeoutFlag();
  Wire.beginTransmission(cfg::kSensorAddress);
  diagnosticAck = Wire.endTransmission();
  Serial.print(F("DIAG target=0x")); Serial.print(cfg::kSensorAddress, HEX);
  logNumber(F(" hz="), cfg::kI2cClockHz);
  logNumber(F(" ack="), diagnosticAck);
  Serial.print(F(" timeout=")); Serial.println(Wire.getWireTimeoutFlag());
  setSensorClock(cfg::kDiagnosticI2cHz);
  Serial.print(F("SCAN hz=")); Serial.println(cfg::kDiagnosticI2cHz);
  uint8_t found = 0;
  for (uint8_t address = 1; address < 0x78; ++address) {
    setSensorClock(cfg::kDiagnosticI2cHz);
    Wire.clearWireTimeoutFlag();
    Wire.beginTransmission(address);
    const uint8_t code = Wire.endTransmission();
    if (code == 0) {
      ++found;
      Serial.print(F("FOUND 0x")); Serial.println(address, HEX);
    }
  }
  Serial.print(F("SCAN found=")); Serial.println(found);
  setSensorClock(cfg::kDiagnosticI2cHz);
  Wire.clearWireTimeoutFlag();
  const uint8_t rx = Wire.requestFrom(cfg::kSensorAddress, static_cast<uint8_t>(4));
  logNumber(F("DIAG slow_rx="), rx);
  logNumber(F(" timeout="), Wire.getWireTimeoutFlag());
  Serial.print(F(" bytes="));
  while (Wire.available()) {
    Serial.print(Wire.read(), HEX); Serial.print(' ');
  }
  Serial.println();
  Serial.flush();
  setSensorClock(cfg::kI2cClockHz);
  Wire.setWireTimeout(cfg::kWireTimeoutUs, true);
  Wire.clearWireTimeoutFlag();
}

float pressurePa(uint16_t raw) {
  // Тип A: 10..90% от 16383 отсчётов, как в old/aspd_4525_test.ino.
  return (raw - 1638.3F) * (cfg::kSensorMaxPa - cfg::kSensorMinPa) /
      13106.4F + cfg::kSensorMinPa;
}

float zeroSdPa() {
  return sqrtf(zeroStats.variance());
}

float zeroSlope() { return zeroStats.slope(); }

void reportZero() {
  logNumber(F("ZERO ready="), zeroReady);
  logNumber(F(" n="), zeroStats.samples);
  logNumber(F(" windows="), zeroStats.windows);
  Serial.print(F(" mean_pa=")); Serial.print(zeroStats.mean, 3);
  Serial.print(F(" span_pa=")); Serial.print(zeroStats.maximum - zeroStats.minimum, 3);
  Serial.print(F(" sd_pa=")); Serial.print(zeroSdPa(), 3);
  Serial.print(F(" window_span_pa=")); Serial.print(zeroStats.maxMean - zeroStats.minMean, 3);
  // При ready=1 сохранённый ноль равен mean_pa; отдельный дубль не выводим.
  Serial.print(F(" slope_pa_s=")); Serial.println(zeroSlope(), 3);
  logNumber(F("ZERO_TIMING late_us="), zeroMaxLateUs);
  logNumber(F(" read_us="), zeroMaxReadUs);
  Serial.print(F(" page_us=")); Serial.println(zeroMaxPageUs);
  Serial.flush();
}

void beginZeroFrame() {
  const uint32_t elapsedMs = (micros() - (flowActive ? flowStartedUs : zeroStartedUs)) / 1000UL;
  const uint32_t durationMs = flowActive ? (returnActive ? cfg::kReturnWaitMs : cfg::kFlowWaitMs) :
      cfg::kZeroTestDurationMs;
  const uint32_t remainingMs = elapsedMs < durationMs ? durationMs - elapsedMs : 0;
  zeroFrameSeconds = (remainingMs + 999) / 1000;
  frameFlowConfirmed = flowConfirmed;
  if (flowActive) {
    const float dp = fabsf(flowDeltaPa);
    const float speed = dp <= cfg::kSpeedZeroBandPa ? 0 : sqrtf(2 * dp / cfg::kAirDensityKgM3);
    flowFrameSpeed10 = static_cast<uint32_t>(speed * (cfg::kSpeedInKmh ? 36.0F : 10.0F) + 0.5F);
  }
  zeroFramePhase = (millis() / cfg::kSpinnerFrameMs) % 8;
  zeroFrameActive = true;
  display.firstPage();
}

// Общий формат текущей и максимальной скорости; буфер вызывающей стороны — 24 байта.
void formatSpeed(char *line, uint32_t speed10) {
  ultoa(speed10 / 10, line, 10);
  uint8_t end = strlen(line);
  line[end++] = '.';
  line[end++] = '0' + speed10 % 10;
  strcpy_P(line + end, cfg::kSpeedInKmh ? PSTR(" км/ч") : PSTR(" м/с"));
}

// Одна страница за вызов: между страницами loop() снова проверяет срок опроса.
void renderZeroPage() {
  const uint32_t startedUs = micros();
  char line[24]; // uint32 digits, decimal digit and UTF-8 unit fit without truncation.
  if (flowActive) {
    formatSpeed(line, flowFrameSpeed10);
  } else {
    ultoa(zeroFrameSeconds, line, 10);
    strcat_P(line, PSTR(" c"));
  }
  display.setFont(cfg::kUiFont);
  if (flowActive) {
    // Три раздельные строки: подсказка, скорость, таймер. Между ними есть отступы.
    drawFlash(0, 15, returnActive ? F("Не дуйте") :
        (frameFlowConfirmed ? F("Принято") : F("Дуйте")));
    drawSpinner(111, 13, zeroFramePhase);
    drawCenteredUtf8(40, line);
    display.drawHLine(16, 45, 96);
    ultoa(zeroFrameSeconds, line, 10);
    strcat_P(line, PSTR(" c"));
    drawCenteredUtf8(61, line);
  } else {
    drawFlash(0, 15, F("Проверка"));
    drawFlash(0, 32, F("нуля..."));
    drawSpinner(111, 18, zeroFramePhase);
    drawCenteredUtf8(46, line);
    drawCenteredFlash(61, F("Не дуйте"));
  }
  zeroFrameActive = display.nextPage();
  const uint32_t pageUs = micros() - startedUs;
  if (pageUs > zeroMaxPageUs) zeroMaxPageUs = pageUs;
}

void serviceZeroScreen() {
  if (!zeroFrameActive && millis() - zeroLastFrameMs >= cfg::kDisplayUpdateMs) {
    zeroLastFrameMs = millis();
    beginZeroFrame();
  }
  uint32_t budget = cfg::kZeroPageBudgetUs;
  if (zeroMaxPageUs + 1000 > budget) budget = zeroMaxPageUs + 1000;
  if (zeroFrameActive && static_cast<int32_t>(nextSampleUs - micros()) >
      static_cast<int32_t>(budget)) renderZeroPage();
}

void startZeroTest() {
  zeroStats = ZeroStatistics();
  zeroBaselinePa = 0;
  zeroActive = true;
  zeroReady = false;
  zeroStartedUs = micros();
  zeroLastFrameMs = millis();
  beginZeroFrame();
  while (zeroFrameActive) renderZeroPage();
  // Отсчёт 10 секунд начинается после начального вывода, без исключённых пауз.
  zeroStartedUs = micros();
  nextSampleUs = zeroStartedUs + cfg::kZeroSamplePeriodUs;
}

void failZeroTest(const __FlashStringHelper *reason) {
  zeroActive = false;
  zeroFrameActive = false;
  finalFrameDrawn = true;
  renderErrorScreen(reason);
  Serial.print(F("ZERO_FAIL reason=")); Serial.println(reason);
  reportZero();
}

void reportFlow(bool passed) {
  Serial.print(returnActive ? F("RETURN passed=") : F("FLOW passed=")); Serial.print(passed);
  logNumber(F(" n="), flowSamples);
  if (!returnActive) { logNumber(F(" confirmed="), flowConfirmed); }
  Serial.print(F(" dp_pa=")); Serial.print(flowDeltaPa, 2);
  Serial.print(F(" peak_pa=")); Serial.println(flowPeakPa, 2);
  Serial.flush();
}

void failFlowTest(const __FlashStringHelper *reason) {
  flowActive = false;
  zeroFrameActive = false;
  finalFrameDrawn = true;
  renderErrorScreen(reason);
  Serial.print(returnActive ? F("RETURN_FAIL reason=") : F("FLOW_FAIL reason=")); Serial.println(reason);
  reportFlow(false);
}

void failMeasurementRead() {
  const __FlashStringHelper *reason = F("Неверный статус");
  switch (linkError) {
    case BUS_ERROR: reason = F("Потеря пакетов"); break;
    case TIMEOUT_ERROR: reason = F("Тайм-аут I2C"); break;
    case STALE_ERROR: reason = F("Старые данные"); break;
    case FAULT_ERROR: reason = F("Статус датчика: 3"); break;
    default: break;
  }
  if (flowActive) failFlowTest(reason);
  else failZeroTest(reason);
  logNumber(F("READ error="), static_cast<uint8_t>(linkError));
  logNumber(F(" status="), lastStatus);
  Serial.print(F(" rx=")); Serial.println(receivedBytes);
}

void startFlowTest() {
  flowActive = true;
  returnActive = false;
  flowConfirmed = false;
  finalFrameDrawn = false;
  flowStartedUs = micros();
  flowDetection = FlowDetection();
  flowDeltaPa = flowPeakPa = 0;
  flowSamples = 0;
  zeroLastFrameMs = millis();
  beginZeroFrame();
  while (zeroFrameActive) renderZeroPage();
  flowStartedUs = micros(); // Окно ожидания начинается после вывода инструкции.
  nextSampleUs = flowStartedUs + cfg::kFlowSamplePeriodUs;
}

void showPeakSpeed() {
  char line[24];
  const float speed = flowPeakPa <= cfg::kSpeedZeroBandPa ? 0 :
      sqrtf(2 * flowPeakPa / cfg::kAirDensityKgM3);
  formatSpeed(line, static_cast<uint32_t>(speed * (cfg::kSpeedInKmh ? 36.0F : 10.0F) + 0.5F));
  zeroFrameActive = false;
  display.firstPage();
  do {
    display.setFont(cfg::kUiFont);
    drawCenteredFlash(15, F("Максимальная"));
    drawCenteredFlash(32, F("скорость"));
    display.drawHLine(16, 38, 96);
    drawCenteredUtf8(58, line);
  } while (display.nextPage());
  delay(cfg::kPeakSpeedDisplayMs); // Расписание возврата создаётся после этого экрана.
}

void startReturnTest() {
  returnActive = true;
  flowDetection = FlowDetection();
  flowSamples = 0;
  flowPeakPa = 0;
  flowStartedUs = micros();
  zeroLastFrameMs = millis();
  beginZeroFrame();
  while (zeroFrameActive) renderZeroPage();
  flowStartedUs = micros();
  nextSampleUs = flowStartedUs + cfg::kFlowSamplePeriodUs;
}

void renderPassedScreen() {
  // Крупные ОК тем же способом, что и БРАК, без второго шрифта.
  const uint8_t glyphs[2][7] = {{14, 17, 17, 17, 17, 17, 14}, {17, 18, 20, 24, 20, 18, 17}};
  display.firstPage();
  do {
    for (uint8_t letter = 0; letter < 2; ++letter) {
      for (uint8_t row = 0; row < 7; ++row) {
        for (uint8_t col = 0; col < 5; ++col) {
          if (glyphs[letter][row] & (16 >> col))
            display.drawBox(36 + letter * 30 + col * 5, 2 + row * 5, 5, 5);
        }
      }
    }
    display.setFont(cfg::kUiFont);
    drawCenteredFlash(61, F("Тест пройден"));
  } while (display.nextPage());
}

void sampleFlow() {
  const uint32_t startedUs = micros();
  if (startedUs - nextSampleUs >= cfg::kFlowSamplePeriodUs) {
    failFlowTest(F("Сбой тайминга теста"));
    return;
  }
  nextSampleUs += cfg::kFlowSamplePeriodUs;
  if (!readSensor()) {
    failMeasurementRead();
    return;
  }
  if (static_cast<int32_t>(micros() - nextSampleUs) >= 0) {
    failFlowTest(F("Сбой тайминга теста"));
    return;
  }
  ++flowSamples;
  flowDeltaPa = pressurePa(rawPressure) - zeroBaselinePa;
  const float magnitude = fabsf(flowDeltaPa);
  if (magnitude > flowPeakPa) flowPeakPa = magnitude;
  const uint32_t elapsedMs = (startedUs - flowStartedUs) / 1000UL;
  if (returnActive) {
    if (flowDetection.update(magnitude <= cfg::kReturnBandPa, elapsedMs, cfg::kReturnHoldMs)) {
      flowActive = false;
      zeroFrameActive = false;
      finalFrameDrawn = true;
      reportFlow(true);
      renderPassedScreen();
    } else if (elapsedMs >= cfg::kReturnWaitMs) failFlowTest(F("Нет возврата к нулю"));
  } else {
    const bool aboveThreshold = (cfg::kFlowAllowNegative ? magnitude : flowDeltaPa) >= cfg::kFlowThresholdPa;
    flowConfirmed = flowDetection.updateLatched(aboveThreshold, elapsedMs, cfg::kFlowHoldMs);
    if (flowConfirmed && elapsedMs >= cfg::kFlowMinimumMs) {
      reportFlow(true);
      showPeakSpeed();
      startReturnTest();
    } else if (elapsedMs >= cfg::kFlowWaitMs) failFlowTest(F("Нет реакции"));
  }
}

void finishZeroWindow() {
  zeroStats.finishWindow(cfg::kZeroWindowMs / 1000.0F);
}

void sampleZero() {
  const uint32_t startedUs = micros();
  const uint32_t lateUs = startedUs - nextSampleUs;
  if (lateUs > zeroMaxLateUs) zeroMaxLateUs = lateUs;
  if (lateUs >= cfg::kZeroSamplePeriodUs) {
    failZeroTest(F("Сбой тайминга теста"));
    return;
  }
  nextSampleUs += cfg::kZeroSamplePeriodUs;
  const bool valid = readSensor();
  const uint32_t readUs = micros() - startedUs;
  if (readUs > zeroMaxReadUs) zeroMaxReadUs = readUs;
  if (!valid) {
    // На этапе нуля любая ошибка останавливает тест, даже в диагностическом режиме связи.
    failMeasurementRead();
    return;
  }
  if (static_cast<int32_t>(micros() - nextSampleUs) >= 0) {
    failZeroTest(F("Сбой тайминга теста"));
    return;
  }
  const float p = pressurePa(rawPressure);
  zeroStats.add(p);
  const uint32_t samplesPerWindow = cfg::kZeroWindowMs * 1000UL / cfg::kZeroSamplePeriodUs;
  if (zeroStats.windowSamples == samplesPerWindow) finishZeroWindow();
  if (zeroStats.maximum - zeroStats.minimum > cfg::kNoiseSpanLimitPa) {
    failZeroTest(F("Шум датчика"));
    return;
  }
  if (zeroStats.windows >= 3 && zeroStats.maxMean - zeroStats.minMean > cfg::kDriftRangeLimitPa) {
    failZeroTest(F("Уход нуля"));
    return;
  }
  const uint32_t totalSamples = cfg::kZeroTestDurationMs * 1000UL / cfg::kZeroSamplePeriodUs;
  if (zeroStats.samples == totalSamples) {
    if (zeroSdPa() > cfg::kNoiseSdLimitPa) failZeroTest(F("Шум датчика"));
    else if (fabsf(zeroSlope()) > cfg::kDriftSlopeLimitPaPerSec)
      failZeroTest(F("Уход нуля"));
    else {
      zeroBaselinePa = zeroStats.mean; // Только после успешного теста, больше не подстраивать.
      zeroReady = true;
      zeroActive = false;
      zeroFrameActive = false;
      reportZero();
      startFlowTest();
    }
    return;
  }
}

void stopLinkTest(bool passed) {
  linkState = passed ? LINK_PASSED : LINK_FAILED;
  if (!passed) renderLinkScreen(); // Показать ошибку до медленного поиска адресов.
  reportLink();
  if (passed) {
    startZeroTest();
    return;
  }
  if (cfg::kProbeOnLinkFailure &&
      (errorCounts[BUS_ERROR] || errorCounts[TIMEOUT_ERROR])) probeSensorAfterFailure();
  if (!passed && cfg::kDiagnoseI2cOnFailure &&
      firstLinkError != TIMING_ERROR) diagnoseI2c();
  finalFrameDrawn = true;
}

void sampleLink() {
  const uint32_t startedUs = micros();
  const uint32_t lateUs = startedUs - nextSampleUs;
  if (lateUs >= cfg::kSamplePeriodUs) {
    missedSamples += lateUs / cfg::kSamplePeriodUs;
    linkError = TIMING_ERROR;
    recordLinkError();
    stopLinkTest(false);
    return;
  }
  nextSampleUs += cfg::kSamplePeriodUs;
  ++attempts;
  const bool valid = readSensor();
  const uint32_t readUs = micros() - startedUs;
  if (readUs > maxReadUs) maxReadUs = readUs;
  if (!valid) {
    recordLinkError();
    if (!cfg::kLinkDiagnosticMode) {
      stopLinkTest(false);
      return;
    }
  } else ++goodSamples;
  // В диагностике границы блоков считаем по всем попыткам, включая ошибочные.
  ++blockSamples;
  if (blockSamples == cfg::kLinkSamplesPerBlock) {
    blockSamples = 0;
    ++completedBlocks;
    // Последнее чтение тоже должно закончиться до следующего слота.
    if (static_cast<int32_t>(micros() - nextSampleUs) >= 0) {
      linkError = TIMING_ERROR;
      recordLinkError();
      ++missedSamples;
      stopLinkTest(false);
      return;
    }
    if (completedBlocks == cfg::kLinkBlocks) {
      stopLinkTest(firstLinkError == NO_ERROR && missedSamples == 0);
      return;
    }
    reportLink();
    renderLinkScreen();
    // Явная пауза между блоками не относится к измеряемому расписанию.
    nextSampleUs = micros() + cfg::kSamplePeriodUs;
  }
}

// Экран перед проходом рисуем целиком один раз: во время движения нет SPI-отрисовки.
void renderServoScreen() {
  display.firstPage();
  do {
    display.setFont(cfg::kUiFont);
    drawCenteredFlash(14, F("ТЕСТ СЕРВО"));
    display.drawHLine(8, 20, 112);
    drawCenteredFlash(36, F("Проход крыла"));
    display.drawHLine(20, 45, 89);
    for (uint8_t i = 0; i < 5; ++i) {
      display.drawPixel(20 + i, 45 - i);
      display.drawPixel(20 + i, 45 + i);
      display.drawPixel(108 - i, 45 - i);
      display.drawPixel(108 - i, 45 + i);
    }
    display.drawDisc(64, 45, 3);
    drawCenteredFlash(63, F("Оцените ход"));
  } while (display.nextPage());
}

// Проверенная последовательность old/PITOT_SERVO_TEST.ino; лицензия рядом.
void traceInit(const __FlashStringHelper *message);
void runLegacyServoTest() {
  traceInit(F("SERVO begin"));
  renderServoScreen();
  traceInit(F("SERVO screen"));
  int pos = cfg::kServoNeutral;
  wingServo.write(pos);
  wingServo.attach(cfg::kServoPin);
  traceInit(F("SERVO active"));
  delay(cfg::kServoNeutralHoldMs);
  for (; pos <= cfg::kServoMax; ++pos) {
    wingServo.write(pos);
    delay(cfg::kServoStepMs);
  }
  for (pos = cfg::kServoMax; pos >= cfg::kServoMin; --pos) {
    wingServo.write(pos);
    delay(cfg::kServoStepMs);
  }
  // Сохранён исходный возврат с min-1 (19° при стандартных настройках).
  for (; pos <= cfg::kServoNeutral; ++pos) {
    wingServo.write(pos);
    delay(cfg::kServoStepMs);
  }
  // Сохраняем импульсы в нейтрали после прохода, как в оригинале.
  traceInit(F("SERVO done"));
}
// Новый полный сеанс серво и Пито без сброса Nano.
void restartPitotFlow() {
  linkState = LINK_RUNNING;
  linkError = firstLinkError = NO_ERROR;
  memset(errorCounts, 0, sizeof(errorCounts));
  memset(firstErrorPacket, 0, sizeof(firstErrorPacket));
  memset(lastPacket, 0, sizeof(lastPacket));
  firstErrorStatus = lastStatus = diagnosticAck = 255;
  firstErrorRx = receivedBytes = 0;
  goodSamples = attempts = missedSamples = maxReadUs = 0;
  blockSamples = completedBlocks = 0;
  rawPressure = rawTemperature = 0;
  zeroActive = zeroReady = zeroFrameActive = false;
  zeroMaxPageUs = zeroMaxLateUs = zeroMaxReadUs = 0;
  // Статистика, кадр и часы этапов инициализируются в startZeroTest/startFlowTest.
  zeroBaselinePa = 0;
  flowActive = returnActive = flowConfirmed = frameFlowConfirmed = false;
  reconnectDetection = ReconnectDetection();
  finalFrameDrawn = false;
  Wire.setWireTimeout(cfg::kWireTimeoutUs, true);
  Wire.clearWireTimeoutFlag();
  setSensorClock(cfg::kI2cClockHz);
  Serial.println(F("RESTART pitot-connected"));
  Serial.flush();
  if (cfg::kServoTestEnabled) runLegacyServoTest();
  delay(cfg::kSensorStartupMs);
  renderLinkScreen();
  nextSampleUs = micros() + cfg::kSamplePeriodUs;
}

void monitorReconnect() {
  const uint32_t nowMs = millis();
  if (nowMs - reconnectLastPollMs < cfg::kReconnectPollMs) return;
  reconnectLastPollMs = nowMs;
  Wire.clearWireTimeoutFlag();
  setSensorClock(cfg::kReconnectI2cHz);
  Wire.beginTransmission(cfg::kSensorAddress);
  const uint8_t code = Wire.endTransmission();
  const uint8_t state = Wire.getWireTimeoutFlag() ? 2 : (code == 0 ? 1 : (code == 2 ? 0 : 2));
  if (reconnectDetection.update(state, cfg::kReconnectConfirmations)) restartPitotFlow();
}

// Общий вывод меток запуска экономит Flash и завершает передачу перед следующим шагом.
void __attribute__((noinline)) traceInit(const __FlashStringHelper *message) {
  Serial.println(message);
  Serial.flush();
}

void setup() {
  Serial.begin(cfg::kSerialBaud);
  Serial.println(F("BOOT wing-tester peak-speed-v1"));
  Serial.print(F("RESET mcusr="));
  Serial.println(startupResetFlags, HEX);
  traceInit(F("INIT oled_begin"));
  display.begin();
  display.setContrast(cfg::kDisplayContrast);
  traceInit(F("INIT oled_done"));
  if (cfg::kServoTestEnabled) {
    runLegacyServoTest();
  }
  Wire.begin();
  setSensorClock(cfg::kI2cClockHz);
  Wire.setWireTimeout(cfg::kWireTimeoutUs, true);
  delay(cfg::kSensorStartupMs);
  renderLinkScreen();
  traceInit(F("INIT ready"));
  nextSampleUs = micros() + cfg::kSamplePeriodUs;
}

void loop() {
  if (finalFrameDrawn) {
    if (cfg::kAutoRestartOnReconnect) monitorReconnect();
    return;
  }
  if (static_cast<int32_t>(micros() - nextSampleUs) >= 0) {
    if (flowActive) sampleFlow();
    else if (zeroActive) sampleZero();
    else sampleLink();
  }
  if ((zeroActive || flowActive) && !finalFrameDrawn) serviceZeroScreen();
}
