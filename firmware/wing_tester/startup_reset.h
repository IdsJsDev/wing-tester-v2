#pragma once

#include <avr/io.h>
#include <avr/wdt.h>

// Run before global constructors; an inherited watchdog can expire in OLED reset delays.
// Bootloaders may clear MCUSR before entering the sketch, so zero means unknown.
uint8_t startupResetFlags __attribute__((section(".noinit")));

extern "C" void captureStartupReset(void)
    __attribute__((naked, used, section(".init3")));
extern "C" void captureStartupReset(void) {
  startupResetFlags = MCUSR;
  MCUSR = 0;
  wdt_disable();
}
