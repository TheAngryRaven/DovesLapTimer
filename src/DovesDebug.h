/**
 * DovesDebug.h - the shared debug-print member templates for every Doves*
 * class. Each class expands DOVES_DEBUG_TEMPLATES() in a private section;
 * the class must have a `Stream* _serial` member.
 *
 * Compile-time kill switch: the runtime _serial null-check keeps every debug
 * string and print call-site in flash on production builds that never attach
 * a debug Stream. Defining DOVES_DISABLE_DEBUG (e.g. -DDOVES_DISABLE_DEBUG
 * in build flags) swaps the templates for empty inlines so the compiler
 * drops it all — several KB on a fully-featured firmware. Debug behavior is
 * unchanged when the macro is not defined.
 */

#ifndef _DOVES_DEBUG_H
#define _DOVES_DEBUG_H

#include <Arduino.h>
// Polyfills std::forward (used by the templates below) on cores with no C++
// standard library — classic AVR.
#include "ArxTypeTraits.h"

#ifdef DOVES_DISABLE_DEBUG
#define DOVES_DEBUG_TEMPLATES()                                     \
  template<typename... Args> void debug_print(Args&&...) {}         \
  template<typename... Args> void debug_println(Args&&...) {}
#else
#define DOVES_DEBUG_TEMPLATES()                                     \
  template<typename... Args>                                        \
  void debug_print(Args&&... args) {                                \
    if (_serial) { _serial->print(std::forward<Args>(args)...); }   \
  }                                                                 \
  template<typename... Args>                                        \
  void debug_println(Args&&... args) {                              \
    if (_serial) { _serial->println(std::forward<Args>(args)...); } \
  }
#endif

#endif
