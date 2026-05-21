/**
 * Host-build mock of Arduino.h for layer-2 unit tests.
 *
 * The library's Arduino surface is tiny — Stream, F(), and sq() — so this
 * stub stays well under 100 lines. Debug print calls become no-ops on host
 * so tests don't spew the library's verbose Serial output.
 *
 * Found via the test/ -I path before any real Arduino headers, which the
 * host compiler doesn't have anyway.
 */

#ifndef ARDUINO_H
#define ARDUINO_H

#include <cstdint>
#include <cstddef>
#include <cmath>
#include <cstring>
#include <utility>

#ifndef NULL
#define NULL nullptr
#endif

// Arduino's F() macro stores string literals in flash on AVR. On host it
// can be the identity — string literals already live in .rodata.
#define F(s) (s)

// Arduino's sq() macro. Library uses it in pointLineSegmentDistance.
#define sq(x) ((x) * (x))

// Minimal Stream stub. The library only ever calls print() / println()
// through templated debug helpers, so accept anything and discard it.
class Stream {
public:
  template <typename... Args>
  void print(Args&&...) {}
  template <typename... Args>
  void println(Args&&...) {}
  void println() {}
};

#endif
