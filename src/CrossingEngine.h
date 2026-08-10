/**
 * CrossingEngine - the reusable line-crossing detection core.
 *
 * Owns the in-zone GPS ring buffer and the crossing interpolation math that
 * previously lived inside DovesLapTimer (_detectLineCrossing +
 * interpolateCrossingPoint). Extracted so point-to-point timers
 * (SprintTimer) can reuse the exact same battle-tested pipeline — including
 * with independent per-line buffers — without duplicating the math.
 *
 * The engine is deliberately stateless about *which* line it is checking:
 * line endpoints and the per-line in-zone flag are passed per call, exactly
 * like the old private helper. One engine can therefore be shared across
 * several lines (DovesLapTimer shares one across S/F + S2 + S3, with the
 * caller enforcing only-one-line-crossing-at-a-time), or dedicated to a
 * single line (SprintTimer gives every line its own engine, so start and
 * finish zones may overlap without contention).
 *
 * See DETECTION.md for the algorithm itself.
 */

#ifndef _DOVES_CROSSING_ENGINE_H
#define _DOVES_CROSSING_ENGINE_H

#include <Arduino.h>
// DovesDebug.h pulls in the ArxTypeTraits std::forward polyfill before any
// std:: use, so this header is safe to include in any order relative to
// DovesLapTimer.h.
#include "DovesDebug.h"
#include "GeoMath.h"

#define CROSSING_LINE_SIDE_A -1
#define CROSSING_LINE_SIDE_EXACT 0
#define CROSSING_LINE_SIDE_B 1

// GPS time base: milliseconds since UTC midnight, wraps 86,399,999 -> 0
#define DOVES_MILLIS_PER_DAY  86400000UL

// Max believable gap between the two consecutive buffered fixes that straddle
// a crossing line. A larger gap means the GPS time base stepped (e.g. u-blox
// re-acquisition) and the pair cannot be interpolated coherently.
#define CROSSING_MAX_FIX_GAP_MS  10000UL

// The straddling pair's summed distance to the line is validated against
// max(crossingThresholdMeters, factor * pair spacing) so that low-rate GPS
// (widely spaced fixes) doesn't fail validation purely on sample density.
#define CROSSING_PAIR_SPACING_FACTOR  1.25

/**
 * @brief Elapsed milliseconds between two milliseconds-since-midnight timestamps.
 *
 * Normalizes across the UTC midnight wrap (86,399,999 -> 0), which happens
 * mid-evening across the Americas. Without this, a lap straddling midnight
 * underflows unsigned subtraction into a ~4.29-billion-ms "lap time".
 *
 * @param startMs Earlier timestamp, in ms since midnight.
 * @param endMs Later timestamp, in ms since midnight.
 * @return Elapsed time in ms, wrap-normalized.
 */
static inline unsigned long timeSinceMidnightDelta(unsigned long startMs, unsigned long endMs) {
  if (endMs < startMs) {
    endMs += DOVES_MILLIS_PER_DAY;
  }
  return endMs - startMs;
}

// Outcome of a single CrossingEngine::detect pass.
enum LineDetectResult {
  LINE_DETECT_NONE,         // not in / near the zone, or exited with an invalid interpolation
  LINE_DETECT_IN_ZONE,      // inside the zone (entered this fix or continuing)
  LINE_DETECT_COMPLETED,    // exited the zone this fix — interpolated outputs valid
};

struct crossingPointBufferEntry {
  double lat; // latitude
  double lng; // longitude
  unsigned long time; // current time in milliseconds
  float odometer; // time traveled since device start and this entry
  float speedKmh; // speed in kmph
};

class CrossingEngine {
public:
  CrossingEngine(double crossingThresholdMeters = 7, Stream *debugSerial = NULL);

  /**
   * @brief Sets the crossing zone threshold, in meters.
   */
  void setThreshold(double meters);
  /**
   * @brief Attaches / detaches a debug output stream.
   */
  void setDebugSerial(Stream *debugSerial);
  /**
   * @brief Selects linear (true, default) or Catmull-Rom (false) position
   * interpolation. Time and odometer are always interpolated linearly.
   */
  void setForceLinear(bool linear);
  bool getForceLinear() const;
  /**
   * @brief Number of crossing-zone exits whose interpolation was rejected.
   *
   * A rejection means the GPS data inside the zone never produced a usable
   * straddling pair (no side change, pair too far from the line, incoherent
   * timestamps, or crossing landing off the line segment).
   */
  unsigned int getRejectedCrossingCount() const;
  /**
   * @brief Clears the buffer, the rejected-crossing counter, and any
   * in-flight zone state. Does NOT clear the configured threshold/mode.
   */
  void reset();

  /**
   * @brief The crossing-zone state machine (formerly
   * DovesLapTimer::_detectLineCrossing). Detects entry, buffers GPS fixes
   * inside the zone, and interpolates the exact crossing point on exit.
   *
   * The caller owns the per-line in-zone flag and the vehicle state
   * (current fix + previous-fix snapshot) — the engine owns only the
   * buffer and the math.
   *
   * @param currentLat / currentLng Current GPS position.
   * @param currentTimeMs Milliseconds since UTC midnight for this fix.
   * @param currentOdometer Total odometer reading at this fix, meters.
   * @param currentSpeedKmh Speed at this fix, km/h.
   * @param prevFix Previous fix snapshot (Catmull-Rom pre-crossing point),
   *   or NULL if no previous fix exists yet.
   * @param pointALat / pointALng Line endpoint A.
   * @param pointBLat / pointBLng Line endpoint B.
   * @param crossingFlag Reference to the caller-owned per-line in-zone flag.
   * @param lineLabel Debug label: 0 = start(/finish), 1 = sprint finish,
   *   2 / 3 = sector.
   * @param[out] outLat / outLng / outTime / outOdometer Interpolated
   *   crossing point — only valid when the return is LINE_DETECT_COMPLETED.
   * @return LINE_DETECT_NONE / IN_ZONE / COMPLETED. See enum docs.
   */
  LineDetectResult detect(
      double currentLat, double currentLng,
      unsigned long currentTimeMs,
      float currentOdometer,
      float currentSpeedKmh,
      const crossingPointBufferEntry *prevFix,
      double pointALat, double pointALng,
      double pointBLat, double pointBLng,
      bool& crossingFlag,
      int lineLabel,
      double& outLat, double& outLng,
      unsigned long& outTime,
      double& outOdometer);

private:
  // Shared debug members — see DovesDebug.h for the DOVES_DISABLE_DEBUG
  // kill-switch rationale.
  DOVES_DEBUG_TEMPLATES()

  /**
   * @brief Finds the straddling fix pair in the buffer and interpolates the
   * crossing point (chronologically unwinding the ring if it wrapped).
   * @return True if a valid crossing was found and the out-params populated.
   */
  bool interpolateCrossingPoint(double& crossingLat, double& crossingLng, unsigned long& crossingTime, double& crossingOdometer, double pointALat, double pointALng, double pointBLat, double pointBLng);
  /**
   * @brief Computes the interpolation weight based on distances and speeds.
   */
  double interpolateWeight(double distA, double distB, float speedA, float speedB);
  /**
   * @brief Catmull-Rom spline interpolation between two points.
   */
  double catmullRom(double p0, double p1, double p2, double p3, double t);

  Stream *_serial;
  double crossingThresholdMeters;
  bool forceLinear = true;
  unsigned int rejectedCrossingCount = 0;

  // Buffer sizing: AVR exposes RAMEND/RAMSTART so we can tell a Mega (8KB) from a Uno (2KB).
  // On modern 32-bit cores (nRF52, ESP32, SAMD, RP2040, etc.) those macros are undefined
  // and would silently evaluate to 0 - defaulting such targets to the small buffer would
  // defeat the whole point of the hotfix. Assume anyone not on classic AVR has plenty of RAM.
  #if defined(RAMEND) && defined(RAMSTART)
    #if ((RAMEND - RAMSTART) > 3000)
      static const int crossingPointBufferSize = 100;
    #else
      static const int crossingPointBufferSize = 25;
    #endif
  #else
    static const int crossingPointBufferSize = 100;
  #endif

  crossingPointBufferEntry crossingPointBuffer[crossingPointBufferSize];
  int crossingPointBufferIndex = 0;
  bool crossingPointBufferFull = false;
};

#endif
