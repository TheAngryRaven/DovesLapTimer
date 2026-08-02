/**
 * CrossingEngine - the reusable line-crossing detection core.
 *
 * The detection + interpolation bodies are extracted verbatim from
 * DovesLapTimer (_detectLineCrossing / interpolateCrossingPoint /
 * interpolateWeight / catmullRom) — the Layer-2 synthetic tests and the
 * Layer-3 NMEA replay goldens pin the numeric behavior across the move.
 */

#include "CrossingEngine.h"

#define debugln debug_println
#define debug debug_print

CrossingEngine::CrossingEngine(double crossingThresholdMeters, Stream *debugSerial) {
  this->crossingThresholdMeters = crossingThresholdMeters;
  _serial = debugSerial;
  memset(crossingPointBuffer, 0, sizeof(crossingPointBuffer));
}

void CrossingEngine::setThreshold(double meters) {
  crossingThresholdMeters = meters;
}

void CrossingEngine::setDebugSerial(Stream *debugSerial) {
  _serial = debugSerial;
}

void CrossingEngine::setForceLinear(bool linear) {
  forceLinear = linear;
}

bool CrossingEngine::getForceLinear() const {
  return forceLinear;
}

unsigned int CrossingEngine::getRejectedCrossingCount() const {
  return rejectedCrossingCount;
}

void CrossingEngine::reset() {
  rejectedCrossingCount = 0;
  crossingPointBufferIndex = 0;
  crossingPointBufferFull = false;
  memset(crossingPointBuffer, 0, sizeof(crossingPointBuffer));
}

/**
 * Crossing-line detection: hypotenuse threshold method.
 *
 * Using the width of the crossing line and "crossingThresholdMeters" to form
 * a right triangle, the calculated hypotenuse is the effective proximity
 * threshold. We measure from the driver to each crossing point; if either
 * distance exceeds the hypotenuse we are not in the zone.
 *
 * Earlier experiments used acute/obtuse-triangle detection (see
 * DovesLapTimer::isObtuseTriangle), which worked on OKC but felt brittle.
 * Hypotenuse-based threshold has been more reliable across short and long
 * track configurations.
 */
LineDetectResult CrossingEngine::detect(
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
    double& outOdometer) {
  double distToLine = INFINITY;

  if (crossingFlag || geoInsideLineThreshold(crossingThresholdMeters, currentLat, currentLng, pointALat, pointALng, pointBLat, pointBLng)) {
    distToLine = geoPointLineSegmentDistance(currentLat, currentLng, pointALat, pointALng, pointBLat, pointBLng);
  }

  if (crossingFlag) {
    if (distToLine > crossingThresholdMeters + 1) {
      // Exited the zone — interpolate, reset buffer, report completion.
      debug(F("Line "));
      debug(lineLabel);
      debugln(F(" crossed, calculating..."));
      crossingFlag = false;

      // Include the exiting fix itself: at low GPS rates (1-5 Hz) the line
      // is often crossed between the last in-zone fix and this one, and
      // without it the buffer holds no straddling pair at all. At high
      // rates it sits beyond the (earlier) genuine pair and is ignored.
      crossingPointBuffer[crossingPointBufferIndex].lat = currentLat;
      crossingPointBuffer[crossingPointBufferIndex].lng = currentLng;
      crossingPointBuffer[crossingPointBufferIndex].time = currentTimeMs;
      crossingPointBuffer[crossingPointBufferIndex].odometer = currentOdometer;
      crossingPointBuffer[crossingPointBufferIndex].speedKmh = currentSpeedKmh;
      crossingPointBufferIndex = (crossingPointBufferIndex + 1) % crossingPointBufferSize;
      if (crossingPointBufferIndex == 0) crossingPointBufferFull = true;

      outLat = 0.0; outLng = 0.0; outOdometer = 0.0; outTime = 0;
      bool validCrossing = interpolateCrossingPoint(outLat, outLng, outTime, outOdometer,
                                                    pointALat, pointALng, pointBLat, pointBLng);

      if (validCrossing) {
        debug(F("  crossingLat: "));  debugln(outLat, 6);
        debug(F("  crossingLng: "));  debugln(outLng, 6);
        debug(F("  crossingOdometer: ")); debugln(outOdometer);
        debug(F("  crossingTime: ")); debugln(outTime);
      } else {
        // Surface the failure — debug serial is usually not connected on
        // track, and a silently swallowed crossing looks like a dead lap
        // counter to the user.
        rejectedCrossingCount++;
      }

      crossingPointBufferIndex = 0;
      crossingPointBufferFull = false;
      memset(crossingPointBuffer, 0, sizeof(crossingPointBuffer));
      // An invalid interpolation (no straddling pair found, or an incoherent
      // one) is reported as NONE so callers never consume garbage out-params.
      // A legitimate crossing at exactly 00:00:00.000 (outTime == 0) is valid.
      return validCrossing ? LINE_DETECT_COMPLETED : LINE_DETECT_NONE;
    }

    // Still in zone — buffer this fix.
    crossingPointBuffer[crossingPointBufferIndex].lat = currentLat;
    crossingPointBuffer[crossingPointBufferIndex].lng = currentLng;
    crossingPointBuffer[crossingPointBufferIndex].time = currentTimeMs;
    crossingPointBuffer[crossingPointBufferIndex].odometer = currentOdometer;
    crossingPointBuffer[crossingPointBufferIndex].speedKmh = currentSpeedKmh;
    crossingPointBufferIndex = (crossingPointBufferIndex + 1) % crossingPointBufferSize;
    if (crossingPointBufferIndex == 0) crossingPointBufferFull = true;

    debug(F("Line "));
    debug(lineLabel);
    debug(F(" distToLine: "));
    debug(distToLine);
    debug(F(" | buffering index["));
    debug(crossingPointBufferIndex);
    debug(F("] full["));
    debugln(crossingPointBufferFull ? F("True") : F("False"));
    return LINE_DETECT_IN_ZONE;
  }

  if (distToLine < crossingThresholdMeters) {
    // Entering the zone for the first time this approach.
    debug(F("Entering line "));
    debug(lineLabel);
    debugln(F(" crossing zone"));
    crossingFlag = true;

    // Insert previous GPS fix as pre-crossing point — gives Catmull-Rom its p0.
    if (prevFix != NULL) {
      crossingPointBuffer[crossingPointBufferIndex] = *prevFix;
      crossingPointBufferIndex = (crossingPointBufferIndex + 1) % crossingPointBufferSize;
    }

    // Capture current point.
    crossingPointBuffer[crossingPointBufferIndex].lat = currentLat;
    crossingPointBuffer[crossingPointBufferIndex].lng = currentLng;
    crossingPointBuffer[crossingPointBufferIndex].time = currentTimeMs;
    crossingPointBuffer[crossingPointBufferIndex].odometer = currentOdometer;
    crossingPointBuffer[crossingPointBufferIndex].speedKmh = currentSpeedKmh;
    crossingPointBufferIndex = (crossingPointBufferIndex + 1) % crossingPointBufferSize;
    return LINE_DETECT_IN_ZONE;
  }

  return LINE_DETECT_NONE;
}

double CrossingEngine::interpolateWeight(double distA, double distB, float speedA, float speedB) {
  // Guard against division by zero - if either speed is essentially zero,
  // fall back to pure distance-based weighting
  const float minSpeed = 0.001f;  // ~0.0005 knots, effectively stationary
  if (speedA < minSpeed || speedB < minSpeed) {
    // Pure distance-based interpolation: weight is proportion of distA to total
    double totalDist = distA + distB;
    if (totalDist < 1e-9) {
      return 0.5;  // Both distances zero, use midpoint
    }
    return distA / totalDist;
  }

  double weightedDistA = distA / speedA;
  double weightedDistB = distB / speedB;

  // Guard against sum being zero (shouldn't happen with above checks, but defensive)
  double weightedSum = weightedDistA + weightedDistB;
  if (weightedSum < 1e-9) {
    return 0.5;
  }

  return weightedDistA / weightedSum;
}

double CrossingEngine::catmullRom(double p0, double p1, double p2, double p3, double t) {
  // Calculate t^2 and t^3
  double t2 = t * t;
  double t3 = t2 * t;

  // Calculate the Catmull-Rom coefficients a, b, c, and d
  double a = -0.5 * p0 + 1.5 * p1 - 1.5 * p2 + 0.5 * p3;
  double b = p0 - 2.5 * p1 + 2 * p2 - 0.5 * p3;
  double c = -0.5 * p0 + 0.5 * p2;
  double d = p1;

  // Calculate and return the interpolated value using the coefficients and powers of t
  return a * t3 + b * t2 + c * t + d;
}

bool CrossingEngine::interpolateCrossingPoint(double& crossingLat, double& crossingLng, unsigned long& crossingTime, double& crossingOdometer, double pointALat, double pointALng, double pointBLat, double pointBLng) {
  int numPoints = crossingPointBufferFull ? crossingPointBufferSize : crossingPointBufferIndex;

  // The buffer is circular: once it wraps (e.g. a kart parked on the grid
  // inside the zone), physical index order no longer equals chronological
  // order, and the seam pair (newest next to oldest) would masquerade as a
  // crossing. Walk entries in chronological order instead.
  const int oldestIndex = crossingPointBufferFull ? crossingPointBufferIndex : 0;
  auto entryAt = [&](int k) -> const crossingPointBufferEntry& {
    return crossingPointBuffer[(oldestIndex + k) % crossingPointBufferSize];
  };

  // Find the first pair of consecutive buffer points on opposite sides of the crossing line.
  // In a normal pass there is exactly one such pair - the two GPS fixes that straddle the line.
  int crossingIndexA = -1;
  int crossingIndexB = -1;
  double crossingSumDistances = INFINITY;

  for (int i = 0; i < numPoints - 1; i++) {
    double distA = geoPointLineSegmentDistance(entryAt(i).lat, entryAt(i).lng, pointALat, pointALng, pointBLat, pointBLng);
    double distB = geoPointLineSegmentDistance(entryAt(i + 1).lat, entryAt(i + 1).lng, pointALat, pointALng, pointBLat, pointBLng);

    int sideA = geoPointOnSideOfLine(entryAt(i).lat, entryAt(i).lng, pointALat, pointALng, pointBLat, pointBLng);
    int sideB = geoPointOnSideOfLine(entryAt(i + 1).lat, entryAt(i + 1).lng, pointALat, pointALng, pointBLat, pointBLng);

    debug(F("i: "));
    debug(i);
    debug(F(" : distA: "));
    debug(distA);
    debug(F(" : sideA: "));
    debug(sideA);
    debug(F(" : distB: "));
    debug(distB);
    debug(F(" sideB: "));
    debug(sideB);
    debug(F(" sum: "));
    debugln(distA + distB, 2);

    // First pair on opposite sides of the line = the crossing pair
    if (sideA != sideB) {
      crossingIndexA = i;
      crossingIndexB = i + 1;
      crossingSumDistances = distA + distB;
      debug(F("crossing pair found, sum: "));
      debugln(crossingSumDistances, 2);
      break;
    }
  }
  debug(F("crossingSumDistances: "));
  debugln(crossingSumDistances);

  if (crossingIndexA == -1 || crossingIndexB == -1) {
    debugln(F("~~~ INVALID CROSSING ~~~ INVALID CROSSING ~~~ INVALID CROSSING ~~~ INVALID CROSSING ~~~"));
    return false;
  }

  {
    const crossingPointBufferEntry& entryA = entryAt(crossingIndexA);
    const crossingPointBufferEntry& entryB = entryAt(crossingIndexB);

    // Validate: the crossing pair must hug the line *relative to the GPS
    // sample spacing*, not in absolute meters. The old absolute check
    // (sum < crossingThresholdMeters) conflated zone size with sample
    // density: at 1 Hz / 70 km/h consecutive fixes are ~19 m apart, every
    // genuine crossing failed the 7 m bound, and the lap counter silently
    // never incremented. For a pair genuinely straddling the line, the sum
    // of the two distances can never exceed the pair's own spacing, so a
    // spacing-scaled bound stays tight at any sample rate.
    double pairSpacing = geoHaversine(entryA.lat, entryA.lng, entryB.lat, entryB.lng);
    double allowedSum = CROSSING_PAIR_SPACING_FACTOR * pairSpacing;
    if (allowedSum < crossingThresholdMeters) {
      allowedSum = crossingThresholdMeters;
    }
    if (crossingSumDistances > allowedSum) {
      debugln(F("~~~ INVALID CROSSING: pair too far from line ~~~"));
      return false;
    }

    // Time deltas are computed in double, normalized across the UTC midnight
    // wrap, and sanity-clamped BEFORE any conversion back to unsigned long —
    // an out-of-range double-to-unsigned conversion is undefined behavior.
    double deltaTime = (double)entryB.time - (double)entryA.time;
    if (deltaTime < 0) {
      deltaTime += (double)DOVES_MILLIS_PER_DAY;
    }
    if (deltaTime > (double)CROSSING_MAX_FIX_GAP_MS) {
      // The straddling pair is not temporally coherent (GPS time step during
      // re-acquisition, or stale buffer contents) — refuse to interpolate.
      debugln(F("~~~ INVALID CROSSING: fix gap too large ~~~"));
      return false;
    }

    // Compute the interpolation factor (t) from distances and speeds at the crossing pair
    double distA = geoPointLineSegmentDistance(entryA.lat, entryA.lng, pointALat, pointALng, pointBLat, pointBLng);
    double distB = geoPointLineSegmentDistance(entryB.lat, entryB.lng, pointALat, pointALng, pointBLat, pointBLng);
    double t = interpolateWeight(distA, distB, entryA.speedKmh, entryB.speedKmh);

    // Geometric sanity: the interpolated point must land on the crossing
    // line *segment* (within the threshold), not on its infinite extension
    // — pointOnSideOfLine treats the line as infinite, so a pair straddling
    // the extension beyond an endpoint would otherwise slip through now
    // that the sum bound scales with fix spacing.
    double linearLat = entryA.lat + t * (entryB.lat - entryA.lat);
    double linearLng = entryA.lng + t * (entryB.lng - entryA.lng);
    if (geoPointLineSegmentDistance(linearLat, linearLng, pointALat, pointALng, pointBLat, pointBLng) > crossingThresholdMeters) {
      debugln(F("~~~ INVALID CROSSING: crossing point off the line segment ~~~"));
      return false;
    }

    debugln(F("~~~ VALID CROSSING ~~~"));

    // Time and odometer are always interpolated linearly (monotonic values that
    // should not overshoot), regardless of the interpolation mode for position.
    double deltaOdometer = entryB.odometer - entryA.odometer;
    crossingOdometer = entryA.odometer + t * deltaOdometer;
    double crossingTimeMs = (double)entryA.time + t * deltaTime;
    if (crossingTimeMs >= (double)DOVES_MILLIS_PER_DAY) {
      crossingTimeMs -= (double)DOVES_MILLIS_PER_DAY;  // zone straddled midnight
    }
    crossingTime = (unsigned long)crossingTimeMs;

    if (forceLinear) {
      crossingLat = linearLat;
      crossingLng = linearLng;
    } else {
      // Catmull-Rom spline interpolation for position only.
      // Requires 4 control points: p0 (before A), p1 (A), p2 (B), p3 (after B)
      bool canUseCatmullRom = (crossingIndexA >= 1) && (crossingIndexB <= numPoints - 2);

      if (!canUseCatmullRom) {
        // Not enough points for Catmull-Rom, fall back to linear
        debugln(F("Catmull-Rom: insufficient control points, using linear fallback"));
        crossingLat = linearLat;
        crossingLng = linearLng;
      } else {
        // We have 4 valid control points for Catmull-Rom
        int index0 = crossingIndexA - 1;
        int index1 = crossingIndexA;
        int index2 = crossingIndexB;
        int index3 = crossingIndexB + 1;

        debugln(F("Catmull-Rom: using spline interpolation"));
        debug(F("  indices: "));
        debug(index0);
        debug(F(", "));
        debug(index1);
        debug(F(", "));
        debug(index2);
        debug(F(", "));
        debugln(index3);

        // Catmull-Rom for lat/lng only - spline smoothing helps with curved paths
        crossingLat = catmullRom(entryAt(index0).lat, entryAt(index1).lat, entryAt(index2).lat, entryAt(index3).lat, t);
        crossingLng = catmullRom(entryAt(index0).lng, entryAt(index1).lng, entryAt(index2).lng, entryAt(index3).lng, t);
      }
    }
    return true;
  }
}
