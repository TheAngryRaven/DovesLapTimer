/**
 * GPS-based lap timing library for go-kart and racing applications.
 * This library does NOT interface with your GPS, simply feed it data and check the state.
 * Supports start/finish line detection, 3-sector split timing, pace comparison, and distance tracking.
 *
 * The development of this library has been overseen, and all documentation has been generated using chatGPT4.
 */

#include "DovesLapTimer.h"
#include "GeoMath.h"

#define debugln debug_println
#define debug debug_print

DovesLapTimer::DovesLapTimer(double crossingThresholdMeters, Stream *debugSerial) {
  this->crossingThresholdMeters = crossingThresholdMeters;

  if (debugSerial == NULL) {
    _serial = nullptr;
  } else {
    _serial = debugSerial;
  }
}

int DovesLapTimer::loop(double currentLat, double currentLng, float currentAltitudeMeters, float currentSpeedKnots) {
  // Reject invalid fixes before they can poison the odometer or timing state.
  // NaN/Inf and (0,0) fixes are routine parser output during fix loss; a
  // single one would otherwise stick in totalDistanceTraveled forever.
  if (!geoCoordinatesValid(currentLat, currentLng)) {
    debugln(F("Rejected invalid GPS coordinates"));
    return -1;
  }
  // Altitude and speed are auxiliary — sanitize rather than drop the fix.
  if (!geoIsFinite(currentAltitudeMeters)) {
    currentAltitudeMeters = positionPrevAlt;
  }
  if (!geoIsFinite(currentSpeedKnots) || currentSpeedKnots < 0) {
    currentSpeedKnots = 0;
  }

  // Update Odometer - only calculate distance if we have a previous position
  if (firstPositionReceived) {
    double jumpDistance = this->haversine(positionPrevLat, positionPrevLng, currentLat, currentLng);
    if (jumpDistance > GPS_MAX_PLAUSIBLE_JUMP_METERS) {
      consecutiveJumpCount++;
      if (consecutiveJumpCount < GPS_JUMP_REACCEPT_COUNT) {
        // Almost certainly a teleport glitch — drop the fix entirely.
        debugln(F("Rejected implausible GPS jump"));
        return -1;
      }
      // Several consecutive far fixes: the new position is real (signal
      // re-acquisition / device moved). Re-seed without crediting the gap
      // to the odometer.
      consecutiveJumpCount = 0;
    } else {
      consecutiveJumpCount = 0;
      // TODO: I think alt is messing up, investigate more... maybe flag?
      double distanceTraveledSinceLastUpdate = this->haversine3D(
        positionPrevLat,
        positionPrevLng,
        positionPrevAlt,
        currentLat,
        currentLng,
        currentAltitudeMeters
      );
      totalDistanceTraveled += distanceTraveledSinceLastUpdate;
    }
  } else {
    firstPositionReceived = true;
  }
  positionPrevLat = currentLat;
  positionPrevLng = currentLng;
  positionPrevAlt = currentAltitudeMeters;

  // update current speed
  currentSpeedkmh = currentSpeedKnots * GEOMATH_KNOTS_TO_KMH;

  // run calculations for each crossing-line
  // Only one line can be "crossing" at a time due to shared buffer.
  // Mutual exclusion: skip start/finish if a sector crossing is active, and vice versa.

  bool nearAnyLine = false;

  // Check start/finish line - requires a configured line (otherwise the
  // endpoint members would be meaningless zeros), and skip if a sector
  // crossing is already in progress
  if (startFinishLineConfigured && (crossing || (!crossingSector2 && !crossingSector3))) {
    if (this->checkStartFinish(currentLat, currentLng)) {
      nearAnyLine = true;
    }
  }

  // Check sector lines if configured and not currently crossing start/finish
  if (areSectorLinesConfigured() && !crossing) {
    // Check sector 2 line
    if (sector2LineConfigured && !crossingSector2 && !crossingSector3) {
      if (checkSectorLine(currentLat, currentLng,
                          sector2PointALat, sector2PointALng,
                          sector2PointBLat, sector2PointBLng,
                          crossingSector2, 2)) {
        nearAnyLine = true;
      }
    } else if (crossingSector2) {
      // Continue processing sector 2 crossing
      if (checkSectorLine(currentLat, currentLng,
                          sector2PointALat, sector2PointALng,
                          sector2PointBLat, sector2PointBLng,
                          crossingSector2, 2)) {
        nearAnyLine = true;
      }
    }

    // Check sector 3 line
    if (sector3LineConfigured && !crossingSector2 && !crossingSector3) {
      if (checkSectorLine(currentLat, currentLng,
                          sector3PointALat, sector3PointALng,
                          sector3PointBLat, sector3PointBLng,
                          crossingSector3, 3)) {
        nearAnyLine = true;
      }
    } else if (crossingSector3) {
      // Continue processing sector 3 crossing
      if (checkSectorLine(currentLat, currentLng,
                          sector3PointALat, sector3PointALng,
                          sector3PointBLat, sector3PointBLng,
                          crossingSector3, 3)) {
        nearAnyLine = true;
      }
    }
  }

  // Save current fix as previous for next iteration's Catmull-Rom pre-crossing point
  prevFixLat = currentLat;
  prevFixLng = currentLng;
  prevFixTime = millisecondsSinceMidnight;
  prevFixOdometer = totalDistanceTraveled;
  prevFixSpeedKmh = currentSpeedkmh;
  hasPrevFix = true;

  return nearAnyLine ? 0 : -1;
}

// TODO: update function to be a bit more portable to allow for split timing
bool DovesLapTimer::checkStartFinish(double currentLat, double currentLng) {
  double cLat = 0.0, cLng = 0.0, cOdo = 0.0;
  unsigned long cTime = 0;

  LineDetectResult ev = _detectLineCrossing(
      currentLat, currentLng,
      startFinishPointALat, startFinishPointALng,
      startFinishPointBLat, startFinishPointBLng,
      crossing,
      0,
      cLat, cLng, cTime, cOdo);

  if (ev == LINE_DETECT_COMPLETED) {
    if (raceStarted) {
      laps++;
      unsigned long lapTime = timeSinceMidnightDelta(currentLapStartTime, cTime);
      double lapDistance = cOdo - currentLapOdometerStart;

      debug(F("Lap Finish Time: "));
      debug(lapTime);
      debug(F(" : "));
      debugln((double)(lapTime / 1000.0), 3);

      lastLapTime = lapTime;
      lastLapDistance = lapDistance;
      if (bestLapTime <= 0 || lastLapTime < bestLapTime) {
        bestLapTime = lastLapTime;
        bestLapDistance = lastLapDistance;
        bestLapNumber = laps;
      }
    } else {
      raceStarted = true;
      debugln(F("Race Started"));
    }
    currentLapStartTime = cTime;
    currentLapOdometerStart = cOdo;
    handleLineCrossing(cTime, 0);
  }

  return ev == LINE_DETECT_IN_ZONE;
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
 * isObtuseTriangle), which worked on OKC but felt brittle. Hypotenuse-based
 * threshold has been more reliable across short and long track configurations.
 */
LineDetectResult DovesLapTimer::_detectLineCrossing(
    double currentLat, double currentLng,
    double pointALat, double pointALng,
    double pointBLat, double pointBLng,
    bool& crossingFlag,
    int lineLabel,
    double& outLat, double& outLng,
    unsigned long& outTime,
    double& outOdometer) {
  double distToLine = INFINITY;

  if (crossingFlag || insideLineThreshold(currentLat, currentLng, pointALat, pointALng, pointBLat, pointBLng)) {
    distToLine = pointLineSegmentDistance(currentLat, currentLng, pointALat, pointALng, pointBLat, pointBLng);
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
      crossingPointBuffer[crossingPointBufferIndex].time = millisecondsSinceMidnight;
      crossingPointBuffer[crossingPointBufferIndex].odometer = totalDistanceTraveled;
      crossingPointBuffer[crossingPointBufferIndex].speedKmh = currentSpeedkmh;
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
    crossingPointBuffer[crossingPointBufferIndex].time = millisecondsSinceMidnight;
    crossingPointBuffer[crossingPointBufferIndex].odometer = totalDistanceTraveled;
    crossingPointBuffer[crossingPointBufferIndex].speedKmh = currentSpeedkmh;
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
    if (hasPrevFix) {
      crossingPointBuffer[crossingPointBufferIndex].lat = prevFixLat;
      crossingPointBuffer[crossingPointBufferIndex].lng = prevFixLng;
      crossingPointBuffer[crossingPointBufferIndex].time = prevFixTime;
      crossingPointBuffer[crossingPointBufferIndex].odometer = prevFixOdometer;
      crossingPointBuffer[crossingPointBufferIndex].speedKmh = prevFixSpeedKmh;
      crossingPointBufferIndex = (crossingPointBufferIndex + 1) % crossingPointBufferSize;
    }

    // Capture current point.
    crossingPointBuffer[crossingPointBufferIndex].lat = currentLat;
    crossingPointBuffer[crossingPointBufferIndex].lng = currentLng;
    crossingPointBuffer[crossingPointBufferIndex].time = millisecondsSinceMidnight;
    crossingPointBuffer[crossingPointBufferIndex].odometer = totalDistanceTraveled;
    crossingPointBuffer[crossingPointBufferIndex].speedKmh = currentSpeedkmh;
    crossingPointBufferIndex = (crossingPointBufferIndex + 1) % crossingPointBufferSize;
    return LINE_DETECT_IN_ZONE;
  }

  return LINE_DETECT_NONE;
}

bool DovesLapTimer::insideLineThreshold(double driverLat, double driverLon, double crossingPointALat, double crossingPointALon, double crossingPointBLat, double crossingPointBLon) {
  // Calculate the distance from the driver to crossing points A and B
  double driverLengthA = haversine(driverLat, driverLon, crossingPointALat, crossingPointALon);
  double driverLengthB = haversine(driverLat, driverLon, crossingPointBLat, crossingPointBLon);

  // Calculate the distance between crossing points A and B
  double crossingLineLength = haversine(crossingPointALat, crossingPointALon, crossingPointBLat, crossingPointBLon);

  // Calculate the maximum allowed distance from the driver to the line formed by crossing points A and B
  double maxLineLength = sqrt(sq(crossingThresholdMeters) + sq(crossingLineLength));

  // // dbg
  // debug(F("crossingLineLength: "));
  // debug(crossingLineLength, 2);
  // debug(F(" | maxLineLength: "));
  // debug(maxLineLength, 2);
  // debug(F(" | driverLengthA: "));
  // debug(driverLengthA, 2);
  // debug(F(" | driverLengthB: "));
  // debug(driverLengthB, 2);
  // // dbg

  // Check if the driver is within the threshold distance from the line formed by crossing points A and B
  return driverLengthA < maxLineLength && driverLengthB < maxLineLength;
}

bool DovesLapTimer::isObtuseTriangle(double lat1, double lon1, double lat2, double lon2, double lat3, double lon3) {
  // Get side lengths
  double a = haversine(lat1, lon1, lat2, lon2);
  double b = haversine(lat1, lon1, lat3, lon3);
  double c = haversine(lat2, lon2, lat3, lon3);

  // Sort the sides in ascending order
  if (a > b) std::swap(a, b);
  if (b > c) std::swap(b, c);
  if (a > b) std::swap(a, b);

  // listen... this has been a long debugging session
  if ( a + b <= c ) {
    // debugln(F("triangle: Impossible"));
    return false;
  } else {
    TRITYPE discriminant = a * a + b * b - c * c;
    if (discriminant < 0) {
      // debugln(F("triangle: Obtuse"));
      return true;
    } else if (discriminant > 0) {
      // debugln(F("triangle: Acute"));
      return false;
    } else {
      // debugln(F("triangle: Right Angled"));
      return false;
    }
  }
}

int DovesLapTimer::pointOnSideOfLine(double driverLat, double driverLng, double pointALat, double pointALng, double pointBLat, double pointBLng) {
  double lineDirectionX = pointBLat - pointALat;
  double lineDirectionY = pointBLng - pointALng;
  double driverToPointAX = driverLat - pointALat;
  double driverToPointAY = driverLng - pointALng;

  double crossProduct = lineDirectionX * driverToPointAY - lineDirectionY * driverToPointAX;

  // todo: defines?
  if (crossProduct > 0) {
    return CROSSING_LINE_SIDE_A; // Driver is on one side of the line
  } else if (crossProduct < 0) {
    return CROSSING_LINE_SIDE_B; // Driver is on the other side of the line
  } else {
    return CROSSING_LINE_SIDE_EXACT; // Driver is exactly on the line
  }
}

double DovesLapTimer::pointLineSegmentDistance(double pointX, double pointY, double startX, double startY, double endX, double endY) {
  double dx = endX - startX;
  double dy = endY - startY;
  double segmentLengthSquared = dx * dx + dy * dy;

  // Use epsilon comparison for floating-point near-zero check
  // This handles degenerate line segments (start == end)
  if (segmentLengthSquared < 1e-12) {
    // The line segment is actually a point (or nearly so)
    return haversine(pointX, pointY, startX, startY);
  }

  double projectionScalar = ((pointX - startX) * dx + (pointY - startY) * dy) / segmentLengthSquared;

  double haversineStart = haversine(pointX, pointY, startX, startY);
  double haversineEnd = haversine(pointX, pointY, endX, endY);

  if (projectionScalar < 0.0) {
    // The projection of the point is outside the line segment, closest to the start point
    return haversineStart;
  } else if (projectionScalar > 1.0) {
    // The projection of the point is outside the line segment, closest to the end point
    return haversineEnd;
  }

  // The projection of the point is within the line segment
  double projectedX = startX + projectionScalar * dx;
  double projectedY = startY + projectionScalar * dy;
  return haversine(pointX, pointY, projectedX, projectedY);
}

double DovesLapTimer::haversine(double lat1, double lon1, double lat2, double lon2) {
  return geoHaversine(lat1, lon1, lat2, lon2);
}

double DovesLapTimer::haversine3D(double prevLat, double prevLng, double prevAlt, double currentLat, double currentLng, double currentAlt) {
  return geoHaversine3D(prevLat, prevLng, prevAlt, currentLat, currentLng, currentAlt);
}

/////////// private functions

double DovesLapTimer::interpolateWeight(double distA, double distB, float speedA, float speedB) {
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
double DovesLapTimer::catmullRom(double p0, double p1, double p2, double p3, double t) {
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

bool DovesLapTimer::interpolateCrossingPoint(double& crossingLat, double& crossingLng, unsigned long& crossingTime, double& crossingOdometer, double pointALat, double pointALng, double pointBLat, double pointBLng) {
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
    double distA = pointLineSegmentDistance(entryAt(i).lat, entryAt(i).lng, pointALat, pointALng, pointBLat, pointBLng);
    double distB = pointLineSegmentDistance(entryAt(i + 1).lat, entryAt(i + 1).lng, pointALat, pointALng, pointBLat, pointBLng);

    int sideA = pointOnSideOfLine(entryAt(i).lat, entryAt(i).lng, pointALat, pointALng, pointBLat, pointBLng);
    int sideB = pointOnSideOfLine(entryAt(i + 1).lat, entryAt(i + 1).lng, pointALat, pointALng, pointBLat, pointBLng);

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
    double pairSpacing = haversine(entryA.lat, entryA.lng, entryB.lat, entryB.lng);
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
    double distA = pointLineSegmentDistance(entryA.lat, entryA.lng, pointALat, pointALng, pointBLat, pointBLng);
    double distB = pointLineSegmentDistance(entryB.lat, entryB.lng, pointALat, pointALng, pointBLat, pointBLng);
    double t = interpolateWeight(distA, distB, entryA.speedKmh, entryB.speedKmh);

    // Geometric sanity: the interpolated point must land on the crossing
    // line *segment* (within the threshold), not on its infinite extension
    // — pointOnSideOfLine treats the line as infinite, so a pair straddling
    // the extension beyond an endpoint would otherwise slip through now
    // that the sum bound scales with fix spacing.
    double linearLat = entryA.lat + t * (entryB.lat - entryA.lat);
    double linearLng = entryA.lng + t * (entryB.lng - entryA.lng);
    if (pointLineSegmentDistance(linearLat, linearLng, pointALat, pointALng, pointBLat, pointBLng) > crossingThresholdMeters) {
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

/////////// direction detection

void DirectionDetector::onLineCrossing(int sectorNumber, unsigned long crossingTime) {
  if (sectorNumber == 0) {
    // Start/finish crossed. If we've collected both S2 and S3 timestamps in
    // the current lap window, resolve direction from their temporal order.
    // Single-sector laps (driver missed a poorly-placed line, or GPS rate
    // too low to catch the zone) are discarded and re-tried next lap.
    if (raceSeen && direction == DIR_UNKNOWN
        && lapS2CrossingTime != 0 && lapS3CrossingTime != 0) {
      direction = (lapS2CrossingTime < lapS3CrossingTime) ? DIR_FORWARD : DIR_REVERSE;
    }
    lapS2CrossingTime = 0;
    lapS3CrossingTime = 0;
    raceSeen = true;
    return;
  }

  if (direction != DIR_UNKNOWN) {
    return;
  }

  if (!raceSeen) {
    return;
  }

  // Latest crossing wins inside a lap so a phantom glitch early in the lap
  // gets overwritten by the real crossing later on.
  if (sectorNumber == 2) {
    lapS2CrossingTime = crossingTime;
  } else if (sectorNumber == 3) {
    lapS3CrossingTime = crossingTime;
  }
}

/////////// sector timing helper methods

void DovesLapTimer::handleLineCrossing(unsigned long crossingTime, int sectorNumber) {
  if (!areSectorLinesConfigured()) {
    // If sector lines not configured, just handle start/finish as before
    return;
  }

  // Feed direction detector with raw (physical) sector number and crossing
  // time so it can compare S2 vs S3 timestamps to infer direction.
  _directionDetector.onLineCrossing(sectorNumber, crossingTime);

  // Remap sector number for reverse direction (swap 2<->3)
  int effectiveSector = sectorNumber;
  if (_directionDetector.isReverse() && sectorNumber >= 2) {
    effectiveSector = (sectorNumber == 2) ? 3 : 2;
    debug(F("Direction reverse: physical S"));
    debug(sectorNumber);
    debug(F(" -> logical S"));
    debugln(effectiveSector);
  }

  if (effectiveSector == 0) {
    // Crossing start/finish line
    if (raceStarted && currentSector == 3) {
      // Completing sector 3 and finishing lap
      currentLapSector3Time = timeSinceMidnightDelta(currentSectorStartTime, crossingTime);

      debug(F("Sector 3 Time: "));
      debugln(currentLapSector3Time);

      // Update best sectors
      updateBestSectors();
    }

    // Start sector 1
    currentSector = 1;
    currentSectorStartTime = crossingTime;
    currentLapSector1Time = 0;
    currentLapSector2Time = 0;
    currentLapSector3Time = 0;

    debug(F("Starting Sector 1"));
    debugln();

  } else if (effectiveSector == 2) {
    // Crossing sector 2 line (logical)
    if (currentSector == 1) {
      // Completing sector 1, starting sector 2
      currentLapSector1Time = timeSinceMidnightDelta(currentSectorStartTime, crossingTime);
      currentSector = 2;
      currentSectorStartTime = crossingTime;

      debug(F("Sector 1 Time: "));
      debug(currentLapSector1Time);
      debug(F(" : "));
      debugln((double)(currentLapSector1Time/1000.0), 3);
    } else {
      // Out of order crossing - invalidate lap
      debug(F("WARNING: Sector 2 crossed out of order (current sector: "));
      debug(currentSector);
      debugln(F(")"));
      currentSector = 0;  // Invalidate
    }

  } else if (effectiveSector == 3) {
    // Crossing sector 3 line (logical)
    if (currentSector == 2) {
      // Completing sector 2, starting sector 3
      currentLapSector2Time = timeSinceMidnightDelta(currentSectorStartTime, crossingTime);
      currentSector = 3;
      currentSectorStartTime = crossingTime;

      debug(F("Sector 2 Time: "));
      debug(currentLapSector2Time);
      debug(F(" : "));
      debugln((double)(currentLapSector2Time/1000.0), 3);
    } else {
      // Out of order crossing - invalidate lap
      debug(F("WARNING: Sector 3 crossed out of order (current sector: "));
      debug(currentSector);
      debugln(F(")"));
      currentSector = 0;  // Invalidate
    }
  }
}

void DovesLapTimer::updateBestSectors() {
  // Only update if all sectors were completed
  if (currentLapSector1Time == 0 || currentLapSector2Time == 0 || currentLapSector3Time == 0) {
    return;
  }

  // Update sector 1
  if (bestSector1Time == 0 || currentLapSector1Time < bestSector1Time) {
    bestSector1Time = currentLapSector1Time;
    bestSector1LapNumber = laps;
    debug(F("New best Sector 1: "));
    debugln(bestSector1Time);
  }

  // Update sector 2
  if (bestSector2Time == 0 || currentLapSector2Time < bestSector2Time) {
    bestSector2Time = currentLapSector2Time;
    bestSector2LapNumber = laps;
    debug(F("New best Sector 2: "));
    debugln(bestSector2Time);
  }

  // Update sector 3
  if (bestSector3Time == 0 || currentLapSector3Time < bestSector3Time) {
    bestSector3Time = currentLapSector3Time;
    bestSector3LapNumber = laps;
    debug(F("New best Sector 3: "));
    debugln(bestSector3Time);
  }
}

bool DovesLapTimer::checkSectorLine(double currentLat, double currentLng,
                                    double pointALat, double pointALng,
                                    double pointBLat, double pointBLng,
                                    bool& crossingFlag, int sectorNumber) {
  double cLat = 0.0, cLng = 0.0, cOdo = 0.0;
  unsigned long cTime = 0;

  LineDetectResult ev = _detectLineCrossing(
      currentLat, currentLng,
      pointALat, pointALng, pointBLat, pointBLng,
      crossingFlag,
      sectorNumber,
      cLat, cLng, cTime, cOdo);

  if (ev == LINE_DETECT_COMPLETED && raceStarted) {
    handleLineCrossing(cTime, sectorNumber);
  }

  return ev == LINE_DETECT_IN_ZONE;
}

/////////// getters and setters

void DovesLapTimer::reset() {
  debugln(F("Resetting laptimer..."));
  // reset main race parameters
  raceStarted = false;
  currentLapStartTime = 0;
  lastLapTime = 0;
  bestLapTime = 0;
  currentLapOdometerStart = 0.0;
  lastLapDistance = 0.0;
  bestLapDistance = 0.0;
  bestLapNumber = 0;
  laps = 0;

  // reset sector timing state
  currentSector = 0;
  currentSectorStartTime = 0;
  crossingSector2 = false;
  crossingSector3 = false;
  currentLapSector1Time = 0;
  currentLapSector2Time = 0;
  currentLapSector3Time = 0;
  bestSector1Time = 0;
  bestSector2Time = 0;
  bestSector3Time = 0;
  bestSector1LapNumber = 0;
  bestSector2LapNumber = 0;
  bestSector3LapNumber = 0;

  // reset direction detection
  _directionDetector.reset();

  // reset time tracking
  millisecondsSinceMidnight = 0;

  // reset odometer and position tracking
  totalDistanceTraveled = 0;
  positionPrevLat = 0;
  positionPrevLng = 0;
  positionPrevAlt = 0;
  firstPositionReceived = false;
  consecutiveJumpCount = 0;
  rejectedCrossingCount = 0;
  prevFixLat = 0;
  prevFixLng = 0;
  prevFixTime = 0;
  prevFixOdometer = 0;
  prevFixSpeedKmh = 0;
  hasPrevFix = false;

  // Reset the crossingPointBuffer index and full status
  crossing = false;
  crossingPointBufferIndex = 0;
  crossingPointBufferFull = false;
  memset(crossingPointBuffer, 0, sizeof(crossingPointBuffer));
}
// A crossing line is usable only if both endpoints are finite and distinct.
// A degenerate line (e.g. the 0.00 placeholders shipped in example sketches)
// can never produce a side-change and would only churn the crossing buffer.
static bool lineIsValid(double aLat, double aLng, double bLat, double bLng) {
  if (!geoIsFinite(aLat) || !geoIsFinite(aLng) || !geoIsFinite(bLat) || !geoIsFinite(bLng)) {
    return false;
  }
  return aLat != bLat || aLng != bLng;
}

void DovesLapTimer::setStartFinishLine(double pointALat, double pointALng, double pointBLat, double pointBLng) {
  startFinishPointALat = pointALat;
  startFinishPointALng = pointALng;
  startFinishPointBLat = pointBLat;
  startFinishPointBLng = pointBLng;
  startFinishLineConfigured = lineIsValid(pointALat, pointALng, pointBLat, pointBLng);
  if (!startFinishLineConfigured) {
    debugln(F("WARNING: invalid start/finish line (degenerate or non-finite) - detection disabled"));
  }
}
void DovesLapTimer::setSector2Line(double pointALat, double pointALng, double pointBLat, double pointBLng) {
  sector2PointALat = pointALat;
  sector2PointALng = pointALng;
  sector2PointBLat = pointBLat;
  sector2PointBLng = pointBLng;
  sector2LineConfigured = lineIsValid(pointALat, pointALng, pointBLat, pointBLng);
  if (!sector2LineConfigured) {
    debugln(F("WARNING: invalid sector 2 line (degenerate or non-finite) - sector disabled"));
  }
}
void DovesLapTimer::setSector3Line(double pointALat, double pointALng, double pointBLat, double pointBLng) {
  sector3PointALat = pointALat;
  sector3PointALng = pointALng;
  sector3PointBLat = pointBLat;
  sector3PointBLng = pointBLng;
  sector3LineConfigured = lineIsValid(pointALat, pointALng, pointBLat, pointBLng);
  if (!sector3LineConfigured) {
    debugln(F("WARNING: invalid sector 3 line (degenerate or non-finite) - sector disabled"));
  }
}
void DovesLapTimer::updateCurrentTime(unsigned long currentTimeMilliseconds) {
  millisecondsSinceMidnight = currentTimeMilliseconds;
}
void DovesLapTimer::forceLinearInterpolation() {
  forceLinear = true;
}
void DovesLapTimer::forceCatmullRomInterpolation() {
  forceLinear = false;
}
bool DovesLapTimer::getRaceStarted() const {
  return raceStarted;
}
bool DovesLapTimer::getCrossing() const {
  return crossing;
}
unsigned long DovesLapTimer::getCurrentLapStartTime() const {
  return currentLapStartTime;
}
unsigned long DovesLapTimer::getCurrentLapTime() const {
  // raceStarted implies currentLapStartTime has been set — even a lap that
  // legitimately started at exactly 00:00:00.000 (start time 0) is valid.
  return raceStarted ? timeSinceMidnightDelta(currentLapStartTime, millisecondsSinceMidnight) : 0;
}
unsigned long DovesLapTimer::getLastLapTime() const {
  return lastLapTime;
}
unsigned long DovesLapTimer::getBestLapTime() const {
  return bestLapTime;
}
float DovesLapTimer::getCurrentLapOdometerStart() const {
  return currentLapOdometerStart;
}
float DovesLapTimer::getCurrentLapDistance() const {
  return currentLapOdometerStart == 0 || raceStarted == false ? 0 : totalDistanceTraveled - currentLapOdometerStart;
}
float DovesLapTimer::getLastLapDistance() const {
  return lastLapDistance;
}
float DovesLapTimer::getBestLapDistance() const {
  return bestLapDistance;
}
float DovesLapTimer::getTotalDistanceTraveled() const {
  return totalDistanceTraveled;
}
int DovesLapTimer::getBestLapNumber() const {
  return bestLapNumber;
}
int DovesLapTimer::getLaps() const {
  return laps;
}
float DovesLapTimer::getPaceDifference() const {
  float currentLapDistance = currentLapOdometerStart == 0 || raceStarted == false ? 0 : totalDistanceTraveled - currentLapOdometerStart;
  unsigned long currentLapTime = timeSinceMidnightDelta(currentLapStartTime, millisecondsSinceMidnight);

  // Avoid division by zero
  if (currentLapDistance == 0 || bestLapDistance == 0) {
    return 0.0;
  }

  // Calculate the pace for the current lap and the best lap
  float currentLapPace = currentLapTime / currentLapDistance;
  float bestLapPace = bestLapTime / bestLapDistance;

  // Calculate the pace difference
  float paceDiff = currentLapPace - bestLapPace;

  return paceDiff;
}
float DovesLapTimer::getCurrentSpeedKmh() const {
  return currentSpeedkmh;
}
float DovesLapTimer::getCurrentSpeedMph() const {
  return currentSpeedkmh * GEOMATH_KMH_TO_MPH;
}

/////////// sector timing getters

unsigned long DovesLapTimer::getBestSector1Time() const {
  return bestSector1Time;
}
unsigned long DovesLapTimer::getBestSector2Time() const {
  return bestSector2Time;
}
unsigned long DovesLapTimer::getBestSector3Time() const {
  return bestSector3Time;
}
unsigned long DovesLapTimer::getCurrentLapSector1Time() const {
  return currentLapSector1Time;
}
unsigned long DovesLapTimer::getCurrentLapSector2Time() const {
  return currentLapSector2Time;
}
unsigned long DovesLapTimer::getCurrentLapSector3Time() const {
  return currentLapSector3Time;
}
unsigned long DovesLapTimer::getOptimalLapTime() const {
  // Only return optimal lap if all sectors have been recorded
  if (bestSector1Time == 0 || bestSector2Time == 0 || bestSector3Time == 0) {
    return 0;
  }
  return bestSector1Time + bestSector2Time + bestSector3Time;
}
int DovesLapTimer::getBestSector1LapNumber() const {
  return bestSector1LapNumber;
}
int DovesLapTimer::getBestSector2LapNumber() const {
  return bestSector2LapNumber;
}
int DovesLapTimer::getBestSector3LapNumber() const {
  return bestSector3LapNumber;
}
int DovesLapTimer::getCurrentSector() const {
  return currentSector;
}
bool DovesLapTimer::areSectorLinesConfigured() const {
  return sector2LineConfigured && sector3LineConfigured;
}
bool DovesLapTimer::isStartFinishLineConfigured() const {
  return startFinishLineConfigured;
}
unsigned int DovesLapTimer::getRejectedCrossingCount() const {
  return rejectedCrossingCount;
}

/////////// direction detection getters

int DovesLapTimer::getDirection() const {
  return _directionDetector.direction;
}
bool DovesLapTimer::isDirectionResolved() const {
  return _directionDetector.direction != DIR_UNKNOWN;
}