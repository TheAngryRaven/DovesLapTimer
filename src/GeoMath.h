/**
 * Geographic math utilities shared across DovesLapTimer, WaypointLapTimer, and CourseDetector.
 *
 * These are static inline functions so any class can include this header without
 * needing a separate .cpp or an instance of DovesLapTimer.
 *
 * DovesLapTimer retains its own haversine()/haversine3D() public methods as thin
 * wrappers for backward compatibility.
 */

#ifndef _GEOMATH_H
#define _GEOMATH_H

#include <math.h>
#include <float.h>

// M_PI is not part of the C/C++ standard — it's a POSIX/BSD extension that
// glibc happens to expose by default but stricter libcs (e.g. under
// -std=c++NN / __STRICT_ANSI__) hide. Define a fallback so the library
// compiles everywhere.
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const double GEOMATH_RADIUS_EARTH = 6371.0 * 1000; // meters

// Unit conversion constants — the single source of truth for the unit
// boundaries the public API spans (knots from NMEA, km/h internally, mph
// for the US-facing speed gates, feet for course lengths). One nautical
// mile is exactly 1852 m and one statute mile exactly 1609.344 m, so the
// derived factors below stay mutually consistent (previously three
// hand-typed literals disagreed at the 7th digit).
static constexpr double GEOMATH_KNOTS_TO_KMH  = 1.852;
static constexpr double GEOMATH_MPH_TO_KMH    = 1.609344;
static constexpr double GEOMATH_KMH_TO_MPH    = 1.0 / GEOMATH_MPH_TO_KMH;
static constexpr double GEOMATH_METERS_TO_FEET = 1.0 / 0.3048; // 1 ft = 0.3048 m exactly

/**
 * @brief Checks a value is a real, finite number (not NaN, not +/-infinity).
 *
 * Implemented without isnan()/isinf() because their availability as
 * unqualified names differs between Arduino cores and host <cmath>.
 *
 * @param v Value to check.
 * @return True if v is finite, false for NaN or infinity.
 */
static inline bool geoIsFinite(double v) {
  return v == v && v <= DBL_MAX && v >= -DBL_MAX;
}

/**
 * @brief Validates a GPS coordinate pair before it is allowed to touch timing state.
 *
 * Rejects NaN/infinity (routine output from NMEA parsers during fix loss),
 * out-of-range latitudes/longitudes, and the exact (0, 0) "null island" fix
 * that parsers emit while the receiver has no solution. A single such fix
 * would otherwise permanently poison odometers and distance tracking.
 *
 * @param lat Latitude in decimal degrees.
 * @param lng Longitude in decimal degrees.
 * @return True if the pair is a plausible real-world coordinate.
 */
static inline bool geoCoordinatesValid(double lat, double lng) {
  if (!geoIsFinite(lat) || !geoIsFinite(lng)) return false;
  if (lat < -90.0 || lat > 90.0 || lng < -180.0 || lng > 180.0) return false;
  if (lat == 0.0 && lng == 0.0) return false;
  return true;
}

static inline double geoHaversine(double lat1, double lon1, double lat2, double lon2) {
  double lat1Rad = lat1 * M_PI / 180.0;
  double lon1Rad = lon1 * M_PI / 180.0;
  double lat2Rad = lat2 * M_PI / 180.0;
  double lon2Rad = lon2 * M_PI / 180.0;

  double sinHalfDLat = sin((lat2Rad - lat1Rad) / 2);
  double sinHalfDLon = sin((lon2Rad - lon1Rad) / 2);

  double a = sinHalfDLat * sinHalfDLat + cos(lat1Rad) * cos(lat2Rad) * sinHalfDLon * sinHalfDLon;
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));

  return GEOMATH_RADIUS_EARTH * c;
}

static inline double geoHaversine3D(double prevLat, double prevLng, double prevAlt, double currentLat, double currentLng, double currentAlt) {
  double dist = geoHaversine(prevLat, prevLng, currentLat, currentLng);
  double altDiff = currentAlt - prevAlt;
  return sqrt(dist * dist + altDiff * altDiff);
}

/**
 * @brief Determines which side of an (infinite) line a point is on.
 *
 * Cross-product sign test in degree space. Extracted from
 * DovesLapTimer::pointOnSideOfLine so CrossingEngine and SprintTimer can
 * share it without a timer instance; the member method delegates here.
 *
 * @return 1 / -1 for the two sides, 0 if exactly on the line
 *         (matches CROSSING_LINE_SIDE_B / _A / _EXACT).
 */
static inline int geoPointOnSideOfLine(double driverLat, double driverLng, double pointALat, double pointALng, double pointBLat, double pointBLng) {
  double lineDirectionX = pointBLat - pointALat;
  double lineDirectionY = pointBLng - pointALng;
  double driverToPointAX = driverLat - pointALat;
  double driverToPointAY = driverLng - pointALng;

  double crossProduct = lineDirectionX * driverToPointAY - lineDirectionY * driverToPointAX;

  if (crossProduct > 0) {
    return -1; // CROSSING_LINE_SIDE_A
  } else if (crossProduct < 0) {
    return 1;  // CROSSING_LINE_SIDE_B
  }
  return 0;    // CROSSING_LINE_SIDE_EXACT
}

/**
 * @brief Shortest distance from a point to a line *segment*, in meters.
 *
 * Inputs are decimal-degree coordinates; the projection onto the segment
 * happens in degree space, but every return path measures the final
 * distance via haversine, so the result is in meters. Extracted from
 * DovesLapTimer::pointLineSegmentDistance (which delegates here).
 */
static inline double geoPointLineSegmentDistance(double pointX, double pointY, double startX, double startY, double endX, double endY) {
  double dx = endX - startX;
  double dy = endY - startY;
  double segmentLengthSquared = dx * dx + dy * dy;

  // Epsilon comparison handles degenerate segments (start == end).
  if (segmentLengthSquared < 1e-12) {
    return geoHaversine(pointX, pointY, startX, startY);
  }

  double projectionScalar = ((pointX - startX) * dx + (pointY - startY) * dy) / segmentLengthSquared;

  if (projectionScalar < 0.0) {
    return geoHaversine(pointX, pointY, startX, startY);
  } else if (projectionScalar > 1.0) {
    return geoHaversine(pointX, pointY, endX, endY);
  }

  double projectedX = startX + projectionScalar * dx;
  double projectedY = startY + projectionScalar * dy;
  return geoHaversine(pointX, pointY, projectedX, projectedY);
}

/**
 * @brief Hypotenuse-based crossing-zone membership test.
 *
 * Forms a right triangle from the line's width and thresholdMeters; the
 * hypotenuse is the effective proximity bound measured from the driver to
 * EACH line endpoint. See DETECTION.md for why this beats a plain
 * distance-to-line check. Extracted from DovesLapTimer::insideLineThreshold
 * (which delegates here with its configured threshold).
 */
static inline bool geoInsideLineThreshold(double thresholdMeters, double driverLat, double driverLon, double crossingPointALat, double crossingPointALon, double crossingPointBLat, double crossingPointBLon) {
  double driverLengthA = geoHaversine(driverLat, driverLon, crossingPointALat, crossingPointALon);
  double driverLengthB = geoHaversine(driverLat, driverLon, crossingPointBLat, crossingPointBLon);
  double crossingLineLength = geoHaversine(crossingPointALat, crossingPointALon, crossingPointBLat, crossingPointBLon);
  double maxLineLength = sqrt(thresholdMeters * thresholdMeters + crossingLineLength * crossingLineLength);
  return driverLengthA < maxLineLength && driverLengthB < maxLineLength;
}

#endif
