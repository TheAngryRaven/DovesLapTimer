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

#endif
