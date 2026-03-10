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

static const double GEOMATH_RADIUS_EARTH = 6371.0 * 1000; // meters

static inline double geoHaversine(double lat1, double lon1, double lat2, double lon2) {
  double lat1Rad = lat1 * M_PI / 180.0;
  double lon1Rad = lon1 * M_PI / 180.0;
  double lat2Rad = lat2 * M_PI / 180.0;
  double lon2Rad = lon2 * M_PI / 180.0;

  double deltaLat = lat2Rad - lat1Rad;
  double deltaLon = lon2Rad - lon1Rad;

  double a = pow(sin(deltaLat / 2), 2) + cos(lat1Rad) * cos(lat2Rad) * pow(sin(deltaLon / 2), 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));

  return GEOMATH_RADIUS_EARTH * c;
}

static inline double geoHaversine3D(double prevLat, double prevLng, double prevAlt, double currentLat, double currentLng, double currentAlt) {
  double dist = geoHaversine(prevLat, prevLng, currentLat, currentLng);
  double altDiff = currentAlt - prevAlt;
  return sqrt(dist * dist + altDiff * altDiff);
}

#endif
