/**
 * CourseDetector - detects which course layout the driver is on.
 *
 * Algorithm:
 *   1. Wait for speed >= 20 mph (driver is moving)
 *   2. Create a "waypoint" at that position
 *   3. Wait for driver to return near the waypoint (completed ~1 lap)
 *   4. Compare driven distance to each course's lengthFt
 *   5. Build ranked list of candidates within tolerance
 *   6. CourseManager validates candidates via raceStarted sanity check
 */

#ifndef _COURSE_DETECTOR_H
#define _COURSE_DETECTOR_H

#include <Arduino.h>
#include "DovesLapTimer.h"
#include "GeoMath.h"

struct CourseInfo {
  const char* name;
  float lengthFt;
};

struct DetectionCandidate {
  int index;
  float ratio;   // distance match quality (0.0 = perfect)
};

class CourseDetector {
public:
  CourseDetector();
  void init(CourseInfo* courses, int count);
  void update(double lat, double lng, float speedKmh, float totalOdometer);
  void acceptCandidate(int index);
  void rejectAllCandidates();
  void reset();
  void setSpeedThresholdMph(float mph);
  void setDetectionProximityMeters(float meters);

  // Getters
  int getState() const;
  int getDetectedCourseIndex() const;
  bool isDetected() const;
  double getWaypointLat() const;
  double getWaypointLng() const;
  bool hasWaypoint() const;
  int getRankedMatchCount() const;
  const DetectionCandidate* getRankedMatches() const;

private:
  void _checkSpeedThreshold(double lat, double lng, float speedKmh, float totalOdometer);
  void _checkWaypointProximity(double lat, double lng, float totalOdometer);
  void _matchCourseRanked(float distanceMeters);

  CourseInfo _courses[MAX_COURSES];
  int _courseCount;
  int _state;
  double _waypointLat;
  double _waypointLng;
  float _waypointOdometer;
  int _detectedCourseIndex;
  DetectionCandidate _rankedMatches[MAX_COURSES];
  int _rankedMatchCount;
  float _speedThresholdMph;
  float _detectionProximityMeters;
};

#endif
