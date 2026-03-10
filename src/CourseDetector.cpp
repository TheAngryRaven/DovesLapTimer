/**
 * CourseDetector - detects which course layout the driver is on.
 * Ported from js/lib/course-detector.js
 */

#include "CourseDetector.h"

CourseDetector::CourseDetector() {
  _courseCount = 0;
  reset();
}

void CourseDetector::init(CourseInfo* courses, int count) {
  _courseCount = (count > MAX_COURSES) ? MAX_COURSES : count;
  for (int i = 0; i < _courseCount; i++) {
    _courses[i].name = courses[i].name;
    _courses[i].lengthFt = courses[i].lengthFt;
  }
  reset();
}

void CourseDetector::reset() {
  _state = DETECT_STATE_IDLE;
  _waypointLat = 0;
  _waypointLng = 0;
  _waypointOdometer = 0;
  _detectedCourseIndex = -1;
  _rankedMatchCount = 0;
}

void CourseDetector::update(double lat, double lng, float speedKmh, float totalOdometer) {
  if (_state == DETECT_STATE_IDLE) {
    _state = DETECT_STATE_WAITING_FOR_SPEED;
  }

  if (_state == DETECT_STATE_WAITING_FOR_SPEED) {
    _checkSpeedThreshold(lat, lng, speedKmh, totalOdometer);
  }

  if (_state == DETECT_STATE_WAYPOINT_SET) {
    _checkWaypointProximity(lat, lng, totalOdometer);
  }
}

void CourseDetector::_checkSpeedThreshold(double lat, double lng, float speedKmh, float totalOdometer) {
  float speedMph = speedKmh * 0.621371;

  if (speedMph >= COURSE_DETECT_SPEED_THRESHOLD_MPH) {
    _waypointLat = lat;
    _waypointLng = lng;
    _waypointOdometer = totalOdometer;
    _state = DETECT_STATE_WAYPOINT_SET;
  }
}

void CourseDetector::_checkWaypointProximity(double lat, double lng, float totalOdometer) {
  float distanceSinceWaypoint = totalOdometer - _waypointOdometer;

  if (distanceSinceWaypoint < COURSE_DETECT_MIN_DISTANCE_METERS) {
    return;
  }

  double distToWaypoint = geoHaversine(lat, lng, _waypointLat, _waypointLng);

  if (distToWaypoint < COURSE_DETECT_WAYPOINT_PROXIMITY_METERS) {
    _matchCourseRanked(distanceSinceWaypoint);
  }
}

void CourseDetector::_matchCourseRanked(float distanceMeters) {
  float distanceFt = distanceMeters * METERS_TO_FEET;

  _rankedMatchCount = 0;

  for (int i = 0; i < _courseCount; i++) {
    float courseLengthFt = _courses[i].lengthFt;
    if (courseLengthFt <= 0) continue;

    float ratio = fabs(distanceFt - courseLengthFt) / courseLengthFt;

    if (ratio <= COURSE_DETECT_DISTANCE_TOLERANCE_PCT) {
      _rankedMatches[_rankedMatchCount].index = i;
      _rankedMatches[_rankedMatchCount].ratio = ratio;
      _rankedMatchCount++;
    }
  }

  if (_rankedMatchCount > 0) {
    // Simple insertion sort by ratio (best match first)
    for (int i = 1; i < _rankedMatchCount; i++) {
      DetectionCandidate key = _rankedMatches[i];
      int j = i - 1;
      while (j >= 0 && _rankedMatches[j].ratio > key.ratio) {
        _rankedMatches[j + 1] = _rankedMatches[j];
        j--;
      }
      _rankedMatches[j + 1] = key;
    }
    _state = DETECT_STATE_CANDIDATES_READY;
  }
  // If no candidates within tolerance, stay in waypoint_set and try again next pass
}

void CourseDetector::acceptCandidate(int index) {
  _detectedCourseIndex = index;
  _rankedMatchCount = 0;
  _state = DETECT_STATE_DETECTED;
}

void CourseDetector::rejectAllCandidates() {
  _rankedMatchCount = 0;
  _state = DETECT_STATE_WAYPOINT_SET;
}

/////////// getters

int CourseDetector::getState() const {
  return _state;
}

int CourseDetector::getDetectedCourseIndex() const {
  return _detectedCourseIndex;
}

bool CourseDetector::isDetected() const {
  return _state == DETECT_STATE_DETECTED;
}

double CourseDetector::getWaypointLat() const {
  return _waypointLat;
}

double CourseDetector::getWaypointLng() const {
  return _waypointLng;
}

bool CourseDetector::hasWaypoint() const {
  return _state != DETECT_STATE_IDLE && _state != DETECT_STATE_WAITING_FOR_SPEED;
}

int CourseDetector::getRankedMatchCount() const {
  return _rankedMatchCount;
}

const DetectionCandidate* CourseDetector::getRankedMatches() const {
  return _rankedMatches;
}
