/**
 * CourseManager - orchestrates multiple DovesLapTimer instances + CourseDetector + WaypointLapTimer.
 * Ported from js/lib/course-manager.js
 */

#include "CourseManager.h"

#define debugln debug_println
#define debug debug_print

CourseManager::CourseManager(TrackConfig& config, double crossingThreshold, Stream *debugSerial)
  : _lapAnythingTimer(debugSerial) {
  _serial = debugSerial;
  _crossingThreshold = crossingThreshold;
  _trackLongName = config.longName;
  _trackShortName = config.shortName;
  _initCourses(config);
}

void CourseManager::_initCourses(TrackConfig& config) {
  _courseCount = (config.courseCount > MAX_COURSES) ? MAX_COURSES : config.courseCount;
  _activeCourseIndex = -1;
  _detectionComplete = false;
  _detectionRejectionCount = 0;
  _lapAnythingActive = false;

  // Create a DovesLapTimer for each course
  for (int i = 0; i < _courseCount; i++) {
    CourseConfig& course = config.courses[i];

    // Use placement new to construct timer in-place (avoids assignment operator issues)
    new (&_courseTimers[i].timer) DovesLapTimer(_crossingThreshold, _serial);
    _courseTimers[i].timer.forceLinearInterpolation();
    _courseTimers[i].name = course.name;
    _courseTimers[i].lengthFt = course.lengthFt;
    _courseTimers[i].active = true;

    // Configure start/finish line
    _courseTimers[i].timer.setStartFinishLine(
      course.startALat, course.startALng,
      course.startBLat, course.startBLng
    );

    // Configure sector lines if present
    if (course.hasSector2) {
      _courseTimers[i].timer.setSector2Line(
        course.sector2ALat, course.sector2ALng,
        course.sector2BLat, course.sector2BLng
      );
    }

    if (course.hasSector3) {
      _courseTimers[i].timer.setSector3Line(
        course.sector3ALat, course.sector3ALng,
        course.sector3BLat, course.sector3BLng
      );
    }

    _courseTimers[i].timer.reset();
  }

  // Mark remaining slots as inactive
  for (int i = _courseCount; i < MAX_COURSES; i++) {
    _courseTimers[i].active = false;
    _courseTimers[i].name = NULL;
    _courseTimers[i].lengthFt = 0;
  }

  // Create the detector with course names and lengths
  if (_courseCount > 0) {
    CourseInfo courseInfos[MAX_COURSES];
    for (int i = 0; i < _courseCount; i++) {
      courseInfos[i].name = config.courses[i].name;
      courseInfos[i].lengthFt = config.courses[i].lengthFt;
    }
    _detector.init(courseInfos, _courseCount);
  }

  // Reset the Lap Anything timer
  _lapAnythingTimer.reset();

  // If no courses loaded, activate Lap Anything immediately
  if (_courseCount == 0) {
    _activateLapAnything();
  }
}

void CourseManager::updateCurrentTime(unsigned long ms) {
  for (int i = 0; i < _courseCount; i++) {
    if (_courseTimers[i].active) {
      _courseTimers[i].timer.updateCurrentTime(ms);
    }
  }

  _lapAnythingTimer.updateCurrentTime(ms);
}

int CourseManager::loop(double lat, double lng, float altMeters, float speedKnots) {
  // Feed all active course timers
  for (int i = 0; i < _courseCount; i++) {
    if (_courseTimers[i].active) {
      _courseTimers[i].timer.loop(lat, lng, altMeters, speedKnots);
    }
  }

  // Feed Lap Anything timer (always running to accumulate data)
  _lapAnythingTimer.loop(lat, lng, altMeters, speedKnots);

  // Feed the detector
  if (!_detectionComplete && _courseCount > 0) {
    float speedKmh = speedKnots * 1.852;

    // Get odometer from first active timer
    float odometer = 0;
    for (int i = 0; i < _courseCount; i++) {
      if (_courseTimers[i].active) {
        odometer = _courseTimers[i].timer.getTotalDistanceTraveled();
        break;
      }
    }

    _detector.update(lat, lng, speedKmh, odometer);

    // Handle candidates_ready state
    if (_detector.getState() == DETECT_STATE_CANDIDATES_READY) {
      _handleCandidatesReady();
    }
  }

  return -1;
}

void CourseManager::_handleCandidatesReady() {
  int matchCount = _detector.getRankedMatchCount();
  const DetectionCandidate* matches = _detector.getRankedMatches();

  for (int i = 0; i < matchCount; i++) {
    int idx = matches[i].index;
    if (idx >= 0 && idx < _courseCount && _courseTimers[idx].active && _courseTimers[idx].timer.getRaceStarted()) {
      // Sanity check passed — accept this candidate
      _detector.acceptCandidate(idx);
      _activeCourseIndex = idx;
      _detectionComplete = true;
      debug(F("Course detected (validated): "));
      debugln(_courseTimers[idx].name);
      return;
    }
  }

  // No candidate had raceStarted — reject all
  _detector.rejectAllCandidates();
  _detectionRejectionCount++;
  debug(F("Detection rejected ("));
  debug(_detectionRejectionCount);
  debug(F("/"));
  debug(COURSE_DETECT_MAX_REJECTIONS);
  debugln(F(") - no candidate has raceStarted"));

  // After max rejections, fall back to Lap Anything
  if (_detectionRejectionCount >= COURSE_DETECT_MAX_REJECTIONS) {
    debugln(F("Max detection rejections reached - activating Lap Anything"));
    _activateLapAnything();
  }
}

void CourseManager::_activateLapAnything() {
  _lapAnythingActive = true;
  _detectionComplete = true;
}

void CourseManager::pruneInactiveCourses() {
  if (!_detectionComplete || _activeCourseIndex < 0) return;

  for (int i = 0; i < _courseCount; i++) {
    if (i != _activeCourseIndex) {
      _courseTimers[i].active = false;
    }
  }
}

void CourseManager::reset() {
  // Reset all timers
  for (int i = 0; i < _courseCount; i++) {
    _courseTimers[i].active = true;
    _courseTimers[i].timer.reset();
  }

  _detector.reset();
  _lapAnythingTimer.reset();
  _activeCourseIndex = -1;
  _detectionComplete = false;
  _detectionRejectionCount = 0;
  _lapAnythingActive = false;

  // If no courses, activate Lap Anything immediately
  if (_courseCount == 0) {
    _activateLapAnything();
  }
}

/////////// getters

bool CourseManager::isDetectionComplete() const {
  return _detectionComplete;
}

int CourseManager::getActiveCourseIndex() const {
  return _activeCourseIndex;
}

const char* CourseManager::getActiveCourseName() const {
  if (_lapAnythingActive) return "Lap Anything";
  if (_activeCourseIndex < 0) return NULL;
  return _courseTimers[_activeCourseIndex].name;
}

int CourseManager::getCourseCount() const {
  return _courseCount;
}

int CourseManager::getDetectionRejectionCount() const {
  return _detectionRejectionCount;
}

DovesLapTimer* CourseManager::getActiveTimer() {
  if (_activeCourseIndex < 0) return NULL;
  CourseTimerEntry& ct = _courseTimers[_activeCourseIndex];
  return ct.active ? &ct.timer : NULL;
}

WaypointLapTimer* CourseManager::getLapAnythingTimer() {
  return &_lapAnythingTimer;
}

bool CourseManager::isLapAnythingActive() const {
  return _lapAnythingActive;
}

const char* CourseManager::getTrackName() const {
  return _trackLongName ? _trackLongName : "Unknown";
}

const char* CourseManager::getShortName() const {
  return _trackShortName ? _trackShortName : "";
}

CourseDetector* CourseManager::getDetector() {
  return &_detector;
}
