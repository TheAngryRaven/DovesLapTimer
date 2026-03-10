/**
 * CourseManager - orchestrates multiple DovesLapTimer instances + CourseDetector + WaypointLapTimer.
 *
 * Feeds ALL course timers the same GPS data simultaneously.
 * Once the CourseDetector identifies candidates, validates via raceStarted sanity check.
 * Falls back to WaypointLapTimer ("Lap Anything") if detection fails.
 *
 * Implements the same updateCurrentTime() / loop() interface as DovesLapTimer,
 * so the caller can feed it via duck typing.
 */

#ifndef _COURSE_MANAGER_H
#define _COURSE_MANAGER_H

#include <Arduino.h>
#include "DovesLapTimer.h"
#include "WaypointLapTimer.h"
#include "CourseDetector.h"

struct CourseConfig {
  const char* name;
  float lengthFt;
  double startALat, startALng, startBLat, startBLng;
  double sector2ALat, sector2ALng, sector2BLat, sector2BLng;
  double sector3ALat, sector3ALng, sector3BLat, sector3BLng;
  bool hasSector2;
  bool hasSector3;
};

struct TrackConfig {
  const char* longName;
  const char* shortName;
  CourseConfig courses[MAX_COURSES];
  int courseCount;
};

struct CourseTimerEntry {
  DovesLapTimer timer;
  const char* name;
  float lengthFt;
  bool active;
};

class CourseManager {
public:
  CourseManager(TrackConfig& config, double crossingThreshold = 7.0, Stream *debugSerial = NULL);

  void updateCurrentTime(unsigned long ms);
  int loop(double lat, double lng, float altMeters, float speedKnots);
  void reset();
  void pruneInactiveCourses();

  // Detection state
  bool isDetectionComplete() const;
  int getActiveCourseIndex() const;
  const char* getActiveCourseName() const;
  int getCourseCount() const;
  int getDetectionRejectionCount() const;

  // Timer access
  DovesLapTimer* getActiveTimer();
  WaypointLapTimer* getLapAnythingTimer();
  bool isLapAnythingActive() const;

  // Track metadata
  const char* getTrackName() const;
  const char* getShortName() const;

  // Detector access
  CourseDetector* getDetector();

private:
  template<typename... Args>
  void debug_print(Args&&... args) {
    if(_serial) { _serial->print(std::forward<Args>(args)...); }
  }
  template<typename... Args>
  void debug_println(Args&&... args) {
    if(_serial) { _serial->println(std::forward<Args>(args)...); }
  }

  void _initCourses(TrackConfig& config);
  void _handleCandidatesReady();
  void _activateLapAnything();

  Stream *_serial;
  double _crossingThreshold;

  CourseTimerEntry _courseTimers[MAX_COURSES];
  int _courseCount;

  CourseDetector _detector;
  WaypointLapTimer _lapAnythingTimer;

  int _activeCourseIndex;
  bool _detectionComplete;
  bool _lapAnythingActive;
  int _detectionRejectionCount;

  const char* _trackLongName;
  const char* _trackShortName;
};

#endif
