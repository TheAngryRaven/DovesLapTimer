/**
 * SprintTimer - point-to-point run timing. See SprintTimer.h for the model.
 */

#include "SprintTimer.h"

#define debugln debug_println
#define debug debug_print

// Line labels for CrossingEngine debug output.
#define SPRINT_LINE_START   0
#define SPRINT_LINE_FINISH  1

SprintTimer::SprintTimer(double crossingThresholdMeters, Stream *debugSerial)
  : _startEngine(crossingThresholdMeters, debugSerial),
    _finishEngine(crossingThresholdMeters, debugSerial),
    _sector2Engine(crossingThresholdMeters, debugSerial),
    _sector3Engine(crossingThresholdMeters, debugSerial) {
  _crossingThresholdMeters = crossingThresholdMeters;
  _serial = (debugSerial == NULL) ? nullptr : debugSerial;
}

int SprintTimer::loop(double currentLat, double currentLng, float currentAltitudeMeters, float currentSpeedKnots) {
  // Fix intake mirrors DovesLapTimer::loop() — reject invalid fixes,
  // sanitize auxiliary values, drop single-fix teleports, re-accept a
  // sustained relocation without crediting the gap to the odometer.
  if (!geoCoordinatesValid(currentLat, currentLng)) {
    debugln(F("Rejected invalid GPS coordinates"));
    return -1;
  }
  if (!geoIsFinite(currentAltitudeMeters)) {
    currentAltitudeMeters = _positionPrevAlt;
  }
  if (!geoIsFinite(currentSpeedKnots) || currentSpeedKnots < 0) {
    currentSpeedKnots = 0;
  }

  if (_firstPositionReceived) {
    double jumpDistance = geoHaversine(_positionPrevLat, _positionPrevLng, currentLat, currentLng);
    if (jumpDistance > GPS_MAX_PLAUSIBLE_JUMP_METERS) {
      _consecutiveJumpCount++;
      if (_consecutiveJumpCount < GPS_JUMP_REACCEPT_COUNT) {
        debugln(F("Rejected implausible GPS jump"));
        return -1;
      }
      _consecutiveJumpCount = 0;
    } else {
      _consecutiveJumpCount = 0;
      _totalDistanceTraveled += geoHaversine3D(
        _positionPrevLat, _positionPrevLng, _positionPrevAlt,
        currentLat, currentLng, currentAltitudeMeters);
    }
  } else {
    _firstPositionReceived = true;
  }
  _positionPrevLat = currentLat;
  _positionPrevLng = currentLng;
  _positionPrevAlt = currentAltitudeMeters;
  _currentSpeedKmh = currentSpeedKnots * GEOMATH_KNOTS_TO_KMH;

  // Every configured line is checked on EVERY fix — independent engines,
  // no mutual exclusion (see header). Detect first, then handle results in
  // a fixed order: finish before start, so that when both lines complete
  // on the same fix (overlapping zones) the active run is completed by the
  // finish before the start crossing opens the next one.
  const crossingPointBufferEntry *prev = _hasPrevFix ? &_prevFix : NULL;
  bool nearAnyLine = false;

  double finLat = 0, finLng = 0, finOdo = 0;
  unsigned long finTime = 0;
  LineDetectResult finishEv = LINE_DETECT_NONE;
  if (_finishLineConfigured) {
    finishEv = _finishEngine.detect(
        currentLat, currentLng, _millisecondsSinceMidnight, _totalDistanceTraveled, _currentSpeedKmh, prev,
        _finishALat, _finishALng, _finishBLat, _finishBLng,
        _crossingFinish, SPRINT_LINE_FINISH, finLat, finLng, finTime, finOdo);
    if (finishEv != LINE_DETECT_NONE) nearAnyLine = true;
  }

  double stLat = 0, stLng = 0, stOdo = 0;
  unsigned long stTime = 0;
  LineDetectResult startEv = LINE_DETECT_NONE;
  if (_startLineConfigured) {
    startEv = _startEngine.detect(
        currentLat, currentLng, _millisecondsSinceMidnight, _totalDistanceTraveled, _currentSpeedKmh, prev,
        _startALat, _startALng, _startBLat, _startBLng,
        _crossingStart, SPRINT_LINE_START, stLat, stLng, stTime, stOdo);
    if (startEv != LINE_DETECT_NONE) nearAnyLine = true;
  }

  double s2Lat = 0, s2Lng = 0, s2Odo = 0;
  unsigned long s2Time = 0;
  LineDetectResult s2Ev = LINE_DETECT_NONE;
  if (_sector2LineConfigured) {
    s2Ev = _sector2Engine.detect(
        currentLat, currentLng, _millisecondsSinceMidnight, _totalDistanceTraveled, _currentSpeedKmh, prev,
        _sector2ALat, _sector2ALng, _sector2BLat, _sector2BLng,
        _crossingSector2, 2, s2Lat, s2Lng, s2Time, s2Odo);
    if (s2Ev != LINE_DETECT_NONE) nearAnyLine = true;
  }

  double s3Lat = 0, s3Lng = 0, s3Odo = 0;
  unsigned long s3Time = 0;
  LineDetectResult s3Ev = LINE_DETECT_NONE;
  if (_sector3LineConfigured) {
    s3Ev = _sector3Engine.detect(
        currentLat, currentLng, _millisecondsSinceMidnight, _totalDistanceTraveled, _currentSpeedKmh, prev,
        _sector3ALat, _sector3ALng, _sector3BLat, _sector3BLng,
        _crossingSector3, 3, s3Lat, s3Lng, s3Time, s3Odo);
    if (s3Ev != LINE_DETECT_NONE) nearAnyLine = true;
  }

  if (finishEv == LINE_DETECT_COMPLETED) {
    _handleFinishCrossing(finTime, finOdo);
  }
  if (startEv == LINE_DETECT_COMPLETED) {
    _handleStartCrossing(stTime, stOdo);
  }
  if (s2Ev == LINE_DETECT_COMPLETED) {
    _handleSplitCrossing(2, s2Time);
  }
  if (s3Ev == LINE_DETECT_COMPLETED) {
    _handleSplitCrossing(3, s3Time);
  }

  // Save current fix as previous for the engines' Catmull-Rom pre-crossing point
  _prevFix.lat = currentLat;
  _prevFix.lng = currentLng;
  _prevFix.time = _millisecondsSinceMidnight;
  _prevFix.odometer = _totalDistanceTraveled;
  _prevFix.speedKmh = _currentSpeedKmh;
  _hasPrevFix = true;

  return nearAnyLine ? 0 : -1;
}

void SprintTimer::updateCurrentTime(unsigned long currentTimeMilliseconds) {
  _millisecondsSinceMidnight = currentTimeMilliseconds;
}

/////////// crossing handlers

void SprintTimer::_handleStartCrossing(unsigned long crossingTime, double crossingOdometer) {
  if (_runActive) {
    // Botched-course re-launch: cancel the run in progress, start fresh.
    _cancelledRuns++;
    debugln(F("Start line re-crossed - run cancelled, starting new run"));
  } else {
    debugln(F("Run started"));
  }
  _raceStarted = true;
  _runActive = true;
  _currentRunStartTime = crossingTime;
  _currentRunOdometerStart = crossingOdometer;
  _currentSegment = 1;
  _segmentStartTime = crossingTime;
  _segmentOrderValid = true;
  for (int i = 0; i <= SPRINT_MAX_SPLITS; i++) {
    _currentRunSegTime[i] = 0;
  }
}

void SprintTimer::_handleFinishCrossing(unsigned long crossingTime, double crossingOdometer) {
  if (!_runActive) {
    // Return loop / staging traffic — a finish crossing with no active run
    // means nothing in sprint.
    debugln(F("Finish crossing ignored (no active run)"));
    return;
  }

  // Close the final segment (only when splits are in play).
  if (areSectorLinesConfigured()) {
    if (_currentSegment == _expectedSegments()) {
      _currentRunSegTime[_currentSegment - 1] = timeSinceMidnightDelta(_segmentStartTime, crossingTime);
    } else {
      // A split was missed (GPS dropout or misplaced line) — the run time
      // stands, but its segment data can't feed best-segment tracking.
      _segmentOrderValid = false;
    }
  }

  unsigned long runTime = timeSinceMidnightDelta(_currentRunStartTime, crossingTime);
  float runDistance = (float)(crossingOdometer - _currentRunOdometerStart);

  _runs++;
  _lastRunTime = runTime;
  _lastRunDistance = runDistance;
  if (_bestRunTime <= 0 || runTime < _bestRunTime) {
    _bestRunTime = runTime;
    _bestRunDistance = runDistance;
    _bestRunNumber = _runs;
  }

  debug(F("Run Finish Time: "));
  debug(runTime);
  debug(F(" : "));
  debugln((double)(runTime / 1000.0), 3);

  if (areSectorLinesConfigured() && _segmentOrderValid) {
    _updateBestSegments();
  }

  _runActive = false;
  _currentSegment = 0;
}

void SprintTimer::_handleSplitCrossing(int sectorLabel, unsigned long crossingTime) {
  if (!_runActive) {
    // Splits mean nothing outside a run (return loop traffic).
    return;
  }

  int closesSegment = _segmentClosedBySector(sectorLabel);
  if (_currentSegment == closesSegment) {
    _currentRunSegTime[closesSegment - 1] = timeSinceMidnightDelta(_segmentStartTime, crossingTime);
    _currentSegment = closesSegment + 1;
    _segmentStartTime = crossingTime;

    debug(F("Segment "));
    debug(closesSegment);
    debug(F(" Time: "));
    debugln(_currentRunSegTime[closesSegment - 1]);
  } else {
    debug(F("WARNING: Sector "));
    debug(sectorLabel);
    debug(F(" crossed out of order (current segment: "));
    debug(_currentSegment);
    debugln(F(") - run sector data invalidated"));
    _segmentOrderValid = false;
  }
}

int SprintTimer::_expectedSegments() const {
  return 1 + (_sector2LineConfigured ? 1 : 0) + (_sector3LineConfigured ? 1 : 0);
}

int SprintTimer::_segmentClosedBySector(int sectorLabel) const {
  if (sectorLabel == 2) {
    return 1;  // sector 2, when configured, is always the first split
  }
  return _sector2LineConfigured ? 2 : 1;  // sector 3 follows sector 2 if present
}

void SprintTimer::_updateBestSegments() {
  // Only runs with every expected segment recorded, in order, may feed the
  // best-segment table — otherwise a short/invalid segment from a missed
  // split would fabricate an unbeatable "best".
  int expected = _expectedSegments();
  for (int i = 0; i < expected; i++) {
    if (_currentRunSegTime[i] == 0) {
      return;
    }
  }
  for (int i = 0; i < expected; i++) {
    if (_bestSegTime[i] == 0 || _currentRunSegTime[i] < _bestSegTime[i]) {
      _bestSegTime[i] = _currentRunSegTime[i];
      _bestSegRunNumber[i] = _runs;
    }
  }
}

/////////// configuration

// A crossing line is usable only if both endpoints are finite and distinct
// (same rule as DovesLapTimer's setters).
static bool sprintLineIsValid(double aLat, double aLng, double bLat, double bLng) {
  if (!geoIsFinite(aLat) || !geoIsFinite(aLng) || !geoIsFinite(bLat) || !geoIsFinite(bLng)) {
    return false;
  }
  return aLat != bLat || aLng != bLng;
}

void SprintTimer::setStartLine(double pointALat, double pointALng, double pointBLat, double pointBLng) {
  _startALat = pointALat; _startALng = pointALng;
  _startBLat = pointBLat; _startBLng = pointBLng;
  _startLineConfigured = sprintLineIsValid(pointALat, pointALng, pointBLat, pointBLng);
  if (!_startLineConfigured) {
    debugln(F("WARNING: invalid start line (degenerate or non-finite) - detection disabled"));
  }
}

void SprintTimer::setFinishLine(double pointALat, double pointALng, double pointBLat, double pointBLng) {
  _finishALat = pointALat; _finishALng = pointALng;
  _finishBLat = pointBLat; _finishBLng = pointBLng;
  _finishLineConfigured = sprintLineIsValid(pointALat, pointALng, pointBLat, pointBLng);
  if (!_finishLineConfigured) {
    debugln(F("WARNING: invalid finish line (degenerate or non-finite) - detection disabled"));
  }
}

void SprintTimer::setSector2Line(double pointALat, double pointALng, double pointBLat, double pointBLng) {
  _sector2ALat = pointALat; _sector2ALng = pointALng;
  _sector2BLat = pointBLat; _sector2BLng = pointBLng;
  _sector2LineConfigured = sprintLineIsValid(pointALat, pointALng, pointBLat, pointBLng);
  if (!_sector2LineConfigured) {
    debugln(F("WARNING: invalid sector 2 line (degenerate or non-finite) - sector disabled"));
  }
}

void SprintTimer::setSector3Line(double pointALat, double pointALng, double pointBLat, double pointBLng) {
  _sector3ALat = pointALat; _sector3ALng = pointALng;
  _sector3BLat = pointBLat; _sector3BLng = pointBLng;
  _sector3LineConfigured = sprintLineIsValid(pointALat, pointALng, pointBLat, pointBLng);
  if (!_sector3LineConfigured) {
    debugln(F("WARNING: invalid sector 3 line (degenerate or non-finite) - sector disabled"));
  }
}

bool SprintTimer::isStartLineConfigured() const {
  return _startLineConfigured;
}

bool SprintTimer::isFinishLineConfigured() const {
  return _finishLineConfigured;
}

bool SprintTimer::areSectorLinesConfigured() const {
  return _sector2LineConfigured || _sector3LineConfigured;
}

void SprintTimer::forceLinearInterpolation() {
  _startEngine.setForceLinear(true);
  _finishEngine.setForceLinear(true);
  _sector2Engine.setForceLinear(true);
  _sector3Engine.setForceLinear(true);
}

void SprintTimer::forceCatmullRomInterpolation() {
  _startEngine.setForceLinear(false);
  _finishEngine.setForceLinear(false);
  _sector2Engine.setForceLinear(false);
  _sector3Engine.setForceLinear(false);
}

void SprintTimer::reset() {
  debugln(F("Resetting sprint timer..."));
  _raceStarted = false;
  _runActive = false;
  _runs = 0;
  _cancelledRuns = 0;
  _currentRunStartTime = 0;
  _currentRunOdometerStart = 0;
  _lastRunTime = 0;
  _lastRunDistance = 0;
  _bestRunTime = 0;
  _bestRunDistance = 0;
  _bestRunNumber = 0;

  _currentSegment = 0;
  _segmentStartTime = 0;
  _segmentOrderValid = true;
  for (int i = 0; i <= SPRINT_MAX_SPLITS; i++) {
    _currentRunSegTime[i] = 0;
    _bestSegTime[i] = 0;
    _bestSegRunNumber[i] = 0;
  }

  _millisecondsSinceMidnight = 0;
  _totalDistanceTraveled = 0;
  _positionPrevLat = 0;
  _positionPrevLng = 0;
  _positionPrevAlt = 0;
  _firstPositionReceived = false;
  _consecutiveJumpCount = 0;
  _currentSpeedKmh = 0;
  _prevFix.lat = 0;
  _prevFix.lng = 0;
  _prevFix.time = 0;
  _prevFix.odometer = 0;
  _prevFix.speedKmh = 0;
  _hasPrevFix = false;

  _crossingStart = false;
  _crossingFinish = false;
  _crossingSector2 = false;
  _crossingSector3 = false;
  _startEngine.reset();
  _finishEngine.reset();
  _sector2Engine.reset();
  _sector3Engine.reset();
}

/////////// getters

bool SprintTimer::isRunActive() const {
  return _runActive;
}

int SprintTimer::getRuns() const {
  return _runs;
}

int SprintTimer::getCancelledRunCount() const {
  return _cancelledRuns;
}

unsigned long SprintTimer::getCurrentRunTime() const {
  return _runActive ? timeSinceMidnightDelta(_currentRunStartTime, _millisecondsSinceMidnight) : 0;
}

unsigned long SprintTimer::getLastRunTime() const {
  return _lastRunTime;
}

unsigned long SprintTimer::getBestRunTime() const {
  return _bestRunTime;
}

int SprintTimer::getBestRunNumber() const {
  return _bestRunNumber;
}

float SprintTimer::getCurrentRunDistance() const {
  return _runActive ? (float)(_totalDistanceTraveled - _currentRunOdometerStart) : 0;
}

float SprintTimer::getLastRunDistance() const {
  return _lastRunDistance;
}

float SprintTimer::getBestRunDistance() const {
  return _bestRunDistance;
}

bool SprintTimer::getRaceStarted() const {
  return _raceStarted;
}

bool SprintTimer::getCrossing() const {
  return _crossingStart || _crossingFinish || _crossingSector2 || _crossingSector3;
}

float SprintTimer::getTotalDistanceTraveled() const {
  return _totalDistanceTraveled;
}

float SprintTimer::getPaceDifference() const {
  if (!_runActive) {
    return 0.0;
  }
  float currentRunDistance = (float)(_totalDistanceTraveled - _currentRunOdometerStart);
  unsigned long currentRunTime = timeSinceMidnightDelta(_currentRunStartTime, _millisecondsSinceMidnight);

  if (currentRunDistance == 0 || _bestRunDistance == 0) {
    return 0.0;
  }

  float currentRunPace = currentRunTime / currentRunDistance;
  float bestRunPace = _bestRunTime / _bestRunDistance;
  return currentRunPace - bestRunPace;
}

float SprintTimer::getCurrentSpeedKmh() const {
  return _currentSpeedKmh;
}

float SprintTimer::getCurrentSpeedMph() const {
  return _currentSpeedKmh * GEOMATH_KMH_TO_MPH;
}

int SprintTimer::getCurrentSector() const {
  return _runActive ? _currentSegment : 0;
}

unsigned long SprintTimer::getCurrentLapSector1Time() const {
  return _currentRunSegTime[0];
}

unsigned long SprintTimer::getCurrentLapSector2Time() const {
  return _currentRunSegTime[1];
}

unsigned long SprintTimer::getCurrentLapSector3Time() const {
  return _currentRunSegTime[2];
}

unsigned long SprintTimer::getBestSector1Time() const {
  return _bestSegTime[0];
}

unsigned long SprintTimer::getBestSector2Time() const {
  return _bestSegTime[1];
}

unsigned long SprintTimer::getBestSector3Time() const {
  return _bestSegTime[2];
}

int SprintTimer::getBestSector1LapNumber() const {
  return _bestSegRunNumber[0];
}

int SprintTimer::getBestSector2LapNumber() const {
  return _bestSegRunNumber[1];
}

int SprintTimer::getBestSector3LapNumber() const {
  return _bestSegRunNumber[2];
}

unsigned long SprintTimer::getOptimalLapTime() const {
  if (!areSectorLinesConfigured()) {
    return 0;
  }
  int expected = _expectedSegments();
  unsigned long sum = 0;
  for (int i = 0; i < expected; i++) {
    if (_bestSegTime[i] == 0) {
      return 0;
    }
    sum += _bestSegTime[i];
  }
  return sum;
}

unsigned int SprintTimer::getRejectedCrossingCount() const {
  return _startEngine.getRejectedCrossingCount()
       + _finishEngine.getRejectedCrossingCount()
       + _sector2Engine.getRejectedCrossingCount()
       + _sector3Engine.getRejectedCrossingCount();
}
