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

  _crossingEngine.setThreshold(crossingThresholdMeters);
  _crossingEngine.setDebugSerial(_serial);
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
  prevFix.lat = currentLat;
  prevFix.lng = currentLng;
  prevFix.time = millisecondsSinceMidnight;
  prevFix.odometer = totalDistanceTraveled;
  prevFix.speedKmh = currentSpeedkmh;
  hasPrevFix = true;

  return nearAnyLine ? 0 : -1;
}

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
 * Thin wrapper over CrossingEngine::detect — the detection state machine and
 * interpolation now live in CrossingEngine.cpp (extracted, not rewritten).
 * This wrapper supplies the timer's vehicle state so the two callers
 * (checkStartFinish / checkSectorLine) keep their historical call shape.
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
  return _crossingEngine.detect(
      currentLat, currentLng,
      millisecondsSinceMidnight,
      totalDistanceTraveled,
      currentSpeedkmh,
      hasPrevFix ? &prevFix : NULL,
      pointALat, pointALng,
      pointBLat, pointBLng,
      crossingFlag,
      lineLabel,
      outLat, outLng, outTime, outOdometer);
}

bool DovesLapTimer::insideLineThreshold(double driverLat, double driverLon, double crossingPointALat, double crossingPointALon, double crossingPointBLat, double crossingPointBLon) {
  return geoInsideLineThreshold(crossingThresholdMeters, driverLat, driverLon, crossingPointALat, crossingPointALon, crossingPointBLat, crossingPointBLon);
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
  return geoPointOnSideOfLine(driverLat, driverLng, pointALat, pointALng, pointBLat, pointBLng);
}

double DovesLapTimer::pointLineSegmentDistance(double pointX, double pointY, double startX, double startY, double endX, double endY) {
  return geoPointLineSegmentDistance(pointX, pointY, startX, startY, endX, endY);
}

double DovesLapTimer::haversine(double lat1, double lon1, double lat2, double lon2) {
  return geoHaversine(lat1, lon1, lat2, lon2);
}

double DovesLapTimer::haversine3D(double prevLat, double prevLng, double prevAlt, double currentLat, double currentLng, double currentAlt) {
  return geoHaversine3D(prevLat, prevLng, prevAlt, currentLat, currentLng, currentAlt);
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
  prevFix.lat = 0;
  prevFix.lng = 0;
  prevFix.time = 0;
  prevFix.odometer = 0;
  prevFix.speedKmh = 0;
  hasPrevFix = false;

  // Reset the crossing engine (buffer + rejected-crossing counter)
  crossing = false;
  _crossingEngine.reset();
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
  _crossingEngine.setForceLinear(true);
}
void DovesLapTimer::forceCatmullRomInterpolation() {
  _crossingEngine.setForceLinear(false);
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
  return _crossingEngine.getRejectedCrossingCount();
}

/////////// direction detection getters

int DovesLapTimer::getDirection() const {
  return _directionDetector.direction;
}
bool DovesLapTimer::isDirectionResolved() const {
  return _directionDetector.direction != DIR_UNKNOWN;
}