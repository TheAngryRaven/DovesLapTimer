/**
 * WaypointLapTimer - universal fallback lap timer ("Lap Anything" mode).
 * Ported from js/lib/waypoint-lap-timer.js
 */

#include "WaypointLapTimer.h"

#define debugln debug_println
#define debug debug_print

WaypointLapTimer::WaypointLapTimer(Stream *debugSerial) {
  _serial = debugSerial;
  _speedThresholdMph = COURSE_DETECT_SPEED_THRESHOLD_MPH;
  _proximityMeters = WAYPOINT_LAP_PROXIMITY_METERS;
  _resetState();
}

void WaypointLapTimer::_resetState() {
  _state = WLT_STATE_IDLE;
  _millisecondsSinceMidnight = 0;

  _waypointLat = 0;
  _waypointLng = 0;
  _waypointOdometer = 0;

  _totalDistanceTraveled = 0;
  _positionPrevLat = 0;
  _positionPrevLng = 0;
  _firstPositionReceived = false;
  _currentSpeedKmh = 0;
  // _speedThresholdMph and _proximityMeters are NOT reset here —
  // they are set once in the constructor and preserved through reset()
  // to match CourseDetector behavior. User-configured values via
  // setSpeedThresholdMph() / setProximityMeters() survive reset.

  _raceStarted = false;
  _crossing = false;
  _currentLapStartTime = 0;
  _lastLapTime = 0;
  _bestLapTime = 0;
  _currentLapOdometerStart = 0;
  _lastLapDistance = 0;
  _bestLapDistance = 0;
  _bestLapNumber = 0;
  _laps = 0;

  _clearProximityBuffer();
}

void WaypointLapTimer::updateCurrentTime(unsigned long currentTimeMilliseconds) {
  _millisecondsSinceMidnight = currentTimeMilliseconds;
}

int WaypointLapTimer::loop(double currentLat, double currentLng, float currentAltitudeMeters, float currentSpeedKnots) {
  // Update odometer
  if (_firstPositionReceived) {
    double dist = geoHaversine(_positionPrevLat, _positionPrevLng, currentLat, currentLng);
    _totalDistanceTraveled += dist;
  } else {
    _firstPositionReceived = true;
  }

  _positionPrevLat = currentLat;
  _positionPrevLng = currentLng;
  _currentSpeedKmh = currentSpeedKnots * 1.852;

  // State machine
  if (_state == WLT_STATE_IDLE) {
    _state = WLT_STATE_WAITING_SPEED;
  }

  if (_state == WLT_STATE_WAITING_SPEED) {
    _checkSpeed(currentLat, currentLng);
  }

  if (_state == WLT_STATE_DRIVING) {
    _checkProximity(currentLat, currentLng);
  }

  if (_state == WLT_STATE_IN_PROXIMITY) {
    _bufferProximityPoint(currentLat, currentLng);
  }

  return -1;
}

void WaypointLapTimer::reset() {
  _resetState();
}

void WaypointLapTimer::setSpeedThresholdMph(float mph) {
  if (mph > 0) _speedThresholdMph = mph;
}

void WaypointLapTimer::setProximityMeters(float meters) {
  if (meters > 0) _proximityMeters = meters;
}

void WaypointLapTimer::_checkSpeed(double lat, double lng) {
  float speedMph = _currentSpeedKmh * 0.621371;

  if (speedMph >= _speedThresholdMph) {
    _waypointLat = lat;
    _waypointLng = lng;
    _waypointOdometer = _totalDistanceTraveled;
    _currentLapStartTime = _millisecondsSinceMidnight;
    _currentLapOdometerStart = _totalDistanceTraveled;
    _state = WLT_STATE_DRIVING;
    debugln(F("Waypoint set (Lap Anything mode)"));
  }
}

void WaypointLapTimer::_checkProximity(double lat, double lng) {
  float distSinceWaypoint = _totalDistanceTraveled - _waypointOdometer;

  if (distSinceWaypoint < WAYPOINT_LAP_MIN_DISTANCE_METERS) {
    return;
  }

  double distToWp = geoHaversine(lat, lng, _waypointLat, _waypointLng);

  if (distToWp < _proximityMeters) {
    _state = WLT_STATE_IN_PROXIMITY;
    _crossing = true;
    _clearProximityBuffer();
    _bufferProximityPoint(lat, lng);
    debugln(F("Entered waypoint proximity"));
  }
}

void WaypointLapTimer::_bufferProximityPoint(double lat, double lng) {
  double distToWp = geoHaversine(lat, lng, _waypointLat, _waypointLng);

  // Check if we've exited proximity
  if (distToWp >= _proximityMeters) {
    _state = WLT_STATE_DRIVING;
    _crossing = false;
    _processProximityBuffer();
    return;
  }

  // Buffer this point
  int idx = _proximityBufferIndex % WAYPOINT_LAP_BUFFER_SIZE;
  _proximityBuffer[idx].lat = lat;
  _proximityBuffer[idx].lng = lng;
  _proximityBuffer[idx].time = _millisecondsSinceMidnight;
  _proximityBuffer[idx].odometer = _totalDistanceTraveled;
  _proximityBuffer[idx].distToWaypoint = distToWp;
  _proximityBufferIndex++;
  if (_proximityBufferCount < WAYPOINT_LAP_BUFFER_SIZE) {
    _proximityBufferCount++;
  }

  // Track closest approach
  if (distToWp < _closestDist) {
    _closestDist = distToWp;
    _closestTime = _millisecondsSinceMidnight;
    _closestOdometer = _totalDistanceTraveled;
  }
}

void WaypointLapTimer::_clearProximityBuffer() {
  _proximityBufferIndex = 0;
  _proximityBufferCount = 0;
  _closestDist = INFINITY;
  _closestTime = 0;
  _closestOdometer = 0;
}

void WaypointLapTimer::_processProximityBuffer() {
  if (_closestTime == 0) {
    debugln(F("No valid closest point found"));
    return;
  }

  unsigned long crossingTime = _closestTime;
  float crossingOdometer = _closestOdometer;

  if (_raceStarted) {
    _laps++;
    unsigned long lapTime = crossingTime - _currentLapStartTime;
    float lapDistance = crossingOdometer - _currentLapOdometerStart;

    _lastLapTime = lapTime;
    _lastLapDistance = lapDistance;
    _currentLapStartTime = crossingTime;
    _currentLapOdometerStart = crossingOdometer;

    if (_bestLapTime == 0 || lapTime < _bestLapTime) {
      _bestLapTime = lapTime;
      _bestLapDistance = lapDistance;
      _bestLapNumber = _laps;
    }

    debug(F("Lap "));
    debug(_laps);
    debug(F(": "));
    debugln((double)(lapTime / 1000.0), 3);
  } else {
    _raceStarted = true;
    _laps++;
    unsigned long lapTime = crossingTime - _currentLapStartTime;
    float lapDistance = crossingOdometer - _currentLapOdometerStart;

    _lastLapTime = lapTime;
    _lastLapDistance = lapDistance;
    _currentLapStartTime = crossingTime;
    _currentLapOdometerStart = crossingOdometer;

    if (_bestLapTime == 0 || lapTime < _bestLapTime) {
      _bestLapTime = lapTime;
      _bestLapDistance = lapDistance;
      _bestLapNumber = _laps;
    }

    debug(F("Lap 1 (from waypoint): "));
    debugln((double)(lapTime / 1000.0), 3);
  }

  // Update waypoint odometer for next lap
  _waypointOdometer = crossingOdometer;
}

/////////// getters

bool WaypointLapTimer::getRaceStarted() const {
  return _raceStarted;
}

bool WaypointLapTimer::getCrossing() const {
  return _crossing;
}

int WaypointLapTimer::getLaps() const {
  return _laps;
}

unsigned long WaypointLapTimer::getCurrentLapTime() const {
  return (_currentLapStartTime <= 0 || !_raceStarted)
    ? 0
    : _millisecondsSinceMidnight - _currentLapStartTime;
}

unsigned long WaypointLapTimer::getLastLapTime() const {
  return _lastLapTime;
}

unsigned long WaypointLapTimer::getBestLapTime() const {
  return _bestLapTime;
}

float WaypointLapTimer::getCurrentLapDistance() const {
  return (_currentLapOdometerStart == 0 || !_raceStarted)
    ? 0
    : _totalDistanceTraveled - _currentLapOdometerStart;
}

float WaypointLapTimer::getTotalDistanceTraveled() const {
  return _totalDistanceTraveled;
}

int WaypointLapTimer::getBestLapNumber() const {
  return _bestLapNumber;
}

float WaypointLapTimer::getPaceDifference() const {
  float currentLapDist = getCurrentLapDistance();
  unsigned long currentLapTime = getCurrentLapTime();

  if (currentLapDist == 0 || _bestLapDistance == 0) {
    return 0.0;
  }

  float currentPace = currentLapTime / currentLapDist;
  float bestPace = _bestLapTime / _bestLapDistance;
  return currentPace - bestPace;
}

float WaypointLapTimer::getLastLapDistance() const {
  return _lastLapDistance;
}

float WaypointLapTimer::getBestLapDistance() const {
  return _bestLapDistance;
}

bool WaypointLapTimer::hasWaypoint() const {
  return _state != WLT_STATE_IDLE && _state != WLT_STATE_WAITING_SPEED;
}

double WaypointLapTimer::getWaypointLat() const {
  return _waypointLat;
}

double WaypointLapTimer::getWaypointLng() const {
  return _waypointLng;
}
