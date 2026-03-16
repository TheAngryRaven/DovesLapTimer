/**
 * WaypointLapTimer - universal fallback lap timer ("Lap Anything" mode).
 *
 * Uses single-point proximity detection instead of crossing lines.
 * Algorithm:
 *   1. Wait for speed >= 20 mph, drop a waypoint
 *   2. Drive away (min distance traveled)
 *   3. On return to waypoint proximity, buffer approach points
 *   4. On exit from proximity, use closest-approach point's time for lap split
 *   5. Repeat for subsequent laps
 *
 * Duck-typed to match DovesLapTimer's public API so the display/logger
 * can use either timer interchangeably.
 */

#ifndef _WAYPOINT_LAP_TIMER_H
#define _WAYPOINT_LAP_TIMER_H

#include <Arduino.h>
#include "DovesLapTimer.h"
#include "GeoMath.h"

// Internal states
#define WLT_STATE_IDLE           0
#define WLT_STATE_WAITING_SPEED  1
#define WLT_STATE_DRIVING        2
#define WLT_STATE_IN_PROXIMITY   3

struct ProximityBufferEntry {
  double lat;
  double lng;
  unsigned long time;
  float odometer;
  float distToWaypoint;
};

class WaypointLapTimer {
public:
  WaypointLapTimer(Stream *debugSerial = NULL);

  void updateCurrentTime(unsigned long currentTimeMilliseconds);
  int loop(double currentLat, double currentLng, float currentAltitudeMeters, float currentSpeedKnots);
  void reset();
  void setSpeedThresholdMph(float mph);
  void setProximityMeters(float meters);

  // Timing getters (duck-typed to DovesLapTimer)
  bool getRaceStarted() const;
  bool getCrossing() const;
  int getLaps() const;
  unsigned long getCurrentLapTime() const;
  unsigned long getLastLapTime() const;
  unsigned long getBestLapTime() const;
  float getCurrentLapDistance() const;
  float getTotalDistanceTraveled() const;
  int getBestLapNumber() const;
  float getPaceDifference() const;

  float getLastLapDistance() const;
  float getBestLapDistance() const;

  // Waypoint access
  bool hasWaypoint() const;
  double getWaypointLat() const;
  double getWaypointLng() const;

  // Sector getters (not supported, return 0)
  int getCurrentSector() const { return 0; }
  bool areSectorLinesConfigured() const { return false; }
  unsigned long getCurrentLapSector1Time() const { return 0; }
  unsigned long getCurrentLapSector2Time() const { return 0; }
  unsigned long getCurrentLapSector3Time() const { return 0; }
  unsigned long getBestSector1Time() const { return 0; }
  unsigned long getBestSector2Time() const { return 0; }
  unsigned long getBestSector3Time() const { return 0; }
  unsigned long getOptimalLapTime() const { return 0; }
  int getBestSector1LapNumber() const { return 0; }
  int getBestSector2LapNumber() const { return 0; }
  int getBestSector3LapNumber() const { return 0; }

  // Direction (not applicable)
  int getDirection() const { return DIR_UNKNOWN; }
  bool isDirectionResolved() const { return false; }

private:
  template<typename... Args>
  void debug_print(Args&&... args) {
    if(_serial) { _serial->print(std::forward<Args>(args)...); }
  }
  template<typename... Args>
  void debug_println(Args&&... args) {
    if(_serial) { _serial->println(std::forward<Args>(args)...); }
  }

  void _resetState();
  void _checkSpeed(double lat, double lng);
  void _checkProximity(double lat, double lng);
  void _bufferProximityPoint(double lat, double lng);
  void _clearProximityBuffer();
  void _processProximityBuffer();

  Stream *_serial;

  int _state;
  unsigned long _millisecondsSinceMidnight;

  // Waypoint
  double _waypointLat;
  double _waypointLng;
  float _waypointOdometer;

  // Odometer / position
  float _totalDistanceTraveled;
  double _positionPrevLat;
  double _positionPrevLng;
  bool _firstPositionReceived;
  float _currentSpeedKmh;
  float _speedThresholdMph;
  float _proximityMeters;

  // Timing
  bool _raceStarted;
  bool _crossing;
  unsigned long _currentLapStartTime;
  unsigned long _lastLapTime;
  unsigned long _bestLapTime;
  float _currentLapOdometerStart;
  float _lastLapDistance;
  float _bestLapDistance;
  int _bestLapNumber;
  int _laps;

  // Proximity buffer
  ProximityBufferEntry _proximityBuffer[WAYPOINT_LAP_BUFFER_SIZE];
  int _proximityBufferIndex;
  int _proximityBufferCount;
  float _closestDist;
  unsigned long _closestTime;
  float _closestOdometer;
};

#endif
