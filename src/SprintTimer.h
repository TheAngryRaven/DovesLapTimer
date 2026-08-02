/**
 * SprintTimer - point-to-point run timing ("sprint mode": autocross,
 * hillclimb, rally-stage style events).
 *
 * Unlike DovesLapTimer there are no laps: a run begins at a START line and
 * ends at a SEPARATE FINISH line, with up to two optional split (sector)
 * lines between them. Multiple runs happen per session — the driver loops
 * back around to the start and goes again.
 *
 * Run semantics (deliberately simple, two states):
 *   - WAITING:  a start-line crossing begins a run.
 *   - RUNNING:  a finish-line crossing completes the run;
 *               a START-line crossing CANCELS the run in progress and starts
 *               a brand new one (botched-course re-launch — the sprint
 *               equivalent of circuit's every-S/F-crossing-closes-and-opens).
 *   - Finish crossings while WAITING are ignored (driving back past the
 *     finish on the return loop must not do anything).
 *   - A DNF is just normal operation: an abandoned run never completes and
 *     records nothing.
 *
 * ALL lines are monitored on every fix — there is no mutual exclusion.
 * Every line owns its own CrossingEngine (buffer + interpolation), so the
 * start and finish zones may sit arbitrarily close together (common at
 * autocross paddocks) without contending for a shared buffer. The cost is
 * RAM: ~2.9 KB per configured line's buffer on 32-bit targets (~11.6 KB for
 * a fully-configured instance). SprintTimer is meant to be instantiated
 * once, not pooled like CourseManager's 8-slot timer array. Not suitable
 * for AVR Uno; tight on Mega.
 *
 * Sector/split model: segments are numbered 1..N in crossing order, where
 * N = 1 + (number of configured splits). Sector 2 (if configured) always
 * closes segment 1; sector 3 closes the next segment; the finish closes the
 * last. A split crossed out of order, or missed entirely, invalidates the
 * run's SECTOR data (best-segment tracking skips that run) but never the
 * run time itself. Zero, one, or two splits are all legal — unlike the
 * circuit timer there is no both-or-nothing sector gate.
 *
 * Direction note: crossing detection is direction-agnostic. A backward
 * start crossing on the return loop opens a bogus run, which the
 * cancel-and-restart rule self-heals at the real launch. A backward FINISH
 * crossing while RUNNING is the only truly bogus completion; it requires
 * re-entering the finish zone mid-run, which a one-way sprint course
 * doesn't do.
 *
 * Duck-typed to DovesLapTimer's public getter surface (laps == runs) so
 * displays/loggers can consume either timer interchangeably; run-native
 * aliases (getRuns, getBestRunTime, ...) are provided for new code.
 */

#ifndef _DOVES_SPRINT_TIMER_H
#define _DOVES_SPRINT_TIMER_H

#include <Arduino.h>
#include "GeoMath.h"
#include "CrossingEngine.h"
#include "DovesLapTimer.h"  // DIR_UNKNOWN, GPS_* validation constants

// The maximum number of split (sector) lines between start and finish.
#define SPRINT_MAX_SPLITS  2

class SprintTimer {
public:
  SprintTimer(double crossingThresholdMeters = 7, Stream *debugSerial = NULL);

  /**
   * @brief Feed a GPS fix. Call updateCurrentTime() first, then this, on
   * every fix — identical contract to DovesLapTimer::loop().
   *
   * @param currentLat Latitude in decimal degrees.
   * @param currentLng Longitude in decimal degrees.
   * @param currentAltitudeMeters Altitude in meters.
   * @param currentSpeedKnots Speed in knots.
   * @return 0 if near any configured line this fix, -1 otherwise (or the
   *   fix was rejected as invalid).
   */
  int loop(double currentLat, double currentLng, float currentAltitudeMeters, float currentSpeedKnots);
  /**
   * @brief Updates the current GPS time since midnight (milliseconds).
   */
  void updateCurrentTime(unsigned long currentTimeMilliseconds);
  /**
   * @brief Reset all run/timing state to zero. Configured lines are kept.
   */
  void reset();

  /////////////////////////////////////////////////////////////////////////////
  // Configuration

  /**
   * @brief Sets the START line (a run begins when it is crossed).
   */
  void setStartLine(double pointALat, double pointALng, double pointBLat, double pointBLng);
  /**
   * @brief Sets the FINISH line (a run completes when it is crossed while
   * a run is active).
   */
  void setFinishLine(double pointALat, double pointALng, double pointBLat, double pointBLng);
  /**
   * @brief Sets the optional sector 2 split line (closes segment 1).
   */
  void setSector2Line(double pointALat, double pointALng, double pointBLat, double pointBLng);
  /**
   * @brief Sets the optional sector 3 split line (closes the segment after
   * sector 2's, or segment 1 if sector 2 is not configured).
   */
  void setSector3Line(double pointALat, double pointALng, double pointBLat, double pointBLng);

  bool isStartLineConfigured() const;
  bool isFinishLineConfigured() const;
  /**
   * @brief True when at least one split line is configured (note: NOT the
   * circuit timer's both-or-nothing rule — a single split is legal).
   */
  bool areSectorLinesConfigured() const;

  void forceLinearInterpolation();
  void forceCatmullRomInterpolation();

  /////////////////////////////////////////////////////////////////////////////
  // Run-native surface

  /** @brief True while a run is in progress (between start and finish). */
  bool isRunActive() const;
  /** @brief Completed run count this session. */
  int getRuns() const;
  /** @brief Runs cancelled by re-crossing the start line mid-run. */
  int getCancelledRunCount() const;
  /** @brief Elapsed ms of the run in progress, 0 while WAITING. */
  unsigned long getCurrentRunTime() const;
  unsigned long getLastRunTime() const;
  unsigned long getBestRunTime() const;
  /** @brief 1-based run number that set the best time (0 = none yet). */
  int getBestRunNumber() const;
  float getCurrentRunDistance() const;
  float getLastRunDistance() const;
  float getBestRunDistance() const;

  /////////////////////////////////////////////////////////////////////////////
  // Duck-typed DovesLapTimer surface (laps == runs)

  /** @brief True once any run has ever started this session. */
  bool getRaceStarted() const;
  /** @brief True while inside any configured line's crossing zone. */
  bool getCrossing() const;
  int getLaps() const { return getRuns(); }
  unsigned long getCurrentLapTime() const { return getCurrentRunTime(); }
  unsigned long getLastLapTime() const { return getLastRunTime(); }
  unsigned long getBestLapTime() const { return getBestRunTime(); }
  int getBestLapNumber() const { return getBestRunNumber(); }
  float getCurrentLapDistance() const { return getCurrentRunDistance(); }
  float getLastLapDistance() const { return getLastRunDistance(); }
  float getBestLapDistance() const { return getBestRunDistance(); }
  float getTotalDistanceTraveled() const;
  /**
   * @brief Pace delta vs the best run in ms per meter (see
   * DovesLapTimer::getPaceDifference). 0 while WAITING.
   */
  float getPaceDifference() const;
  float getCurrentSpeedKmh() const;
  float getCurrentSpeedMph() const;

  /**
   * @brief Current segment (1..N) while a run is active, 0 while WAITING.
   */
  int getCurrentSector() const;
  unsigned long getCurrentLapSector1Time() const;
  unsigned long getCurrentLapSector2Time() const;
  unsigned long getCurrentLapSector3Time() const;
  unsigned long getBestSector1Time() const;
  unsigned long getBestSector2Time() const;
  unsigned long getBestSector3Time() const;
  int getBestSector1LapNumber() const;
  int getBestSector2LapNumber() const;
  int getBestSector3LapNumber() const;
  /**
   * @brief Sum of best segment times (theoretical best run). 0 unless at
   * least one split is configured and every segment has a best recorded.
   */
  unsigned long getOptimalLapTime() const;

  /** @brief Total zone exits whose interpolation was rejected, all lines. */
  unsigned int getRejectedCrossingCount() const;

  // Direction (not applicable point-to-point)
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

  /** @brief Segments in a complete run: 1 + configured splits. */
  int _expectedSegments() const;
  /** @brief Which segment a given split line (2 or 3) closes. */
  int _segmentClosedBySector(int sectorLabel) const;
  void _handleStartCrossing(unsigned long crossingTime, double crossingOdometer);
  void _handleFinishCrossing(unsigned long crossingTime, double crossingOdometer);
  void _handleSplitCrossing(int sectorLabel, unsigned long crossingTime);
  void _updateBestSegments();

  Stream *_serial;
  double _crossingThresholdMeters;

  // One engine per line: independent buffers, so zones may overlap and all
  // lines stay hot on every fix. This is the deliberate opposite of
  // DovesLapTimer's shared-buffer design — see the header comment.
  CrossingEngine _startEngine;
  CrossingEngine _finishEngine;
  CrossingEngine _sector2Engine;
  CrossingEngine _sector3Engine;

  // Line endpoints + configuration + in-zone flags
  double _startALat = 0, _startALng = 0, _startBLat = 0, _startBLng = 0;
  double _finishALat = 0, _finishALng = 0, _finishBLat = 0, _finishBLng = 0;
  double _sector2ALat = 0, _sector2ALng = 0, _sector2BLat = 0, _sector2BLng = 0;
  double _sector3ALat = 0, _sector3ALng = 0, _sector3BLat = 0, _sector3BLng = 0;
  bool _startLineConfigured = false;
  bool _finishLineConfigured = false;
  bool _sector2LineConfigured = false;
  bool _sector3LineConfigured = false;
  bool _crossingStart = false;
  bool _crossingFinish = false;
  bool _crossingSector2 = false;
  bool _crossingSector3 = false;

  // Vehicle state (same intake pipeline as DovesLapTimer::loop)
  unsigned long _millisecondsSinceMidnight = 0;
  float _totalDistanceTraveled = 0;
  double _positionPrevLat = 0;
  double _positionPrevLng = 0;
  float _positionPrevAlt = 0;
  bool _firstPositionReceived = false;
  int _consecutiveJumpCount = 0;
  float _currentSpeedKmh = 0;
  crossingPointBufferEntry _prevFix = {0, 0, 0, 0, 0};
  bool _hasPrevFix = false;

  // Run state
  bool _raceStarted = false;   // any run ever started this session
  bool _runActive = false;     // RUNNING vs WAITING
  int _runs = 0;
  int _cancelledRuns = 0;
  unsigned long _currentRunStartTime = 0;
  double _currentRunOdometerStart = 0;
  unsigned long _lastRunTime = 0;
  float _lastRunDistance = 0;
  unsigned long _bestRunTime = 0;
  float _bestRunDistance = 0;
  int _bestRunNumber = 0;

  // Segment (split) state for the run in progress
  int _currentSegment = 0;             // 0 = WAITING, 1..N while RUNNING
  unsigned long _segmentStartTime = 0;
  bool _segmentOrderValid = true;
  unsigned long _currentRunSegTime[SPRINT_MAX_SPLITS + 1] = {0, 0, 0};

  // Best segment times across runs (valid, in-order runs only)
  unsigned long _bestSegTime[SPRINT_MAX_SPLITS + 1] = {0, 0, 0};
  int _bestSegRunNumber[SPRINT_MAX_SPLITS + 1] = {0, 0, 0};
};

#endif
