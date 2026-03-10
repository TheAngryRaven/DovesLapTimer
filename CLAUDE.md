# DovesLapTimer - Project Guide

> **IMPORTANT**: Always keep this CLAUDE.md file updated when making changes to the codebase.
> When adding new features, modifying APIs, fixing bugs, or changing architecture, update the
> relevant sections below so future sessions don't waste tokens re-exploring the project.

## Project Overview

**What**: GPS-based lap timing Arduino library for go-kart / racing applications.
**Author**: Michael Champagne (crimsondove)
**Version**: 4.0.0
**Repo**: https://github.com/TheAngryRaven/DovesLapTimer
**License**: GPL v3
**Dependency**: ArxTypeTraits (auto-included by Arduino Library Manager)

The library does NOT interface with GPS hardware directly. You feed it coordinates and time,
it handles crossing detection, lap timing, sector splits, distance tracking, pace comparison,
course detection, direction detection, and waypoint-based fallback timing.

## Directory Structure

```
DovesLapTimer/
├── src/
│   ├── DovesLapTimer.h          # Header - class definition, structs, constants, DirectionDetector
│   ├── DovesLapTimer.cpp        # Implementation - crossing detection, sector timing, direction
│   ├── GeoMath.h                # Shared haversine/haversine3D (static inline, no .cpp needed)
│   ├── WaypointLapTimer.h       # Single-point proximity-based lap timer ("Lap Anything")
│   ├── WaypointLapTimer.cpp     # WaypointLapTimer implementation
│   ├── CourseDetector.h         # Course detection state machine
│   ├── CourseDetector.cpp       # CourseDetector implementation
│   ├── CourseManager.h          # Multi-timer orchestrator
│   └── CourseManager.cpp        # CourseManager implementation
├── examples/
│   ├── basic_oled_example/      # Full working example with OLED + uBlox GPS
│   │   ├── basic_oled_example.ino
│   │   ├── display_config.h     # OLED init (SSD1306 or SH110X)
│   │   ├── gps_config.h         # uBlox binary config commands
│   │   └── images.h             # Bitmap data for UI
│   ├── sector_timing_example/   # Demonstrates 3-sector timing
│   │   └── sector_timing_example.ino
│   └── real_track_data_debug/   # Replays real NMEA data (no GPS needed)
│       ├── real_track_data_debug.ino
│       ├── gps_race_data_2laps.h
│       ├── gps_race_data_lap.h
│       ├── gps_race_data_long_lap.h
│       └── gps_race_data_praga_laps.h
├── wokwi/                       # Wokwi simulator custom GPS chip
│   ├── gps-fake.chip.c
│   └── gps-fake.chip.json
├── images/                      # Diagrams for HELPME.md
├── library.properties           # Arduino library metadata
├── README.md                    # User-facing docs and API reference
├── HELPME.md                    # Technical deep-dive on crossing detection algorithm
├── LICENSE                      # GPL v3
└── .github/FUNDING.yml
```

## Architecture & Key Concepts

### Class Hierarchy (v4.0)

```
CourseManager (orchestrator)
├── DovesLapTimer[MAX_COURSES]   # One per course layout, line-crossing detection
│   └── DirectionDetector        # Inline struct, detects forward/reverse
├── CourseDetector               # State machine: speed → waypoint → distance match
├── WaypointLapTimer             # Fallback "Lap Anything" proximity-based timer
└── GeoMath.h                    # Shared static haversine functions
```

### Core Class: `DovesLapTimer`

**Constructor**: `DovesLapTimer(double crossingThresholdMeters = 7, Stream *debugSerial = NULL)`

### Main Loop Flow
1. User calls `updateCurrentTime(millisecondsSinceMidnight)` with GPS time
2. User calls `loop(lat, lng, altMeters, speedKnots)` on each GPS fix
3. `loop()` updates odometer (haversine3D), speed, then checks all crossing lines
4. `checkStartFinish()` and `checkSectorLine()` handle detection + interpolation

### Crossing Detection Algorithm (the hard part)
- Uses **hypotenuse-based threshold** (not simple distance-to-line)
- Calculates hypotenuse from `crossingLineWidth` and `crossingThresholdMeters`
- Checks driver distance to EACH endpoint of crossing line vs hypotenuse
- When inside threshold: buffers GPS points in `crossingPointBuffer`
- When exiting threshold: calls `interpolateCrossingPoint()` to find exact crossing

### Interpolation Methods
- **Linear** (default, `forceLinear = true`): distance+speed weighted interpolation
- **Catmull-Rom spline**: uses 4 control points, falls back to linear if insufficient points

### Sector Timing
- 3 sectors: Start/Finish -> Sector2 -> Sector3 -> Start/Finish
- Requires both sector 2 and sector 3 lines to be configured
- Validates crossing order (rejects out-of-order crossings)
- Tracks best sector times and lap numbers independently

### Direction Detection (v4.0)
- `DirectionDetector` struct inline in `DovesLapTimer.h`
- After start/finish crossed (`raceSeen = true`), first sector line determines direction
- S2 first = forward (`DIR_FORWARD`), S3 first = reverse (`DIR_REVERSE`)
- Once resolved, direction is locked. `handleLineCrossing()` remaps S2↔S3 when reverse
- Getters: `getDirection()`, `isDirectionResolved()`

### Course Detection (v4.0)
- `CourseDetector` state machine: IDLE → WAITING_FOR_SPEED → WAYPOINT_SET → CANDIDATES_READY → DETECTED
- Drops waypoint when speed >= 20 mph, waits for return within 10m after 200m+ traveled
- Compares driven distance (feet) to each course's `lengthFt` within 25% tolerance
- Builds ranked candidates, CourseManager validates via `raceStarted` sanity check

### WaypointLapTimer ("Lap Anything") (v4.0)
- Fallback when no course is detected (after 3 rejections)
- Drops waypoint at speed, uses proximity buffer (30m zone) for closest-approach timing
- Duck-typed to match DovesLapTimer's public API
- No sector support (getters return 0)

### CourseManager (v4.0)
- Orchestrates multiple DovesLapTimers + CourseDetector + WaypointLapTimer
- Feeds ALL active timers the same GPS data simultaneously
- Validates detection candidates via `raceStarted` check
- Falls back to Lap Anything after `COURSE_DETECT_MAX_REJECTIONS` (3) failures
- `pruneInactiveCourses()` deactivates non-detected timers to save memory

### Memory Management
- Circular buffer `crossingPointBuffer` sized by available RAM:
  - `>3000 bytes RAM`: 100 entries
  - `<=3000 bytes RAM`: 25 entries
- Buffer is shared between all line crossings (start/finish + sectors)
- Mutual exclusion: only one line can be "crossing" at a time
- CourseManager peak: ~24 KB during detection (8 courses), drops to ~5 KB after pruning

### Shared GeoMath (v4.0)
- `GeoMath.h` provides `geoHaversine()` and `geoHaversine3D()` as `static inline` functions
- Used by WaypointLapTimer and CourseDetector (no DovesLapTimer instance needed)
- DovesLapTimer retains its own `haversine()`/`haversine3D()` methods for backward compat

## Public API Quick Reference

### DovesLapTimer Setup Methods
| Method | Description |
|--------|-------------|
| `setStartFinishLine(aLat, aLng, bLat, bLng)` | Define start/finish line |
| `setSector2Line(aLat, aLng, bLat, bLng)` | Define sector 2 split line |
| `setSector3Line(aLat, aLng, bLat, bLng)` | Define sector 3 split line |
| `updateCurrentTime(millisSinceMidnight)` | Set GPS time (call before loop) |
| `forceLinearInterpolation()` | Use linear interpolation (default) |
| `forceCatmullRomInterpolation()` | Use Catmull-Rom spline |
| `reset()` | Reset all state to zero |

### DovesLapTimer Timing Getters
| Method | Returns |
|--------|---------|
| `getCurrentLapTime()` | Current lap elapsed ms |
| `getLastLapTime()` | Last completed lap ms |
| `getBestLapTime()` | Best lap ms |
| `getCurrentLapSector1Time()` | Current lap S1 ms |
| `getCurrentLapSector2Time()` | Current lap S2 ms |
| `getCurrentLapSector3Time()` | Current lap S3 ms |
| `getBestSector1Time()` | Best S1 ms (any lap) |
| `getBestSector2Time()` | Best S2 ms (any lap) |
| `getBestSector3Time()` | Best S3 ms (any lap) |
| `getOptimalLapTime()` | Sum of best sectors |

### DovesLapTimer State/Distance Getters
| Method | Returns |
|--------|---------|
| `getRaceStarted()` | True after first line crossing |
| `getCrossing()` | True while inside crossing zone |
| `getLaps()` | Completed lap count |
| `getBestLapNumber()` | Lap # of best time |
| `getCurrentSector()` | 0=not started, 1/2/3 |
| `areSectorLinesConfigured()` | True if both sector lines set |
| `getCurrentLapDistance()` | Current lap meters |
| `getLastLapDistance()` | Last lap meters |
| `getBestLapDistance()` | Best lap meters |
| `getTotalDistanceTraveled()` | Odometer meters |
| `getPaceDifference()` | Pace delta vs best lap |
| `getDirection()` | DIR_UNKNOWN/DIR_FORWARD/DIR_REVERSE |
| `isDirectionResolved()` | True once direction is known |

### WaypointLapTimer API (duck-typed to DovesLapTimer)
Same timing/state getters as DovesLapTimer. Sector getters return 0. Additional:
| Method | Returns |
|--------|---------|
| `hasWaypoint()` | True if waypoint has been set |
| `getWaypointLat()` | Waypoint latitude |
| `getWaypointLng()` | Waypoint longitude |

### CourseDetector API
| Method | Returns |
|--------|---------|
| `init(CourseInfo*, count)` | Initialize with course array |
| `update(lat, lng, speedKmh, odometer)` | Drive state machine |
| `acceptCandidate(index)` | Accept detected course |
| `rejectAllCandidates()` | Reject, return to waypoint_set |
| `getState()` | Current detection state |
| `isDetected()` | True if course detected |
| `getDetectedCourseIndex()` | Index of detected course |

### CourseManager API
| Method | Returns |
|--------|---------|
| `updateCurrentTime(ms)` | Feed time to all timers |
| `loop(lat, lng, alt, speedKnots)` | Feed GPS to all timers + detector |
| `reset()` | Reset everything |
| `pruneInactiveCourses()` | Free non-detected timer memory |
| `isDetectionComplete()` | True if course detected or Lap Anything active |
| `getActiveTimer()` | Pointer to detected course's DovesLapTimer |
| `getLapAnythingTimer()` | Pointer to WaypointLapTimer |
| `isLapAnythingActive()` | True if in Lap Anything fallback mode |
| `getActiveCourseName()` | Name of detected course or "Lap Anything" |

### Config Structs (for CourseManager)
```cpp
struct CourseConfig {
  const char* name; float lengthFt;
  double startALat, startALng, startBLat, startBLng;
  double sector2ALat, sector2ALng, sector2BLat, sector2BLng;
  double sector3ALat, sector3ALng, sector3BLat, sector3BLng;
  bool hasSector2, hasSector3;
};

struct TrackConfig {
  const char* longName; const char* shortName;
  CourseConfig courses[MAX_COURSES]; int courseCount;
};
```

### Public Utility Methods (also used by examples for display)
| Method | Description |
|--------|-------------|
| `insideLineThreshold(...)` | Check if point is in crossing detection zone |
| `pointLineSegmentDistance(...)` | Distance from point to line segment (meters) |
| `haversine(lat1, lon1, lat2, lon2)` | Great-circle distance (meters) |
| `haversine3D(...)` | 3D distance including altitude |
| `isObtuseTriangle(...)` | Triangle type check |
| `pointOnSideOfLine(...)` | Which side of line a point is on |

## Constants (defined in DovesLapTimer.h)

| Constant | Value | Description |
|----------|-------|-------------|
| `MAX_COURSES` | 8 | Max course layouts per track |
| `COURSE_DETECT_SPEED_THRESHOLD_MPH` | 20 | Speed to trigger waypoint drop |
| `COURSE_DETECT_WAYPOINT_PROXIMITY_METERS` | 10.0 | Return-to-waypoint trigger distance |
| `COURSE_DETECT_MIN_DISTANCE_METERS` | 200.0 | Min travel before proximity check |
| `COURSE_DETECT_DISTANCE_TOLERANCE_PCT` | 0.25 | 25% tolerance for distance matching |
| `COURSE_DETECT_MAX_REJECTIONS` | 3 | Rejections before Lap Anything fallback |
| `WAYPOINT_LAP_MIN_DISTANCE_METERS` | 100.0 | Min travel for waypoint lap timer |
| `WAYPOINT_LAP_PROXIMITY_METERS` | 30.0 | Proximity zone for waypoint timing |
| `WAYPOINT_LAP_BUFFER_SIZE` | 50 | Proximity buffer entries |
| `DIR_UNKNOWN/DIR_FORWARD/DIR_REVERSE` | 0/1/2 | Direction detection states |
| `DETECT_STATE_*` | 0-4 | Course detection state machine states |

## Known Issues & TODOs (from code comments)

1. ~~**Catmull-Rom interpolation bug**~~: Fixed - spline now only used for lat/lng (not time/odometer), and previous GPS fix inserted as pre-crossing control point
2. **Altitude messing up distance**: `loop()` line 27 has TODO: "I think alt is messing up, investigate more... maybe flag?"
3. **Early abort bug**: checkStartFinish line 148-149 TODO about aborting early causing re-check immediately
4. **`checkStartFinish` portability**: Line 106 TODO to make more portable for split timing
5. ~~**License mismatch**~~: Resolved - GPL v3, library.properties updated
6. ~~**Header comment outdated**~~: Fixed - updated to mention 3-sector timing
7. ~~**No keywords.txt**~~: Added
8. ~~**No .gitignore**~~: Added

## Supported Hardware

- **MCU**: Seeed XIAO nRF52840 (recommended, 256 KB RAM), Arduino Mega+ (minimum)
- **GPS**: Matek SAM-M10Q (25Hz), Matek SAM-M8Q (18Hz) - uBlox modules
- **Display**: 128x64 I2C OLED (SSD1306 or SH110X)
- **GPS Library**: Adafruit_GPS
- **Display Library**: Adafruit GFX + Adafruit_SSD1306 or Adafruit_SH110x

## Testing

Use the `real_track_data_debug` example to replay real NMEA data without GPS hardware.
Track data from Orlando Kart Center is included. Compare results against MyLaps timing.

## Cross-Reference: Related Repos

- DovesLapTimer (this repo) - Core timing library
- DovesSchizoTest - JS webapp prototype (source for v4.0 features)
