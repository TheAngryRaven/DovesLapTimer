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
- After start/finish crossed (`raceSeen = true`), captures **physical** S2 and S3
  crossing timestamps in a per-lap window (`lapS2CrossingTime`, `lapS3CrossingTime`)
- At the next start/finish, if BOTH timestamps were captured this lap, direction
  resolves: `s2 < s3` → `DIR_FORWARD`, otherwise `DIR_REVERSE`
- Single-sector laps (driver missed a poorly-placed line or GPS rate too low to
  catch the zone) are discarded — the window resets and we re-try next lap
- Latest in-lap crossing wins: a phantom GPS glitch early in the lap gets
  overwritten by the real crossing that follows
- Once resolved, direction is locked. `handleLineCrossing()` remaps S2↔S3 when reverse
- Getters: `getDirection()`, `isDirectionResolved()`

### Course Detection (v4.0)
- `CourseDetector` state machine: IDLE → WAITING_FOR_SPEED → WAYPOINT_SET → CANDIDATES_READY → DETECTED
- Drops waypoint when speed >= 20 mph, waits for return within 10m after 200m+ traveled
- Compares driven distance (feet) to each course's `lengthFt` within 25% tolerance
- Builds ranked candidates, CourseManager validates via `raceStarted` sanity check
- `rejectAllCandidates(currentOdometer)` advances `_waypointOdometer` to "now"
  so a rejection requires another full lap before re-ranking — otherwise the
  driver is still inside the proximity zone and the next GPS fix would
  re-trigger ranking, burning through `COURSE_DETECT_MAX_REJECTIONS` in a few
  frames and jumping straight to Lap Anything

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
- Circular buffer `crossingPointBuffer` sized per platform:
  - Classic AVR with `RAMEND - RAMSTART > 3000` (e.g., Mega): 100 entries
  - Classic AVR with `RAMEND - RAMSTART <= 3000` (e.g., Uno): 25 entries
  - Any non-AVR (`RAMEND`/`RAMSTART` undefined — nRF52, ESP32, SAMD, RP2040): 100 entries
- Buffer is reset (memset + index=0) at the end of every crossing — wraparound within a single crossing is a non-concern at kart speeds
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
2. **Altitude messing up distance**: `loop()` has a TODO: "I think alt is messing up, investigate more... maybe flag?"
3. ~~**Early abort bug**~~: Abandoned — commented-out `crossingStartedLineSide` tracking plus the `CROSSING_LINE_SIDE_NONE` define have been removed; the underlying "abort early" optimization was never implemented and the current hypotenuse-threshold flow is reliable in practice.
4. **`checkStartFinish` portability**: TODO at the top of `checkStartFinish` to make more portable for split timing
5. ~~**License mismatch**~~: Resolved - GPL v3, library.properties updated
6. ~~**Header comment outdated**~~: Fixed - updated to mention 3-sector timing
7. ~~**No keywords.txt**~~: Added (refreshed 2026-04-17 for full v4.0 API coverage)
8. ~~**No .gitignore**~~: Added
9. **nRF52840 RAM budget unaffected by AVR macros**: `DovesLapTimer.h` buffer-size `#if` now uses `defined(RAMEND) && defined(RAMSTART)` as the AVR detection gate; non-AVR targets get the 100-entry buffer unconditionally.
10. **Haversine hot-path micro-opt (2026-04-17)**: `pow(x, 2)` replaced with `x*x` in `GeoMath.h::geoHaversine` and `DovesLapTimer.cpp::pointLineSegmentDistance` for consistency with the existing `sq()` usage elsewhere — same output, fewer library calls.
11. ~~**Direction detector mis-classified forward as reverse**~~: Fixed (2026-05-20). `DirectionDetector::onLineCrossing` used to lock direction from whichever single sector line (S2 or S3) was hit first after `raceSeen`. A poorly-placed sector line that the racing line missed (or low GPS sample rate that skipped the zone) caused S3 to be hit "first", flipping direction to reverse forever. Now requires BOTH physical S2 and S3 to be crossed within a lap window and decides from their temporal order at the next start/finish. Same root cause as the ported webapp bug `courseDetection.ts:67-90`.
12. ~~**Direction lockout on glitched first crossing**~~: Fixed (2026-05-20). Same change as #11. The old "first crossing wins" rule meant a single GPS teleport that triggered a phantom S3 crossing locked direction to reverse permanently. The new logic requires both sector lines and lets the *latest* in-lap crossing overwrite earlier phantoms, dramatically shrinking the surface area for a single glitch to mis-lock. Same root cause as the ported webapp bug `lapCalculation.ts:107`. Note: a glitch that fabricates BOTH sector crossings on the same lap could still mis-lock — a multi-lap voting threshold would harden this further but was deferred.
13. ~~**CourseManager burned through MAX_REJECTIONS in a few GPS frames**~~: Fixed (2026-05-20). When `_handleCandidatesReady` rejected all candidates, `rejectAllCandidates()` only reset state to `WAYPOINT_SET`; the driver was still inside waypoint proximity with `distanceSinceWaypoint` still well above the 200m gate, so the very next GPS fix re-triggered ranking → same candidates → reject → repeat. At 25 Hz this burned all 3 rejections in ~120 ms, falling straight to Lap Anything without the driver completing another real lap. Fix: `rejectAllCandidates(currentOdometer)` now advances `_waypointOdometer` to "now", so the next ranking pass requires another full lap (200m+ traveled and returned to proximity).

## Supported Hardware

- **MCU**: Seeed XIAO nRF52840 (recommended, 256 KB RAM), Arduino Mega+ (minimum)
- **GPS**: Matek SAM-M10Q (25Hz), Matek SAM-M8Q (18Hz) - uBlox modules
- **Display**: 128x64 I2C OLED (SSD1306 or SH110X)
- **GPS Library**: Adafruit_GPS
- **Display Library**: Adafruit GFX + Adafruit_SSD1306 or Adafruit_SH110x

## Testing

Use the `real_track_data_debug` example to replay real NMEA data without GPS hardware.
Track data from Orlando Kart Center is included. Compare results against MyLaps timing.

## Continuous Integration

GitHub Actions workflows live in `.github/workflows/`:

- **arduino-lint.yml** — runs `arduino/arduino-lint-action@v2` at `compliance: specification`
  (Library Manager compatibility level). Uploads `arduino-lint-report.json` as an artifact.
- **compile-examples.yml** — uses `arduino/compile-sketches@v1` to compile every example
  against a matrix of boards:
  - `arduino:avr:mega` — `sector_timing_example` only (smoke test; the OLED + GPS
    examples are over Mega's SRAM budget — the README itself calls Mega "really
    pushing it", so we only claim the core library compiles on AVR Mega)
  - `arduino:avr:uno` — `sector_timing_example` only (smoke test on small AVR; the other two
    require `Serial1` which the Uno lacks)
  - `esp32:esp32:esp32` — all 3 examples (custom platform URL pulls the ESP32 BSP)
  - `Seeeduino:nrf52:xiaonRF52840` — all 3 examples (recommended hardware; uses the
    Adafruit-based nrf52 core, not the older mbed core, since that's what real XIAO
    users compile against and it matches the FPU/double-precision pitch in the README)
  - Pulled libs: ArxTypeTraits, Adafruit GPS Library, Adafruit GFX, Adafruit SSD1306,
    Adafruit SH110X
  - `enable-deltas-report: true` uploads `sketches-reports/` so PRs surface flash/SRAM deltas

Both workflows trigger on push to `main`/`master`, on any PR, and manually via
`workflow_dispatch`. Status badges are linked at the top of README.md.

This is a **layer 1** CI setup — structural validation only (lint + compile). Future work:
layer 2 (native unit tests for pure-math modules like `GeoMath`, `DirectionDetector`,
`CourseDetector`) and layer 3 (NMEA replay regression tests against golden lap times).

## Cross-Reference: Related Repos

- DovesLapTimer (this repo) - Core timing library
- DovesSchizoTest - JS webapp prototype (source for v4.0 features)
