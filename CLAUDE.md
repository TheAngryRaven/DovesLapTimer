# DovesLapTimer - Project Guide

> **IMPORTANT**: Always keep this CLAUDE.md file updated when making changes to the codebase.
> When adding new features, modifying APIs, fixing bugs, or changing architecture, update the
> relevant sections below so future sessions don't waste tokens re-exploring the project.

## Development Standards (read this first)

This codebase was deliberately brought up to a professional bar. **Keep it there** — don't
let it regress. When making any change:

1. **Add tests whenever possible.** This library has a three-layer test suite
   (`test/`) that is the whole reason changes can be made confidently. New behavior
   should land *with* a test:
   - Pure logic / math / state machines → a Layer 2 unit test (`test/test_*.cpp`).
   - Anything that affects lap-time output → verify against the Layer 3 NMEA replay
     suite; if you intentionally change timing, update the pinned goldens **and** say
     why in the commit/PR.
   - Always run `cd test && make run` before committing. All green, no exceptions.
   - A bug fix should come with a test that fails before the fix and passes after.
2. **Keep the CHANGELOG updated.** Every user-facing or notable change goes under the
   `## [Unreleased]` heading in `CHANGELOG.md` (Keep a Changelog format:
   Added/Changed/Fixed). On release, move those entries under the new version heading.
3. **Keep doc comments and the README honest.** Public API changes update the
   `@brief`/`@param`/`@return` comments (they feed the Doxygen site). If user-facing
   behavior changes, update `README.md`. Don't let docs claim something the code
   doesn't do — that's how the Catmull-Rom confusion happened.
4. **Respect the architecture boundary.** The library does NOT touch GPS hardware or
   serial. It takes coordinates + time and returns timing. Hardware/parsing belongs in
   `examples/`, never the core.
5. **Stay memory-conscious.** Targets go down to AVR Mega. No heap allocation in the
   GPS hot path; watch SRAM.
6. **One logical change per PR.** Branch off `master`, let CI (lint + compile matrix +
   unit tests + docs) go green, fill out the PR template. See `CONTRIBUTING.md` for the
   full workflow.

In short: tests with changes, changelog updated, docs truthful, no broken windows.

## Project Overview

**What**: GPS-based lap timing Arduino library for go-kart / racing applications.
**Author**: Michael Champagne (crimsondove)
**Version**: 4.1.0
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
├── images/                      # Diagrams for DETECTION.md
├── library.properties           # Arduino library metadata
├── README.md                    # User-facing docs and API reference
├── DETECTION.md                 # Technical deep-dive on crossing detection algorithm (+ validation evidence)
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
3. `loop()` validates the fix (NaN/Inf/out-of-range/(0,0) rejected; >500m
   single-fix teleports dropped, sustained relocation re-accepted after 3
   fixes without crediting the gap; non-finite alt/speed sanitized)
4. `loop()` updates odometer (haversine3D), speed, then checks all crossing
   lines (start/finish only if `startFinishLineConfigured`)
5. `checkStartFinish()` and `checkSectorLine()` handle detection + interpolation

Both call a shared private helper `_detectLineCrossing()` that runs the zone
state machine, manages the GPS-fix buffer, and produces interpolated crossing
output via out-params. Callers branch on the helper's `LineDetectResult`
(NONE / IN_ZONE / COMPLETED) and do their own post-crossing accounting
(`checkStartFinish` does lap timing, `checkSectorLine` just delegates to
`handleLineCrossing`). A zone exit whose interpolation is invalid (no
straddling pair, or pair gap > `CROSSING_MAX_FIX_GAP_MS`) reports NONE so
callers never consume garbage out-params — the old `time != 0` validity
sentinel is gone (a crossing at exactly 00:00:00.000 UTC is legitimate).

### Time Base (midnight rollover)
All timestamps are milliseconds since UTC midnight and wrap 86,399,999 → 0.
Every duration is computed via `timeSinceMidnightDelta(start, end)` (static
inline in `DovesLapTimer.h`), which normalizes across the wrap. The crossing
interpolator computes its fix-pair delta in `double` (wrap-normalized, then
clamped to `CROSSING_MAX_FIX_GAP_MS`) before converting back to `unsigned
long`, avoiding out-of-range double→unsigned UB. Known limitation: a
*backwards* GPS time step that isn't a midnight wrap is indistinguishable
from one and produces a near-day-length delta — the interpolator clamp
catches it inside a crossing zone; lap-level deltas do not.

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
- Fallback when no course is detected (after rejections / no-match passes / distance failsafe)
- Drops waypoint at speed, tracks closest approach inside a 30m proximity zone
  for the lap split (three scalars — the old 50-entry buffer was write-only
  dead memory and was removed; ~150 B per instance now)
- Duck-typed to match DovesLapTimer's public API
- No sector support (getters return 0)

### CourseManager (v4.0)
- Orchestrates multiple DovesLapTimers + CourseDetector + WaypointLapTimer
- Feeds ALL active timers the same GPS data simultaneously
- Validates detection candidates via `raceStarted` check
- Falls back to Lap Anything via three routes: `COURSE_DETECT_MAX_REJECTIONS`
  (3) candidate rejections, `COURSE_DETECT_MAX_NO_MATCH_PASSES` (3) completed
  proximity passes that matched no course length (the detector never produces
  candidates with a wrong/missing config — pre-fix this hung detection
  forever), or the odometer exceeding
  `COURSE_DETECT_FALLBACK_DISTANCE_FACTOR` (4) × longest course (floored at
  `COURSE_DETECT_FALLBACK_MIN_METERS`, 2000m) for drivers who never return
  to the waypoint
- Activating Lap Anything deactivates ALL course timers (they'd never be
  surfaced again; pre-fix they kept burning CPU on every fix forever)
- `pruneInactiveCourses()` stops *processing* non-detected timers — saves
  CPU per fix, frees ZERO bytes (the 8-slot array is statically allocated)

### Memory Management
- Circular buffer `crossingPointBuffer` sized per platform:
  - Classic AVR with `RAMEND - RAMSTART > 3000` (e.g., Mega): 100 entries
  - Classic AVR with `RAMEND - RAMSTART <= 3000` (e.g., Uno): 25 entries
  - Any non-AVR (`RAMEND`/`RAMSTART` undefined — nRF52, ESP32, SAMD, RP2040): 100 entries
- Buffer is reset (memset + index=0) at the end of every crossing. The buffer
  CAN wrap within a single crossing (kart parked in the zone — standing start,
  red flag); the interpolator unwinds the ring chronologically before scanning
  so the seam pair (newest adjacent to oldest) can't masquerade as a crossing
- Buffer is shared between all line crossings (start/finish + sectors)
- Mutual exclusion: only one line can be "crossing" at a time
- CourseManager: ~29 KB with 64-bit doubles (8 × ~3.6 KB by-value timer array,
  allocated for MAX_COURSES regardless of configured course count). Pruning /
  Lap-Anything deactivation save CPU only — no memory is ever released. Does
  NOT fit on AVR Mega (8 KB SRAM)
- Classic AVR additionally runs with 32-bit `double` — a `#warning` fires at
  compile time; lap counting works but odometer/interpolation accuracy is
  degraded (see README hardware notes)

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
| `isStartFinishLineConfigured()` | True once a valid start/finish line set |
| `getCurrentLapDistance()` | Current lap meters |
| `getLastLapDistance()` | Last lap meters |
| `getBestLapDistance()` | Best lap meters |
| `getTotalDistanceTraveled()` | Odometer meters |
| `getPaceDifference()` | Pace delta vs best lap |
| `getDirection()` | DIR_UNKNOWN/DIR_FORWARD/DIR_REVERSE |
| `isDirectionResolved()` | True once direction is known |
| `getRejectedCrossingCount()` | Zone exits whose interpolation was rejected |

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
| `getNoMatchCount()` | Completed proximity passes that matched no course |

### CourseManager API
| Method | Returns |
|--------|---------|
| `updateCurrentTime(ms)` | Feed time to all timers |
| `loop(lat, lng, alt, speedKnots)` | Feed GPS to all timers + detector |
| `reset()` | Reset everything |
| `pruneInactiveCourses()` | Stop feeding non-detected timers (CPU only, frees no RAM) |
| `isCourseTimerActive(index)` | True while that course's timer is still fed |
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
| `COURSE_DETECT_MAX_NO_MATCH_PASSES` | 3 | No-match laps before Lap Anything fallback |
| `COURSE_DETECT_FALLBACK_DISTANCE_FACTOR` | 4.0 | × longest course = detection distance failsafe |
| `COURSE_DETECT_FALLBACK_MIN_METERS` | 2000.0 | Floor for the distance failsafe |
| `CROSSING_PAIR_SPACING_FACTOR` | 1.25 | Crossing-pair sum bound scales with fix spacing |
| `GEOMATH_KNOTS_TO_KMH` etc. | — | Shared unit-conversion constants (`GeoMath.h`) |
| `DIR_UNKNOWN/DIR_FORWARD/DIR_REVERSE` | 0/1/2 | Direction detection states |
| `DETECT_STATE_*` | 0-4 | Course detection state machine states |
| `DOVES_MILLIS_PER_DAY` | 86400000 | GPS time-of-day wrap point (UTC midnight) |
| `GPS_MAX_PLAUSIBLE_JUMP_METERS` | 500.0 | Single-fix jump beyond this = glitch, dropped |
| `GPS_JUMP_REACCEPT_COUNT` | 3 | Consecutive far fixes before re-seeding position |
| `CROSSING_MAX_FIX_GAP_MS` | 10000 | Max believable gap between crossing-pair fixes |

## Known Issues & TODOs (from code comments)

1. ~~**Catmull-Rom interpolation bug**~~: Fixed - spline is now used **only for the
   reported crossing point's lat/lng**. Time and odometer are always interpolated
   linearly to avoid spline overshoot producing wrong lap times. Practical consequence:
   `forceCatmullRomInterpolation()` does NOT change lap-time output vs. linear — it
   only affects `crossingLat`/`crossingLng` (which the public API doesn't currently
   expose). The mode setting is preserved for future API additions and for users who
   inspect interpolated coords via debug logs; layer-3 replay tests assert
   `catmull_lap_times == linear_lap_times` on all four real-track fixtures to keep
   this contract enforced.
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
14. ~~**Start/finish line never initialized — `loop()` read indeterminate memory**~~: Fixed (2026-06-11, code-review C1). The four `startFinishPoint*` doubles had no initializers and no configured flag, and `loop()` unconditionally called `checkStartFinish()` — calling `loop()` before `setStartFinishLine()` was UB with potential phantom crossings. All line members are now zero-initialized, `startFinishLineConfigured` gates detection, `isStartFinishLineConfigured()` exposes it, and all three line setters reject degenerate (A==B, e.g. example-sketch 0.00 placeholders) or non-finite lines.
15. ~~**Midnight rollover / backwards GPS time corrupted every time computation**~~: Fixed (2026-06-11, code-review C2). See "Time Base (midnight rollover)" section above. Tests: `test/test_midnight_rollover.cpp`.
16. ~~**Course detection falsely matched longer layouts on no-match laps**~~: Fixed (2026-06-11, code-review C3). `_checkWaypointProximity` now re-anchors `_waypointOdometer` after a completed proximity pass with no match, so `distanceSinceWaypoint` can't accumulate across laps and match a 2x-length layout (same root-cause class as #13).
17. ~~**Zero GPS input validation — NaN/(0,0) fix poisoned state forever**~~: Fixed (2026-06-11, code-review H1). `DovesLapTimer::loop()`, `WaypointLapTimer::loop()`, and `CourseDetector::update()` validate input via `GeoMath.h::geoCoordinatesValid()`/`geoIsFinite()`; teleport fixes are dropped with a 3-fix re-accept (see Main Loop Flow). Tests: `test/test_input_validation.cpp`.
18. ~~**Crossing-buffer wraparound broke the interpolator's chronology assumption**~~: Fixed (2026-06-11, code-review H2). The interpolator unwinds the ring chronologically before scanning (see Memory Management). Triggered by a kart parked on/near the line — standing start, red flag. Tests: `test/test_buffer_wraparound.cpp`.
19. **AVR `double` is 32-bit — degraded precision** (code-review H3): documented + `#warning` on 4-byte-double targets (2026-06-11). Lap counting works; odometer/interpolation accuracy degraded. The *full* fix (local-tangent-plane offsets from a reference point so float precision suffices) was deferred — see PR notes.
20. ~~**Low-rate GPS silently killed lap counting**~~: Fixed (2026-06-11, code-review H4). Crossing validation now scales with the straddle pair's spacing (`CROSSING_PAIR_SPACING_FACTOR`), the zone-exiting fix is buffered, an on-segment geometric check backstops the looser bound, and rejections surface via `getRejectedCrossingCount()`. Tests: `test/test_low_rate_gps.cpp`.
21. ~~**Lap-Anything fallback unreachable when no candidates ever ranked**~~: Fixed (2026-06-11, code-review H5). No-match pass counter + odometer failsafe both feed the fallback (see CourseManager section). Tests: `test/test_course_manager.cpp`.
22. ~~**"Pruning saves memory" was false; Lap-Anything left 8 timers running**~~: Fixed/corrected (2026-06-11, code-review H6+H7). Docs now state pruning saves CPU only (~29 KB statically allocated either way; CourseManager cannot fit on AVR Mega); `_activateLapAnything()` deactivates all course timers. The placement-new storage-pool redesign that would actually release memory was deferred — see PR notes.
23. ~~**WaypointLapTimer carried 1.6 KB of write-only buffer**~~: Removed (2026-06-11, code-review H8). Closest-approach scalars suffice; `WAYPOINT_LAP_BUFFER_SIZE` and `ProximityBufferEntry` deleted. Instance size ~150 B.
24. **Five-unit API surface** (code-review H14): knots into `loop()`, km/h into `CourseDetector::update()`, mph thresholds, feet course lengths, meters everywhere else. Conversions now go through shared `GeoMath.h` constants so they can't drift, but standardizing new API on one speed unit remains future work (breaking change).

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
  - `Seeeduino:nrf52:xiaonRF52840Sense` — all 3 examples (recommended hardware; uses the
    Adafruit-based nrf52 core, not the older mbed core, since that's what real XIAO
    users compile against and it matches the FPU/double-precision pitch in the README)
  - Pulled libs: ArxTypeTraits, Adafruit GPS Library, Adafruit GFX, Adafruit SSD1306,
    Adafruit SH110X, Adafruit TinyUSB Library (last one needed so the Seeed nRF52
    BSP's `Serial`/`Adafruit_USBD_CDC` references resolve at link time)
  - Pre-compile step `pip install --user adafruit-nrfutil` runs only on the XIAO
    cell — the BSP's link recipe shells out to it even at compile-only time
  - `enable-deltas-report: true` uploads `sketches-reports/` so PRs surface flash/SRAM deltas

- **unit-tests.yml** — builds and runs the host-native test suite in `test/`
  on ubuntu-latest with `g++`. No board matrix; the suite is pure C++ and
  doesn't depend on any Arduino toolchain. Fails the run on any failed assert.

- **docs.yml** — builds Doxygen HTML from `src/` + `examples/` + `README.md` +
  `DETECTION.md` using the doxygen-awesome-css theme vendored under `docs/`.
  On push to master, deploys the output (`docs-build/html/`) to the
  `gh-pages` branch via `peaceiris/actions-gh-pages` (v4, SHA-pinned) with
  `force_orphan: true` (keeps the gh-pages branch lean — overwrites
  rather than accumulating history). On PR, builds-only without
  deploying so we catch a broken Doxyfile before merge. Live site:
  https://theangryraven.github.io/DovesLapTimer/ (GitHub repo Settings →
  Pages must be set to source = gh-pages branch, folder = /).

- **coverage.yml** — builds the `test/` suite with `g++ --coverage`, runs it,
  and summarizes line coverage of `src/` with `gcovr`. Gated on every PR via
  `make coverage` (fails under `COVERAGE_GATE`, currently **80%** — sits just
  below measured coverage; keep ratcheting it up alongside new tests via the
  `COVERAGE_GATE ?=` line in `test/Makefile`). On push to master it generates
  a shields.io endpoint JSON (`make coverage-badge`) and publishes it to the
  `badges` branch (orphan, badge JSON only) via `peaceiris/actions-gh-pages`. The
  README badge reads that JSON through `img.shields.io/endpoint`. On every PR
  it also posts/updates a per-PR coverage summary comment (gcovr `--markdown`
  via first-party `actions/github-script` — no third-party service, runs
  entirely in-runner). Current line coverage ~84.5% (gate at 80) — every
  module now has direct tests, including `CourseManager`
  (`test_course_manager.cpp`) and `WaypointLapTimer`
  (`test_waypoint_lap_timer.cpp`).

Supply chain: every `uses:` is pinned to a verified commit SHA (with a
trailing `# vX` comment naming the release), `.github/dependabot.yml`
(github-actions ecosystem, weekly, grouped) keeps the pins from rotting, and
the gh-pages/badge deploy steps additionally require
`github.ref == 'refs/heads/master'` so a `workflow_dispatch` from a feature
branch cannot overwrite production docs or the badge.

All five workflows trigger on push to `main`/`master`, on any PR, and
manually via `workflow_dispatch`. Status badges are linked at the top of
README.md.

### Test layers

- **Layer 1 — structural** (`arduino-lint.yml`, `compile-examples.yml`):
  lint pass + every example compiles across the board matrix. Catches
  missing includes, broken API signatures, fatal SRAM/flash overruns.
- **Layer 2 — host unit tests** (`unit-tests.yml`): runs `test/` on the
  host via `make run`. Covers `GeoMath`, `DirectionDetector`,
  `CourseDetector` state machine, `CourseManager` orchestration
  (`test_course_manager.cpp`), `WaypointLapTimer`
  (`test_waypoint_lap_timer.cpp`), a synthetic-track integration
  pass over the full `DovesLapTimer` pipeline, plus regression suites for
  midnight rollover (`test_midnight_rollover.cpp`), adversarial GPS input
  (`test_input_validation.cpp`), crossing-buffer wraparound
  (`test_buffer_wraparound.cpp`), and low-rate GPS crossing detection
  (`test_low_rate_gps.cpp`). Catches algorithm
  regressions that compile fine but produce wrong numbers. See
  `test/README.md` for layout and how to add a suite.
- **Layer 3 — NMEA replay regression** (`unit-tests.yml`, same job): replays the
  four `examples/real_track_data_debug/gps_race_data_*.h` fixtures through the
  lap timer via `test/replay_runner.h` (minimal `$GPGGA` + `$GPRMC` parser, no
  Adafruit_GPS dependency). Asserts the output matches the per-lap DoveTimer
  golden values pinned in each fixture header to ±50ms, plus a ±200ms check
  against MyLaps magnetic-loop times where recorded. Catches interpolation
  regressions on real-world noisy GPS data, not just the clean synthetic track.
  21 replay tests across 4 fixtures.

## Cross-Reference: Related Repos

- DovesLapTimer (this repo) - Core timing library
- DovesSchizoTest - JS webapp prototype (source for v4.0 features)
