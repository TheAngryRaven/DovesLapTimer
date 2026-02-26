# DovesLapTimer - Project Guide

> **IMPORTANT**: Always keep this CLAUDE.md file updated when making changes to the codebase.
> When adding new features, modifying APIs, fixing bugs, or changing architecture, update the
> relevant sections below so future sessions don't waste tokens re-exploring the project.

## Project Overview

**What**: GPS-based lap timing Arduino library for go-kart / racing applications.
**Author**: Michael Champagne (crimsondove)
**Version**: 3.1.1
**Repo**: https://github.com/TheAngryRaven/DovesLapTimer
**License**: GPL v3
**Dependency**: ArxTypeTraits (auto-included by Arduino Library Manager)

The library does NOT interface with GPS hardware directly. You feed it coordinates and time,
it handles crossing detection, lap timing, sector splits, distance tracking, and pace comparison.

## Directory Structure

```
DovesLapTimer/
├── src/
│   ├── DovesLapTimer.h          # Header - class definition, structs, public API (535 lines)
│   └── DovesLapTimer.cpp        # Implementation - all logic (929 lines)
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
├── LICENSE                      # GPL v3 (conflicts with library.properties MIT claim)
└── .github/FUNDING.yml
```

## Architecture & Key Concepts

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
- Known issue: Catmull-Rom has a bug (noted in basic_oled_example setup comment)

### Sector Timing
- 3 sectors: Start/Finish -> Sector2 -> Sector3 -> Start/Finish
- Requires both sector 2 and sector 3 lines to be configured
- Validates crossing order (rejects out-of-order crossings)
- Tracks best sector times and lap numbers independently

### Memory Management
- Circular buffer `crossingPointBuffer` sized by available RAM:
  - `>3000 bytes RAM`: 100 entries
  - `<=3000 bytes RAM`: 25 entries
- Buffer is shared between all line crossings (start/finish + sectors)
- Mutual exclusion: only one line can be "crossing" at a time

## Public API Quick Reference

### Setup Methods
| Method | Description |
|--------|-------------|
| `setStartFinishLine(aLat, aLng, bLat, bLng)` | Define start/finish line |
| `setSector2Line(aLat, aLng, bLat, bLng)` | Define sector 2 split line |
| `setSector3Line(aLat, aLng, bLat, bLng)` | Define sector 3 split line |
| `updateCurrentTime(millisSinceMidnight)` | Set GPS time (call before loop) |
| `forceLinearInterpolation()` | Use linear interpolation (default) |
| `forceCatmullRomInterpolation()` | Use Catmull-Rom spline (has known issues) |
| `reset()` | Reset all state to zero |

### Timing Getters
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

### State/Distance Getters
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

### Public Utility Methods (also used by examples for display)
| Method | Description |
|--------|-------------|
| `insideLineThreshold(...)` | Check if point is in crossing detection zone |
| `pointLineSegmentDistance(...)` | Distance from point to line segment (meters) |
| `haversine(lat1, lon1, lat2, lon2)` | Great-circle distance (meters) |
| `haversine3D(...)` | 3D distance including altitude |
| `isObtuseTriangle(...)` | Triangle type check |
| `pointOnSideOfLine(...)` | Which side of line a point is on |

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

- **MCU**: Seeed XIAO nRF52840 (recommended), Arduino Mega+ (minimum)
- **GPS**: Matek SAM-M10Q (25Hz), Matek SAM-M8Q (18Hz) - uBlox modules
- **Display**: 128x64 I2C OLED (SSD1306 or SH110X)
- **GPS Library**: Adafruit_GPS
- **Display Library**: Adafruit GFX + Adafruit_SSD1306 or Adafruit_SH110x

## Testing

Use the `real_track_data_debug` example to replay real NMEA data without GPS hardware.
Track data from Orlando Kart Center is included. Compare results against MyLaps timing.

## Cross-Reference: Related Repos

(Update this section as the other two repos are set up)
- DovesLapTimer (this repo) - Core timing library
- [Add other repos here as they are created]
