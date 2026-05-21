# Changelog

All notable changes to this project will be documented in this file. The
format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- Project governance docs: `CONTRIBUTING.md` (dev setup, the 3-layer testing
  philosophy, PR workflow, release process), `SECURITY.md`, GitHub issue
  forms (bug report + feature request) and a pull-request template.

## [4.1.0] – 2026-05-21

A quality-and-confidence release. No breaking API changes; everything from
v4.0 keeps working. The library now has a real test harness, modern API
docs, less duplication, and accurate documentation of where Catmull-Rom
actually matters.

### Added
- **Layer 1 CI** — three GitHub Actions workflows:
  `arduino-lint` (Library Manager compliance), `compile-examples` (every
  example compiles across Arduino Mega, Uno, ESP32, and XIAO nRF52840),
  status badges on the README.
- **Layer 2 host-native unit tests** (`test/`) — 43 tests covering
  `GeoMath`, `DirectionDetector`, `CourseDetector` state machine, and a
  synthetic-track integration pass over the full `DovesLapTimer`
  pipeline. Builds and runs with plain `g++` — no Arduino toolchain or
  hardware required. `cd test && make run`.
- **Layer 3 NMEA replay regression tests** — 23 tests replay the four
  real Orlando Kart Center GPS recordings from `examples/real_track_data_debug/`
  through the lap timer and assert lap times match pinned goldens within
  ±10 ms, plus a ±200 ms sanity bound against MyLaps magnetic-loop
  ground truth where it was recorded.
- **API documentation site** — Doxygen-generated HTML with the
  doxygen-awesome-css theme (dark mode toggle, sidebar nav, code-copy
  buttons, interactive TOC). Lives at
  https://theangryraven.github.io/DovesLapTimer/. New `docs` workflow
  builds on every PR and deploys to the `gh-pages` branch on push to
  master.
- **README Quickstart + Testing & validation sections** so a new user
  hits a working code example within the first 50 lines.
- **`LineDetectResult` enum** and `_detectLineCrossing` private helper
  in `DovesLapTimer.h` — exposed for future split-timing API work.
- **Configurable thresholds via setters**: `setSpeedThresholdMph` /
  `setDetectionProximityMeters` on `CourseDetector`.

### Changed
- **`checkStartFinish` and `checkSectorLine` refactored** to share a
  single `_detectLineCrossing` zone state machine. Net –31 lines, single
  source of truth for entry/buffer/interpolate logic, no behavior
  change (proven via the 64-test layer-2/3 net).
- **NMEA replay test tolerances tightened from ±50 ms → ±10 ms.** The
  library is deterministic at 0 ms vs. itself; 10 ms is sub-meter at
  kart speeds, tight enough to catch real interpolation regressions
  without being flaky.
- **`sector_timing_example` sketch now runs out of the box** without a
  GPS module. Drives the lap timer through a synthetic 100m × 100m
  counter-clockwise loop so users can see sector splits + best/optimal
  lap accumulate on Serial in real time. Three-step swap-in path
  documented in the header for replacing the synthetic feed with a
  real GPS module.
- **`forceCatmullRomInterpolation()` documented honestly** — the
  spline is intentionally scoped to crossing-point lat/lng only; time
  and odometer are always interpolated linearly. `getLastLapTime()` /
  `getBestLapTime()` / sector times are identical between modes. The
  layer-3 replay tests now enforce this contract.
- **README "Direction Detection" bullet** updated to match the
  current "both sector lines per lap, glitch-resistant" behavior
  (previously described the pre-fix "first crossing wins" rule).

### Fixed
- **DirectionDetector mis-classified forward as reverse** when the
  racing line missed a sector or GPS rate was too low to catch the
  zone — the detector used to lock direction from whichever single
  sector line was hit first. Now requires both physical S2 and S3
  to be crossed within a lap window and decides from their temporal
  order at the next start/finish.
- **DirectionDetector lockout on a glitched first crossing** —
  pre-fix, a single GPS teleport that triggered a phantom S3 crossing
  locked direction to reverse permanently. New logic lets the latest
  in-lap crossing overwrite earlier phantoms and requires both
  sectors, dramatically shrinking the surface area for a single
  glitch to mis-lock.
- **CourseManager burned through `MAX_REJECTIONS` in a few GPS
  frames** when `rejectAllCandidates()` reset state to
  `WAYPOINT_SET` without advancing the lap-window origin — the driver
  was still inside proximity with `distanceSinceWaypoint` well above
  the 200 m gate, so the very next GPS fix re-triggered ranking. Fix:
  `rejectAllCandidates(currentOdometer)` advances `_waypointOdometer`
  to "now" so re-ranking requires another full lap.
- **`Doxyfile` doc-comment typos** — `curentLng` → `currentLng` and
  `currentSpeed` → `currentSpeedKnots` in `DovesLapTimer.h` `@param`
  tags so they match actual signatures.
- **`HELPME.md` heading hierarchy** — h4 after h2 (skipped h3)
  demoted to h3.

### CI / Tooling
- Added `peaceiris/actions-gh-pages@v3` for docs deployment.
- doxygen-awesome-css v2.3.4 vendored under `docs/` (MIT-licensed)
  so the docs build doesn't depend on the network at CI time.
- Test build uses `-include mock/Arduino.h` to inject the host
  Arduino stub, mirroring what the Arduino IDE auto-injects for
  `.ino` sketches. No library source modification needed for host
  tests to compile.

## [4.0.0] – 2026

### Added
- **Automatic course detection** via `CourseManager`. Drive a lap
  and the library figures out which course layout you're on by
  matching driven distance against known courses. Supports up to
  8 layouts per track.
- **Direction detection** based on sector-line crossing order.
- **"Lap Anything" fallback** (`WaypointLapTimer`) — proximity-based
  lap timing when no course is detected. Works on any track without
  pre-configured crossing lines.
- **3-sector split timing** with best sector times, optimal lap
  calculation, and per-sector lap-number tracking.
- Shared `GeoMath` helpers (`haversine`, `haversine3D`) used by
  every detector.
- Configurable speed / proximity / detection thresholds.

### Earlier history
For pre-4.0 history (initial sector timing, Catmull-Rom interpolation
work, etc.) see the git log directly.

[4.1.0]: https://github.com/TheAngryRaven/DovesLapTimer/releases/tag/v4.1.0
[4.0.0]: https://github.com/TheAngryRaven/DovesLapTimer/releases/tag/v4.0.0
