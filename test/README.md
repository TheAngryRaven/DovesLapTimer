# Layer-2 unit tests

Host-native unit tests for the pure-math and state-machine modules of
DovesLapTimer. Compiles and runs on Linux / macOS with `g++` or `clang++`
— no Arduino hardware, no emulator.

## What's covered

### Layer 2 — module unit tests

| Suite | Module(s) | Tests |
|---|---|---|
| `test_geomath` | `GeoMath.h` | haversine + haversine3D sanity, real distances |
| `test_direction_detector` | `DirectionDetector` in `DovesLapTimer.h/cpp` | resolution outcomes, single-sector discard, glitch overwrite (CLAUDE.md issues #11 / #12), post-resolution lock |
| `test_course_detector` | `CourseDetector.h/cpp` | full state machine + rejection cooldown (CLAUDE.md issue #13) + no-match lap-window re-anchor (issue #16) |
| `test_synthetic_track` | full `DovesLapTimer` pipeline | drives a deterministic 100m × 100m CCW square loop, asserts lap / sector times + cross-lap consistency + direction resolution |
| `test_midnight_rollover` | time-base handling | lap / sector / crossing-zone timing across the UTC midnight wrap, `timeSinceMidnightDelta` (CLAUDE.md issue #15) |
| `test_input_validation` | GPS input guards | NaN/Inf/(0,0)/out-of-range rejection, teleport drop + relocation re-seed, unconfigured / degenerate line safety (issues #14 / #17) |
| `test_buffer_wraparound` | crossing-buffer ring | parked-in-zone wraparound: park-cross-park and standing-start scenarios (issue #18) |
| `test_low_rate_gps` | crossing validation | 1 Hz / 5 Hz crossing detection, spacing-scaled validation, `getRejectedCrossingCount()` surfacing (issue #20) |
| `test_waypoint_lap_timer` | `WaypointLapTimer.h/cpp` | waypoint drop, closest-approach lap splits on a 400m circle, lap accounting, reset semantics |
| `test_course_manager` | `CourseManager.h/cpp` | detection accept/reject, no-match + distance-failsafe fallbacks (issue #21), Lap-Anything timer deactivation, prune, reset |

### Layer 3 — NMEA replay regression

Replays real GPS recordings from `examples/real_track_data_debug/` through the
lap timer and asserts the output matches the per-lap golden values pinned in
each fixture header (DoveTimer LINEAR / CATMULL, plus MyLaps where recorded).

| Suite | Fixture | What it covers |
|---|---|---|
| `test_nmea_lap` | `gps_race_data_lap.h` | OKC, 1 lap, rental kart. Linear + Catmull-Rom, both vs. MyLaps. |
| `test_nmea_2laps` | `gps_race_data_2laps.h` | OKC, 2 laps, rental kart. Linear + Catmull-Rom. |
| `test_nmea_long_lap` | `gps_race_data_long_lap.h` | OKC pro track, 1 lap. Linear + Catmull-Rom. |
| `test_nmea_praga` | `gps_race_data_praga_laps.h` | OKC, 2 laps, Praga Dark. Linear only (no Catmull baseline). |

Pinned tolerance is 50 ms against the documented DoveTimer golden times — tight
enough to catch any interpolation regression. Where MyLaps magnetic-loop times
are available, a looser 200 ms tolerance proves we stay close to real-world
ground truth.

If you intentionally improve the algorithm and the pinned values shift,
update the goldens in the fixture headers AND the matching `EXPECT_NEAR`
calls in the test file.

The replay parser (`replay_runner.h`) handles `$GPGGA` and `$GPRMC`
sentences — enough for these fixtures — without depending on Adafruit_GPS.

The synthetic-track suite (layer 2) is an integration test for clean
deterministic input; the NMEA-replay suites (layer 3) are regression
tests against captured real-world noise.

## Running locally

```sh
cd test
make run        # build + run all suites; exits non-zero on any failure
make coverage   # build instrumented, run, print gcovr line-coverage summary
make clean      # remove build/ + coverage artifacts
```

`make coverage` enforces a gate (`COVERAGE_GATE`, default 1%) and fails if line
coverage of `src/` drops below it. Raise the bar as coverage grows:
`make coverage COVERAGE_GATE=40`, or edit the default in the Makefile.

## How it works

The Arduino IDE auto-injects `<Arduino.h>` at the top of every TU it
compiles. On a host compiler that header doesn't exist, so `test/mock/`
provides a tiny stub (`Stream` no-op, `F()` identity, `sq()` macro) plus
a passthrough for `ArxTypeTraits.h` that just forwards to `<type_traits>`
and `<utility>`.

The Makefile force-injects the mock via `-include mock/Arduino.h`,
mirroring the IDE's auto-include — no library source modifications needed.

Each `test_*.cpp` declares plain test functions and lists them in `main()`
via `RUN_TEST(name)`. `test_runner.h` provides `EXPECT_TRUE` /
`EXPECT_EQ` / `EXPECT_NEAR` macros that record failures; each binary
returns the failure count so the Makefile / CI can detect a red run.

## Adding tests

1. Write `test_<module>.cpp` with `void test_<name>()` functions.
2. List each one in `main()` via `RUN_TEST(<name>)`.
3. `make run` picks it up automatically (the Makefile globs `test_*.cpp`).
