# Layer-2 unit tests

Host-native unit tests for the pure-math and state-machine modules of
DovesLapTimer. Compiles and runs on Linux / macOS with `g++` or `clang++`
— no Arduino hardware, no emulator.

## What's covered

| Suite | Module(s) | Tests |
|---|---|---|
| `test_geomath` | `GeoMath.h` | haversine + haversine3D sanity, real distances |
| `test_direction_detector` | `DirectionDetector` in `DovesLapTimer.h/cpp` | resolution outcomes, single-sector discard, glitch overwrite (CLAUDE.md issues #11 / #12), post-resolution lock |
| `test_course_detector` | `CourseDetector.h/cpp` | full state machine + rejection cooldown (CLAUDE.md issue #13) |
| `test_synthetic_track` | full `DovesLapTimer` pipeline | drives a deterministic 100m × 100m CCW square loop, asserts lap / sector times + cross-lap consistency + direction resolution |

The synthetic-track suite is an integration test — if the crossing
detection or interpolation regresses, lap-to-lap variance > 5ms or
absolute timing drift > 100ms will fail the run.

## Running locally

```sh
cd test
make run        # build + run all suites; exits non-zero on any failure
make clean      # remove build/
```

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
