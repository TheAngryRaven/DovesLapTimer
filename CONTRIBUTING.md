# Contributing to DovesLapTimer

Thanks for your interest in improving DovesLapTimer! This is a GPS lap-timing
library for go-karts and race cars, and contributions — bug reports, fixes,
new features, docs — are all welcome.

## Ground rules

- Be respectful. See the [Code of Conduct](CODE_OF_CONDUCT.md).
- This library does **not** talk to GPS hardware directly. It takes
  `(lat, lng, altitude_m, speed_knots, time_ms_since_midnight)` and returns
  lap timing. Keep that boundary — hardware/serial parsing belongs in the
  examples, not the core.
- The library targets memory-constrained MCUs (down to AVR Mega). Watch your
  SRAM and avoid heap allocation in the hot path.

## Development setup

You don't need any Arduino hardware to develop or test the core library.

```sh
git clone https://github.com/TheAngryRaven/DovesLapTimer.git
cd DovesLapTimer/test
make run          # builds + runs the full host-native test suite with g++
```

That's it — the test harness stubs the small Arduino surface the library uses
(`Stream`, `F()`, `sq()`) so everything compiles and runs on a plain Linux/macOS
host. See [`test/README.md`](test/README.md) for the layout.

To compile the example sketches you'll need the Arduino IDE/CLI with the
relevant board packages — but CI does this for you on every PR, so it's
optional locally.

## The testing philosophy (please read before adding code)

DovesLapTimer is tested in three layers. **New behavior should land with a
test in the appropriate layer.** This is what lets us refactor the timing core
confidently — a regression of even a few milliseconds turns CI red.

1. **Layer 1 — structural** (`arduino-lint`, `compile-examples`): lint pass +
   every example compiles across Arduino Mega, Uno, ESP32, and XIAO nRF52840.
   Catches missing includes, broken signatures, SRAM/flash overruns.
2. **Layer 2 — module unit tests** (`test/test_*.cpp`): host-native tests for
   `GeoMath`, `DirectionDetector`, the `CourseDetector` state machine, and a
   synthetic-track integration pass over the full pipeline. Catches algorithm
   regressions that compile fine but produce wrong numbers.
3. **Layer 3 — NMEA replay regression** (`test/test_nmea_*.cpp`): replays real
   Orlando Kart Center GPS recordings and asserts lap times match pinned
   goldens within ±10 ms. Catches interpolation regressions on real-world
   noisy data.

**Adding a test:** write `test/test_<thing>.cpp` with `void test_<name>()`
functions, list them in `main()` via `RUN_TEST(<name>)`, and `make run` picks
it up automatically (the Makefile globs `test_*.cpp`). Use the
`EXPECT_TRUE` / `EXPECT_EQ` / `EXPECT_NEAR` macros from `test_runner.h`.

If you intentionally change timing output and a layer-3 golden shifts, update
the pinned value in the test **and** the fixture header comment, and explain
why in your PR.

## Pull request workflow

1. Fork and branch off `master`. Name branches descriptively
   (e.g. `fix-sector-overflow`, `add-elevation-getter`).
2. Make your change with a matching test.
3. Run `cd test && make run` locally — all green.
4. Update docs if you touched the public API:
   - Doc comments (`@brief` / `@param` / `@return`) — these feed the
     [API docs site](https://theangryraven.github.io/DovesLapTimer/).
   - `README.md` if user-facing behavior changed.
   - `CHANGELOG.md` under an "Unreleased" heading (Keep a Changelog format).
5. Open a PR. CI runs all four workflows (lint, compile, unit tests, docs).
   Fill out the PR template.

Keep PRs focused — one logical change per PR is much easier to review than a
grab-bag.

## Commit messages

Write a clear subject line in the imperative ("fix sector overflow", "add
elevation getter"), and use the body to explain the *why*, not just the *what*.

## Releasing (maintainers)

1. Bump `version=` in `library.properties`.
2. Move the `CHANGELOG.md` "Unreleased" entries under a new version heading.
3. **Doc-claim spot check**: for every behavior touched this release, read the
   matching README section and header `@brief`/`@return` aloud against the
   code. Docs have drifted into outright lies before (the README described a
   direction-detection algorithm a previous release had removed; two getters
   documented the wrong units). If the sentence describes code that no longer
   exists, fix it before tagging.
4. Tag the commit (`vX.Y.Z`) and push the tag — or create a GitHub Release,
   which creates the tag.
5. The Arduino Library Manager indexer auto-ingests the new tag within a couple
   hours. No registry re-submission needed.

## Questions

Open an issue with the **question** label, or start a discussion. For anything
sensitive, see [SECURITY.md](SECURITY.md).
