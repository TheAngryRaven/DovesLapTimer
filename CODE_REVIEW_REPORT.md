# DovesLapTimer — Professional Codebase Review

**Date:** 2026-06-10
**Method:** Five independent deep-dive reviews (core algorithm, orchestration layer, test suite + CI, robustness/security/performance, API/docs/process), cross-checked and deduplicated. Every finding below was verified against the actual source — file:line references included. The test suite was executed (66/66 pass, ~6 s), and memory footprints were measured with a compiled probe.

---

## Overall Score: 4.5 / 10

*(Scale: 10 = Linux-kernel-grade, unattainable for a solo developer. 5 = solid commercial firmware with known debts. 3 = typical published Arduino library.)*

| Subsystem | Score | One-line verdict |
|---|---|---|
| Core algorithm (`DovesLapTimer`, `GeoMath`) | 4/10 | The crossing state machine is genuinely good; the time arithmetic, input validation, and initialization around it are not. |
| Orchestration (`CourseManager`/`CourseDetector`/`WaypointLapTimer`) | 4/10 | Readable state machines, but one field-failure bug, an unreachable fallback, a false memory claim, and 0% test coverage. |
| Tests + CI | 6/10 | Real three-layer suite, top-decile for Arduino — but the headline v4.0 modules are untested and several gates are decorative. |
| Robustness / security / performance | 5/10 | No buffer overflows found anywhere; but GPS — the only input — is treated as trusted. |
| API / docs / process | 6/10 | Rich, mostly-verifiable docs; capped by one flat behavioral contradiction and a five-unit-system public API. |

**The defining pattern:** the machinery is excellent (three-layer tests, golden replays, CI matrix, changelog discipline), but the *synchronization points nothing enforces* have all drifted — README↔code, fixture goldens↔tests, Doxyfile↔version, keywords.txt↔API, memory docs↔measured reality. And the single biggest gap between rhetoric and reality: the project's first rule is "tests with changes," while its two largest v4.0 modules (568 lines, the product differentiators) have literally zero tests.

**For balance, what's genuinely strong:** modulo-guarded buffer indexing throughout, zero heap allocation in the hot path, no `sprintf`/`strcpy` in `src/`, division-by-zero guards in the interpolator, const-correct getters, vacuous-pass-proof replay assertions, regression tests pinned to specific historical bugs, honest DETECTION.md, complete governance docs, and clean recent commit history. This is far above the Arduino-library median. It is not yet at the professional bar it sets for itself.

---

# Issue List (sorted by criticality)

## CRITICAL — correctness bugs with realistic field triggers; fix first

### C1. Start/finish line coordinates are never initialized — `loop()` reads indeterminate memory
`src/DovesLapTimer.h:616-619`, `src/DovesLapTimer.cpp:15-23`, `:62`
The four `startFinishPoint*` doubles have no in-class initializers and the constructor sets only `crossingThresholdMeters` and `_serial`. Sector lines have `sector2LineConfigured`/`sector3LineConfigured` guards; the start/finish line has **no** configured flag, and `loop()` unconditionally calls `checkStartFinish()`. Calling `loop()` before `setStartFinishLine()` feeds uninitialized doubles into `haversine()` — undefined behavior that can fire phantom crossings. The shipped `basic_oled_example.ino:19-22` defaults its line to `0.00` placeholders, so even the "configured" example exercises a degenerate line silently.
**Fix:** add `startFinishLineConfigured` flag (mirroring the sector flags), gate `checkStartFinish()` on it, and zero-initialize all line members.

### C2. Midnight rollover / backwards GPS time → unsigned underflow corrupts every time computation (and reaches UB)
`src/DovesLapTimer.cpp:133, 470-472, 583, 606, 626, 790, 821`; `src/WaypointLapTimer.cpp:181, 239`
The entire time base is `millisecondsSinceMidnight` (wraps 86,399,999 → 0 at UTC midnight — mid-evening across the Americas), and every duration is naked `unsigned long` subtraction. A lap straddling midnight yields a ~4.29-billion-ms "lap time" that is then accepted as `bestLapTime` on a first lap (`:143`) and pollutes state until reset. Worse, in the interpolator (`:470-472`) the subtraction happens in `unsigned long` *before* the `double` conversion; if the crossing zone straddles midnight (or a u-blox time-step during re-acquisition), `crossingTime = A.time + t * deltaTime` converts an out-of-range double to `unsigned long` — undefined behavior per [conv.fpint]. Also, `cTime != 0` guards (`:130`, `:689`) discard a legitimate crossing at exactly 00:00:00.000. No handling, no documented limitation, no test.
**Fix:** normalize deltas at every subtraction site (`if (now < start) now += 86400000UL;`), compute the interpolator delta as `(double)b.time - (double)a.time` with a sanity clamp, and add midnight-rollover unit tests.

### C3. Course detection: failed distance match never re-anchors the lap window → false match against a longer layout
`src/CourseDetector.cpp:103` (the comment documents the gap: *"If no candidates within tolerance, stay in waypoint_set and try again next pass"*)
`distanceSinceWaypoint = totalOdometer - _waypointOdometer` grows monotonically across laps when no match is found. Concrete failure: track has 3,000 ft and 6,000 ft layouts; driver runs the 3,000 ft course but it's missing from config (or the 10 m proximity zone was missed on lap 1). Lap 1: 3,000 ft, no match. Lap 2: cumulative ~6,000 ft → **falsely matches the 6,000 ft layout**. The `raceStarted` sanity check does not catch this when layouts share a start/finish line — the normal case for multi-layout tracks, i.e. the exact use case this feature exists for. This is the same root-cause class as the already-fixed rejection bug (CLAUDE.md issue #13): the odometer re-anchor was added to the rejection path but not the silent-no-match path.
**Fix:** re-anchor `_waypointOdometer` (and consider re-dropping the waypoint) after *every* completed proximity pass, match or no match. Add a regression test mirroring the #13 test.

---

## HIGH — silent failures, false claims, unsafe defaults

### H1. Zero GPS input validation — one NaN/Inf/(0,0) fix permanently poisons state and silently hangs detection
`src/DovesLapTimer.cpp:25-43`
NaN lat/lng (routine from parsers during fix loss) → `totalDistanceTraveled += NaN` sticks **forever**: lap distance, best-lap distance, and `getPaceDifference()` are NaN for the session, and `CourseDetector::_checkWaypointProximity` never fires (NaN comparisons all false) so detection silently hangs. A single (0,0) glitch fix adds ~9,000 km to the odometer, destroying the 25%-tolerance course match. NaN speed flows through `interpolateWeight` (`:381` — `NaN < minSpeed` is false) into `t = NaN` → NaN→unsigned conversion UB. Meanwhile crossing detection is *accidentally* protected (`NaN < threshold` is false), so timing keeps working while distances rot — the user keeps trusting the output. There is no max-jump/teleport rejection anywhere.
**Fix:** validate at the top of `loop()` (reject NaN/Inf, |lat|>90, |lng|>180, exact (0,0), implausible inter-fix jumps); add adversarial-input tests.

### H2. Crossing-buffer wraparound breaks the interpolator's chronology assumption — triggered by a standing start on the grid
`src/DovesLapTimer.cpp:218-219` (wrap), `:417-446` (scan assumes index order == chronological order)
Once the ring wraps (>100 fixes in-zone ≈ 4–5.5 s at 18–25 Hz), the seam pair compares the newest fix against the oldest; if they straddle the line, the "first opposite-side pair" search latches onto it and interpolates between points minutes apart. CLAUDE.md's "wraparound is a non-concern at kart speeds" ignores a kart *parked on* the start/finish line (grid start, red flag, stop-and-go) — exactly when the session's first crossing is computed.
**Fix:** freeze buffering when full, or unwind the ring chronologically before scanning. Add a test that sits in-zone past `bufferSize`.

### H3. On the advertised AVR targets, `double` is 32-bit — the precision premise collapses, silently
All of `src/` (e.g. `GeoMath.h:32`); CI compiles for `arduino:avr:mega` and `:uno`
On classic AVR `double == float` (~7 significant digits). At the included Orlando track (lat 28.41 / lng −81.38): position quantizes at ~0.21 m (lat) / ~0.75 m (lng), and the haversine half-angle subtraction `sin((lat2Rad-lat1Rad)/2)` differences (~8e-8 rad per 25 Hz fix) sit *below* float ulp — per-fix odometer increments carry up to ~100% relative error; the side-of-line cross product degrades toward sign noise. Laps still count (7 m threshold survives), but the precision the library is built around does not exist on Mega/Uno, and there is no `#warning`, doc caveat, or `static_assert(sizeof(double) == 8)`. The host test suite (64-bit double) is unrepresentative of these targets by construction.
**Fix:** either drop the AVR-support claim to "compiles, degraded accuracy — see docs," or implement local-tangent-plane offsets from a reference point. At minimum, emit `#warning` on 4-byte-double targets and document it.

### H4. Low-rate GPS silently kills all lap counting — crossing validation conflates zone size with sample density
`src/DovesLapTimer.cpp:459`: `if (crossingSumDistances < crossingThresholdMeters && ...)`
`crossingSumDistances` ≈ inter-fix spacing of the straddle pair. At 1 Hz / 70 km/h that's ~19 m — **every** crossing fails the default 7 m check, `outTime` stays 0, and `checkStartFinish` silently does nothing. No error surfaces through the public API ("INVALID CROSSING" goes only to debug serial); the lap counter just never increments.
**Fix:** validate against `max(threshold, k × observed fix spacing)`, and surface rejected crossings (counter or callback) instead of silence.

### H5. The Lap-Anything fallback is unreachable from the most likely failure mode — detection can hang forever
`src/CourseManager.cpp:161-164`
The fallback fires only after `COURSE_DETECT_MAX_REJECTIONS`, and the counter increments only in `_handleCandidatesReady()` — which requires the detector to *produce candidates*. If the driven lap never matches any configured course within 25% (wrong config, or all `lengthFt <= 0`, which `CourseDetector.cpp:79` skips), the detector sits in `WAYPOINT_SET` forever: `isDetectionComplete()` false forever, `getActiveTimer()` NULL forever — while the WaypointLapTimer has been silently timing laps the whole time and is never surfaced. No lap-count or distance timeout exists.
**Fix:** add a no-match counter or distance/lap timeout that also feeds the fallback path.

### H6. "Pruning saves memory" is false, and `CourseManager` cannot fit on the advertised AVR minimum
`src/CourseManager.h:94`, `src/CourseManager.cpp:172-180`
`CourseTimerEntry _courseTimers[MAX_COURSES]` is a by-value member array — measured **28,736 B** of the manager's **30,800 B** total (host; ~17 KB on Mega with 4-byte doubles vs. its 8 KB SRAM). `pruneInactiveCourses()` only sets `active = false`; **zero bytes are ever released**. CLAUDE.md's "drops to ~5 KB after pruning" is fiction — pruning saves CPU, not memory. And the library advertises "Arduino Mega+ (minimum)" with no `static_assert`, `#warning`, or doc note that this class bricks a Mega.
**Fix:** correct the documentation immediately (cheap); longer-term, placement-new into a raw storage pool sized by `courseCount`, or gate `CourseManager` off AVR explicitly.

### H7. Lap-Anything mode leaves all 8 course timers running forever — and prune is unusable in exactly that mode
`src/CourseManager.cpp:167-170`, `:173`
`_activateLapAnything()` sets two flags but deactivates nothing, so `loop()` keeps feeding all 8 full crossing pipelines every fix at 25 Hz, forever. The one API that could stop it early-returns: `pruneInactiveCourses()` bails when `_activeCourseIndex < 0` — which is precisely the Lap-Anything state.
**Fix:** `_activateLapAnything()` should deactivate all course entries itself.

### H8. `WaypointLapTimer._proximityBuffer` is 1.6 KB of write-only dead memory
`src/WaypointLapTimer.h:136`, `src/WaypointLapTimer.cpp:143-152` (writes), `:170-218` (never read)
50 × 32 B populated on every in-proximity fix; `_processProximityBuffer()` reads only the separately-tracked `_closestDist/_closestTime/_closestOdometer` scalars. On a codebase whose rule 5 is "stay memory-conscious, targets down to AVR," 1.6 KB of SRAM per instance buying nothing is indefensible.
**Fix:** delete the buffer (and rename `_processProximityBuffer`), or implement the closest-approach interpolation it was reserved for.

### H9. README "Direction Detection" documents the *removed* algorithm — and the changelog claims it was fixed
`README.md:434-436` vs `src/DovesLapTimer.cpp:524-555`; `CHANGELOG.md:81-83`
The README section still says "the first sector line you cross determines direction" — the exact algorithm removed in the 2026-05-20 fix (issues #11/#12); the code resolves direction only at the next start/finish when BOTH S2 and S3 were crossed. The changelog asserts the README was updated; only the "What's New" bullet (`README.md:59`) was — the README now contradicts itself. Two more public doc/units lies: `getPaceDifference()` is documented as "seconds" in README:255 and "milliseconds" in `DovesLapTimer.h:334` — the code returns **ms per meter** (`cpp:829-833`); `pointLineSegmentDistance` header doc claims degrees (`h:154-156`) — every return path is meters via `haversine`.
**Fix:** rewrite README:434-436, correct both unit docs, and add doc-claim spot checks to the release checklist.

### H10. Local test gate gives false greens — Makefile tracks no header dependencies
`test/Makefile:35` — reproduced: `touch ../src/DovesLapTimer.h && make` → "Nothing to be done"
Binaries depend on `$(LIB_SRCS)` and the mocks but **not** `../src/*.h`, `replay_runner.h`, or fixtures. `DovesLapTimer.h` is 658 lines containing the entire `DirectionDetector` implementation. The project's stated pre-commit gate ("`cd test && make run`, all green, no exceptions") passes on stale binaries after any header edit; local and CI can disagree.
**Fix:** add `-MMD -MP` dep generation (or list headers as prerequisites). One-line class of fix.

### H11. The two largest v4.0 modules have zero tests
`src/CourseManager.cpp` (263 lines) and `src/WaypointLapTimer.cpp` (305 lines) appear nowhere in `test/` except as link inputs. The multi-timer orchestration, candidate accept/reject via `raceStarted`, the Lap-Anything fallback, and `pruneInactiveCourses()` are all unverified — and the issue-#13 fix is tested only at the `CourseDetector` level, not through `CourseManager` where the bug manifested. C3, H5, H6, H7, and H8 above all live in this untested code; that is not a coincidence.
**Fix:** add `test_course_manager.cpp` and `test_waypoint_lap_timer.cpp` covering detection-accept, reject→fallback, no-match (C3/H5), prune behavior, and waypoint lap accounting.

### H12. CI supply chain: all actions on floating tags, third-party action holds `contents: write`, no Dependabot
All five workflows: `actions/checkout@v4`, `arduino/compile-sketches@v1`, `actions/github-script@v7`, and `peaceiris/actions-gh-pages@v3` (third-party, a generation old, running with `contents: write` in `docs.yml:36` and `coverage.yml:84`). A hijacked tag could push arbitrary content to gh-pages/badges or exfiltrate the token. `.github/dependabot.yml` does not exist, so nothing monitors any of this. Related: `workflow_dispatch` on `docs.yml:35` / `coverage.yml:72-89` lets a manual run from **any branch** overwrite production gh-pages and the coverage badge.
**Fix:** pin all actions to commit SHAs, add `dependabot.yml` (github-actions ecosystem), restrict deploy steps to `github.ref == 'refs/heads/master'`.

### H13. The coverage gate gates nothing
`test/Makefile:63` (`COVERAGE_GATE ?= 1`), `coverage.yml:28-32`
Actual coverage is ~51%; the gate is 1%. You could delete every behavioral test and a single `geoHaversine` call would keep CI green. "Intentionally low, raise as coverage grows" was written; it was never raised.
**Fix:** set the gate to ~48% today (just below current), and raise it with H11's new suites.

### H14. The public API is a five-unit-system trap with no shared conversion constants
`DovesLapTimer::loop(..., speedKnots)` (`h:101`) · `CourseDetector::update(..., speedKmh, ...)` (`CourseDetector.h:34`) · `setSpeedThresholdMph(mph)` (`h:41`) · `CourseInfo.lengthFt` (`CourseManager.h:22`) vs. meters everywhere else · ms-since-midnight time. One user integration touches all five. Internally the conversions use three different literals scattered across files: `* 1.852` (3 sites), `* 0.621371` (2 sites), `/ 1.60934f` (2 sites) — and 1/1.60934 ≠ 0.621371 at the 7th digit. The chain is currently *correct* (verified), but there is nothing keeping it that way.
**Fix:** shared `constexpr` conversion constants in `GeoMath.h` (which exists for exactly this); long-term, standardize new API on one speed unit.

---

## MEDIUM — architecture, performance, silent misuse, test-quality debt

### M1. `DovesLapTimer` is a god class
`src/DovesLapTimer.h:562-656` — ~52 member variables, ~45 public methods, ≥7 responsibilities (odometer, speed conversion, generic geo utilities, crossing-zone state machine, buffer management, interpolation, lap timing, sector timing, direction feeding, pace). The v4.0 refactor created `GeoMath.h`, yet `haversine`/`haversine3D` remain instance methods that delegate (`cpp:367-373`), and the stateless line-geometry helpers stayed members. The 12-coordinate three-line representation wants `struct GeoLine { GeoPoint a, b; }`; the 9 sector members want their own type.
**Fix (incremental):** geometry → free functions; introduce `GeoPoint`/`GeoLine`; extract sector state machine.

### M2. Public-header namespace pollution
`DovesLapTimer.h:13` — global `using TRITYPE = double;` (used once). `h:19-45` — ~20 unprefixed object-like macros (`MAX_COURSES`, `DIR_FORWARD`, `METERS_TO_FEET`, `DETECT_STATE_*`; plus `WLT_STATE_*` in `WaypointLapTimer.h:24-27`) — high collision odds in user sketches. `h:638` `radiusEarth` is dead (GeoMath's constant is the one used). `crossingPointBufferEntry` (`h:55`) is a lowercase-named private detail exposed globally. The detector/waypoint constants live in the core header — a dumping ground for sibling modules.
**Fix:** `constexpr` in a namespace or `DOVES_`-prefix; move sibling constants out; delete `TRITYPE` and `radiusEarth`.

### M3. Duck-typing contract has already drifted, with no enforcement
`CourseManager.h:8-9` promises interchangeable timers; reality: `WaypointLapTimer` lacks `getCurrentLapStartTime()`/`getCurrentLapOdometerStart()`; `DovesLapTimer::loop` returns 0/−1 (undocumented) while `CourseManager::loop`/`WaypointLapTimer::loop` return constant −1; `WaypointLapTimer` ignores altitude (2D odometer, `-Wunused-parameter` warning at `WaypointLapTimer.cpp:54` under the project's own flags) while `DovesLapTimer` is 3D — the twins report different distances for the same drive. Every consumer must branch on `isLapAnythingActive()` and duplicate rendering code. The vtable-avoidance argument is moot since this subsystem can't run on AVR anyway (H6).
**Fix:** small abstract `ILapTimer` interface, or at minimum a `static_assert`/test-suite API-parity check; make `loop()` `void` or document the return.

### M4. ~40% of `WaypointLapTimer` is copy-pasted, plus internal self-duplication
~120-140 of 306 lines duplicate `DovesLapTimer` (odometer update, lap accounting, ~60 lines of getters including the same `<= 0`-on-unsigned wart). The two branches of `_processProximityBuffer()` (`WaypointLapTimer.cpp:179-218`) are 36 character-identical lines differing by one assignment and a debug string. The `debug_print` template pair is pasted into three headers; `#define debug`/`#define debugln` into three .cpp files. The `loop()` sector blocks in `DovesLapTimer.cpp:70-103` execute the same call in both branches of an `if/else if`; the linear-interpolation fallback is duplicated verbatim at `cpp:476-480`/`:490-494`. Zero tests guard either copy against drift.

### M5. Hot-path performance: two-thirds of per-fix trig is constant or redundant
`src/DovesLapTimer.cpp:262-286` — `insideLineThreshold` recomputes the **fixed line's own length** by haversine on every fix instead of caching it in the setters. Per fix with 3 lines: 10 haversines (~20 sin, 20 cos, 10 atan2, 20 sqrt), all *software* double-precision on every supported MCU (nRF52840's FPU is single-precision). CourseManager pre-prune: ~83 haversines ≈ 400+ double transcendentals per fix; at 25 Hz that's order 15–25% of a 64 MHz M4. `pointLineSegmentDistance` (`:350-351`) always computes both endpoint haversines before checking the projection scalar. On zone exit, `interpolateCrossingPoint` can do ~600 haversines in one `loop()` call — a multi-ms stall at the timing-critical moment.
**Fix:** cache line lengths in setters (easy 30%+); short-circuit endpoint distances; consider equirectangular approximation for the sub-100 m zone checks.

### M6. Debug logging floods the UART exactly while crossing the line
`DovesLapTimer.cpp:221-228` (per in-zone fix), `:432-443` (per buffer pair on exit); `real_track_data_debug.ino:119` runs `Serial.begin(9600)` while in-zone output exceeds 960 B/s — `Serial.print` blocks when the TX buffer fills, and on real hardware the GPS UART overruns and drops sentences during the crossing. The `F()` literals also cost flash even with `_serial == nullptr`.
**Fix:** compile-time debug gate (`#ifdef DOVES_DEBUG`), and bump the example's baud.

### M7. Orchestration-layer input validation holes
`CourseDetector::acceptCandidate(int index)` (`CourseDetector.cpp:106-110`): no bounds check — `acceptCandidate(-5)` yields `DETECT_STATE_DETECTED` with a garbage index. `_initCourses` clamps only the upper bound (`CourseManager.cpp:21`): negative `courseCount` silently deadlocks the manager (detector never fed, fallback never armed). NULL course `name` flows into `debugln(name)` → `strlen(NULL)` crash on most cores when debug is enabled; `getActiveCourseName()` can return NULL while `getTrackName()` guards with "Unknown" — inconsistent contract.

### M8. Planar geometry in raw degree space — no `cos(latitude)` longitude scaling
`DovesLapTimer.cpp:318-334` (`pointOnSideOfLine`), `:336-364` (`pointLineSegmentDistance`)
One degree of longitude is `cos(lat)` shorter than one of latitude (≈12% compression at the Orlando track, ≈30% at 45°N), so the projection scalar and "closest point" are computed in a stretched frame; the subsequent haversine faithfully measures distance to the *wrong* point, biasing zone entry/exit and the interpolation weight. Side *sign* is unaffected, and error is small at low latitude — a classic "works where the author tested it" bug that grows with latitude. No antimeridian handling either (undocumented).
**Fix:** scale longitude deltas by `cos(lat)` before projecting (one line per function); document operating limits.

### M9. Mutual-exclusion between lines silently drops crossings
`DovesLapTimer.cpp:61, 68` — while a sector crossing is active, start/finish is skipped and vice versa (shared buffer). With overlapping zones (short tracks; the hypotenuse inflation makes zones larger than the threshold suggests), whichever line is entered first wins and the other crossing is *lost*, desynchronizing the sector state machine into the out-of-order rejection path. Sector buffering also runs before race start (`raceStarted` checked only at completion, `:689`), blocking start/finish detection in that window.
**Fix:** per-line buffers, or document a minimum line-separation requirement and detect violations at setup.

### M10. `CourseManager` constructor materializes 3.5 KB temporaries on the stack
`CourseManager.cpp:31` — `_courseTimers[i].timer = DovesLapTimer(...)` copy-assigns a measured 3,576 B temporary over an already default-constructed member, inside a constructor typically run as a global initializer. ESP32's default loopTask stack is 8 KB — one statement eats ~44% of it.
**Fix:** a `configure(threshold, serial)` re-init method instead of assignment-from-temporary.

### M11. Detector is fed the noisier of two odometers — the one with an open "alt is messing up" TODO
`CourseManager.cpp:113-121` feeds `CourseDetector` the 3D altitude-inclusive odometer (`DovesLapTimer.cpp:28` TODO) for a precision-sensitive 25%-tolerance length match, while `WaypointLapTimer` uses 2D. The two timers report different total distances for the same drive.
**Fix:** resolve the altitude TODO (clamp/flag altitude deltas), or feed the detector 2D distance consistently.

### M12. Test-quality debt (grouped)
- **Catmull-Rom has zero behavioral assertions** — all tests assert `catmull == linear`, true by design (spline scoped to lat/lng, which nothing checks). The spline could return NaN coordinates with a green suite.
- **`DovesLapTimer::reset()` never tested** — especially mid-crossing with a half-filled buffer; `reset()` also misses `currentSpeedkmh` (`cpp:698-750`), so `getCurrentSpeedKmh()` reports stale pre-reset speed.
- **No adversarial tests**: NaN/Inf, time backwards, buffer wrap (H2), zero-length line, dropout mid-zone.
- **Replay suites never configure sector lines** (`replay_runner.h:124-129` has no sector fields) — sector timing, out-of-order rejection, and direction detection are never exercised on real noisy data.
- **`LapRecord records[3];` uninitialized** in `test_synthetic_track.cpp:130` and 5 siblings — regressions yield garbage comparisons instead of clean diagnostics (`= {}` fixes it).
- **Goldens drifted from their own process**: `gps_race_data_lap.h:9-10` documents 68748/68745 ms, tests pin 68742; `test/README.md:31`/CLAUDE.md claim ±50 ms tolerance, tests use ±10 ms; CLAUDE.md says 21 replay tests, there are 23. `long_lap`/`praga` fixtures have no independent ground truth.
- **`EXPECT_EQ` failure output omits actual values** (`test_runner.h:46-55`).
- **Replay NMEA parser**: no checksum validation (`replay_runner.h:69`), and `strtok_r` collapses empty fields (`:74`) — a void RMC sentence shifts fields, skips the `fix=false` branch, and the replay keeps feeding stale coordinates after fix loss. Latent with current fixtures, wrong for general NMEA.

### M13. CI rigor gaps (grouped)
- **No ASan/UBSan lane** — the cheapest possible catch for C1/C2-class bugs (the host suite already exists; one Makefile target).
- **No static analysis** (cppcheck/clang-tidy), no `-Werror` (the `WaypointLapTimer.cpp:54` unused-parameter warning ships, repeated 8×, contradicting "no broken windows"), no clang build (test/README claims macOS/clang support, never exercised), no format check.
- **No caching** — ESP32 + Seeed BSPs (hundreds of MB) re-downloaded every run; apt/pip installs every run.
- **No `concurrency` groups or `timeout-minutes`** on any workflow; push+PR double-runs the 4-board matrix; no path filters (docs-only changes run everything).
- **No release automation** and no scheduled run — floating BSP indexes mean upstream toolchain breakage surfaces only on the next unrelated push.

### M14. Doc/site/release drift (grouped)
- `Doxyfile:12` `PROJECT_NUMBER = "4.0.0"` vs `library.properties` 4.1.0 — the published docs site advertises the previous major. The release checklist (`CONTRIBUTING.md:84-91`) omits the Doxyfile, so it will drift again.
- **No git tags exist** (`git tag -l` empty) yet `CHANGELOG.md:145-146` links to release tags and Library Manager ingestion depends on them.
- `keywords.txt` missing `getCurrentSpeedKmh`/`getCurrentSpeedMph` (added in PR #39) — went stale within five weeks of claiming full coverage.
- Three of five public headers (`CourseManager.h`, `CourseDetector.h`, `WaypointLapTimer.h`) have **no per-method Doxygen at all** — the published API site shows bare signatures for the entire headline v4.0 API; `WARN_NO_PARAMDOC=NO` ensures CI never notices.
- README:11 describes the algorithm as using "the 4 points closest to the line" — it actually uses the first opposite-side consecutive pair.
- `CHANGELOG [Unreleased]` missing: HELPME.md→DETECTION.md rename (breaks old links), coverage-gate/badge CI work, CODE_OF_CONDUCT.
- No GPL/SPDX headers in any source file (`grep -ri 'GPL\|Copyright\|SPDX' src/` → 0 hits); `GPL-3.0` is a deprecated SPDX id (use `GPL-3.0-only`/`-or-later`); `library.properties` `includes=` omits `GeoMath.h`.

### M15. Flagship example teaches a data-losing pattern
`basic_oled_example.ino:134` reads **one UART byte per `loop()`** while `displayLoop()` blocks ~25–100 ms on a full I²C framebuffer push 3×/s — at 115200 baud/18 Hz, one refresh accumulates 70–270 bytes against a 64–256 B RX buffer → routine sentence loss. `gpsSetup()` hardcodes config array lengths (`GPS_SendConfig(uart115200NmeaOnly, 28)`) instead of `sizeof` — a future `gps_config.h` edit becomes a PROGMEM overread. `while (!Serial);` is the default-on state of both hardware examples (`basic_oled_example.ino:87-88`) — the timer bricks in the kart without a USB host.
**Fix:** drain the UART in a `while (gps->available())` loop, make display refresh time-budgeted, use `sizeof`, default `HAS_DEBUG` off.

---

## LOW — polish, hygiene, latent warts

- **L1. Dead/experimental code shipped as public API**: `isObtuseTriangle` (`cpp:288-316`) called by nothing anywhere, billed in keywords.txt, complete with "this has been a long debugging session" comment; commented-out haversine call (`cpp:37-42`) and debug block (`:273-282`); stale `// todo: defines?` (`:326`).
- **L2. `<= 0` comparisons on `unsigned long`** (`cpp:143, 790`; `WaypointLapTimer.cpp:239`) — means `== 0`, trips `-Wtype-limits`, signals domain confusion. Related: time/odometer **zero-as-sentinel** holes — a legitimate crossing at exactly midnight tick 0 is discarded (`cpp:130, 689`; `WaypointLapTimer.cpp:171`); `getCurrentLapDistance()` (`cpp:802`) uses exact float equality `currentLapOdometerStart == 0` as "not started."
- **L3. Stateless geometry methods are non-const members** (`h:116-198`: `haversine`, `pointLineSegmentDistance`, `pointOnSideOfLine`, `insideLineThreshold`, …) — a `const DovesLapTimer&` can't call them, yet the examples use them as utilities. Also `CourseManager(TrackConfig&)` / `CourseDetector::init(CourseInfo*)` should take `const`; config `name` pointer lifetime is undocumented.
- **L4. Header hygiene**: `DovesLapTimer.h` uses `Stream`/`F()`/`sq()` but includes only `ArxTypeTraits.h`, compiling via a transitive `Arduino.h` — every sibling header includes it explicitly. `CourseDetector.h`/`WaypointLapTimer.h` include the full 3.5 KB `DovesLapTimer.h` solely for shared `#define`s.
- **L5. Magic numbers**: exit hysteresis `crossingThresholdMeters + 1` (`cpp:188` — why 1 m? GPS noise exceeds it), `1e-12` (`:343`), `1e-9` (`:384, :395`), unit literals (see H14). Wrong-units comments: `cpp:380` says "knots," parameters are km/h; `h:59` says "time traveled," field is distance.
- **L6. `loop()` return contract**: declared `int`, returns 0 when near a line, −1 otherwise — inverse of intuitive truthiness, no `@return` anywhere; `checkStartFinish` doc block describes a parameter that doesn't exist.
- **L7. Same-frame double-buffering landmine**: `WaypointLapTimer::loop()` sequential `if`s buffer the entry point twice (`WaypointLapTimer.cpp:80, 126`) — currently masked because the buffer is dead (H8); becomes a bug the day someone makes it live.
- **L8. `GeoMath.h`**: `static const` gives every TU its own copy (use `constexpr`); clamp `a` with `fmin(a, 1.0)` against NaN at antipodes for a "shared utility" header.
- **L9. Test framework**: no crash isolation within a suite binary (segfault silently skips later tests; exit code still fails — mitigated), no fixtures (each replay assertion re-runs a full NMEA replay), 32 redundant TU compiles.
- **L10. Wokwi chip**: unchecked `malloc` per message at 18 Hz, dead loop-guard logic with an inverted comment (`gps-fake.chip.c:7727-7743`). Simulator-only.
- **L11. Commit/process slips**: "Spelling lol" commit a week after CONTRIBUTING forbade it; direct-to-master GitHub-UI commits bypassing the documented PR workflow; "all documentation has been generated using chatGPT4" banner still atop the flagship header (`h:6`).
- **L12. `totalDistanceTraveled` is `float`** — ulp ≈ 8 mm at 131 km accumulated; fine for karting, undocumented for the endurance use the README gestures at.

---

# Suggested work-through order

1. **C1, C2, C3** — the three field-failure correctness bugs, each with a regression test (per project rule 1).
2. **H1, H2, H4** — input validation, buffer-wrap, low-rate rejection: hardens the core against the real world.
3. **H10, H13, H11** — fix the Makefile deps, raise the coverage gate to ~48%, then write the CourseManager/WaypointLapTimer suites (which will force confronting H5–H8).
4. **H5, H6, H7, H8** — orchestration-layer fixes; correct the CLAUDE.md/README memory claims in the same PR (H6 doc half is a 5-minute fix — do it immediately).
5. **H9, M14** — doc-honesty pass: README direction section, pace/distance units, Doxyfile version, keywords.txt, tags. Cheap, high credibility value.
6. **H12, M13** — pin actions to SHAs + dependabot + ASan/UBSan lane + `-Werror` (fix the one warning first).
7. **H3, H14, M8** — decide the AVR story honestly; centralize unit constants; add `cos(lat)` scaling.
8. **M1–M7, M9–M12, M15** — architecture and example cleanups, as capacity allows.
9. **L1–L12** — polish pass.

Items 1–6 are mostly small, surgical changes; nothing before item 7 requires design work. Completing 1–6 alone would move the overall score to roughly **6/10**; the architectural items (god class, duplication, duck-type contract) are what stand between 6 and 8.
