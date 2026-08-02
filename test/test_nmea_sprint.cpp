/**
 * Layer-3 differential replay: SprintTimer vs DovesLapTimer on REAL data.
 *
 * Fixture: examples/real_track_data_debug/gps_race_data_2laps.h
 *   (Orlando Kart Center, "Normal" layout, two laps, real noisy GPS)
 *
 * Idea (from the sprint-mode plan): treat the circuit lap as a pseudo-sprint
 * by using the OKC start/finish line as the sprint START and the OKC
 * sector 2 line as the sprint FINISH. Every recorded lap then contains
 * exactly one point-to-point run whose duration must equal that lap's
 * SECTOR 1 time as computed by the circuit timer on the very same fixes.
 *
 * That makes this a self-validating differential test: two independent
 * state machines (lap accounting vs run accounting) over the same
 * crossing engine and the same real-world data must agree — no synthetic
 * geometry, no hand-picked goldens. It does NOT exercise sprint-specific
 * semantics the circuit data can't produce (staging stops, cancel/restart,
 * ignored finishes) — those live in test_sprint_timer.cpp's synthetic
 * suite. Better point-to-point fixtures (a real autocross log) can join
 * later without displacing either.
 *
 * Line coordinates are the OKC "Normal" course from the DovesDataLogger
 * repo's SDCARD/TRACKS/OKC.json — its S/F line matches this fixture's
 * documented S/F to the last digit.
 */

#include "test_runner.h"
#include "replay_runner.h"
#include "../src/SprintTimer.h"
#include "../examples/real_track_data_debug/gps_race_data_2laps.h"

static const int num_gps_logs = sizeof(gps_logs) / sizeof(gps_logs[0]);

// OKC "Normal" course (SDCARD/TRACKS/OKC.json, DovesDataLogger repo)
static constexpr double SF_A_LAT = 28.4127081705638, SF_A_LNG = -81.3797326641803;
static constexpr double SF_B_LAT = 28.4127303867932, SF_B_LNG = -81.3795704875378;
static constexpr double S2_A_LAT = 28.4119049886871, S2_A_LNG = -81.3790708193926;
static constexpr double S2_B_LAT = 28.4118316342961, S2_B_LNG = -81.3791856652217;
static constexpr double S3_A_LAT = 28.4115010664104, S3_A_LNG = -81.3799856475317;
static constexpr double S3_B_LAT = 28.4115084390461, S3_B_LNG = -81.3798064021136;

struct DifferentialResult {
  std::vector<unsigned long> circuitSector1;  // per lap: S/F -> S2, circuit timer
  std::vector<unsigned long> sprintRuns;      // per run:  S/F -> S2, sprint timer
  int circuitLaps = 0;
  int sprintRuns_n = 0;
  int cancelled = 0;
  bool sprintRunActiveAtEnd = false;
};

// Feed the identical fix stream to both timers, capturing the circuit
// timer's sector-1 completions (snapshot at the 1 -> 2 sector transition,
// while the value is still in the current-lap accumulator) and the sprint
// timer's completed runs.
static DifferentialResult runDifferential() {
  DovesLapTimer circuit(7.0);
  circuit.setStartFinishLine(SF_A_LAT, SF_A_LNG, SF_B_LAT, SF_B_LNG);
  circuit.setSector2Line(S2_A_LAT, S2_A_LNG, S2_B_LAT, S2_B_LNG);
  circuit.setSector3Line(S3_A_LAT, S3_A_LNG, S3_B_LAT, S3_B_LNG);
  circuit.forceLinearInterpolation();

  SprintTimer sprint(7.0);
  sprint.setStartLine(SF_A_LAT, SF_A_LNG, SF_B_LAT, SF_B_LNG);
  sprint.setFinishLine(S2_A_LAT, S2_A_LNG, S2_B_LAT, S2_B_LNG);
  sprint.forceLinearInterpolation();

  DifferentialResult r;
  NmeaState state;
  int lastSector = 0;
  int lastRuns = 0;

  for (int i = 0; i < num_gps_logs; i++) {
    if (!parseNmeaLine(gps_logs[i], state)) continue;
    if (!state.fix) continue;

    circuit.updateCurrentTime(state.time_ms);
    circuit.loop(state.lat, state.lng, state.alt_m, state.speed_knots);
    sprint.updateCurrentTime(state.time_ms);
    sprint.loop(state.lat, state.lng, state.alt_m, state.speed_knots);

    int sector = circuit.getCurrentSector();
    if (sector == 2 && lastSector == 1) {
      r.circuitSector1.push_back(circuit.getCurrentLapSector1Time());
    }
    lastSector = sector;

    if (sprint.getRuns() > lastRuns) {
      r.sprintRuns.push_back(sprint.getLastRunTime());
      lastRuns = sprint.getRuns();
    }
  }

  r.circuitLaps = circuit.getLaps();
  r.sprintRuns_n = sprint.getRuns();
  r.cancelled = sprint.getCancelledRunCount();
  r.sprintRunActiveAtEnd = sprint.isRunActive();
  return r;
}

void test_three_runs_from_three_laps() {
  DifferentialResult r = runDifferential();
  // The "2laps" fixture actually contains THREE full laps (the name refers
  // to the two golden-pinned ones); each lap contains exactly one complete
  // S/F -> S2 sequence. The lap-3-closing S/F crossing opens a FOURTH
  // sprint run that the fixture ends before finishing — exactly the
  // expected "abandoned run records nothing" behavior on real data.
  EXPECT_EQ(r.circuitLaps, 3);
  EXPECT_EQ(r.sprintRuns_n, 3);
  EXPECT_EQ((int)r.sprintRuns.size(), 3);
  EXPECT_EQ((int)r.circuitSector1.size(), 3);
}

void test_run_times_match_pinned_goldens() {
  // Golden pins in the layer-3 style (measured on this fixture, ±50 ms):
  // sector-1 durations of OKC "Normal" laps 1-3 as point-to-point runs.
  DifferentialResult r = runDifferential();
  EXPECT_TRUE(r.sprintRuns.size() >= 3);
  EXPECT_NEAR(r.sprintRuns[0], 28378UL, 50.0);
  EXPECT_NEAR(r.sprintRuns[1], 27903UL, 50.0);
  EXPECT_NEAR(r.sprintRuns[2], 28304UL, 50.0);
}

void test_run_times_match_circuit_sector1_times() {
  // The core differential assertion: same fixes, same engine, two
  // independent accounting layers -> identical crossing interpolations ->
  // run time == that lap's sector-1 time, to the millisecond (10 ms slack
  // for any rounding asymmetry).
  DifferentialResult r = runDifferential();
  size_t n = r.sprintRuns.size() < r.circuitSector1.size()
           ? r.sprintRuns.size() : r.circuitSector1.size();
  EXPECT_TRUE(n >= 3);
  for (size_t i = 0; i < n; i++) {
    EXPECT_NEAR(r.sprintRuns[i], r.circuitSector1[i], 10.0);
  }
}

void test_run_times_are_plausible() {
  // Coarse sanity independent of the circuit timer: OKC "Normal" is a
  // ~69s lap and sector 1 is its opening chunk — every run must be a
  // sane fraction of the lap, not 0 and not a whole lap.
  DifferentialResult r = runDifferential();
  for (size_t i = 0; i < r.sprintRuns.size(); i++) {
    EXPECT_TRUE(r.sprintRuns[i] > 2000UL);
    EXPECT_TRUE(r.sprintRuns[i] < 60000UL);
  }
}

void test_no_cancellations_on_circuit_data() {
  // At OKC the start line is never re-crossed between a run's start (S/F)
  // and its finish (S2), so the cancel/restart path must never trigger.
  DifferentialResult r = runDifferential();
  EXPECT_EQ(r.cancelled, 0);
}

void test_final_run_left_open_at_fixture_end() {
  // The lap-3-closing S/F crossing starts sprint run 4; the data ends
  // before the S2 line is reached again. DNF-as-normal-op: nothing
  // recorded, run simply still active.
  DifferentialResult r = runDifferential();
  EXPECT_TRUE(r.sprintRunActiveAtEnd);
  EXPECT_EQ(r.sprintRuns_n, 3);
}

int main() {
  printf("=== NMEA differential replay: SprintTimer vs circuit sector 1 (OKC 2 laps) ===\n");

  RUN_TEST(three_runs_from_three_laps);
  RUN_TEST(run_times_match_pinned_goldens);
  RUN_TEST(run_times_match_circuit_sector1_times);
  RUN_TEST(run_times_are_plausible);
  RUN_TEST(no_cancellations_on_circuit_data);
  RUN_TEST(final_run_left_open_at_fixture_end);

  TEST_SUMMARY();
}
