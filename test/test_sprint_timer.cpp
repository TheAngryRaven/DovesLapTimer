/**
 * Layer-2 integration tests for SprintTimer (point-to-point run timing).
 *
 * Drives the timer over a deterministic synthetic OPEN course — a 500 m
 * straight with a start line at x=50 m, splits at x=150 m (S2) and
 * x=300 m (S3), and a finish line at x=450 m — then loops back on a
 * parallel return leg 40 m north (outside every crossing zone).
 *
 * Step: 5 m per 200 ms fix (25 m/s ≈ 48.6 kn), matching the circuit
 * synthetic test. Expected values are exact by construction:
 *   - run time:   400 m  = 16000 ms
 *   - segment 1:  100 m  =  4000 ms   (start → S2)
 *   - segment 2:  150 m  =  6000 ms   (S2 → S3)
 *   - segment 3:  150 m  =  6000 ms   (S3 → finish)
 *
 * Covers the decided sprint semantics:
 *   - runs complete with correct, deterministic times across multiple runs
 *   - start crossing while RUNNING cancels + restarts (botched-course rule)
 *   - finish crossings while WAITING are ignored
 *   - a backward start crossing opens a run (self-heal documented behavior)
 *   - 0 / 1 / 2 splits are all legal; single split has a real optimal
 *   - a missed split invalidates the run's sector data but not the run
 */

#include "test_runner.h"
#include "../src/SprintTimer.h"

// =============================================================================
// Synthetic open course
// =============================================================================

static constexpr double BASE_LAT = 28.40000;
static constexpr double BASE_LNG = -81.40000;
// Haversine-consistent meters→degrees at BASE_LAT (R = 6371 km).
static constexpr double M_PER_DEG_LAT = 111194.9266;
static constexpr double M_PER_DEG_LNG = 111194.9266 * 0.879649;  // cos(28.4°)

static double latAt(double yMeters) { return BASE_LAT + yMeters / M_PER_DEG_LAT; }
static double lngAt(double xMeters) { return BASE_LNG + xMeters / M_PER_DEG_LNG; }

static constexpr double START_X  = 50.0;
static constexpr double S2_X     = 150.0;
static constexpr double S3_X     = 300.0;
static constexpr double FINISH_X = 450.0;
static constexpr double COURSE_END_X = 500.0;
static constexpr double RETURN_Y = 40.0;   // return leg offset, outside all zones
static constexpr double LINE_HALF_WIDTH = 10.0;  // lines span y in [-10, +10]

static constexpr double STEP_METERS   = 5.0;
static constexpr unsigned long MS_PER_STEP = 200;
static constexpr float SIM_SPEED_KNOTS = 48.0f;
static constexpr float SIM_ALT_METERS  = 50.0f;

static void setLine(SprintTimer &t, void (SprintTimer::*setter)(double, double, double, double), double xMeters) {
  (t.*setter)(latAt(-LINE_HALF_WIDTH), lngAt(xMeters), latAt(LINE_HALF_WIDTH), lngAt(xMeters));
}

static void configureFullCourse(SprintTimer &t) {
  setLine(t, &SprintTimer::setStartLine, START_X);
  setLine(t, &SprintTimer::setFinishLine, FINISH_X);
  setLine(t, &SprintTimer::setSector2Line, S2_X);
  setLine(t, &SprintTimer::setSector3Line, S3_X);
}

// Simple fix-feeding driver: walks the timer along a piecewise path of
// (x, y) waypoints in STEP_METERS increments, advancing sim time per step.
struct PathDriver {
  SprintTimer &timer;
  unsigned long simTimeMs = 0;
  double x = 0, y = 0;

  explicit PathDriver(SprintTimer &t) : timer(t) {}

  void feed() {
    timer.updateCurrentTime(simTimeMs);
    timer.loop(latAt(y), lngAt(x), SIM_ALT_METERS, SIM_SPEED_KNOTS);
    simTimeMs += MS_PER_STEP;
  }

  // Move in a straight line to (targetX, targetY) in STEP_METERS steps,
  // feeding a fix at every step (including the final position).
  void driveTo(double targetX, double targetY) {
    double dx = targetX - x;
    double dy = targetY - y;
    double dist = sqrt(dx * dx + dy * dy);
    int steps = (int)(dist / STEP_METERS + 0.5);
    if (steps < 1) steps = 1;
    for (int i = 1; i <= steps; i++) {
      x += dx / steps;
      y += dy / steps;
      feed();
    }
  }

  // One full out-and-back circuit: out along y=0 crossing every line, then
  // return on the y=RETURN_Y leg and rejoin the start of the course.
  void lapCourse() {
    driveTo(COURSE_END_X, 0);
    driveTo(COURSE_END_X, RETURN_Y);
    driveTo(0, RETURN_Y);
    driveTo(0, 0);
  }
};

// =============================================================================
// Tests
// =============================================================================

void test_initial_state() {
  SprintTimer t(7.0);
  configureFullCourse(t);
  EXPECT_TRUE(t.isStartLineConfigured());
  EXPECT_TRUE(t.isFinishLineConfigured());
  EXPECT_TRUE(t.areSectorLinesConfigured());
  EXPECT_FALSE(t.isRunActive());
  EXPECT_FALSE(t.getRaceStarted());
  EXPECT_EQ(t.getRuns(), 0);
  EXPECT_EQ(t.getCurrentRunTime(), 0UL);
  EXPECT_EQ(t.getLaps(), 0);  // duck-typed alias
}

void test_single_run_time_exact() {
  SprintTimer t(7.0);
  configureFullCourse(t);
  PathDriver d(t);

  d.driveTo(COURSE_END_X, 0);

  EXPECT_EQ(t.getRuns(), 1);
  EXPECT_TRUE(t.getRaceStarted());
  EXPECT_FALSE(t.isRunActive());  // finished, back to WAITING
  EXPECT_NEAR(t.getLastRunTime(), 16000UL, 100.0);
  EXPECT_NEAR(t.getLastRunDistance(), 400.0f, 10.0);
  EXPECT_EQ(t.getBestRunNumber(), 1);
}

void test_run_active_between_start_and_finish() {
  SprintTimer t(7.0);
  configureFullCourse(t);
  PathDriver d(t);

  d.driveTo(100, 0);  // past start (50), before S2
  EXPECT_TRUE(t.isRunActive());
  EXPECT_TRUE(t.getCurrentRunTime() > 0);
  EXPECT_EQ(t.getCurrentSector(), 1);

  d.driveTo(200, 0);  // past S2
  EXPECT_EQ(t.getCurrentSector(), 2);

  d.driveTo(350, 0);  // past S3
  EXPECT_EQ(t.getCurrentSector(), 3);

  d.driveTo(COURSE_END_X, 0);  // past finish
  EXPECT_FALSE(t.isRunActive());
  EXPECT_EQ(t.getCurrentSector(), 0);
  EXPECT_EQ(t.getCurrentRunTime(), 0UL);  // the *waiting* signal
}

void test_three_runs_deterministic() {
  SprintTimer t(7.0);
  configureFullCourse(t);
  PathDriver d(t);

  unsigned long runTimes[3];
  for (int i = 0; i < 3; i++) {
    d.lapCourse();
    runTimes[i] = t.getLastRunTime();
  }

  EXPECT_EQ(t.getRuns(), 3);
  EXPECT_EQ(t.getCancelledRunCount(), 0);
  EXPECT_NEAR(runTimes[0], 16000UL, 100.0);
  EXPECT_NEAR(runTimes[0], runTimes[1], 5.0);
  EXPECT_NEAR(runTimes[1], runTimes[2], 5.0);
}

void test_segment_times_exact() {
  SprintTimer t(7.0);
  configureFullCourse(t);
  PathDriver d(t);

  d.driveTo(COURSE_END_X, 0);

  EXPECT_NEAR(t.getCurrentLapSector1Time(), 4000UL, 100.0);
  EXPECT_NEAR(t.getCurrentLapSector2Time(), 6000UL, 100.0);
  EXPECT_NEAR(t.getCurrentLapSector3Time(), 6000UL, 100.0);
  EXPECT_NEAR(t.getBestSector1Time(), 4000UL, 100.0);
  EXPECT_NEAR(t.getBestSector2Time(), 6000UL, 100.0);
  EXPECT_NEAR(t.getBestSector3Time(), 6000UL, 100.0);
  EXPECT_EQ(t.getBestSector1LapNumber(), 1);

  // Segments sum to the run time, and optimal == best on identical runs.
  unsigned long segSum = t.getBestSector1Time() + t.getBestSector2Time() + t.getBestSector3Time();
  EXPECT_NEAR(segSum, t.getBestRunTime(), 5.0);
  EXPECT_NEAR(t.getOptimalLapTime(), t.getBestRunTime(), 5.0);
}

void test_start_recross_cancels_and_restarts() {
  // Botched course: driver gets to x=250 (past S2), gives up, loops back
  // around on the return leg, re-crosses the start, and runs the full
  // course. The aborted attempt must be cancelled — one completed run,
  // one cancellation, and the completed run's time is a clean 16000 ms
  // from the SECOND start crossing.
  SprintTimer t(7.0);
  configureFullCourse(t);
  PathDriver d(t);

  d.driveTo(250, 0);           // start crossed, S2 crossed, mid-course
  EXPECT_TRUE(t.isRunActive());
  d.driveTo(250, RETURN_Y);    // bail out north
  d.driveTo(0, RETURN_Y);      // return leg (no zones)
  d.driveTo(0, 0);             // rejoin course start
  EXPECT_TRUE(t.isRunActive());  // aborted run still nominally active
  d.driveTo(COURSE_END_X, 0);  // full clean run

  EXPECT_EQ(t.getRuns(), 1);
  EXPECT_EQ(t.getCancelledRunCount(), 1);
  EXPECT_NEAR(t.getLastRunTime(), 16000UL, 100.0);
  EXPECT_NEAR(t.getBestRunTime(), 16000UL, 100.0);
}

void test_finish_crossing_ignored_while_waiting() {
  // Drive east-to-west across the finish line with no run active: nothing
  // may start, complete, or count.
  SprintTimer t(7.0);
  configureFullCourse(t);
  PathDriver d(t);
  d.x = COURSE_END_X;
  d.y = 0;

  d.driveTo(400, 0);  // backward across the finish (450)

  EXPECT_EQ(t.getRuns(), 0);
  EXPECT_FALSE(t.isRunActive());
  EXPECT_FALSE(t.getRaceStarted());
  EXPECT_EQ(t.getLastRunTime(), 0UL);
}

void test_backward_start_crossing_opens_run() {
  // Documented self-heal behavior: crossing detection is direction
  // agnostic, so a backward start crossing opens a (bogus) run — which
  // the next forward start crossing cancels. The timed result of the real
  // launch is unaffected.
  SprintTimer t(7.0);
  configureFullCourse(t);
  PathDriver d(t);
  d.x = COURSE_END_X;
  d.y = 0;

  d.driveTo(0, 0);  // backward down the whole course, crossing start last
  EXPECT_TRUE(t.isRunActive());   // bogus run opened...
  EXPECT_EQ(t.getRuns(), 0);

  d.driveTo(COURSE_END_X, 0);     // ...real launch cancels it, clean run
  EXPECT_EQ(t.getRuns(), 1);
  EXPECT_EQ(t.getCancelledRunCount(), 1);
  EXPECT_NEAR(t.getLastRunTime(), 16000UL, 100.0);
}

void test_single_split_is_legal() {
  // Only S2 configured: two segments (start→S2 = 4000, S2→finish = 12000),
  // and a real optimal — unlike the circuit timer's both-or-nothing gate.
  SprintTimer t(7.0);
  setLine(t, &SprintTimer::setStartLine, START_X);
  setLine(t, &SprintTimer::setFinishLine, FINISH_X);
  setLine(t, &SprintTimer::setSector2Line, S2_X);
  PathDriver d(t);

  EXPECT_TRUE(t.areSectorLinesConfigured());
  d.driveTo(COURSE_END_X, 0);

  EXPECT_EQ(t.getRuns(), 1);
  EXPECT_NEAR(t.getCurrentLapSector1Time(), 4000UL, 100.0);
  EXPECT_NEAR(t.getCurrentLapSector2Time(), 12000UL, 100.0);
  EXPECT_EQ(t.getCurrentLapSector3Time(), 0UL);
  EXPECT_NEAR(t.getOptimalLapTime(), t.getBestRunTime(), 5.0);
}

void test_no_splits_still_times_runs() {
  SprintTimer t(7.0);
  setLine(t, &SprintTimer::setStartLine, START_X);
  setLine(t, &SprintTimer::setFinishLine, FINISH_X);
  PathDriver d(t);

  EXPECT_FALSE(t.areSectorLinesConfigured());
  d.driveTo(COURSE_END_X, 0);

  EXPECT_EQ(t.getRuns(), 1);
  EXPECT_NEAR(t.getLastRunTime(), 16000UL, 100.0);
  EXPECT_EQ(t.getOptimalLapTime(), 0UL);
  EXPECT_EQ(t.getBestSector1Time(), 0UL);
}

void test_missed_split_invalidates_sector_data_not_run() {
  // Detour around S2 (northward bulge from x=125 to x=175), then finish
  // normally. The run completes and counts; segment data must not feed
  // the best-segment table.
  SprintTimer t(7.0);
  configureFullCourse(t);
  PathDriver d(t);

  d.driveTo(125, 0);
  d.driveTo(125, RETURN_Y);   // detour north, S2 zone never entered
  d.driveTo(175, RETURN_Y);
  d.driveTo(175, 0);
  d.driveTo(COURSE_END_X, 0); // S3 + finish crossed normally

  EXPECT_EQ(t.getRuns(), 1);
  EXPECT_TRUE(t.getLastRunTime() > 16000UL);  // detour cost real time
  EXPECT_EQ(t.getBestSector1Time(), 0UL);     // no best segments recorded
  EXPECT_EQ(t.getBestSector2Time(), 0UL);
  EXPECT_EQ(t.getOptimalLapTime(), 0UL);
}

void test_best_run_tracking_across_unequal_runs() {
  // Run 1 clean (16000). Run 2 with a mid-course detour (slower). Best
  // must stay run 1.
  SprintTimer t(7.0);
  configureFullCourse(t);
  PathDriver d(t);

  d.lapCourse();  // run 1: 16000
  d.driveTo(200, 0);
  d.driveTo(200, RETURN_Y);   // detour (S3 not yet crossed)
  d.driveTo(240, RETURN_Y);
  d.driveTo(240, 0);
  d.driveTo(COURSE_END_X, 0); // run 2 completes, slower

  EXPECT_EQ(t.getRuns(), 2);
  EXPECT_TRUE(t.getLastRunTime() > t.getBestRunTime());
  EXPECT_EQ(t.getBestRunNumber(), 1);
  EXPECT_NEAR(t.getBestRunTime(), 16000UL, 100.0);
}

void test_reset_clears_state_keeps_lines() {
  SprintTimer t(7.0);
  configureFullCourse(t);
  PathDriver d(t);

  d.lapCourse();
  EXPECT_EQ(t.getRuns(), 1);

  t.reset();
  EXPECT_EQ(t.getRuns(), 0);
  EXPECT_FALSE(t.getRaceStarted());
  EXPECT_FALSE(t.isRunActive());
  EXPECT_EQ(t.getBestRunTime(), 0UL);
  EXPECT_EQ(t.getCancelledRunCount(), 0);
  EXPECT_TRUE(t.isStartLineConfigured());   // lines survive reset
  EXPECT_TRUE(t.isFinishLineConfigured());

  // And the course still times correctly after reset.
  PathDriver d2(t);
  d2.driveTo(COURSE_END_X, 0);
  EXPECT_EQ(t.getRuns(), 1);
  EXPECT_NEAR(t.getLastRunTime(), 16000UL, 100.0);
}

void test_degenerate_lines_rejected() {
  SprintTimer t(7.0);
  t.setStartLine(0.0, 0.0, 0.0, 0.0);
  t.setFinishLine(latAt(0), lngAt(100), latAt(0), lngAt(100));  // A == B
  EXPECT_FALSE(t.isStartLineConfigured());
  EXPECT_FALSE(t.isFinishLineConfigured());

  // Feeding fixes with nothing configured must be safe and do nothing.
  PathDriver d(t);
  d.driveTo(COURSE_END_X, 0);
  EXPECT_EQ(t.getRuns(), 0);
  EXPECT_FALSE(t.getRaceStarted());
}

void test_duck_typed_surface_matches_run_surface() {
  SprintTimer t(7.0);
  configureFullCourse(t);
  PathDriver d(t);
  d.lapCourse();
  d.lapCourse();

  EXPECT_EQ(t.getLaps(), t.getRuns());
  EXPECT_EQ(t.getLastLapTime(), t.getLastRunTime());
  EXPECT_EQ(t.getBestLapTime(), t.getBestRunTime());
  EXPECT_EQ(t.getBestLapNumber(), t.getBestRunNumber());
  EXPECT_EQ(t.getDirection(), DIR_UNKNOWN);
  EXPECT_FALSE(t.isDirectionResolved());
  // Duck-typed equivalents of the circuit timer's remaining getters.
  EXPECT_TRUE(t.isStartFinishLineConfigured());  // both lines set above
  EXPECT_TRUE(t.getCurrentLapStartTime() > 0);   // last run's start time
  EXPECT_TRUE(t.getCurrentLapOdometerStart() > 0);
}

void test_same_fix_split_and_finish_dispatch_in_crossing_order() {
  // Overlapping zones near the finish (explicitly supported): with a 7 m
  // threshold, a line's in-crossing window spans ~±8 m along the course, so
  // S2 at x=437 and the finish at x=444 keep BOTH engines in-crossing at
  // x=440..445. One 20 m fix from 445 to 465 then exits both zones on the
  // SAME loop() pass. The split's interpolated crossing (437 m) precedes
  // the finish's (444 m) and must be dispatched first — the old fixed
  // finish-first order closed the run before its final split landed,
  // wrongly invalidating the run's segment data.
  SprintTimer t(7.0);
  setLine(t, &SprintTimer::setStartLine, START_X);
  setLine(t, &SprintTimer::setFinishLine, 444.0);
  setLine(t, &SprintTimer::setSector2Line, 437.0);
  PathDriver d(t);

  d.driveTo(445, 0);        // 5 m steps, ends with both engines in-crossing
  EXPECT_TRUE(t.isRunActive());
  EXPECT_TRUE(t.getCrossing());
  d.x = 465;
  d.feed();                 // single 20 m fix: exits both zones together

  EXPECT_EQ(t.getRuns(), 1);
  EXPECT_FALSE(t.isRunActive());
  // Run start(50) -> finish(444): 394 m at 25 m/s = 15760 ms.
  EXPECT_NEAR(t.getLastRunTime(), 15760UL, 100.0);
  // Segment data survived: start->S2 = 387 m = 15480 ms, S2->finish = 7 m
  // = 280 ms. Pre-fix both best segments were 0 (order flagged invalid).
  EXPECT_NEAR(t.getBestSector1Time(), 15480UL, 100.0);
  EXPECT_NEAR(t.getBestSector2Time(), 280UL, 100.0);
  EXPECT_NEAR(t.getOptimalLapTime(), t.getBestRunTime(), 5.0);
}

void test_loop_return_matches_circuit_contract() {
  // DovesLapTimer::loop() returns 0 only while INSIDE a crossing zone; the
  // zone-exit fix (the one whose crossing completes) returns -1. SprintTimer
  // must match — displays keying "freeze while crossing" off loop()==0 see
  // identical behavior on either timer. Zone reality with a 7 m threshold:
  // enter at perpendicular distance < 7 m, exit at > 8 m, so with 5 m steps
  // the in-zone fixes for the start line at x=50 are 45/50/55 and x=60 is
  // the exit fix.
  SprintTimer t(7.0);
  setLine(t, &SprintTimer::setStartLine, START_X);
  setLine(t, &SprintTimer::setFinishLine, FINISH_X);

  unsigned long simTime = 0;
  for (double x = 5; x <= 120; x += 5) {
    t.updateCurrentTime(simTime);
    int ret = t.loop(latAt(0), lngAt(x), SIM_ALT_METERS, SIM_SPEED_KNOTS);
    simTime += MS_PER_STEP;

    if (x >= 45 && x <= 55) {
      EXPECT_EQ(ret, 0);   // inside the start-line zone
    } else if (x == 60) {
      // Zone-exit fix: the crossing completes here, and the circuit timer
      // returns -1 on this fix (0 is IN_ZONE only) — SprintTimer must too.
      EXPECT_EQ(ret, -1);
      EXPECT_TRUE(t.isRunActive());  // the completion still dispatched
    } else {
      EXPECT_EQ(ret, -1);  // clear of the zone
    }
  }
}

int main() {
  printf("=== SprintTimer point-to-point tests ===\n");

  RUN_TEST(initial_state);
  RUN_TEST(single_run_time_exact);
  RUN_TEST(run_active_between_start_and_finish);
  RUN_TEST(three_runs_deterministic);
  RUN_TEST(segment_times_exact);
  RUN_TEST(start_recross_cancels_and_restarts);
  RUN_TEST(finish_crossing_ignored_while_waiting);
  RUN_TEST(backward_start_crossing_opens_run);
  RUN_TEST(single_split_is_legal);
  RUN_TEST(no_splits_still_times_runs);
  RUN_TEST(missed_split_invalidates_sector_data_not_run);
  RUN_TEST(best_run_tracking_across_unequal_runs);
  RUN_TEST(reset_clears_state_keeps_lines);
  RUN_TEST(degenerate_lines_rejected);
  RUN_TEST(duck_typed_surface_matches_run_surface);
  RUN_TEST(same_fix_split_and_finish_dispatch_in_crossing_order);
  RUN_TEST(loop_return_matches_circuit_contract);

  TEST_SUMMARY();
}
