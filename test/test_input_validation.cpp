/**
 * Layer-2 adversarial-input tests (CLAUDE.md review issues C1 + H1).
 *
 * C1: loop() before setStartFinishLine() must be a safe no-op (previously
 *     it fed uninitialized doubles into haversine — UB and phantom
 *     crossings), and degenerate placeholder lines (the 0.00 defaults in
 *     basic_oled_example) must not count as configured.
 *
 * H1: a single NaN / Inf / (0,0) / out-of-range fix must not poison the
 *     odometer or speed state, and single-fix teleports must be rejected
 *     while sustained relocations re-seed without crediting the gap.
 */

#include "test_runner.h"
#include <limits>
#include "../src/DovesLapTimer.h"
#include "../src/WaypointLapTimer.h"
#include "../src/CourseDetector.h"

static constexpr double BASE_LAT = 28.41265;
static constexpr double BASE_LNG = -81.37920;
static constexpr float  ALT      = 50.0f;
static constexpr float  SPEED    = 20.0f;  // knots

static const double NAN_D = std::numeric_limits<double>::quiet_NaN();
static const double INF_D = std::numeric_limits<double>::infinity();

// ~5m east per step at this latitude
static constexpr double LNG_STEP_5M = 5.0 / 97924.0;

// Feed `count` valid eastbound fixes starting at index `startStep`.
static void feedValidFixes(DovesLapTimer &t, int startStep, int count,
                           unsigned long &timeMs) {
  for (int i = 0; i < count; i++) {
    t.updateCurrentTime(timeMs);
    t.loop(BASE_LAT, BASE_LNG + (startStep + i) * LNG_STEP_5M, ALT, SPEED);
    timeMs += 200;
  }
}

// =============================================================================
// C1 — unconfigured / degenerate start/finish line
// =============================================================================

void test_loop_without_startfinish_line_is_safe_noop() {
  DovesLapTimer timer(7.0);
  EXPECT_FALSE(timer.isStartFinishLineConfigured());

  unsigned long timeMs = 1000;
  feedValidFixes(timer, 0, 50, timeMs);

  // No line configured -> no phantom crossings, no race start
  EXPECT_FALSE(timer.getRaceStarted());
  EXPECT_EQ(timer.getLaps(), 0);
  // Odometer still works: 49 steps x ~5m
  EXPECT_NEAR(timer.getTotalDistanceTraveled(), 245.0, 5.0);
}

void test_degenerate_placeholder_line_not_configured() {
  // Mirrors basic_oled_example's shipped 0.00 placeholders
  DovesLapTimer timer(7.0);
  timer.setStartFinishLine(0.00, 0.00, 0.00, 0.00);
  EXPECT_FALSE(timer.isStartFinishLineConfigured());

  timer.setStartFinishLine(28.41255, -81.37920, 28.41275, -81.37920);
  EXPECT_TRUE(timer.isStartFinishLineConfigured());
}

void test_degenerate_sector_lines_not_configured() {
  DovesLapTimer timer(7.0);
  timer.setSector2Line(1.0, 2.0, 1.0, 2.0);     // zero-length
  timer.setSector3Line(NAN_D, 2.0, 3.0, 4.0);   // non-finite
  EXPECT_FALSE(timer.areSectorLinesConfigured());
}

// =============================================================================
// H1 — NaN / Inf / (0,0) / out-of-range fixes
// =============================================================================

void test_nan_fix_does_not_poison_odometer() {
  DovesLapTimer timer(7.0);
  unsigned long timeMs = 1000;
  feedValidFixes(timer, 0, 10, timeMs);
  float before = timer.getTotalDistanceTraveled();
  EXPECT_TRUE(before == before);  // not NaN

  timer.updateCurrentTime(timeMs);
  timer.loop(NAN_D, BASE_LNG, ALT, SPEED);
  timer.loop(BASE_LAT, NAN_D, ALT, SPEED);
  timeMs += 200;

  // Rejected fixes add nothing; continuing from the same area still works
  feedValidFixes(timer, 10, 10, timeMs);
  float after = timer.getTotalDistanceTraveled();
  EXPECT_TRUE(after == after);  // still not NaN
  EXPECT_NEAR(after - before, 50.0, 5.0);  // 10 more 5m steps only
}

void test_null_island_fix_rejected() {
  DovesLapTimer timer(7.0);
  unsigned long timeMs = 1000;
  feedValidFixes(timer, 0, 10, timeMs);
  float before = timer.getTotalDistanceTraveled();

  // A (0,0) glitch fix would add ~9000km to the odometer if accepted
  timer.updateCurrentTime(timeMs);
  timer.loop(0.0, 0.0, ALT, SPEED);

  EXPECT_NEAR(timer.getTotalDistanceTraveled(), before, 0.1);
}

void test_out_of_range_and_inf_fixes_rejected() {
  DovesLapTimer timer(7.0);
  unsigned long timeMs = 1000;
  feedValidFixes(timer, 0, 5, timeMs);
  float before = timer.getTotalDistanceTraveled();

  timer.loop(95.0, BASE_LNG, ALT, SPEED);      // |lat| > 90
  timer.loop(BASE_LAT, 200.0, ALT, SPEED);     // |lng| > 180
  timer.loop(INF_D, BASE_LNG, ALT, SPEED);
  timer.loop(BASE_LAT, -INF_D, ALT, SPEED);

  EXPECT_NEAR(timer.getTotalDistanceTraveled(), before, 0.1);
}

void test_nan_speed_and_altitude_sanitized() {
  DovesLapTimer timer(7.0);
  unsigned long timeMs = 1000;
  feedValidFixes(timer, 0, 5, timeMs);

  timer.updateCurrentTime(timeMs);
  timer.loop(BASE_LAT, BASE_LNG + 5 * LNG_STEP_5M, (float)NAN_D, (float)NAN_D);

  float speed = timer.getCurrentSpeedKmh();
  EXPECT_TRUE(speed == speed);   // not NaN
  EXPECT_NEAR(speed, 0.0, 0.001);
  float dist = timer.getTotalDistanceTraveled();
  EXPECT_TRUE(dist == dist);     // NaN altitude must not reach haversine3D
}

// =============================================================================
// H1 — teleport rejection / sustained-relocation re-seed
// =============================================================================

void test_single_teleport_rejected() {
  DovesLapTimer timer(7.0);
  unsigned long timeMs = 1000;
  feedValidFixes(timer, 0, 10, timeMs);
  float before = timer.getTotalDistanceTraveled();

  // One fix 5km away (GPS multipath teleport), then back on course
  timer.updateCurrentTime(timeMs);
  timer.loop(BASE_LAT + 0.05, BASE_LNG, ALT, SPEED);
  timeMs += 200;
  feedValidFixes(timer, 10, 5, timeMs);

  // The ~11km round trip must not be credited — only the 5 real steps
  EXPECT_NEAR(timer.getTotalDistanceTraveled() - before, 25.0, 5.0);
}

void test_sustained_relocation_reseeds_without_credit() {
  DovesLapTimer timer(7.0);
  unsigned long timeMs = 1000;
  feedValidFixes(timer, 0, 10, timeMs);
  float before = timer.getTotalDistanceTraveled();

  const double farLat = BASE_LAT + 0.05;  // ~5.5km away

  // GPS_JUMP_REACCEPT_COUNT consecutive far fixes: first two dropped,
  // third re-seeds the position without crediting the 5.5km gap
  for (int i = 0; i < GPS_JUMP_REACCEPT_COUNT; i++) {
    timer.updateCurrentTime(timeMs);
    timer.loop(farLat, BASE_LNG + i * LNG_STEP_5M, ALT, SPEED);
    timeMs += 200;
  }
  EXPECT_NEAR(timer.getTotalDistanceTraveled(), before, 0.1);

  // Movement at the new location accumulates normally again
  for (int i = GPS_JUMP_REACCEPT_COUNT; i < GPS_JUMP_REACCEPT_COUNT + 4; i++) {
    timer.updateCurrentTime(timeMs);
    timer.loop(farLat, BASE_LNG + i * LNG_STEP_5M, ALT, SPEED);
    timeMs += 200;
  }
  EXPECT_NEAR(timer.getTotalDistanceTraveled() - before, 20.0, 5.0);
}

// =============================================================================
// H1 — CourseDetector and WaypointLapTimer guards
// =============================================================================

void test_course_detector_ignores_invalid_input() {
  CourseInfo courses[] = {{"Course A", 1000.0f}};
  CourseDetector det;
  det.init(courses, 1);

  // NaN coordinates at speed must NOT capture a NaN waypoint (which would
  // make every later proximity comparison false and hang detection)
  det.update(NAN_D, BASE_LNG, 35.0f, 100.0f);
  det.update(BASE_LAT, NAN_D, 35.0f, 100.0f);
  det.update(0.0, 0.0, 35.0f, 100.0f);
  EXPECT_FALSE(det.hasWaypoint());

  // A valid fix afterwards sets a real waypoint
  det.update(BASE_LAT, BASE_LNG, 35.0f, 100.0f);
  EXPECT_TRUE(det.hasWaypoint());
  EXPECT_NEAR(det.getWaypointLat(), BASE_LAT, 1e-9);
}

void test_waypoint_lap_timer_ignores_invalid_input() {
  WaypointLapTimer wlt;
  wlt.updateCurrentTime(1000);

  // Invalid fixes at speed: no waypoint, no odometer poisoning
  wlt.loop(NAN_D, BASE_LNG, ALT, 48.0f);
  wlt.loop(0.0, 0.0, ALT, 48.0f);
  EXPECT_FALSE(wlt.hasWaypoint());
  float dist = wlt.getTotalDistanceTraveled();
  EXPECT_TRUE(dist == dist);
  EXPECT_NEAR(dist, 0.0, 0.1);

  // Valid fast fix drops a real waypoint
  wlt.loop(BASE_LAT, BASE_LNG, ALT, 48.0f);
  EXPECT_TRUE(wlt.hasWaypoint());
  EXPECT_NEAR(wlt.getWaypointLat(), BASE_LAT, 1e-9);
}

int main() {
  printf("=== Input validation tests ===\n");

  RUN_TEST(loop_without_startfinish_line_is_safe_noop);
  RUN_TEST(degenerate_placeholder_line_not_configured);
  RUN_TEST(degenerate_sector_lines_not_configured);

  RUN_TEST(nan_fix_does_not_poison_odometer);
  RUN_TEST(null_island_fix_rejected);
  RUN_TEST(out_of_range_and_inf_fixes_rejected);
  RUN_TEST(nan_speed_and_altitude_sanitized);

  RUN_TEST(single_teleport_rejected);
  RUN_TEST(sustained_relocation_reseeds_without_credit);

  RUN_TEST(course_detector_ignores_invalid_input);
  RUN_TEST(waypoint_lap_timer_ignores_invalid_input);

  TEST_SUMMARY();
}
