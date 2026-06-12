/**
 * Layer-2 tests for WaypointLapTimer ("Lap Anything" mode) — review issue H11.
 *
 * Drives the proximity state machine end to end: waypoint drop at speed,
 * closest-approach lap splits on a deterministic 400m circle, lap
 * accounting (last/best/distance), and reset semantics. Until now this
 * 300-line module had zero direct tests.
 */

#include "test_runner.h"
#include "../src/WaypointLapTimer.h"

static constexpr double CENTER_LAT = 28.41265;
static constexpr double CENTER_LNG = -81.37920;
static constexpr double RADIUS_M   = 63.66;       // 2*pi*r ~= 400m lap
static constexpr unsigned int  STEPS_PER_LAP = 80; // 5m per step
static constexpr unsigned long MS_PER_STEP   = 200;
static constexpr float SPEED_FAST_KNOTS = 48.0f;   // ~55 mph, above 20mph gate
static constexpr float SPEED_SLOW_KNOTS = 5.0f;    // ~5.8 mph, below gate
static constexpr float ALT = 50.0f;

// Position on the circle at a given step; step 0 = easternmost point.
static void circleFix(unsigned long step, double &lat, double &lng) {
  const double mPerDegLat = 111320.0;
  const double mPerDegLng = 111320.0 * cos(CENTER_LAT * M_PI / 180.0);
  double theta = (2.0 * M_PI * (step % STEPS_PER_LAP)) / STEPS_PER_LAP;
  lat = CENTER_LAT + (RADIUS_M * sin(theta)) / mPerDegLat;
  lng = CENTER_LNG + (RADIUS_M * cos(theta)) / mPerDegLng;
}

static void driveCircle(WaypointLapTimer &wlt, unsigned long steps,
                        float speedKnots, unsigned long startTimeMs = 1000) {
  for (unsigned long step = 0; step < steps; step++) {
    double lat, lng;
    circleFix(step, lat, lng);
    wlt.updateCurrentTime(startTimeMs + step * MS_PER_STEP);
    wlt.loop(lat, lng, ALT, speedKnots);
  }
}

// =============================================================================
// Waypoint drop
// =============================================================================

void test_initial_state() {
  WaypointLapTimer wlt;
  EXPECT_FALSE(wlt.hasWaypoint());
  EXPECT_FALSE(wlt.getRaceStarted());
  EXPECT_EQ(wlt.getLaps(), 0);
  EXPECT_EQ(wlt.getCurrentLapTime(), 0UL);
}

void test_no_waypoint_below_speed_threshold() {
  WaypointLapTimer wlt;
  driveCircle(wlt, 40, SPEED_SLOW_KNOTS);
  EXPECT_FALSE(wlt.hasWaypoint());
  EXPECT_FALSE(wlt.getRaceStarted());
}

void test_waypoint_drops_at_speed() {
  WaypointLapTimer wlt;
  double lat0, lng0;
  circleFix(0, lat0, lng0);

  wlt.updateCurrentTime(1000);
  wlt.loop(lat0, lng0, ALT, SPEED_FAST_KNOTS);

  EXPECT_TRUE(wlt.hasWaypoint());
  EXPECT_NEAR(wlt.getWaypointLat(), lat0, 1e-9);
  EXPECT_NEAR(wlt.getWaypointLng(), lng0, 1e-9);
}

// =============================================================================
// Lap splits via closest approach
// =============================================================================

void test_laps_counted_on_circle() {
  WaypointLapTimer wlt;
  driveCircle(wlt, 3 * STEPS_PER_LAP + 20, SPEED_FAST_KNOTS);

  EXPECT_TRUE(wlt.getRaceStarted());
  EXPECT_TRUE(wlt.getLaps() >= 3);
}

void test_lap_time_matches_circle_period() {
  // Every lap passes exactly through the waypoint; the closest-approach
  // split lands on the same circle position each time, so lap time must
  // equal the lap period (80 steps x 200ms).
  WaypointLapTimer wlt;
  driveCircle(wlt, 3 * STEPS_PER_LAP + 20, SPEED_FAST_KNOTS);

  EXPECT_NEAR(wlt.getLastLapTime(), STEPS_PER_LAP * MS_PER_STEP, 300.0);
  EXPECT_NEAR(wlt.getBestLapTime(), STEPS_PER_LAP * MS_PER_STEP, 300.0);
}

void test_lap_distance_matches_circumference() {
  WaypointLapTimer wlt;
  driveCircle(wlt, 3 * STEPS_PER_LAP + 20, SPEED_FAST_KNOTS);

  // 80 chords of a 400m circle ~= 399.7m
  EXPECT_NEAR(wlt.getLastLapDistance(), 400.0, 20.0);
  EXPECT_NEAR(wlt.getBestLapDistance(), 400.0, 20.0);
}

void test_best_lap_number_tracked() {
  WaypointLapTimer wlt;
  driveCircle(wlt, 3 * STEPS_PER_LAP + 20, SPEED_FAST_KNOTS);

  int best = wlt.getBestLapNumber();
  EXPECT_TRUE(best >= 1);
  EXPECT_TRUE(best <= wlt.getLaps());
}

void test_crossing_flag_only_inside_proximity() {
  // Mid-lap (far side of the circle) the crossing flag must be false.
  WaypointLapTimer wlt;
  driveCircle(wlt, STEPS_PER_LAP / 2, SPEED_FAST_KNOTS);
  EXPECT_FALSE(wlt.getCrossing());
}

// =============================================================================
// Reset / configuration
// =============================================================================

void test_reset_clears_timing_state() {
  WaypointLapTimer wlt;
  driveCircle(wlt, 2 * STEPS_PER_LAP + 20, SPEED_FAST_KNOTS);
  EXPECT_TRUE(wlt.getLaps() >= 1);

  wlt.reset();
  EXPECT_FALSE(wlt.hasWaypoint());
  EXPECT_FALSE(wlt.getRaceStarted());
  EXPECT_EQ(wlt.getLaps(), 0);
  EXPECT_NEAR(wlt.getTotalDistanceTraveled(), 0.0, 0.01);
}

void test_custom_speed_threshold_survives_reset() {
  WaypointLapTimer wlt;
  wlt.setSpeedThresholdMph(100.0f);  // above our 55mph sim speed

  driveCircle(wlt, 10, SPEED_FAST_KNOTS);
  EXPECT_FALSE(wlt.hasWaypoint());  // custom threshold respected

  wlt.reset();
  driveCircle(wlt, 10, SPEED_FAST_KNOTS);
  EXPECT_FALSE(wlt.hasWaypoint());  // threshold survived reset (documented)
}

int main() {
  printf("=== WaypointLapTimer tests ===\n");

  RUN_TEST(initial_state);
  RUN_TEST(no_waypoint_below_speed_threshold);
  RUN_TEST(waypoint_drops_at_speed);

  RUN_TEST(laps_counted_on_circle);
  RUN_TEST(lap_time_matches_circle_period);
  RUN_TEST(lap_distance_matches_circumference);
  RUN_TEST(best_lap_number_tracked);
  RUN_TEST(crossing_flag_only_inside_proximity);

  RUN_TEST(reset_clears_timing_state);
  RUN_TEST(custom_speed_threshold_survives_reset);

  TEST_SUMMARY();
}
