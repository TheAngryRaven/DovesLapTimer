/**
 * Layer-2 tests for crossing-buffer wraparound (CLAUDE.md review issue H2).
 *
 * The crossing buffer is a 100-entry ring. A kart parked inside the
 * crossing zone (standing start on the grid, red flag, stop-and-go) wraps
 * it in 4-5.5s at 18-25Hz. Before the fix, the interpolator scanned in
 * physical index order, so the seam pair (newest entry adjacent to oldest)
 * masqueraded as the line crossing — either fabricating a crossing from
 * points minutes apart or invalidating the real one.
 *
 * Geometry: S/F line spans lat 28.41255..28.41275 at lng -81.37920.
 * The "driver" moves only in longitude; west of the line is one side,
 * east is the other. ~3m of longitude at this latitude = 3.064e-5 deg.
 */

#include "test_runner.h"
#include "../src/DovesLapTimer.h"

static constexpr double SF_A_LAT = 28.41255, SF_A_LNG = -81.37920;
static constexpr double SF_B_LAT = 28.41275, SF_B_LNG = -81.37920;
static constexpr double DRIVER_LAT = 28.41265;            // line midpoint
static constexpr double LNG_PER_METER = 1.0 / 97924.0;    // at this latitude
static constexpr unsigned long MS_PER_FIX = 200;
static constexpr float ALT = 50.0f;

static double lngAtMetersEast(double meters) {
  return SF_A_LNG + meters * LNG_PER_METER;
}

static void feedFix(DovesLapTimer &t, double metersEastOfLine,
                    float speedKnots, unsigned long &timeMs) {
  t.updateCurrentTime(timeMs);
  t.loop(DRIVER_LAT, lngAtMetersEast(metersEastOfLine), ALT, speedKnots);
  timeMs += MS_PER_FIX;
}

// =============================================================================
// Park on one side of the line, creep across, park on the other side,
// then drive out. >120 in-zone fixes wraps the 100-entry ring while the
// genuine crossing pair stays inside the retained window. Pre-fix, the
// physical-order scan latched onto the seam pair (3m west entry from
// minutes earlier next to an 8m east entry) whose distance sum failed
// validation -> the crossing was thrown away entirely.
// =============================================================================

void test_park_cross_park_still_detects_crossing() {
  DovesLapTimer timer(7.0);
  timer.setStartFinishLine(SF_A_LAT, SF_A_LNG, SF_B_LAT, SF_B_LNG);

  unsigned long timeMs = 100000;

  // 60 fixes parked 3m west of the line (in zone, side A)
  for (int i = 0; i < 60; i++) feedFix(timer, -3.0, 0.0f, timeMs);
  unsigned long lastWestTime = timeMs - MS_PER_FIX;

  // creep across to 3m east (side B), park there for 60 fixes
  for (int i = 0; i < 60; i++) feedFix(timer, 3.0, 0.0f, timeMs);

  // drive out east: 8m (still in zone), 13m (exits, triggers interpolation)
  feedFix(timer, 8.0, 10.0f, timeMs);
  feedFix(timer, 13.0, 10.0f, timeMs);

  EXPECT_TRUE(timer.getRaceStarted());
  // Genuine crossing: midpoint of the last-west / first-east fix pair
  // (equal 3m distances, both stationary -> t = 0.5)
  unsigned long expected = lastWestTime + MS_PER_FIX / 2;
  EXPECT_NEAR(timer.getCurrentLapStartTime(), expected, 250.0);
}

// =============================================================================
// Standing start: parked on the grid just shy of the line long enough to
// wrap the ring, then launch through. The crossing pair is in the newest
// fixes; the unwound chronological scan must find it and date the race
// start at the launch, not somewhere in the parked history.
// =============================================================================

void test_standing_start_grid_crossing_time_correct() {
  DovesLapTimer timer(7.0);
  timer.setStartFinishLine(SF_A_LAT, SF_A_LNG, SF_B_LAT, SF_B_LNG);

  unsigned long timeMs = 100000;

  // 150 fixes parked 3m west (wraps the 100-entry buffer)
  for (int i = 0; i < 150; i++) feedFix(timer, -3.0, 0.0f, timeMs);

  // launch: 2m per fix eastward across the line and out of the zone
  unsigned long beforeLaunch = timeMs;
  feedFix(timer, -1.0, 20.0f, timeMs);
  feedFix(timer,  1.0, 20.0f, timeMs);   // crossing happens here
  feedFix(timer,  3.0, 20.0f, timeMs);
  feedFix(timer,  5.0, 20.0f, timeMs);
  feedFix(timer,  7.0, 20.0f, timeMs);
  feedFix(timer,  9.0, 20.0f, timeMs);   // exits zone

  EXPECT_TRUE(timer.getRaceStarted());
  // Crossing between the -1m and +1m fixes -> t = 0.5 between them
  unsigned long expected = beforeLaunch + MS_PER_FIX / 2;
  EXPECT_NEAR(timer.getCurrentLapStartTime(), expected, 250.0);
}

// =============================================================================
// After a wrapped standing start, a normal flying lap must still time
// correctly — the buffer reset on zone exit must leave no stale state.
// =============================================================================

void test_normal_lap_after_wrapped_standing_start() {
  DovesLapTimer timer(7.0);
  timer.setStartFinishLine(SF_A_LAT, SF_A_LNG, SF_B_LAT, SF_B_LNG);

  unsigned long timeMs = 100000;

  // Standing start as above
  for (int i = 0; i < 150; i++) feedFix(timer, -3.0, 0.0f, timeMs);
  for (double m = -1.0; m <= 9.0; m += 2.0) feedFix(timer, m, 20.0f, timeMs);
  EXPECT_TRUE(timer.getRaceStarted());
  unsigned long raceStart = timer.getCurrentLapStartTime();

  // "Lap": drive far east, come back through the line westbound at speed.
  // 5m per fix; out to 100m, back through to -15m.
  for (double m = 14.0; m <= 100.0; m += 5.0) feedFix(timer, m, 20.0f, timeMs);
  for (double m = 100.0; m >= -15.0; m -= 5.0) feedFix(timer, m, 20.0f, timeMs);

  EXPECT_EQ(timer.getLaps(), 1);
  // Lap time = (westbound crossing) - (race start); both interpolated.
  // Sanity bound: total sim time elapsed is well under 25s.
  unsigned long lapTime = timer.getLastLapTime();
  EXPECT_TRUE(lapTime > 0);
  EXPECT_TRUE(lapTime < (timeMs - raceStart));
}

int main() {
  printf("=== Crossing-buffer wraparound tests ===\n");

  RUN_TEST(park_cross_park_still_detects_crossing);
  RUN_TEST(standing_start_grid_crossing_time_correct);
  RUN_TEST(normal_lap_after_wrapped_standing_start);

  TEST_SUMMARY();
}
