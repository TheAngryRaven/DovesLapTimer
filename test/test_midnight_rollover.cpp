/**
 * Layer-2 tests for UTC midnight rollover (CLAUDE.md review issue C2).
 *
 * The GPS time base is milliseconds-since-midnight, which wraps
 * 86,399,999 -> 0 at UTC midnight — mid-evening across the Americas.
 * Before the fix, every duration was a naked unsigned subtraction: a lap
 * straddling midnight produced a ~4.29-billion-ms "lap time" that then
 * stuck as bestLapTime, and a crossing zone straddling midnight pushed an
 * out-of-range double through a double->unsigned conversion (UB).
 *
 * Reuses the synthetic 100m x 100m square track from test_synthetic_track
 * with a configurable simulation start time.
 */

#include "test_runner.h"
#include "../src/DovesLapTimer.h"
#include "../src/WaypointLapTimer.h"

static constexpr double TRACK_SOUTH_LAT = 28.41265;
static constexpr double TRACK_NORTH_LAT = 28.41355;
static constexpr double TRACK_WEST_LNG  = -81.37970;
static constexpr double TRACK_EAST_LNG  = -81.37870;
static constexpr unsigned int  STEPS_PER_LAP   = 80;
static constexpr unsigned long SIM_MS_PER_STEP = 200;
static constexpr float         SIM_SPEED_KNOTS = 48.0f;
static constexpr float         SIM_ALT_METERS  = 50.0f;

static constexpr double SF_A_LAT = 28.41255, SF_A_LNG = -81.37920;
static constexpr double SF_B_LAT = 28.41275, SF_B_LNG = -81.37920;

static void generateSyntheticFix(unsigned long step, double &lat, double &lng) {
  const unsigned int sideSteps = STEPS_PER_LAP / 4;
  const unsigned int phase = step % STEPS_PER_LAP;
  const double t = (phase % sideSteps) / (double)sideSteps;

  if (phase < sideSteps) {
    lat = TRACK_SOUTH_LAT;
    lng = TRACK_WEST_LNG + t * (TRACK_EAST_LNG - TRACK_WEST_LNG);
  } else if (phase < 2 * sideSteps) {
    lat = TRACK_SOUTH_LAT + t * (TRACK_NORTH_LAT - TRACK_SOUTH_LAT);
    lng = TRACK_EAST_LNG;
  } else if (phase < 3 * sideSteps) {
    lat = TRACK_NORTH_LAT;
    lng = TRACK_EAST_LNG + t * (TRACK_WEST_LNG - TRACK_EAST_LNG);
  } else {
    lat = TRACK_NORTH_LAT + t * (TRACK_SOUTH_LAT - TRACK_NORTH_LAT);
    lng = TRACK_WEST_LNG;
  }
}

// Drive targetLaps laps starting the simulated GPS clock at startTimeMs.
// The clock is wrapped at DOVES_MILLIS_PER_DAY exactly like a real GPS
// time-of-day field. Completed lap times land in lapTimes[].
static void driveLapsFrom(DovesLapTimer &timer, unsigned long startTimeMs,
                          int targetLaps, unsigned long *lapTimes) {
  int lastLapCount = 0;
  int recIdx = 0;
  const unsigned long maxSteps = STEPS_PER_LAP * (unsigned long)(targetLaps + 2) + 50;

  for (unsigned long step = 0; step < maxSteps; step++) {
    double lat, lng;
    generateSyntheticFix(step, lat, lng);
    timer.updateCurrentTime((startTimeMs + step * SIM_MS_PER_STEP) % DOVES_MILLIS_PER_DAY);
    timer.loop(lat, lng, SIM_ALT_METERS, SIM_SPEED_KNOTS);

    int laps = timer.getLaps();
    if (laps > lastLapCount && recIdx < targetLaps) {
      lapTimes[recIdx++] = timer.getLastLapTime();
      lastLapCount = laps;
    }
    if (laps >= targetLaps) break;
  }
}

// =============================================================================
// timeSinceMidnightDelta helper
// =============================================================================

void test_delta_normal_case() {
  EXPECT_EQ(timeSinceMidnightDelta(1000UL, 5000UL), 4000UL);
  EXPECT_EQ(timeSinceMidnightDelta(0UL, 0UL), 0UL);
}

void test_delta_across_midnight() {
  // 86,399,000 -> 1,000 is 2 seconds across the wrap
  EXPECT_EQ(timeSinceMidnightDelta(86399000UL, 1000UL), 2000UL);
  // one ms before midnight -> exactly midnight
  EXPECT_EQ(timeSinceMidnightDelta(86399999UL, 0UL), 1UL);
}

// =============================================================================
// Lap straddling midnight
// =============================================================================

void test_lap_straddling_midnight_has_correct_time() {
  // Race-start crossing happens ~2000ms into the sim; start the clock so
  // midnight falls ~8000ms into lap 1 (mid-lap, away from any crossing).
  DovesLapTimer timer(7.0);
  timer.setStartFinishLine(SF_A_LAT, SF_A_LNG, SF_B_LAT, SF_B_LNG);

  unsigned long lapTimes[2] = {0, 0};
  driveLapsFrom(timer, DOVES_MILLIS_PER_DAY - 10000UL, 2, lapTimes);

  EXPECT_EQ(timer.getLaps(), 2);
  EXPECT_NEAR(lapTimes[0], 16000UL, 100.0);  // straddles midnight
  EXPECT_NEAR(lapTimes[1], 16000UL, 100.0);
  // The bogus pre-fix behavior left a ~4.29e9 ms best lap behind
  EXPECT_TRUE(timer.getBestLapTime() < 20000UL);
}

void test_crossing_zone_straddling_midnight() {
  // The lap-1-completing S/F crossing happens ~18000ms into the sim
  // (step 90). Start the clock so midnight falls between the two fixes
  // that straddle the line — the interpolator's delta is then negative
  // before normalization, which pre-fix went through unsigned underflow
  // and an out-of-range double->unsigned conversion.
  DovesLapTimer timer(7.0);
  timer.setStartFinishLine(SF_A_LAT, SF_A_LNG, SF_B_LAT, SF_B_LNG);

  unsigned long lapTimes[2] = {0, 0};
  driveLapsFrom(timer, DOVES_MILLIS_PER_DAY - 17900UL, 2, lapTimes);

  EXPECT_EQ(timer.getLaps(), 2);
  EXPECT_NEAR(lapTimes[0], 16000UL, 100.0);
  EXPECT_NEAR(lapTimes[1], 16000UL, 100.0);
  // The crossing time must have wrapped back into [0, day) range
  EXPECT_TRUE(timer.getCurrentLapStartTime() < DOVES_MILLIS_PER_DAY);
}

void test_current_lap_time_across_midnight() {
  DovesLapTimer timer(7.0);
  timer.setStartFinishLine(SF_A_LAT, SF_A_LNG, SF_B_LAT, SF_B_LNG);

  // Start the lap just before midnight, then poll getCurrentLapTime after
  // the wrap without completing the lap.
  unsigned long start = DOVES_MILLIS_PER_DAY - 5000UL;
  for (unsigned long step = 0; step < 40; step++) {  // half a lap
    double lat, lng;
    generateSyntheticFix(step, lat, lng);
    timer.updateCurrentTime((start + step * SIM_MS_PER_STEP) % DOVES_MILLIS_PER_DAY);
    timer.loop(lat, lng, SIM_ALT_METERS, SIM_SPEED_KNOTS);
  }

  EXPECT_TRUE(timer.getRaceStarted());
  // ~40 steps in, race started at ~step 10 -> current lap ~6000ms.
  // Pre-fix this returned ~4.29e9 once millisecondsSinceMidnight wrapped.
  unsigned long current = timer.getCurrentLapTime();
  EXPECT_TRUE(current < 10000UL);
  EXPECT_TRUE(current > 3000UL);
}

void test_sector_times_sane_across_midnight() {
  static constexpr double S2_A_LAT = 28.41310, S2_A_LNG = -81.37860;
  static constexpr double S2_B_LAT = 28.41310, S2_B_LNG = -81.37880;
  static constexpr double S3_A_LAT = 28.41345, S3_A_LNG = -81.37925;
  static constexpr double S3_B_LAT = 28.41365, S3_B_LNG = -81.37925;

  DovesLapTimer timer(7.0);
  timer.setStartFinishLine(SF_A_LAT, SF_A_LNG, SF_B_LAT, SF_B_LNG);
  timer.setSector2Line(S2_A_LAT, S2_A_LNG, S2_B_LAT, S2_B_LNG);
  timer.setSector3Line(S3_A_LAT, S3_A_LNG, S3_B_LAT, S3_B_LNG);

  // Midnight lands ~22000ms in: inside lap 2's sector 1/2 region.
  unsigned long lapTimes[3] = {0, 0, 0};
  driveLapsFrom(timer, DOVES_MILLIS_PER_DAY - 22000UL, 3, lapTimes);

  EXPECT_EQ(timer.getLaps(), 3);
  for (int i = 0; i < 3; i++) {
    EXPECT_NEAR(lapTimes[i], 16000UL, 100.0);
  }
  // Sum of best sectors must still equal a sane lap, not include a
  // 4.29e9 ms underflow artifact.
  unsigned long optimal = timer.getOptimalLapTime();
  EXPECT_TRUE(optimal > 0);
  EXPECT_NEAR(optimal, 16000UL, 200.0);
}

// =============================================================================
// WaypointLapTimer across midnight
// =============================================================================

void test_waypoint_lap_timer_across_midnight() {
  // Drive a 400m circle (80 steps x 5m) through the waypoint each lap.
  // Midnight lands mid-lap-2.
  WaypointLapTimer wlt;
  const double cLat = 28.41265, cLng = -81.37920;
  const double rMeters = 63.66;  // 2*pi*r ~= 400m
  const double mPerDegLat = 111320.0;
  const double mPerDegLng = 111320.0 * cos(cLat * M_PI / 180.0);
  const unsigned long start = DOVES_MILLIS_PER_DAY - 20000UL;

  for (unsigned long step = 0; step < 180; step++) {
    double theta = (2.0 * M_PI * (step % STEPS_PER_LAP)) / STEPS_PER_LAP;
    double lat = cLat + (rMeters * sin(theta)) / mPerDegLat;
    double lng = cLng + (rMeters * cos(theta)) / mPerDegLng;
    wlt.updateCurrentTime((start + step * SIM_MS_PER_STEP) % DOVES_MILLIS_PER_DAY);
    wlt.loop(lat, lng, SIM_ALT_METERS, SIM_SPEED_KNOTS);
  }

  EXPECT_TRUE(wlt.getRaceStarted());
  EXPECT_TRUE(wlt.getLaps() >= 2);
  // Lap 2 straddles midnight — must still be ~16000ms, not ~4.29e9.
  EXPECT_NEAR(wlt.getLastLapTime(), 16000UL, 300.0);
}

int main() {
  printf("=== Midnight rollover tests ===\n");

  RUN_TEST(delta_normal_case);
  RUN_TEST(delta_across_midnight);

  RUN_TEST(lap_straddling_midnight_has_correct_time);
  RUN_TEST(crossing_zone_straddling_midnight);
  RUN_TEST(current_lap_time_across_midnight);
  RUN_TEST(sector_times_sane_across_midnight);

  RUN_TEST(waypoint_lap_timer_across_midnight);

  TEST_SUMMARY();
}
