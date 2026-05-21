/**
 * Layer-3 NMEA replay regression test.
 *
 * Fixture: examples/real_track_data_debug/gps_race_data_lap.h
 *   Orlando Kart Center, 4/29/23, one lap, white Tony rental.
 *
 * Golden times (from fixture header):
 *   MyLaps    : 1:08.807 ms (68807) — magnetic-loop ground truth
 *   DoveTimer : 1:08.748 ms (68748) — LINEAR interpolation
 *   DoveTimer : 1:08.745 ms (68745) — CATMULL-ROM interpolation
 *
 * The pinned 68748 / 68745 values are this library's previously verified
 * outputs — tight tolerance catches algorithm regressions. The 68807
 * MyLaps value is the actual on-track ground truth — looser tolerance
 * proves we're still close to reality.
 */

#include "test_runner.h"
#include "replay_runner.h"
#include "../examples/real_track_data_debug/gps_race_data_lap.h"

static const int num_gps_logs = sizeof(gps_logs) / sizeof(gps_logs[0]);

static constexpr double SF_A_LAT = 28.41270817056385;
static constexpr double SF_A_LNG = -81.37973266418031;
static constexpr double SF_B_LAT = 28.41273038679321;
static constexpr double SF_B_LNG = -81.37957048753776;

static ReplayConfig makeCfg(bool catmull) {
  ReplayConfig c;
  c.sfA_lat = SF_A_LAT; c.sfA_lng = SF_A_LNG;
  c.sfB_lat = SF_B_LAT; c.sfB_lng = SF_B_LNG;
  c.useCatmullRom = catmull;
  return c;
}

// =============================================================================
// LINEAR interpolation
// =============================================================================

void test_linear_completes_one_lap() {
  ReplayResult r = runNmeaReplay(gps_logs, num_gps_logs, makeCfg(false));
  EXPECT_TRUE(r.raceStarted);
  EXPECT_EQ((int)r.lapTimes.size(), 1);
}

void test_linear_matches_dovetimer_golden() {
  ReplayResult r = runNmeaReplay(gps_logs, num_gps_logs, makeCfg(false));
  // Documented: DoveTimer LINEAR = 1:08.748 = 68748 ms
  EXPECT_TRUE(r.lapTimes.size() >= 1);
  if (r.lapTimes.size() >= 1) {
    EXPECT_NEAR(r.lapTimes[0], 68748UL, 50.0);
  }
}

void test_linear_close_to_mylaps() {
  ReplayResult r = runNmeaReplay(gps_logs, num_gps_logs, makeCfg(false));
  // MyLaps magnetic-loop: 1:08.807 = 68807 ms. GPS vs. magnetic loop
  // typically differ by 50-100ms due to antenna placement / interpolation,
  // so 200ms tolerance is realistic.
  EXPECT_TRUE(r.lapTimes.size() >= 1);
  if (r.lapTimes.size() >= 1) {
    EXPECT_NEAR(r.lapTimes[0], 68807UL, 200.0);
  }
}

// =============================================================================
// CATMULL-ROM interpolation
// =============================================================================

void test_catmull_completes_one_lap() {
  ReplayResult r = runNmeaReplay(gps_logs, num_gps_logs, makeCfg(true));
  EXPECT_TRUE(r.raceStarted);
  EXPECT_EQ((int)r.lapTimes.size(), 1);
}

void test_catmull_matches_dovetimer_golden() {
  ReplayResult r = runNmeaReplay(gps_logs, num_gps_logs, makeCfg(true));
  // Documented: DoveTimer CATMULL = 1:08.745 = 68745 ms
  EXPECT_TRUE(r.lapTimes.size() >= 1);
  if (r.lapTimes.size() >= 1) {
    EXPECT_NEAR(r.lapTimes[0], 68745UL, 50.0);
  }
}

void test_catmull_close_to_mylaps() {
  ReplayResult r = runNmeaReplay(gps_logs, num_gps_logs, makeCfg(true));
  EXPECT_TRUE(r.lapTimes.size() >= 1);
  if (r.lapTimes.size() >= 1) {
    EXPECT_NEAR(r.lapTimes[0], 68807UL, 200.0);
  }
}

int main() {
  printf("=== NMEA replay: OKC single lap (gps_race_data_lap.h) ===\n");

  RUN_TEST(linear_completes_one_lap);
  RUN_TEST(linear_matches_dovetimer_golden);
  RUN_TEST(linear_close_to_mylaps);

  RUN_TEST(catmull_completes_one_lap);
  RUN_TEST(catmull_matches_dovetimer_golden);
  RUN_TEST(catmull_close_to_mylaps);

  TEST_SUMMARY();
}
