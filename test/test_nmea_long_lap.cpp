/**
 * Layer-3 NMEA replay regression test.
 *
 * Fixture: examples/real_track_data_debug/gps_race_data_long_lap.h
 *   Orlando Kart Center pro track (long course), single lap, Praga.
 *
 * Golden times (from fixture header):
 *   DoveTimer LINEAR  : 0:58.611 ms (58611)
 *   DoveTimer CATMULL : 0:58.611 ms (58611)
 *   MyLaps            : not recorded
 *
 * Note: linear and Catmull-Rom report identical times for this fixture,
 * which suggests the crossing happens at a point where the four spline
 * control points produce a near-linear interpolant — useful side check.
 */

#include "test_runner.h"
#include "replay_runner.h"
#include "../examples/real_track_data_debug/gps_race_data_long_lap.h"

static const int num_gps_logs = sizeof(gps_logs) / sizeof(gps_logs[0]);

static ReplayConfig makeCfg(bool catmull) {
  ReplayConfig c;
  c.sfA_lat = 28.41270817056385; c.sfA_lng = -81.37973266418031;
  c.sfB_lat = 28.41273038679321; c.sfB_lng = -81.37957048753776;
  c.useCatmullRom = catmull;
  return c;
}

void test_linear_completes_one_lap() {
  ReplayResult r = runNmeaReplay(gps_logs, num_gps_logs, makeCfg(false));
  EXPECT_EQ((int)r.lapTimes.size(), 1);
}

void test_linear_matches_golden() {
  ReplayResult r = runNmeaReplay(gps_logs, num_gps_logs, makeCfg(false));
  EXPECT_TRUE(r.lapTimes.size() >= 1);
  if (r.lapTimes.size() >= 1) EXPECT_NEAR(r.lapTimes[0], 58611UL, 50.0);
}

void test_catmull_completes_one_lap() {
  ReplayResult r = runNmeaReplay(gps_logs, num_gps_logs, makeCfg(true));
  EXPECT_EQ((int)r.lapTimes.size(), 1);
}

void test_catmull_matches_golden() {
  ReplayResult r = runNmeaReplay(gps_logs, num_gps_logs, makeCfg(true));
  EXPECT_TRUE(r.lapTimes.size() >= 1);
  if (r.lapTimes.size() >= 1) EXPECT_NEAR(r.lapTimes[0], 58611UL, 50.0);
}

int main() {
  printf("=== NMEA replay: OKC long lap (gps_race_data_long_lap.h) ===\n");

  RUN_TEST(linear_completes_one_lap);
  RUN_TEST(linear_matches_golden);

  RUN_TEST(catmull_completes_one_lap);
  RUN_TEST(catmull_matches_golden);

  TEST_SUMMARY();
}
