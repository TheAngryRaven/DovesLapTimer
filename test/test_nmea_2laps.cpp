/**
 * Layer-3 NMEA replay regression test.
 *
 * Fixture: examples/real_track_data_debug/gps_race_data_2laps.h
 *   Orlando Kart Center, two laps, white Tony rental.
 *
 * Golden times (from fixture header):
 *   LAP 1
 *     DoveTimer LINEAR  : 1:09.958 ms (69958)
 *     DoveTimer CATMULL : 1:09.942 ms (69942)
 *     MyLaps            : not recorded
 *   LAP 2
 *     MyLaps            : 1:08.807 ms (68807) — magnetic-loop ground truth
 *     DoveTimer LINEAR  : 1:08.748 ms (68748)
 *     DoveTimer CATMULL : 1:08.745 ms (68745)
 */

#include "test_runner.h"
#include "replay_runner.h"
#include "../examples/real_track_data_debug/gps_race_data_2laps.h"

static const int num_gps_logs = sizeof(gps_logs) / sizeof(gps_logs[0]);

static ReplayConfig makeCfg(bool catmull) {
  ReplayConfig c;
  c.sfA_lat = 28.41270817056385; c.sfA_lng = -81.37973266418031;
  c.sfB_lat = 28.41273038679321; c.sfB_lng = -81.37957048753776;
  c.useCatmullRom = catmull;
  return c;
}

void test_linear_completes_two_laps() {
  // Fixture is named "two laps" but the recording continues a few seconds
  // into a third lap. Golden values only cover laps 1 and 2 — assert >= 2.
  ReplayResult r = runNmeaReplay(gps_logs, num_gps_logs, makeCfg(false));
  EXPECT_TRUE((int)r.lapTimes.size() >= 2);
}

void test_linear_lap1_matches_golden() {
  ReplayResult r = runNmeaReplay(gps_logs, num_gps_logs, makeCfg(false));
  EXPECT_TRUE(r.lapTimes.size() >= 1);
  if (r.lapTimes.size() >= 1) EXPECT_NEAR(r.lapTimes[0], 69958UL, 50.0);
}

void test_linear_lap2_matches_golden() {
  ReplayResult r = runNmeaReplay(gps_logs, num_gps_logs, makeCfg(false));
  EXPECT_TRUE(r.lapTimes.size() >= 2);
  if (r.lapTimes.size() >= 2) EXPECT_NEAR(r.lapTimes[1], 68748UL, 50.0);
}

void test_linear_lap2_close_to_mylaps() {
  ReplayResult r = runNmeaReplay(gps_logs, num_gps_logs, makeCfg(false));
  EXPECT_TRUE(r.lapTimes.size() >= 2);
  if (r.lapTimes.size() >= 2) EXPECT_NEAR(r.lapTimes[1], 68807UL, 200.0);
}

void test_catmull_completes_two_laps() {
  ReplayResult r = runNmeaReplay(gps_logs, num_gps_logs, makeCfg(true));
  EXPECT_TRUE((int)r.lapTimes.size() >= 2);
}

void test_catmull_lap1_matches_golden() {
  ReplayResult r = runNmeaReplay(gps_logs, num_gps_logs, makeCfg(true));
  EXPECT_TRUE(r.lapTimes.size() >= 1);
  if (r.lapTimes.size() >= 1) EXPECT_NEAR(r.lapTimes[0], 69942UL, 50.0);
}

void test_catmull_lap2_matches_golden() {
  ReplayResult r = runNmeaReplay(gps_logs, num_gps_logs, makeCfg(true));
  EXPECT_TRUE(r.lapTimes.size() >= 2);
  if (r.lapTimes.size() >= 2) EXPECT_NEAR(r.lapTimes[1], 68745UL, 50.0);
}

void test_catmull_lap2_close_to_mylaps() {
  ReplayResult r = runNmeaReplay(gps_logs, num_gps_logs, makeCfg(true));
  EXPECT_TRUE(r.lapTimes.size() >= 2);
  if (r.lapTimes.size() >= 2) EXPECT_NEAR(r.lapTimes[1], 68807UL, 200.0);
}

int main() {
  printf("=== NMEA replay: OKC two laps (gps_race_data_2laps.h) ===\n");

  RUN_TEST(linear_completes_two_laps);
  RUN_TEST(linear_lap1_matches_golden);
  RUN_TEST(linear_lap2_matches_golden);
  RUN_TEST(linear_lap2_close_to_mylaps);

  RUN_TEST(catmull_completes_two_laps);
  RUN_TEST(catmull_lap1_matches_golden);
  RUN_TEST(catmull_lap2_matches_golden);
  RUN_TEST(catmull_lap2_close_to_mylaps);

  TEST_SUMMARY();
}
