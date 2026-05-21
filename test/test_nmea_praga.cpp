/**
 * Layer-3 NMEA replay regression test.
 *
 * Fixture: examples/real_track_data_debug/gps_race_data_praga_laps.h
 *   Orlando Kart Center, two laps, Praga Dark with blown Tillotson motor.
 *
 * Golden times (from fixture header):
 *   LAP 1: DoveTimer LINEAR  : 1:02.727 ms (62727)
 *   LAP 2: DoveTimer LINEAR  : 1:01.317 ms (61317)
 *   MyLaps / CATMULL : not recorded
 *
 * Only the LINEAR path is tested — the fixture predates the CATMULL
 * baseline being captured.
 */

#include "test_runner.h"
#include "replay_runner.h"
#include "../examples/real_track_data_debug/gps_race_data_praga_laps.h"

static const int num_gps_logs = sizeof(gps_logs) / sizeof(gps_logs[0]);

static ReplayConfig makeCfg() {
  ReplayConfig c;
  c.sfA_lat = 28.41270817056385; c.sfA_lng = -81.37973266418031;
  c.sfB_lat = 28.41273038679321; c.sfB_lng = -81.37957048753776;
  c.useCatmullRom = false;
  return c;
}

void test_linear_completes_two_laps() {
  ReplayResult r = runNmeaReplay(gps_logs, num_gps_logs, makeCfg());
  EXPECT_EQ((int)r.lapTimes.size(), 2);
}

void test_linear_lap1_matches_golden() {
  ReplayResult r = runNmeaReplay(gps_logs, num_gps_logs, makeCfg());
  EXPECT_TRUE(r.lapTimes.size() >= 1);
  if (r.lapTimes.size() >= 1) EXPECT_NEAR(r.lapTimes[0], 62727UL, 50.0);
}

void test_linear_lap2_matches_golden() {
  ReplayResult r = runNmeaReplay(gps_logs, num_gps_logs, makeCfg());
  EXPECT_TRUE(r.lapTimes.size() >= 2);
  if (r.lapTimes.size() >= 2) EXPECT_NEAR(r.lapTimes[1], 61317UL, 50.0);
}

int main() {
  printf("=== NMEA replay: OKC praga two laps (gps_race_data_praga_laps.h) ===\n");

  RUN_TEST(linear_completes_two_laps);
  RUN_TEST(linear_lap1_matches_golden);
  RUN_TEST(linear_lap2_matches_golden);

  TEST_SUMMARY();
}
