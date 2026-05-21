/**
 * Layer-3 NMEA replay regression test.
 *
 * Fixture: examples/real_track_data_debug/gps_race_data_2laps.h
 *   Orlando Kart Center, two laps, white Tony rental. (Recording actually
 *   continues a few seconds into a third lap.)
 *
 * Golden times (current library output, captured 2026-05-21):
 *   LAP 1: 69961 ms (linear == catmull)
 *   LAP 2: 68742 ms (linear == catmull)
 *
 * Lap times are identical between modes because Catmull-Rom is scoped to
 * crossing-point lat/lng only — time/odometer are always interpolated
 * linearly (CLAUDE.md known-issue #1). Fixture-header values from
 * pre-2026 captures (LINEAR 69958/68748, CATMULL 69942/68745) predate
 * that scope fix and no longer represent current behavior.
 *
 * MyLaps magnetic-loop ground truth on LAP 2: 68807 ms — checked with
 * a looser ±200ms bound.
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
  ReplayResult r = runNmeaReplay(gps_logs, num_gps_logs, makeCfg(false));
  EXPECT_TRUE((int)r.lapTimes.size() >= 2);
}

void test_linear_lap1_matches_golden() {
  ReplayResult r = runNmeaReplay(gps_logs, num_gps_logs, makeCfg(false));
  EXPECT_TRUE(r.lapTimes.size() >= 1);
  if (r.lapTimes.size() >= 1) EXPECT_NEAR(r.lapTimes[0], 69961UL, 10.0);
}

void test_linear_lap2_matches_golden() {
  ReplayResult r = runNmeaReplay(gps_logs, num_gps_logs, makeCfg(false));
  EXPECT_TRUE(r.lapTimes.size() >= 2);
  if (r.lapTimes.size() >= 2) EXPECT_NEAR(r.lapTimes[1], 68742UL, 10.0);
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
  if (r.lapTimes.size() >= 1) EXPECT_NEAR(r.lapTimes[0], 69961UL, 10.0);
}

void test_catmull_lap2_matches_golden() {
  ReplayResult r = runNmeaReplay(gps_logs, num_gps_logs, makeCfg(true));
  EXPECT_TRUE(r.lapTimes.size() >= 2);
  if (r.lapTimes.size() >= 2) EXPECT_NEAR(r.lapTimes[1], 68742UL, 10.0);
}

void test_catmull_lap_times_match_linear() {
  ReplayResult lin = runNmeaReplay(gps_logs, num_gps_logs, makeCfg(false));
  ReplayResult cat = runNmeaReplay(gps_logs, num_gps_logs, makeCfg(true));
  EXPECT_TRUE(lin.lapTimes.size() >= 2 && cat.lapTimes.size() >= 2);
  if (lin.lapTimes.size() >= 2 && cat.lapTimes.size() >= 2) {
    EXPECT_EQ(cat.lapTimes[0], lin.lapTimes[0]);
    EXPECT_EQ(cat.lapTimes[1], lin.lapTimes[1]);
  }
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
  RUN_TEST(catmull_lap_times_match_linear);

  TEST_SUMMARY();
}
