/**
 * Layer-3 NMEA replay regression test.
 *
 * Fixture: examples/real_track_data_debug/gps_race_data_lap.h
 *   Orlando Kart Center, 4/29/23, one lap, white Tony rental.
 *
 * Golden times (current library output, captured 2026-05-21):
 *   Linear  : 68742 ms (1:08.742)
 *   Catmull : 68742 ms — identical to linear; the spline interpolation
 *             is intentionally scoped to crossing-point lat/lng only,
 *             so lap times are always linear-interpolated regardless
 *             of mode (see CLAUDE.md known-issue #1).
 *
 * Fixture-header values (1:08.748 linear, 1:08.745 catmull) are from
 * pre-2026 captures before the spline-scope fix and no longer represent
 * current behavior. MyLaps magnetic-loop ground truth (1:08.807) is
 * still useful as a sanity bound (within ±200ms is great for GPS vs.
 * a buried magnet).
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

void test_linear_matches_pinned_golden() {
  ReplayResult r = runNmeaReplay(gps_logs, num_gps_logs, makeCfg(false));
  EXPECT_TRUE(r.lapTimes.size() >= 1);
  if (r.lapTimes.size() >= 1) {
    EXPECT_NEAR(r.lapTimes[0], 68742UL, 10.0);
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
//
// Identical lap times to linear (spline scope is lat/lng-only). These tests
// still exercise the alternate code path to keep it from rotting.
// =============================================================================

void test_catmull_completes_one_lap() {
  ReplayResult r = runNmeaReplay(gps_logs, num_gps_logs, makeCfg(true));
  EXPECT_TRUE(r.raceStarted);
  EXPECT_EQ((int)r.lapTimes.size(), 1);
}

void test_catmull_matches_pinned_golden() {
  ReplayResult r = runNmeaReplay(gps_logs, num_gps_logs, makeCfg(true));
  EXPECT_TRUE(r.lapTimes.size() >= 1);
  if (r.lapTimes.size() >= 1) {
    EXPECT_NEAR(r.lapTimes[0], 68742UL, 10.0);
  }
}

void test_catmull_lap_time_matches_linear() {
  ReplayResult lin = runNmeaReplay(gps_logs, num_gps_logs, makeCfg(false));
  ReplayResult cat = runNmeaReplay(gps_logs, num_gps_logs, makeCfg(true));
  EXPECT_TRUE(lin.lapTimes.size() >= 1 && cat.lapTimes.size() >= 1);
  if (lin.lapTimes.size() >= 1 && cat.lapTimes.size() >= 1) {
    // Spline is scoped to crossing-point lat/lng only -> lap times must match.
    EXPECT_EQ(cat.lapTimes[0], lin.lapTimes[0]);
  }
}

int main() {
  printf("=== NMEA replay: OKC single lap (gps_race_data_lap.h) ===\n");

  RUN_TEST(linear_completes_one_lap);
  RUN_TEST(linear_matches_pinned_golden);
  RUN_TEST(linear_close_to_mylaps);

  RUN_TEST(catmull_completes_one_lap);
  RUN_TEST(catmull_matches_pinned_golden);
  RUN_TEST(catmull_lap_time_matches_linear);

  TEST_SUMMARY();
}
