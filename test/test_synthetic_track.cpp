/**
 * Layer-2 integration test for the full DovesLapTimer pipeline.
 *
 * Drives the lap timer through a deterministic synthetic track — the same
 * 100m × 100m counter-clockwise square used by sector_timing_example —
 * and asserts:
 *
 *   - 3 laps complete
 *   - lap times match expected (80 steps × 200ms = 16000ms)
 *   - sector times match expected (20 steps × 200ms = 4000ms each)
 *   - laps are identical lap-to-lap (deterministic interpolation)
 *   - sum of sectors ≈ lap time
 *   - direction resolves to FORWARD
 *   - optimal lap ≈ best lap (since every lap is the same)
 *
 * If the crossing-detection / interpolation algorithm regresses, the
 * cross-lap consistency check will flag it before anyone notices in real
 * data.
 */

#include "test_runner.h"
#include "../src/DovesLapTimer.h"

// =============================================================================
// Synthetic track (mirrors examples/sector_timing_example)
// =============================================================================

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
static constexpr double S2_A_LAT = 28.41310, S2_A_LNG = -81.37860;
static constexpr double S2_B_LAT = 28.41310, S2_B_LNG = -81.37880;
static constexpr double S3_A_LAT = 28.41345, S3_A_LNG = -81.37925;
static constexpr double S3_B_LAT = 28.41365, S3_B_LNG = -81.37925;

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

// =============================================================================
// Drive the lap timer through N simulated laps.
//
// Captures per-lap completion times via a callback-like accumulator so we
// can assert lap-to-lap consistency, not just final state.
// =============================================================================

struct LapRecord {
  unsigned long lapTime;
  unsigned long sector1;
  unsigned long sector2;
  // sector3 is only set briefly inside loop() and reset before we can
  // observe it externally — covered separately via getBestSector3Time().
};

static void driveLaps(DovesLapTimer &timer, int targetLaps,
                      LapRecord *records) {
  unsigned long simTimeMs = 0;
  int lastLapCount = 0;
  int lastSector = 0;
  int recIdx = 0;
  // Generous step ceiling so a bug that fails to detect crossings can't
  // hang the test forever.
  const unsigned long maxSteps = STEPS_PER_LAP * (unsigned long)(targetLaps + 2) + 50;

  for (unsigned long step = 0; step < maxSteps; step++) {
    double lat, lng;
    generateSyntheticFix(step, lat, lng);
    timer.updateCurrentTime(simTimeMs);
    timer.loop(lat, lng, SIM_ALT_METERS, SIM_SPEED_KNOTS);

    int sector = timer.getCurrentSector();
    int laps = timer.getLaps();

    // Snapshot sector1 / sector2 the moment they finish, while they're
    // still in the current-lap accumulators.
    if (sector != lastSector && recIdx < targetLaps) {
      if (lastSector == 1) {
        records[recIdx].sector1 = timer.getCurrentLapSector1Time();
      } else if (lastSector == 2) {
        records[recIdx].sector2 = timer.getCurrentLapSector2Time();
      }
    }
    lastSector = sector;

    if (laps > lastLapCount && recIdx < targetLaps) {
      records[recIdx].lapTime = timer.getLastLapTime();
      recIdx++;
      lastLapCount = laps;
    }

    simTimeMs += SIM_MS_PER_STEP;
    if (laps >= targetLaps) break;
  }
}

// =============================================================================
// Tests
// =============================================================================

void test_three_laps_complete() {
  DovesLapTimer timer(7.0);
  timer.setStartFinishLine(SF_A_LAT, SF_A_LNG, SF_B_LAT, SF_B_LNG);
  timer.setSector2Line(S2_A_LAT, S2_A_LNG, S2_B_LAT, S2_B_LNG);
  timer.setSector3Line(S3_A_LAT, S3_A_LNG, S3_B_LAT, S3_B_LNG);

  LapRecord records[3];
  driveLaps(timer, 3, records);

  EXPECT_EQ(timer.getLaps(), 3);
  EXPECT_TRUE(timer.getRaceStarted());
}

void test_lap_time_matches_expected() {
  // Expected: 80 steps × 200ms = 16000ms. Allow 100ms slack for
  // interpolation jitter at the crossing.
  DovesLapTimer timer(7.0);
  timer.setStartFinishLine(SF_A_LAT, SF_A_LNG, SF_B_LAT, SF_B_LNG);
  timer.setSector2Line(S2_A_LAT, S2_A_LNG, S2_B_LAT, S2_B_LNG);
  timer.setSector3Line(S3_A_LAT, S3_A_LNG, S3_B_LAT, S3_B_LNG);

  LapRecord records[3];
  driveLaps(timer, 3, records);

  EXPECT_NEAR(records[0].lapTime, 16000UL, 100.0);
  EXPECT_NEAR(records[1].lapTime, 16000UL, 100.0);
  EXPECT_NEAR(records[2].lapTime, 16000UL, 100.0);
}

void test_lap_times_are_deterministic_across_laps() {
  // The synthetic track is deterministic — every lap should produce
  // identical times. Variance > 5ms means interpolation is unstable.
  DovesLapTimer timer(7.0);
  timer.setStartFinishLine(SF_A_LAT, SF_A_LNG, SF_B_LAT, SF_B_LNG);
  timer.setSector2Line(S2_A_LAT, S2_A_LNG, S2_B_LAT, S2_B_LNG);
  timer.setSector3Line(S3_A_LAT, S3_A_LNG, S3_B_LAT, S3_B_LNG);

  LapRecord records[3];
  driveLaps(timer, 3, records);

  EXPECT_NEAR(records[0].lapTime, records[1].lapTime, 5.0);
  EXPECT_NEAR(records[1].lapTime, records[2].lapTime, 5.0);
}

void test_sector_times_match_expected() {
  // The synthetic track's sector lines are not evenly spaced around the
  // loop — S3 sits at lng=-81.37925 on the north side and S/F at
  // lng=-81.37920 on the south side, so sector 3 wraps the entire west
  // side plus half the south side. Expected step counts per sector
  // (each step = 200ms sim time):
  //   Sector 1 (S/F → S2):  ~20 steps = 4000ms
  //   Sector 2 (S2 → S3):   ~21 steps = 4200ms
  //   Sector 3 (S3 → S/F):  ~39 steps = 7800ms
  //   Total:                  80 steps = 16000ms
  // Sectors 1 and 2 captured per-lap via mid-lap snapshotting.
  // Sector 3 only verified via getBestSector3Time() since it's reset
  // inside the lap-completing loop() call before we can observe it.
  DovesLapTimer timer(7.0);
  timer.setStartFinishLine(SF_A_LAT, SF_A_LNG, SF_B_LAT, SF_B_LNG);
  timer.setSector2Line(S2_A_LAT, S2_A_LNG, S2_B_LAT, S2_B_LNG);
  timer.setSector3Line(S3_A_LAT, S3_A_LNG, S3_B_LAT, S3_B_LNG);

  LapRecord records[3];
  driveLaps(timer, 3, records);

  EXPECT_NEAR(records[1].sector1, 4000UL, 100.0);
  EXPECT_NEAR(records[1].sector2, 4200UL, 100.0);
  EXPECT_NEAR(timer.getBestSector3Time(), 7800UL, 100.0);
}

void test_sectors_are_deterministic_across_laps() {
  // All laps identical -> sector1/sector2 values should match across laps.
  DovesLapTimer timer(7.0);
  timer.setStartFinishLine(SF_A_LAT, SF_A_LNG, SF_B_LAT, SF_B_LNG);
  timer.setSector2Line(S2_A_LAT, S2_A_LNG, S2_B_LAT, S2_B_LNG);
  timer.setSector3Line(S3_A_LAT, S3_A_LNG, S3_B_LAT, S3_B_LNG);

  LapRecord records[3];
  driveLaps(timer, 3, records);

  EXPECT_NEAR(records[0].sector1, records[1].sector1, 5.0);
  EXPECT_NEAR(records[1].sector1, records[2].sector1, 5.0);
  EXPECT_NEAR(records[0].sector2, records[1].sector2, 5.0);
  EXPECT_NEAR(records[1].sector2, records[2].sector2, 5.0);
}

void test_sectors_sum_to_lap_time() {
  // Best sector1 + best sector2 + best sector3 should match best lap
  // time (all laps identical so best == any lap, and sectors of any lap
  // cover the whole lap).
  DovesLapTimer timer(7.0);
  timer.setStartFinishLine(SF_A_LAT, SF_A_LNG, SF_B_LAT, SF_B_LNG);
  timer.setSector2Line(S2_A_LAT, S2_A_LNG, S2_B_LAT, S2_B_LNG);
  timer.setSector3Line(S3_A_LAT, S3_A_LNG, S3_B_LAT, S3_B_LNG);

  LapRecord records[3];
  driveLaps(timer, 3, records);

  unsigned long sectorSum = timer.getBestSector1Time()
                          + timer.getBestSector2Time()
                          + timer.getBestSector3Time();
  EXPECT_NEAR(sectorSum, timer.getBestLapTime(), 5.0);
}

void test_optimal_lap_close_to_best_lap_on_uniform_track() {
  // Every lap is identical -> best sector times all come from the same
  // lap -> optimal lap = best lap (within rounding).
  DovesLapTimer timer(7.0);
  timer.setStartFinishLine(SF_A_LAT, SF_A_LNG, SF_B_LAT, SF_B_LNG);
  timer.setSector2Line(S2_A_LAT, S2_A_LNG, S2_B_LAT, S2_B_LNG);
  timer.setSector3Line(S3_A_LAT, S3_A_LNG, S3_B_LAT, S3_B_LNG);

  LapRecord records[3];
  driveLaps(timer, 3, records);

  unsigned long optimal = timer.getOptimalLapTime();
  unsigned long best = timer.getBestLapTime();
  EXPECT_TRUE(optimal > 0);
  EXPECT_TRUE(best > 0);
  EXPECT_NEAR(optimal, best, 5.0);
}

void test_direction_resolves_to_forward() {
  // S/F line is set up so a CCW driver hits sectors in 2→3 order,
  // which is "forward" by the detector's S2<S3 rule.
  DovesLapTimer timer(7.0);
  timer.setStartFinishLine(SF_A_LAT, SF_A_LNG, SF_B_LAT, SF_B_LNG);
  timer.setSector2Line(S2_A_LAT, S2_A_LNG, S2_B_LAT, S2_B_LNG);
  timer.setSector3Line(S3_A_LAT, S3_A_LNG, S3_B_LAT, S3_B_LNG);

  LapRecord records[3];
  driveLaps(timer, 3, records);

  EXPECT_TRUE(timer.isDirectionResolved());
  EXPECT_EQ(timer.getDirection(), DIR_FORWARD);
}

int main() {
  printf("=== Synthetic track integration tests ===\n");

  RUN_TEST(three_laps_complete);
  RUN_TEST(lap_time_matches_expected);
  RUN_TEST(lap_times_are_deterministic_across_laps);
  RUN_TEST(sector_times_match_expected);
  RUN_TEST(sectors_are_deterministic_across_laps);
  RUN_TEST(sectors_sum_to_lap_time);
  RUN_TEST(optimal_lap_close_to_best_lap_on_uniform_track);
  RUN_TEST(direction_resolves_to_forward);

  TEST_SUMMARY();
}
