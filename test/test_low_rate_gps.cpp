/**
 * Layer-2 tests for low-sample-rate GPS crossing detection — review issue H4.
 *
 * The old crossing validation required the straddle pair's summed distance
 * to the line to be under crossingThresholdMeters in absolute meters. That
 * conflates zone size with sample density: at 1 Hz / ~70 km/h consecutive
 * fixes are ~19 m apart, every genuine crossing failed the check, and the
 * lap counter silently never incremented (the failure went only to debug
 * serial). Validation now scales with the pair's own spacing, the exiting
 * fix is included in the buffer so a straddle pair exists at all, and
 * rejected crossings are surfaced via getRejectedCrossingCount().
 *
 * Geometry: S/F line spans lat 28.41255..28.41275 at lng -81.37920; the
 * driver moves only in longitude, perpendicular through the line.
 */

#include "test_runner.h"
#include "../src/DovesLapTimer.h"

static constexpr double SF_A_LAT = 28.41255, SF_A_LNG = -81.37920;
static constexpr double SF_B_LAT = 28.41275, SF_B_LNG = -81.37920;
static constexpr double DRIVER_LAT = 28.41265;          // line midpoint
static constexpr double LNG_PER_METER = 1.0 / 97924.0;  // at this latitude
static constexpr float ALT = 50.0f;

static void feedFix(DovesLapTimer &t, double metersEastOfLine,
                    float speedKnots, unsigned long timeMs) {
  t.updateCurrentTime(timeMs);
  t.loop(DRIVER_LAT, SF_A_LNG + metersEastOfLine * LNG_PER_METER, ALT, speedKnots);
}

// =============================================================================
// 1 Hz crossing: 19m between fixes, threshold widened to 15m as a 1 Hz
// user would. Pre-fix the straddle pair's 19m sum always failed the 15m
// absolute bound -> zero laps forever.
// =============================================================================

void test_one_hz_crossing_detected() {
  DovesLapTimer timer(15.0);  // enlarged zone, appropriate for 1 Hz
  timer.setStartFinishLine(SF_A_LAT, SF_A_LNG, SF_B_LAT, SF_B_LNG);

  // ~70 km/h at 1 Hz = 19m per fix, perpendicular through the line.
  // Two passes: first arms raceStarted, second completes a lap.
  const float speedKnots = 37.0f;  // ~68.5 km/h
  unsigned long t = 10000;
  for (int pass = 0; pass < 2; pass++) {
    for (double m = -47.5; m <= 50.0; m += 19.0) {
      feedFix(timer, m, speedKnots, t);
      t += 1000;
    }
    // teleport-guard-friendly return trip: come back through the line
    if (pass == 0) {
      for (double m = 50.0 - 19.0 / 2; m >= -50.0; m -= 19.0) {
        feedFix(timer, m, speedKnots, t);
        t += 1000;
      }
    }
  }

  EXPECT_TRUE(timer.getRaceStarted());
  EXPECT_TRUE(timer.getLaps() >= 1);
  EXPECT_EQ(timer.getRejectedCrossingCount(), 0U);
}

void test_five_hz_crossing_detected_with_default_threshold() {
  // 5 Hz / ~70 km/h = ~3.8m spacing — must work with the stock 7m zone.
  DovesLapTimer timer(7.0);
  timer.setStartFinishLine(SF_A_LAT, SF_A_LNG, SF_B_LAT, SF_B_LNG);

  unsigned long t = 10000;
  for (double m = -19.0; m <= 19.0; m += 3.8) {
    feedFix(timer, m, 37.0f, t);
    t += 200;
  }

  EXPECT_TRUE(timer.getRaceStarted());
  EXPECT_EQ(timer.getRejectedCrossingCount(), 0U);
}

// =============================================================================
// Rejected crossings are surfaced, not silent
// =============================================================================

void test_rejected_crossing_increments_counter() {
  // Drive into the zone and back out the same side: a zone exit with no
  // side change can't be interpolated. Previously this was visible only
  // on debug serial; now the public counter records it.
  DovesLapTimer timer(7.0);
  timer.setStartFinishLine(SF_A_LAT, SF_A_LNG, SF_B_LAT, SF_B_LNG);

  unsigned long t = 10000;
  feedFix(timer, -20.0, 20.0f, t);  t += 200;
  feedFix(timer, -5.0,  20.0f, t);  t += 200;  // enters zone, west side
  feedFix(timer, -3.0,  20.0f, t);  t += 200;  // still west
  feedFix(timer, -20.0, 20.0f, t);  t += 200;  // U-turn, exits same side

  EXPECT_FALSE(timer.getRaceStarted());
  EXPECT_EQ(timer.getRejectedCrossingCount(), 1U);

  // A genuine crossing afterwards still works and doesn't bump the counter
  for (double m = -20.0; m <= 20.0; m += 4.0) {
    feedFix(timer, m, 20.0f, t);
    t += 200;
  }
  EXPECT_TRUE(timer.getRaceStarted());
  EXPECT_EQ(timer.getRejectedCrossingCount(), 1U);
}

void test_reset_clears_rejected_crossing_counter() {
  DovesLapTimer timer(7.0);
  timer.setStartFinishLine(SF_A_LAT, SF_A_LNG, SF_B_LAT, SF_B_LNG);

  unsigned long t = 10000;
  feedFix(timer, -20.0, 20.0f, t);  t += 200;
  feedFix(timer, -5.0,  20.0f, t);  t += 200;
  feedFix(timer, -20.0, 20.0f, t);  t += 200;
  EXPECT_EQ(timer.getRejectedCrossingCount(), 1U);

  timer.reset();
  EXPECT_EQ(timer.getRejectedCrossingCount(), 0U);
}

int main() {
  printf("=== Low-rate GPS crossing tests ===\n");

  RUN_TEST(one_hz_crossing_detected);
  RUN_TEST(five_hz_crossing_detected_with_default_threshold);

  RUN_TEST(rejected_crossing_increments_counter);
  RUN_TEST(reset_clears_rejected_crossing_counter);

  TEST_SUMMARY();
}
