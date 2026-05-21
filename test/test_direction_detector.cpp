/**
 * Layer-2 tests for DirectionDetector (struct in DovesLapTimer.h,
 * onLineCrossing impl in DovesLapTimer.cpp).
 *
 * Covers initial state, both resolution outcomes, the single-sector
 * discard window, glitch overwrite (CLAUDE.md known-issue #12), and the
 * post-resolution lock. The two bugs at #11 and #12 would each be caught
 * by the "forward_resolves_after_full_lap" / "glitch_overwrite_*" tests.
 */

#include "test_runner.h"
#include "../src/DovesLapTimer.h"

// =============================================================================
// Initial state
// =============================================================================

void test_initial_state_is_unknown() {
  DirectionDetector d;
  EXPECT_EQ(d.direction, DIR_UNKNOWN);
  EXPECT_FALSE(d.raceSeen);
  EXPECT_EQ(d.lapS2CrossingTime, 0UL);
  EXPECT_EQ(d.lapS3CrossingTime, 0UL);
  EXPECT_FALSE(d.isResolved());
  EXPECT_FALSE(d.isReverse());
}

void test_reset_restores_initial_state() {
  DirectionDetector d;
  d.onLineCrossing(0, 100);
  d.onLineCrossing(2, 200);
  d.onLineCrossing(3, 300);
  d.onLineCrossing(0, 400);  // resolves FORWARD
  EXPECT_TRUE(d.isResolved());

  d.reset();
  EXPECT_EQ(d.direction, DIR_UNKNOWN);
  EXPECT_FALSE(d.raceSeen);
  EXPECT_EQ(d.lapS2CrossingTime, 0UL);
  EXPECT_EQ(d.lapS3CrossingTime, 0UL);
}

// =============================================================================
// raceSeen gating — sector crossings before the first S/F do nothing
// =============================================================================

void test_sector_crossing_before_raceseen_is_ignored() {
  DirectionDetector d;
  d.onLineCrossing(2, 1000);
  d.onLineCrossing(3, 2000);
  EXPECT_EQ(d.lapS2CrossingTime, 0UL);  // not recorded
  EXPECT_EQ(d.lapS3CrossingTime, 0UL);
  EXPECT_FALSE(d.isResolved());
}

void test_first_startfinish_arms_detection() {
  DirectionDetector d;
  d.onLineCrossing(0, 5000);
  EXPECT_TRUE(d.raceSeen);
  EXPECT_FALSE(d.isResolved());  // can't resolve yet — no S2/S3 seen
}

// =============================================================================
// Resolution from full lap (S/F → S2 → S3 → S/F or S/F → S3 → S2 → S/F)
// =============================================================================

void test_forward_resolves_after_full_lap() {
  DirectionDetector d;
  d.onLineCrossing(0, 0);      // arm
  d.onLineCrossing(2, 1000);
  d.onLineCrossing(3, 2000);
  d.onLineCrossing(0, 3000);   // resolve at next S/F
  EXPECT_EQ(d.direction, DIR_FORWARD);
  EXPECT_TRUE(d.isResolved());
  EXPECT_FALSE(d.isReverse());
}

void test_reverse_resolves_after_full_lap() {
  DirectionDetector d;
  d.onLineCrossing(0, 0);
  d.onLineCrossing(3, 1000);   // S3 first means driving the track backwards
  d.onLineCrossing(2, 2000);
  d.onLineCrossing(0, 3000);
  EXPECT_EQ(d.direction, DIR_REVERSE);
  EXPECT_TRUE(d.isReverse());
}

// =============================================================================
// Single-sector lap discard (CLAUDE.md known-issue #11)
//
// If the driver crosses only one sector line in a lap (poorly-placed line,
// or GPS sample rate too low to catch it), direction must NOT resolve from
// that single data point. It used to lock as REVERSE from a lone S3.
// =============================================================================

void test_single_s2_lap_does_not_resolve() {
  DirectionDetector d;
  d.onLineCrossing(0, 0);
  d.onLineCrossing(2, 1000);
  // no S3 this lap
  d.onLineCrossing(0, 3000);
  EXPECT_EQ(d.direction, DIR_UNKNOWN);
  // window must reset so next lap can try again
  EXPECT_EQ(d.lapS2CrossingTime, 0UL);
  EXPECT_EQ(d.lapS3CrossingTime, 0UL);
}

void test_single_s3_lap_does_not_resolve() {
  DirectionDetector d;
  d.onLineCrossing(0, 0);
  d.onLineCrossing(3, 1000);
  d.onLineCrossing(0, 3000);
  EXPECT_EQ(d.direction, DIR_UNKNOWN);
}

void test_single_sector_lap_then_complete_lap_resolves() {
  DirectionDetector d;
  d.onLineCrossing(0, 0);
  d.onLineCrossing(2, 1000);   // single-sector lap (discarded)
  d.onLineCrossing(0, 3000);   // resolution fails, retries next lap
  EXPECT_EQ(d.direction, DIR_UNKNOWN);

  d.onLineCrossing(2, 4000);   // full lap this time
  d.onLineCrossing(3, 5000);
  d.onLineCrossing(0, 6000);
  EXPECT_EQ(d.direction, DIR_FORWARD);
}

// =============================================================================
// Glitch overwrite (CLAUDE.md known-issue #12)
//
// A GPS teleport that fabricates a phantom early sector crossing must be
// overwritten by the real later crossing. Pre-fix, "first crossing wins"
// locked direction to whatever the glitch said.
// =============================================================================

void test_glitch_s3_overwritten_by_real_s3_later() {
  DirectionDetector d;
  d.onLineCrossing(0, 0);
  d.onLineCrossing(3, 100);    // phantom S3 glitch right after S/F
  d.onLineCrossing(2, 1000);   // real S2 mid-lap
  d.onLineCrossing(3, 2000);   // real S3 — must overwrite the phantom
  d.onLineCrossing(0, 3000);
  // Without overwrite: lapS3=100, lapS2=1000 → S2>S3 → REVERSE (wrong)
  // With overwrite:    lapS3=2000, lapS2=1000 → S2<S3 → FORWARD (correct)
  EXPECT_EQ(d.direction, DIR_FORWARD);
}

void test_glitch_s2_overwritten_by_real_s2_later() {
  DirectionDetector d;
  d.onLineCrossing(0, 0);
  d.onLineCrossing(2, 100);    // phantom S2
  d.onLineCrossing(3, 1000);
  d.onLineCrossing(2, 2000);   // real S2 — overwrites phantom
  d.onLineCrossing(0, 3000);
  // lapS2=2000, lapS3=1000 → S2>S3 → REVERSE (correct, driver IS reverse)
  EXPECT_EQ(d.direction, DIR_REVERSE);
}

// =============================================================================
// Post-resolution lock
// =============================================================================

void test_direction_does_not_flip_after_resolution() {
  DirectionDetector d;
  d.onLineCrossing(0, 0);
  d.onLineCrossing(2, 1000);
  d.onLineCrossing(3, 2000);
  d.onLineCrossing(0, 3000);   // FORWARD
  EXPECT_EQ(d.direction, DIR_FORWARD);

  // Subsequent crossings that look "reverse" must be ignored.
  d.onLineCrossing(3, 4000);
  d.onLineCrossing(2, 5000);
  d.onLineCrossing(0, 6000);
  EXPECT_EQ(d.direction, DIR_FORWARD);
}

void test_sector_crossings_ignored_after_resolution() {
  DirectionDetector d;
  d.onLineCrossing(0, 0);
  d.onLineCrossing(2, 1000);
  d.onLineCrossing(3, 2000);
  d.onLineCrossing(0, 3000);   // FORWARD locked

  d.onLineCrossing(2, 5000);
  // After resolution the lap window is not maintained — S2 is no-op.
  EXPECT_EQ(d.lapS2CrossingTime, 0UL);
}

int main() {
  printf("=== DirectionDetector tests ===\n");

  RUN_TEST(initial_state_is_unknown);
  RUN_TEST(reset_restores_initial_state);

  RUN_TEST(sector_crossing_before_raceseen_is_ignored);
  RUN_TEST(first_startfinish_arms_detection);

  RUN_TEST(forward_resolves_after_full_lap);
  RUN_TEST(reverse_resolves_after_full_lap);

  RUN_TEST(single_s2_lap_does_not_resolve);
  RUN_TEST(single_s3_lap_does_not_resolve);
  RUN_TEST(single_sector_lap_then_complete_lap_resolves);

  RUN_TEST(glitch_s3_overwritten_by_real_s3_later);
  RUN_TEST(glitch_s2_overwritten_by_real_s2_later);

  RUN_TEST(direction_does_not_flip_after_resolution);
  RUN_TEST(sector_crossings_ignored_after_resolution);

  TEST_SUMMARY();
}
