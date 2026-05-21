/**
 * Layer-2 tests for CourseDetector.
 *
 * Covers the state machine (IDLE → WAITING_FOR_SPEED → WAYPOINT_SET →
 * CANDIDATES_READY → DETECTED), distance-based matching with tolerance,
 * ranked output, and the rejection-cooldown behavior that fixes
 * CLAUDE.md known-issue #13.
 *
 * All tests use a fixed lat/lng so geoHaversine returns 0 — we're testing
 * the state machine, not the geo math.
 */

#include "test_runner.h"
#include "../src/CourseDetector.h"

static constexpr double WAYPOINT_LAT = 28.41265;
static constexpr double WAYPOINT_LNG = -81.37970;
static constexpr float  SPEED_FAST_KMH = 35.0f;   // ~21.7 mph, above 20mph gate
static constexpr float  SPEED_SLOW_KMH = 10.0f;   // ~6.2 mph, below gate

// =============================================================================
// Initial state + IDLE -> WAITING_FOR_SPEED transition
// =============================================================================

void test_initial_state_is_idle() {
  CourseDetector det;
  EXPECT_EQ(det.getState(), DETECT_STATE_IDLE);
  EXPECT_FALSE(det.hasWaypoint());
  EXPECT_FALSE(det.isDetected());
  EXPECT_EQ(det.getDetectedCourseIndex(), -1);
  EXPECT_EQ(det.getRankedMatchCount(), 0);
}

void test_slow_speed_stays_waiting_for_speed() {
  CourseInfo courses[] = {{"Course A", 1000.0f}};
  CourseDetector det;
  det.init(courses, 1);
  det.update(WAYPOINT_LAT, WAYPOINT_LNG, SPEED_SLOW_KMH, 0.0f);
  EXPECT_EQ(det.getState(), DETECT_STATE_WAITING_FOR_SPEED);
  EXPECT_FALSE(det.hasWaypoint());
}

// =============================================================================
// WAITING_FOR_SPEED -> WAYPOINT_SET on speed threshold
// =============================================================================

void test_fast_speed_drops_waypoint() {
  CourseInfo courses[] = {{"Course A", 1000.0f}};
  CourseDetector det;
  det.init(courses, 1);
  det.update(WAYPOINT_LAT, WAYPOINT_LNG, SPEED_FAST_KMH, 100.0f);
  EXPECT_EQ(det.getState(), DETECT_STATE_WAYPOINT_SET);
  EXPECT_TRUE(det.hasWaypoint());
  EXPECT_NEAR(det.getWaypointLat(), WAYPOINT_LAT, 1e-9);
  EXPECT_NEAR(det.getWaypointLng(), WAYPOINT_LNG, 1e-9);
}

// =============================================================================
// WAYPOINT_SET state — proximity + min-distance gates
// =============================================================================

void test_proximity_without_min_distance_does_not_rank() {
  CourseInfo courses[] = {{"Course A", 1000.0f}};
  CourseDetector det;
  det.init(courses, 1);
  det.update(WAYPOINT_LAT, WAYPOINT_LNG, SPEED_FAST_KMH, 100.0f);

  // 50m driven — proximity OK (same position), but distance < 200m gate
  det.update(WAYPOINT_LAT, WAYPOINT_LNG, SPEED_FAST_KMH, 150.0f);
  EXPECT_EQ(det.getState(), DETECT_STATE_WAYPOINT_SET);
  EXPECT_EQ(det.getRankedMatchCount(), 0);
}

void test_min_distance_without_proximity_does_not_rank() {
  CourseInfo courses[] = {{"Course A", 1000.0f}};
  CourseDetector det;
  det.init(courses, 1);
  det.update(WAYPOINT_LAT, WAYPOINT_LNG, SPEED_FAST_KMH, 100.0f);

  // 300m driven, but driver is 1km away from waypoint — no rank yet
  det.update(WAYPOINT_LAT + 0.01, WAYPOINT_LNG, SPEED_FAST_KMH, 400.0f);
  EXPECT_EQ(det.getState(), DETECT_STATE_WAYPOINT_SET);
  EXPECT_EQ(det.getRankedMatchCount(), 0);
}

// =============================================================================
// Course matching — distance vs lengthFt with 25% tolerance
// =============================================================================

void test_returning_to_waypoint_after_full_lap_ranks_match() {
  // Course A is 1000ft (~305m). Drive ~305m and return -> exact match.
  CourseInfo courses[] = {{"Course A", 1000.0f}};
  CourseDetector det;
  det.init(courses, 1);
  det.update(WAYPOINT_LAT, WAYPOINT_LNG, SPEED_FAST_KMH, 100.0f);

  // Drive 305m (1000ft), return to waypoint
  det.update(WAYPOINT_LAT, WAYPOINT_LNG, SPEED_FAST_KMH, 405.0f);
  EXPECT_EQ(det.getState(), DETECT_STATE_CANDIDATES_READY);
  EXPECT_EQ(det.getRankedMatchCount(), 1);
  EXPECT_EQ(det.getRankedMatches()[0].index, 0);
}

void test_distance_outside_tolerance_does_not_match() {
  // Course A is 1000ft (~305m). Drive 700m (~2296ft) — 129% off, way outside 25%.
  CourseInfo courses[] = {{"Course A", 1000.0f}};
  CourseDetector det;
  det.init(courses, 1);
  det.update(WAYPOINT_LAT, WAYPOINT_LNG, SPEED_FAST_KMH, 100.0f);

  det.update(WAYPOINT_LAT, WAYPOINT_LNG, SPEED_FAST_KMH, 800.0f);  // 700m driven
  EXPECT_EQ(det.getState(), DETECT_STATE_WAYPOINT_SET);  // stays, no match
  EXPECT_EQ(det.getRankedMatchCount(), 0);
}

void test_multiple_courses_sorted_by_match_quality() {
  // Drive 305m (~1000ft). Three courses with varying distances from that.
  // Best match should be Course B (1000ft), then A (900ft, 10% off),
  // then C (1100ft, 10% off). D (1500ft) is outside tolerance.
  CourseInfo courses[] = {
    {"Course A",  900.0f},
    {"Course B", 1000.0f},   // perfect match
    {"Course C", 1100.0f},
    {"Course D", 1500.0f},   // outside tolerance
  };
  CourseDetector det;
  det.init(courses, 4);
  det.update(WAYPOINT_LAT, WAYPOINT_LNG, SPEED_FAST_KMH, 100.0f);
  det.update(WAYPOINT_LAT, WAYPOINT_LNG, SPEED_FAST_KMH, 405.0f);  // ~305m

  EXPECT_EQ(det.getState(), DETECT_STATE_CANDIDATES_READY);
  EXPECT_EQ(det.getRankedMatchCount(), 3);  // D excluded
  EXPECT_EQ(det.getRankedMatches()[0].index, 1);  // B first (lowest ratio)
}

// =============================================================================
// Accept / reject transitions
// =============================================================================

void test_accept_candidate_moves_to_detected() {
  CourseInfo courses[] = {{"Course A", 1000.0f}};
  CourseDetector det;
  det.init(courses, 1);
  det.update(WAYPOINT_LAT, WAYPOINT_LNG, SPEED_FAST_KMH, 100.0f);
  det.update(WAYPOINT_LAT, WAYPOINT_LNG, SPEED_FAST_KMH, 405.0f);

  det.acceptCandidate(0);
  EXPECT_EQ(det.getState(), DETECT_STATE_DETECTED);
  EXPECT_TRUE(det.isDetected());
  EXPECT_EQ(det.getDetectedCourseIndex(), 0);
}

// =============================================================================
// Rejection cooldown (CLAUDE.md known-issue #13)
//
// rejectAllCandidates(currentOdometer) must advance the waypoint odometer
// to "now". Otherwise the driver is still inside proximity with
// distanceSinceWaypoint above the 200m gate, so the next GPS fix
// immediately re-ranks the same candidates — burning MAX_REJECTIONS in
// a handful of frames and falling straight to Lap Anything.
// =============================================================================

void test_rejection_imposes_cooldown_until_another_full_lap() {
  CourseInfo courses[] = {{"Course A", 1000.0f}};
  CourseDetector det;
  det.init(courses, 1);
  det.update(WAYPOINT_LAT, WAYPOINT_LNG, SPEED_FAST_KMH, 100.0f);
  det.update(WAYPOINT_LAT, WAYPOINT_LNG, SPEED_FAST_KMH, 405.0f);
  EXPECT_EQ(det.getState(), DETECT_STATE_CANDIDATES_READY);

  det.rejectAllCandidates(405.0f);
  EXPECT_EQ(det.getState(), DETECT_STATE_WAYPOINT_SET);

  // The very next GPS fix is still in proximity. Without the cooldown fix,
  // distanceSinceWaypoint would still be 305m (above 200m gate) and we'd
  // immediately re-rank.
  det.update(WAYPOINT_LAT, WAYPOINT_LNG, SPEED_FAST_KMH, 410.0f);
  EXPECT_EQ(det.getState(), DETECT_STATE_WAYPOINT_SET);  // stays, NO re-rank
  EXPECT_EQ(det.getRankedMatchCount(), 0);

  // Drive another 300m+, return to waypoint — NOW we re-rank
  det.update(WAYPOINT_LAT, WAYPOINT_LNG, SPEED_FAST_KMH, 715.0f);
  EXPECT_EQ(det.getState(), DETECT_STATE_CANDIDATES_READY);
}

// =============================================================================
// reset() restores initial state
// =============================================================================

void test_reset_restores_initial_state() {
  CourseInfo courses[] = {{"Course A", 1000.0f}};
  CourseDetector det;
  det.init(courses, 1);
  det.update(WAYPOINT_LAT, WAYPOINT_LNG, SPEED_FAST_KMH, 100.0f);
  det.update(WAYPOINT_LAT, WAYPOINT_LNG, SPEED_FAST_KMH, 405.0f);
  det.acceptCandidate(0);
  EXPECT_TRUE(det.isDetected());

  det.reset();
  EXPECT_EQ(det.getState(), DETECT_STATE_IDLE);
  EXPECT_FALSE(det.isDetected());
  EXPECT_EQ(det.getDetectedCourseIndex(), -1);
}

int main() {
  printf("=== CourseDetector tests ===\n");

  RUN_TEST(initial_state_is_idle);
  RUN_TEST(slow_speed_stays_waiting_for_speed);
  RUN_TEST(fast_speed_drops_waypoint);

  RUN_TEST(proximity_without_min_distance_does_not_rank);
  RUN_TEST(min_distance_without_proximity_does_not_rank);

  RUN_TEST(returning_to_waypoint_after_full_lap_ranks_match);
  RUN_TEST(distance_outside_tolerance_does_not_match);
  RUN_TEST(multiple_courses_sorted_by_match_quality);

  RUN_TEST(accept_candidate_moves_to_detected);
  RUN_TEST(rejection_imposes_cooldown_until_another_full_lap);

  RUN_TEST(reset_restores_initial_state);

  TEST_SUMMARY();
}
