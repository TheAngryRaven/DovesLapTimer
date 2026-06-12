/**
 * Layer-2 tests for CourseManager orchestration — review issues H5, H7, H11.
 *
 * Drives the full detect/validate/accept/reject/fallback flow on the
 * synthetic 100m x 100m square track (400m lap, ~1312 ft):
 *
 *   - candidate accept via the raceStarted sanity check
 *   - reject -> Lap Anything after MAX_REJECTIONS (issue #13 exercised
 *     through CourseManager, where the original bug manifested)
 *   - H5: no-match passes -> Lap Anything (config lists no matching length)
 *   - H5: distance failsafe -> Lap Anything (driver never re-enters
 *     waypoint proximity)
 *   - H7: Lap Anything activation deactivates every course timer
 *   - pruneInactiveCourses() deactivates non-detected timers
 */

#include "test_runner.h"
#include "../src/CourseManager.h"

static constexpr double TRACK_SOUTH_LAT = 28.41265;
static constexpr double TRACK_NORTH_LAT = 28.41355;
static constexpr double TRACK_WEST_LNG  = -81.37970;
static constexpr double TRACK_EAST_LNG  = -81.37870;
static constexpr unsigned int  STEPS_PER_LAP = 80;   // 5m per step, 400m lap
static constexpr unsigned long MS_PER_STEP   = 200;
static constexpr float SPEED_KNOTS = 48.0f;          // ~55 mph
static constexpr float ALT = 50.0f;
static constexpr float LAP_LENGTH_FT = 1312.34f;     // 400m

// S/F line on the south side — crossed every lap
static constexpr double SF_A_LAT = 28.41255, SF_A_LNG = -81.37920;
static constexpr double SF_B_LAT = 28.41275, SF_B_LNG = -81.37920;

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

static void driveLaps(CourseManager &mgr, unsigned long steps) {
  for (unsigned long step = 0; step < steps; step++) {
    double lat, lng;
    generateSyntheticFix(step, lat, lng);
    mgr.updateCurrentTime(1000 + step * MS_PER_STEP);
    mgr.loop(lat, lng, ALT, SPEED_KNOTS);
  }
}

// A course whose length matches the synthetic lap and whose S/F line is
// genuinely crossed -> passes the raceStarted sanity check.
static CourseConfig matchingCourse(const char *name) {
  CourseConfig c = {};
  c.name = name;
  c.lengthFt = LAP_LENGTH_FT;
  c.startALat = SF_A_LAT; c.startALng = SF_A_LNG;
  c.startBLat = SF_B_LAT; c.startBLng = SF_B_LNG;
  c.hasSector2 = false;
  c.hasSector3 = false;
  return c;
}

// =============================================================================
// Construction
// =============================================================================

void test_zero_courses_activates_lap_anything_immediately() {
  TrackConfig cfg = {};
  cfg.longName = "Empty";
  cfg.shortName = "E";
  cfg.courseCount = 0;

  CourseManager mgr(cfg);
  EXPECT_TRUE(mgr.isDetectionComplete());
  EXPECT_TRUE(mgr.isLapAnythingActive());
}

// =============================================================================
// Detection accept (raceStarted sanity check passes)
// =============================================================================

void test_detection_accepts_matching_course() {
  TrackConfig cfg = {};
  cfg.longName = "Synthetic";
  cfg.shortName = "SYN";
  cfg.courseCount = 1;
  cfg.courses[0] = matchingCourse("Square 400m");

  CourseManager mgr(cfg);
  driveLaps(mgr, STEPS_PER_LAP + 30);  // one lap back to the waypoint + slack

  EXPECT_TRUE(mgr.isDetectionComplete());
  EXPECT_FALSE(mgr.isLapAnythingActive());
  EXPECT_EQ(mgr.getActiveCourseIndex(), 0);
  EXPECT_TRUE(mgr.getActiveTimer() != NULL);
  EXPECT_TRUE(strcmp(mgr.getActiveCourseName(), "Square 400m") == 0);
}

void test_active_timer_keeps_lapping_after_detection() {
  TrackConfig cfg = {};
  cfg.longName = "Synthetic";
  cfg.shortName = "SYN";
  cfg.courseCount = 1;
  cfg.courses[0] = matchingCourse("Square 400m");

  CourseManager mgr(cfg);
  driveLaps(mgr, 3 * STEPS_PER_LAP + 30);

  DovesLapTimer *timer = mgr.getActiveTimer();
  EXPECT_TRUE(timer != NULL);
  EXPECT_TRUE(timer->getLaps() >= 2);
  EXPECT_NEAR(timer->getLastLapTime(), STEPS_PER_LAP * MS_PER_STEP, 100.0);
}

// =============================================================================
// Reject path -> Lap Anything (issue #13 exercised through CourseManager)
// =============================================================================

void test_rejections_fall_back_to_lap_anything() {
  // Length matches so candidates ARE produced, but the S/F line is far off
  // track -> raceStarted never true -> every candidate set is rejected.
  TrackConfig cfg = {};
  cfg.longName = "WrongLine";
  cfg.shortName = "WL";
  cfg.courseCount = 1;
  cfg.courses[0] = matchingCourse("Ghost Course");
  cfg.courses[0].startALat = SF_A_LAT + 1.0;  // ~111km away, never crossed
  cfg.courses[0].startBLat = SF_B_LAT + 1.0;

  CourseManager mgr(cfg);
  // Each rejection re-anchors the detector's lap window, so one rejection
  // per lap: 3 laps to exhaust the budget, plus slack.
  driveLaps(mgr, 5 * STEPS_PER_LAP);

  EXPECT_EQ(mgr.getDetectionRejectionCount(), COURSE_DETECT_MAX_REJECTIONS);
  EXPECT_TRUE(mgr.isLapAnythingActive());
  EXPECT_TRUE(mgr.isDetectionComplete());
  EXPECT_TRUE(mgr.getActiveTimer() == NULL);
}

// =============================================================================
// H5 — no-match passes reach the fallback (previously hung forever)
// =============================================================================

void test_no_match_config_falls_back_to_lap_anything() {
  // Configured length is 10000 ft (~3km); driven laps are ~1312 ft. The
  // detector never produces candidates, so the rejection counter never
  // moves — pre-fix, detection sat in WAYPOINT_SET forever and
  // getActiveTimer() stayed NULL while WaypointLapTimer's laps were unseen.
  TrackConfig cfg = {};
  cfg.longName = "WrongLength";
  cfg.shortName = "WLN";
  cfg.courseCount = 1;
  cfg.courses[0] = matchingCourse("Mismatched Course");
  cfg.courses[0].lengthFt = 10000.0f;

  CourseManager mgr(cfg);
  driveLaps(mgr, 5 * STEPS_PER_LAP);

  EXPECT_TRUE(mgr.isLapAnythingActive());
  EXPECT_TRUE(mgr.isDetectionComplete());
  EXPECT_EQ(mgr.getDetectionRejectionCount(), 0);  // proves the no-match path
  EXPECT_TRUE(mgr.getDetector()->getNoMatchCount() >= COURSE_DETECT_MAX_NO_MATCH_PASSES);
  // The fallback timer was timing laps all along and is now surfaced
  EXPECT_TRUE(mgr.getLapAnythingTimer()->getLaps() >= 1);
}

void test_distance_failsafe_falls_back_when_waypoint_never_revisited() {
  // Drive a straight line: the detector drops a waypoint and the driver
  // never comes back, so neither candidates nor no-match passes ever
  // happen. Only the odometer failsafe can unhang this.
  TrackConfig cfg = {};
  cfg.longName = "Straight";
  cfg.shortName = "STR";
  cfg.courseCount = 1;
  cfg.courses[0] = matchingCourse("Unreachable Course");

  CourseManager mgr(cfg);
  // failsafe = max(2000m, 4 x 400m) = 2000m; drive 2300m east in 5m steps
  const double mPerDegLng = 111320.0 * cos(TRACK_SOUTH_LAT * M_PI / 180.0);
  for (unsigned long step = 0; step < 460; step++) {
    double lng = TRACK_WEST_LNG + (step * 5.0) / mPerDegLng;
    mgr.updateCurrentTime(1000 + step * MS_PER_STEP);
    mgr.loop(TRACK_SOUTH_LAT, lng, ALT, SPEED_KNOTS);
  }

  EXPECT_TRUE(mgr.isLapAnythingActive());
  EXPECT_TRUE(mgr.isDetectionComplete());
  EXPECT_EQ(mgr.getDetectionRejectionCount(), 0);
  EXPECT_EQ(mgr.getDetector()->getNoMatchCount(), 0);
}

// =============================================================================
// H7 — Lap Anything activation stops feeding all course timers
// =============================================================================

void test_lap_anything_deactivates_all_course_timers() {
  TrackConfig cfg = {};
  cfg.longName = "WrongLength";
  cfg.shortName = "WLN";
  cfg.courseCount = 3;
  cfg.courses[0] = matchingCourse("A");
  cfg.courses[1] = matchingCourse("B");
  cfg.courses[2] = matchingCourse("C");
  for (int i = 0; i < 3; i++) cfg.courses[i].lengthFt = 10000.0f;  // never match

  CourseManager mgr(cfg);
  for (int i = 0; i < 3; i++) {
    EXPECT_TRUE(mgr.isCourseTimerActive(i));  // all fed before fallback
  }

  driveLaps(mgr, 5 * STEPS_PER_LAP);

  EXPECT_TRUE(mgr.isLapAnythingActive());
  for (int i = 0; i < 3; i++) {
    EXPECT_FALSE(mgr.isCourseTimerActive(i));  // none fed after fallback
  }
}

// =============================================================================
// pruneInactiveCourses
// =============================================================================

void test_prune_deactivates_non_detected_courses() {
  TrackConfig cfg = {};
  cfg.longName = "TwoLayouts";
  cfg.shortName = "2L";
  cfg.courseCount = 2;
  cfg.courses[0] = matchingCourse("Right Layout");
  cfg.courses[1] = matchingCourse("Wrong Layout");
  cfg.courses[1].lengthFt = 5000.0f;  // outside tolerance, never a candidate

  CourseManager mgr(cfg);
  driveLaps(mgr, STEPS_PER_LAP + 30);
  EXPECT_EQ(mgr.getActiveCourseIndex(), 0);
  EXPECT_TRUE(mgr.isCourseTimerActive(1));  // still fed until pruned

  mgr.pruneInactiveCourses();
  EXPECT_TRUE(mgr.isCourseTimerActive(0));
  EXPECT_FALSE(mgr.isCourseTimerActive(1));
  EXPECT_TRUE(mgr.getActiveTimer() != NULL);
}

// =============================================================================
// reset
// =============================================================================

void test_reset_restores_detection_state() {
  TrackConfig cfg = {};
  cfg.longName = "Synthetic";
  cfg.shortName = "SYN";
  cfg.courseCount = 1;
  cfg.courses[0] = matchingCourse("Square 400m");

  CourseManager mgr(cfg);
  driveLaps(mgr, STEPS_PER_LAP + 30);
  EXPECT_TRUE(mgr.isDetectionComplete());

  mgr.reset();
  EXPECT_FALSE(mgr.isDetectionComplete());
  EXPECT_FALSE(mgr.isLapAnythingActive());
  EXPECT_TRUE(mgr.getActiveTimer() == NULL);
  EXPECT_TRUE(mgr.isCourseTimerActive(0));
  EXPECT_EQ(mgr.getDetector()->getNoMatchCount(), 0);
}

int main() {
  printf("=== CourseManager tests ===\n");

  RUN_TEST(zero_courses_activates_lap_anything_immediately);

  RUN_TEST(detection_accepts_matching_course);
  RUN_TEST(active_timer_keeps_lapping_after_detection);

  RUN_TEST(rejections_fall_back_to_lap_anything);

  RUN_TEST(no_match_config_falls_back_to_lap_anything);
  RUN_TEST(distance_failsafe_falls_back_when_waypoint_never_revisited);

  RUN_TEST(lap_anything_deactivates_all_course_timers);

  RUN_TEST(prune_deactivates_non_detected_courses);

  RUN_TEST(reset_restores_detection_state);

  TEST_SUMMARY();
}
