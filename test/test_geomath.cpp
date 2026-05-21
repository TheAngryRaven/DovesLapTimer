/**
 * Layer-2 tests for GeoMath.h (haversine + haversine3D).
 *
 * Pure math, no Arduino dependencies — proves the test infrastructure
 * works before pulling in heavier modules.
 */

#include "test_runner.h"
#include "../src/GeoMath.h"

// =============================================================================
// haversine — great-circle distance between two lat/lng points (meters)
// =============================================================================

void test_haversine_zero_distance() {
  double d = geoHaversine(40.7128, -74.0060, 40.7128, -74.0060);
  EXPECT_NEAR(d, 0.0, 0.001);
}

void test_haversine_nyc_to_la() {
  // NYC (40.7128, -74.0060) -> LA (34.0522, -118.2437)
  // Reference distance: ~3935.75 km
  double d = geoHaversine(40.7128, -74.0060, 34.0522, -118.2437);
  EXPECT_NEAR(d, 3935746.0, 5000.0);  // ±5km tolerance
}

void test_haversine_one_degree_latitude_at_equator() {
  // 1° of latitude is ~111 km regardless of longitude
  double d = geoHaversine(0.0, 0.0, 1.0, 0.0);
  EXPECT_NEAR(d, 111195.0, 100.0);  // ±100m
}

void test_haversine_one_degree_longitude_at_equator() {
  // At the equator, 1° of longitude is also ~111 km
  double d = geoHaversine(0.0, 0.0, 0.0, 1.0);
  EXPECT_NEAR(d, 111195.0, 100.0);
}

void test_haversine_one_degree_longitude_at_60deg_lat() {
  // At 60° latitude, 1° of longitude is ~55.6 km (cos(60°) = 0.5)
  double d = geoHaversine(60.0, 0.0, 60.0, 1.0);
  EXPECT_NEAR(d, 55597.0, 100.0);
}

void test_haversine_small_distance_kart_scale() {
  // 10m apart at Orlando Kart Center latitude.
  // 0.0001° latitude ~= 11.1m.
  double d = geoHaversine(28.4127, -81.3797, 28.4128, -81.3797);
  EXPECT_NEAR(d, 11.12, 0.1);  // sub-meter precision
}

void test_haversine_symmetric() {
  // distance(A, B) == distance(B, A)
  double d1 = geoHaversine(28.4127, -81.3797, 28.4136, -81.3787);
  double d2 = geoHaversine(28.4136, -81.3787, 28.4127, -81.3797);
  EXPECT_NEAR(d1, d2, 1e-9);
}

// =============================================================================
// haversine3D — distance including altitude delta (Pythagorean composition)
// =============================================================================

void test_haversine3d_zero_altitude_matches_2d() {
  // Same altitude on both ends -> identical to haversine
  double d3d = geoHaversine3D(28.4127, -81.3797, 50.0,
                              28.4128, -81.3797, 50.0);
  double d2d = geoHaversine(28.4127, -81.3797, 28.4128, -81.3797);
  EXPECT_NEAR(d3d, d2d, 0.001);
}

void test_haversine3d_pure_vertical() {
  // Same horizontal position, 10m altitude difference -> distance is 10m
  double d = geoHaversine3D(28.4127, -81.3797, 50.0,
                            28.4127, -81.3797, 60.0);
  EXPECT_NEAR(d, 10.0, 0.001);
}

void test_haversine3d_diagonal() {
  // 11.12m horizontal + 5m vertical -> sqrt(11.12² + 5²) ≈ 12.19m
  double d = geoHaversine3D(28.4127, -81.3797, 50.0,
                            28.4128, -81.3797, 55.0);
  double expected = std::sqrt(11.12 * 11.12 + 5.0 * 5.0);
  EXPECT_NEAR(d, expected, 0.1);
}

void test_haversine3d_altitude_drop_same_distance_as_climb() {
  // sign of altitude delta shouldn't matter (squared in formula)
  double climb = geoHaversine3D(28.4127, -81.3797, 50.0,
                                28.4128, -81.3797, 60.0);
  double drop  = geoHaversine3D(28.4127, -81.3797, 60.0,
                                28.4128, -81.3797, 50.0);
  EXPECT_NEAR(climb, drop, 1e-9);
}

int main() {
  printf("=== GeoMath tests ===\n");

  RUN_TEST(haversine_zero_distance);
  RUN_TEST(haversine_nyc_to_la);
  RUN_TEST(haversine_one_degree_latitude_at_equator);
  RUN_TEST(haversine_one_degree_longitude_at_equator);
  RUN_TEST(haversine_one_degree_longitude_at_60deg_lat);
  RUN_TEST(haversine_small_distance_kart_scale);
  RUN_TEST(haversine_symmetric);

  RUN_TEST(haversine3d_zero_altitude_matches_2d);
  RUN_TEST(haversine3d_pure_vertical);
  RUN_TEST(haversine3d_diagonal);
  RUN_TEST(haversine3d_altitude_drop_same_distance_as_climb);

  TEST_SUMMARY();
}
