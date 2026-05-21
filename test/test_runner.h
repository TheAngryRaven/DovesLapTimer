/**
 * Minimal test runner — hand-rolled, no framework dependency.
 *
 * Each test file defines plain functions and lists them in main() via
 * RUN_TEST(name). EXPECT_* macros record failures; main() returns the
 * failure count so make / CI can detect a red run.
 */

#ifndef TEST_RUNNER_H
#define TEST_RUNNER_H

#include <cstdio>
#include <cstdlib>
#include <cmath>

static int g_tests_run = 0;
static int g_tests_failed = 0;
static int g_current_test_failures = 0;

#define RUN_TEST(name)                                          \
  do {                                                          \
    g_tests_run++;                                              \
    g_current_test_failures = 0;                                \
    printf("  %-50s ", #name);                                  \
    fflush(stdout);                                             \
    test_##name();                                              \
    if (g_current_test_failures > 0) {                          \
      printf(" FAIL\n");                                        \
      g_tests_failed++;                                         \
    } else {                                                    \
      printf(" ok\n");                                          \
    }                                                           \
  } while (0)

#define EXPECT_TRUE(cond)                                       \
  do {                                                          \
    if (!(cond)) {                                              \
      printf("\n    EXPECT_TRUE failed: %s  (line %d)",         \
             #cond, __LINE__);                                  \
      g_current_test_failures++;                                \
    }                                                           \
  } while (0)

#define EXPECT_FALSE(cond) EXPECT_TRUE(!(cond))

#define EXPECT_EQ(a, b)                                         \
  do {                                                          \
    auto _aval = (a);                                           \
    auto _bval = (b);                                           \
    if (!(_aval == _bval)) {                                    \
      printf("\n    EXPECT_EQ failed: %s == %s  (line %d)",     \
             #a, #b, __LINE__);                                 \
      g_current_test_failures++;                                \
    }                                                           \
  } while (0)

#define EXPECT_NEAR(a, b, tol)                                  \
  do {                                                          \
    double _aval = (double)(a);                                 \
    double _bval = (double)(b);                                 \
    double _tval = (double)(tol);                               \
    if (std::fabs(_aval - _bval) > _tval) {                     \
      printf("\n    EXPECT_NEAR failed: |%g - %g| > %g  (line %d)",\
             _aval, _bval, _tval, __LINE__);                    \
      g_current_test_failures++;                                \
    }                                                           \
  } while (0)

#define TEST_SUMMARY()                                          \
  do {                                                          \
    printf("\n%d tests run, %d failed.\n",                      \
           g_tests_run, g_tests_failed);                        \
    return g_tests_failed > 0 ? 1 : 0;                          \
  } while (0)

#endif
