/**
 * Minimal Unity-like test framework for BMS-HETI unit tests
 */

#ifndef UNITY_H
#define UNITY_H

#include <stdio.h>
#include <math.h>
#include <string.h>

/* ============================================================
 * Global counters
 * ============================================================ */

extern int unity_tests_run;
extern int unity_tests_passed;
extern int unity_tests_failed;
extern const char *unity_current_test;

/* ============================================================
 * Assertion macros
 * ============================================================ */

#define TEST_ASSERT(cond) \
    do { \
        unity_tests_run++; \
        if (cond) { \
            unity_tests_passed++; \
        } else { \
            unity_tests_failed++; \
            printf("  FAIL: %s:%d  (%s)\n", __FILE__, __LINE__, #cond); \
        } \
    } while (0)

#define TEST_ASSERT_EQUAL_FLOAT(expected, actual, tol) \
    do { \
        unity_tests_run++; \
        float _e = (float)(expected); \
        float _a = (float)(actual); \
        if (fabsf(_e - _a) <= (float)(tol)) { \
            unity_tests_passed++; \
        } else { \
            unity_tests_failed++; \
            printf("  FAIL: %s:%d  expected %.6f, got %.6f (tol %.6f)\n", \
                   __FILE__, __LINE__, (double)_e, (double)_a, (double)(tol)); \
        } \
    } while (0)

#define TEST_ASSERT_FLOAT_RANGE(val, lo, hi) \
    do { \
        unity_tests_run++; \
        float _v = (float)(val); \
        if (_v >= (float)(lo) && _v <= (float)(hi)) { \
            unity_tests_passed++; \
        } else { \
            unity_tests_failed++; \
            printf("  FAIL: %s:%d  %.6f not in [%.6f, %.6f]\n", \
                   __FILE__, __LINE__, (double)_v, (double)(lo), (double)(hi)); \
        } \
    } while (0)

#define TEST_ASSERT_EQUAL_INT(expected, actual) \
    do { \
        unity_tests_run++; \
        int _e = (int)(expected); \
        int _a = (int)(actual); \
        if (_e == _a) { \
            unity_tests_passed++; \
        } else { \
            unity_tests_failed++; \
            printf("  FAIL: %s:%d  expected %d, got %d\n", \
                   __FILE__, __LINE__, _e, _a); \
        } \
    } while (0)

/* ============================================================
 * Test suite helpers
 * ============================================================ */

#define RUN_TEST(fn) \
    do { \
        unity_current_test = #fn; \
        printf("  [ ] %s\n", unity_current_test); \
        int _before_fail = unity_tests_failed; \
        fn(); \
        if (unity_tests_failed == _before_fail) \
            printf("  [PASS] %s\n", unity_current_test); \
    } while (0)

#define UNITY_BEGIN(suite_name) \
    printf("\n=== %s ===\n", suite_name)

#define UNITY_END() \
    printf("\nResults: %d/%d passed, %d failed\n", \
           unity_tests_passed, unity_tests_run, unity_tests_failed)

#endif /* UNITY_H */
