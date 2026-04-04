/**
 * test_framework.h
 * Minimal unit-test framework for BMS-HETI
 */

#ifndef TEST_FRAMEWORK_H
#define TEST_FRAMEWORK_H

#include <stdio.h>
#include <math.h>

/* Shared counters – defined once in test_main.c */
extern int tests_passed;
extern int tests_failed;

/* ── Assertions ─────────────────────────────────────────────────── */

#define ASSERT_TRUE(cond)                                              \
    do {                                                               \
        if (cond) {                                                    \
            tests_passed++;                                            \
        } else {                                                       \
            tests_failed++;                                            \
            printf("  FAIL [%s:%d] condition is false\n",             \
                   __FILE__, __LINE__);                                \
        }                                                              \
    } while (0)

#define ASSERT_FALSE(cond)  ASSERT_TRUE(!(cond))

#define ASSERT_INT_EQ(expected, actual)                                \
    do {                                                               \
        long long _e = (long long)(expected);                          \
        long long _a = (long long)(actual);                            \
        if (_e == _a) {                                                \
            tests_passed++;                                            \
        } else {                                                       \
            tests_failed++;                                            \
            printf("  FAIL [%s:%d] expected %lld, got %lld\n",        \
                   __FILE__, __LINE__, _e, _a);                        \
        }                                                              \
    } while (0)

/**
 * Floating-point equality within an absolute tolerance.
 */
#define ASSERT_FLOAT_NEAR(expected, actual, tol)                       \
    do {                                                               \
        float _e = (float)(expected);                                  \
        float _a = (float)(actual);                                    \
        float _t = (float)(tol);                                       \
        if (fabsf(_e - _a) <= _t) {                                    \
            tests_passed++;                                            \
        } else {                                                       \
            tests_failed++;                                            \
            printf("  FAIL [%s:%d] expected %.6f, got %.6f "          \
                   "(diff %.6f > tol %.6f)\n",                         \
                   __FILE__, __LINE__,                                 \
                   (double)_e, (double)_a,                             \
                   (double)fabsf(_e - _a), (double)_t);                \
        }                                                              \
    } while (0)

/* ── Test runner helper ─────────────────────────────────────────── */

#define RUN_TEST(fn)                                                   \
    do {                                                               \
        printf("  running %-45s ", #fn);                              \
        int _before = tests_failed;                                    \
        fn();                                                          \
        if (tests_failed == _before)                                   \
            printf("OK\n");                                            \
    } while (0)

#define TEST_SUITE_BEGIN(name)                                         \
    void run_##name##_tests(void) {                                    \
        printf("\n[Suite] %s\n", #name);

#define TEST_SUITE_END() }

/* ── Summary ────────────────────────────────────────────────────── */

#define PRINT_RESULTS()                                                \
    do {                                                               \
        int _total = tests_passed + tests_failed;                      \
        printf("\n=========================================\n");        \
        printf(" Results: %d / %d passed", tests_passed, _total);     \
        if (tests_failed == 0)                                         \
            printf("  -- ALL PASSED\n");                               \
        else                                                           \
            printf("  -- %d FAILED\n", tests_failed);                  \
        printf("=========================================\n");         \
    } while (0)

#endif /* TEST_FRAMEWORK_H */
