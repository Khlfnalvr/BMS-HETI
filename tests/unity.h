/**
 * unity.h - Minimal unit test framework for BMS-HETI
 *
 * Usage:
 *   TEST(name)         - declare a test function
 *   RUN_TEST(name)     - run a test and record result
 *   ASSERT_TRUE(cond)  - fail if condition is false
 *   ASSERT_FLOAT_EQ(a, b, tol) - fail if |a-b| > tol
 *   ASSERT_INT_EQ(a, b)        - fail if a != b
 *   PRINT_RESULTS()    - print summary
 */

#ifndef UNITY_H
#define UNITY_H

#include <stdio.h>
#include <math.h>
#include <string.h>

/* ── internal state ─────────────────────────────────────────────────── */
static int _unity_passed = 0;
static int _unity_failed = 0;
static int _unity_aborted = 0;  /* set when current test fails */
static const char *_unity_current_test = "";

/* ── macros ─────────────────────────────────────────────────────────── */

#define TEST(name)  void test_##name(void)

#define RUN_TEST(name) do {                                         \
    _unity_current_test = #name;                                    \
    _unity_aborted = 0;                                             \
    test_##name();                                                  \
    if (!_unity_aborted) {                                          \
        printf("  [PASS] %s\n", #name);                            \
        _unity_passed++;                                            \
    }                                                               \
} while (0)

#define ASSERT_TRUE(cond) do {                                      \
    if (!(cond)) {                                                  \
        printf("  [FAIL] %s  (%s:%d)  condition false: %s\n",      \
               _unity_current_test, __FILE__, __LINE__, #cond);    \
        _unity_failed++;                                            \
        _unity_aborted = 1;                                         \
        return;                                                     \
    }                                                               \
} while (0)

#define ASSERT_FALSE(cond)  ASSERT_TRUE(!(cond))

#define ASSERT_FLOAT_EQ(a, b, tol) do {                             \
    float _a = (float)(a);                                          \
    float _b = (float)(b);                                          \
    float _t = (float)(tol);                                        \
    if (fabsf(_a - _b) > _t) {                                      \
        printf("  [FAIL] %s  (%s:%d)  %.6f != %.6f (tol %.6f)\n",  \
               _unity_current_test, __FILE__, __LINE__,             \
               (double)_a, (double)_b, (double)_t);                \
        _unity_failed++;                                            \
        _unity_aborted = 1;                                         \
        return;                                                     \
    }                                                               \
} while (0)

#define ASSERT_INT_EQ(a, b) do {                                    \
    long long _a = (long long)(a);                                  \
    long long _b = (long long)(b);                                  \
    if (_a != _b) {                                                 \
        printf("  [FAIL] %s  (%s:%d)  %lld != %lld\n",             \
               _unity_current_test, __FILE__, __LINE__, _a, _b);   \
        _unity_failed++;                                            \
        _unity_aborted = 1;                                         \
        return;                                                     \
    }                                                               \
} while (0)

#define ASSERT_FLOAT_GE(a, b) do {                                  \
    float _a = (float)(a);                                          \
    float _b = (float)(b);                                          \
    if (_a < _b) {                                                  \
        printf("  [FAIL] %s  (%s:%d)  %.6f < %.6f\n",              \
               _unity_current_test, __FILE__, __LINE__,             \
               (double)_a, (double)_b);                             \
        _unity_failed++;                                            \
        _unity_aborted = 1;                                         \
        return;                                                     \
    }                                                               \
} while (0)

#define ASSERT_FLOAT_LE(a, b) do {                                  \
    float _a = (float)(a);                                          \
    float _b = (float)(b);                                          \
    if (_a > _b) {                                                  \
        printf("  [FAIL] %s  (%s:%d)  %.6f > %.6f\n",              \
               _unity_current_test, __FILE__, __LINE__,             \
               (double)_a, (double)_b);                             \
        _unity_failed++;                                            \
        _unity_aborted = 1;                                         \
        return;                                                     \
    }                                                               \
} while (0)

#define PRINT_RESULTS() do {                                        \
    int _total = _unity_passed + _unity_failed;                     \
    printf("\n--- Results: %d/%d passed", _unity_passed, _total);   \
    if (_unity_failed) printf(", %d FAILED", _unity_failed);        \
    printf(" ---\n");                                               \
} while (0)

#define RESULTS_OK()  (_unity_failed == 0)

#endif /* UNITY_H */
