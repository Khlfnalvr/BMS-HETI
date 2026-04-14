/**
 * test_framework.h
 * Minimal C unit test framework for BMS-HETI tests.
 */

#ifndef TEST_FRAMEWORK_H
#define TEST_FRAMEWORK_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

static int _pass_count = 0;
static int _fail_count = 0;

/* ── Assertion macros ─────────────────────────────────────────────────────── */

#define TEST_FAIL_MSG(msg, ...) do {                                            \
    printf(" FAIL\n    [%s:%d] " msg "\n", __FILE__, __LINE__, ##__VA_ARGS__);  \
    _fail_count++;                                                               \
    return;                                                                      \
} while (0)

#define TEST_ASSERT_TRUE(cond) do {                                             \
    if (!(cond))                                                                 \
        TEST_FAIL_MSG("Expected TRUE: %s", #cond);                              \
} while (0)

#define TEST_ASSERT_FALSE(cond) do {                                            \
    if (cond)                                                                    \
        TEST_FAIL_MSG("Expected FALSE: %s", #cond);                             \
} while (0)

#define TEST_ASSERT_EQUAL_INT(expected, actual) do {                            \
    int _e = (int)(expected), _a = (int)(actual);                               \
    if (_e != _a)                                                                \
        TEST_FAIL_MSG("Expected %d, got %d", _e, _a);                           \
} while (0)

#define TEST_ASSERT_FLOAT_WITHIN(delta, expected, actual) do {                  \
    float _e = (float)(expected), _a = (float)(actual), _d = (float)(delta);   \
    float _diff = fabsf(_a - _e);                                               \
    if (_diff > _d)                                                              \
        TEST_FAIL_MSG("Expected %.6f ±%.6f, got %.6f (diff=%.6f)",             \
                      (double)_e, (double)_d, (double)_a, (double)_diff);       \
} while (0)

#define TEST_ASSERT_EQUAL_FLOAT(expected, actual)                               \
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, expected, actual)

#define TEST_ASSERT_FLOAT_BETWEEN(lo, hi, actual) do {                          \
    float _lo = (float)(lo), _hi = (float)(hi), _a = (float)(actual);          \
    if (_a < _lo || _a > _hi)                                                   \
        TEST_FAIL_MSG("Expected [%.4f, %.4f], got %.6f",                        \
                      (double)_lo, (double)_hi, (double)_a);                    \
} while (0)

/* ── Test runner ──────────────────────────────────────────────────────────── */

#define RUN_TEST(fn) do {                                                        \
    printf("  %-50s", #fn);                                                      \
    fflush(stdout);                                                              \
    int _prev = _fail_count;                                                     \
    fn();                                                                        \
    if (_fail_count == _prev) { printf(" PASS\n"); _pass_count++; }             \
} while (0)

#define TEST_SUITE_BEGIN(name) \
    printf("\n=== %s ===\n", name)

#define TEST_SUITE_END() do {                                                    \
    printf("\nTotal: %d passed, %d failed\n", _pass_count, _fail_count);        \
    if (_fail_count > 0) exit(1);                                                \
} while (0)

#endif /* TEST_FRAMEWORK_H */
