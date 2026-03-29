/**
 * Minimal test framework for BMS-HETI unit tests
 */

#ifndef TEST_FRAMEWORK_H
#define TEST_FRAMEWORK_H

#include <stdio.h>
#include <math.h>

static int test_pass_count = 0;
static int test_fail_count = 0;

#define ASSERT_TRUE(cond) \
    do { \
        if (cond) { \
            printf("  PASS: %s\n", #cond); \
            test_pass_count++; \
        } else { \
            printf("  FAIL: %s  (line %d)\n", #cond, __LINE__); \
            test_fail_count++; \
        } \
    } while (0)

#define ASSERT_FALSE(cond) ASSERT_TRUE(!(cond))

#define ASSERT_FLOAT_EQ(a, b, eps) \
    do { \
        float _a = (float)(a); \
        float _b = (float)(b); \
        if (fabsf(_a - _b) <= (float)(eps)) { \
            printf("  PASS: %s == %s  (%.6f)\n", #a, #b, _a); \
            test_pass_count++; \
        } else { \
            printf("  FAIL: %s == %s  (%.6f != %.6f, diff=%.6f)  (line %d)\n", \
                   #a, #b, _a, _b, fabsf(_a - _b), __LINE__); \
            test_fail_count++; \
        } \
    } while (0)

#define ASSERT_INT_EQ(a, b) \
    do { \
        int _a = (int)(a); \
        int _b = (int)(b); \
        if (_a == _b) { \
            printf("  PASS: %s == %s  (%d)\n", #a, #b, _a); \
            test_pass_count++; \
        } else { \
            printf("  FAIL: %s == %s  (%d != %d)  (line %d)\n", \
                   #a, #b, _a, _b, __LINE__); \
            test_fail_count++; \
        } \
    } while (0)

#define ASSERT_FLOAT_GE(a, b) ASSERT_TRUE((float)(a) >= (float)(b))
#define ASSERT_FLOAT_LE(a, b) ASSERT_TRUE((float)(a) <= (float)(b))

#define RUN_TEST(fn) \
    do { \
        printf("\n[TEST] %s\n", #fn); \
        fn(); \
    } while (0)

#define PRINT_RESULTS() \
    do { \
        printf("\n========================================\n"); \
        printf("Results: %d passed, %d failed\n", test_pass_count, test_fail_count); \
        printf("========================================\n"); \
    } while (0)

#define RETURN_TEST_STATUS() (test_fail_count > 0 ? 1 : 0)

#endif /* TEST_FRAMEWORK_H */
