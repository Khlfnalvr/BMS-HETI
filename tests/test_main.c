/**
 * Test runner — executes all BMS-HETI unit test suites
 */

#include <stdio.h>
#include "unity.h"

/* Forward declarations for each suite */
void test_cc_run(void);
void test_battery_params_run(void);
void test_aukf_run(void);
void test_soc_estimator_run(void);

int main(void) {
    printf("========================================\n");
    printf("  BMS-HETI Unit Test Suite\n");
    printf("========================================\n");

    test_cc_run();
    test_battery_params_run();
    test_aukf_run();
    test_soc_estimator_run();

    printf("\n========================================\n");
    UNITY_END();
    printf("========================================\n");

    return (unity_tests_failed > 0) ? 1 : 0;
}
