/**
 * test_main.c
 * Entry point for the BMS-HETI unit test suite.
 */

#include <stdio.h>
#include <stdlib.h>
#include "test_framework.h"

/* Shared pass/fail counters referenced by test_framework.h macros */
int tests_passed = 0;
int tests_failed = 0;

/* Forward declarations of suite runners */
void run_battery_params_tests(void);
void run_cc_tests(void);
void run_aukf_tests(void);
void run_soc_estimator_tests(void);

int main(void) {
    printf("BMS-HETI Unit Test Suite\n");
    printf("========================\n");

    run_battery_params_tests();
    run_cc_tests();
    run_aukf_tests();
    run_soc_estimator_tests();

    PRINT_RESULTS();

    return (tests_failed == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
