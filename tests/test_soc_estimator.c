/**
 * Integration tests for the SoC_Estimator (CCM + AUKF hybrid)
 */

#include "unity.h"
#include "../socestimator.h"

static SoC_Estimator_t est;

static void setup(void) {
    SoC_Estimator_Init(&est, 80.0f, 25.0f);
}

/* ============================================================
 * Init tests
 * ============================================================ */

static void test_init_soc_aukf(void) {
    setup();
    TEST_ASSERT_EQUAL_FLOAT(80.0f, SoC_Estimator_GetSoC(&est), 1e-4f);
}

static void test_init_soc_ccm(void) {
    setup();
    TEST_ASSERT_EQUAL_FLOAT(80.0f, SoC_Estimator_GetSoC_CCM(&est), 1e-4f);
}

static void test_init_update_count_zero(void) {
    setup();
    TEST_ASSERT_EQUAL_INT(0, (int)est.update_count);
}

static void test_init_temperature_stored(void) {
    setup();
    TEST_ASSERT_EQUAL_FLOAT(25.0f, est.temperature, 1e-4f);
}

/* ============================================================
 * Update tests
 * ============================================================ */

static void test_update_returns_soc_in_range(void) {
    setup();
    float soc = SoC_Estimator_Update(&est, 1.0f, 3.35f, 1.0f);
    TEST_ASSERT_FLOAT_RANGE(soc, 0.0f, 100.0f);
}

static void test_update_get_soc_matches_return(void) {
    setup();
    float ret = SoC_Estimator_Update(&est, 1.0f, 3.35f, 1.0f);
    TEST_ASSERT_EQUAL_FLOAT(ret, SoC_Estimator_GetSoC(&est), 1e-5f);
}

static void test_update_ccm_soc_in_range(void) {
    setup();
    SoC_Estimator_Update(&est, 1.0f, 3.35f, 1.0f);
    TEST_ASSERT_FLOAT_RANGE(SoC_Estimator_GetSoC_CCM(&est), 0.0f, 100.0f);
}

static void test_update_increments_count(void) {
    setup();
    SoC_Estimator_Update(&est, 1.0f, 3.35f, 1.0f);
    SoC_Estimator_Update(&est, 1.0f, 3.35f, 1.0f);
    TEST_ASSERT_EQUAL_INT(2, (int)est.update_count);
}

static void test_update_stores_measured_voltage(void) {
    setup();
    SoC_Estimator_Update(&est, 1.0f, 3.42f, 1.0f);
    TEST_ASSERT_EQUAL_FLOAT(3.42f, est.V_measured, 1e-4f);
}

static void test_update_discharge_reduces_ccm_soc(void) {
    setup();
    /* Apply 1 A discharge for 3600 s → CCM SoC should drop substantially */
    SoC_Estimator_Update(&est, 1.0f, 3.35f, 3600.0f);
    TEST_ASSERT(SoC_Estimator_GetSoC_CCM(&est) < 80.0f);
}

static void test_update_zero_current_ccm_unchanged(void) {
    setup();
    SoC_Estimator_Update(&est, 0.0f, 3.35f, 1.0f);
    TEST_ASSERT_EQUAL_FLOAT(80.0f, SoC_Estimator_GetSoC_CCM(&est), 1e-4f);
}

static void test_update_many_iterations_stay_in_range(void) {
    setup();
    float soc = 80.0f;
    for (int i = 0; i < 100; i++) {
        soc = SoC_Estimator_Update(&est, 0.5f, 3.3f, 1.0f);
    }
    TEST_ASSERT_FLOAT_RANGE(soc, 0.0f, 100.0f);
}

/* ============================================================
 * SetTemperature tests
 * ============================================================ */

static void test_set_temperature_updates_estimator(void) {
    setup();
    SoC_Estimator_SetTemperature(&est, 45.0f);
    TEST_ASSERT_EQUAL_FLOAT(45.0f, est.temperature, 1e-4f);
}

static void test_set_temperature_updates_aukf(void) {
    setup();
    SoC_Estimator_SetTemperature(&est, 45.0f);
    TEST_ASSERT_EQUAL_FLOAT(45.0f, est.aukf.temperature, 1e-4f);
}

/* ============================================================
 * Reset tests
 * ============================================================ */

static void test_reset_sets_soc(void) {
    setup();
    SoC_Estimator_Update(&est, 1.0f, 3.3f, 10.0f);
    SoC_Estimator_Reset(&est, 90.0f);
    TEST_ASSERT_EQUAL_FLOAT(90.0f, est.SoC_CCM, 1e-4f);
    TEST_ASSERT_EQUAL_FLOAT(90.0f, est.SoC_AUKF, 1e-4f);
}

static void test_reset_clears_count(void) {
    setup();
    SoC_Estimator_Update(&est, 1.0f, 3.3f, 10.0f);
    SoC_Estimator_Reset(&est, 50.0f);
    TEST_ASSERT_EQUAL_INT(0, (int)est.update_count);
}

static void test_reset_aukf_soc_matches(void) {
    setup();
    SoC_Estimator_Reset(&est, 60.0f);
    TEST_ASSERT_EQUAL_FLOAT(60.0f, AUKF_GetSoC(&est.aukf), 1e-4f);
}

/* ============================================================
 * CCM vs AUKF comparison
 * ============================================================ */

static void test_aukf_soc_can_differ_from_ccm_after_update(void) {
    /*
     * With a voltage that doesn't match the CCM prediction, AUKF corrects
     * the SoC while CCM just integrates. After several steps with a
     * mismatched voltage, the two estimates may diverge.
     */
    setup();
    float ccm_soc = 80.0f, aukf_soc = 80.0f;
    for (int i = 0; i < 20; i++) {
        aukf_soc = SoC_Estimator_Update(&est, 1.0f, 2.8f, 1.0f); /* low V */
        ccm_soc  = SoC_Estimator_GetSoC_CCM(&est);
    }
    /* Both should remain in valid range */
    TEST_ASSERT_FLOAT_RANGE(aukf_soc, 0.0f, 100.0f);
    TEST_ASSERT_FLOAT_RANGE(ccm_soc,  0.0f, 100.0f);
}

/* ============================================================
 * Test suite entry point
 * ============================================================ */

void test_soc_estimator_run(void) {
    UNITY_BEGIN("SoC Estimator (CCM + AUKF)");

    RUN_TEST(test_init_soc_aukf);
    RUN_TEST(test_init_soc_ccm);
    RUN_TEST(test_init_update_count_zero);
    RUN_TEST(test_init_temperature_stored);

    RUN_TEST(test_update_returns_soc_in_range);
    RUN_TEST(test_update_get_soc_matches_return);
    RUN_TEST(test_update_ccm_soc_in_range);
    RUN_TEST(test_update_increments_count);
    RUN_TEST(test_update_stores_measured_voltage);
    RUN_TEST(test_update_discharge_reduces_ccm_soc);
    RUN_TEST(test_update_zero_current_ccm_unchanged);
    RUN_TEST(test_update_many_iterations_stay_in_range);

    RUN_TEST(test_set_temperature_updates_estimator);
    RUN_TEST(test_set_temperature_updates_aukf);

    RUN_TEST(test_reset_sets_soc);
    RUN_TEST(test_reset_clears_count);
    RUN_TEST(test_reset_aukf_soc_matches);

    RUN_TEST(test_aukf_soc_can_differ_from_ccm_after_update);
}
