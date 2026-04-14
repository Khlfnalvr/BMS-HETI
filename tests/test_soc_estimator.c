/**
 * test_soc_estimator.c
 * Unit tests for the SoC_Estimator module (SOCEstimator.c).
 *
 * The estimator is the integration layer that chains:
 *   CoulombCounter → AUKF
 * Tests focus on observable API behaviour rather than internal filter math.
 */

#include "test_framework.h"
#include "../SOCEstimator.h"

/* ── Shared fixture ───────────────────────────────────────────────────────── */

static SoC_Estimator_t est;

static void setup(float soc, float temp) {
    SoC_Estimator_Init(&est, soc, temp);
}

/* ── Init tests ───────────────────────────────────────────────────────────── */

static void test_init_soc_ccm_set(void) {
    setup(60.0f, 25.0f);
    TEST_ASSERT_EQUAL_FLOAT(60.0f, est.SoC_CCM);
}

static void test_init_soc_aukf_set(void) {
    setup(60.0f, 25.0f);
    TEST_ASSERT_EQUAL_FLOAT(60.0f, est.SoC_AUKF);
}

static void test_init_temperature_stored(void) {
    setup(50.0f, 45.0f);
    TEST_ASSERT_EQUAL_FLOAT(45.0f, est.temperature);
}

static void test_init_update_count_zero(void) {
    setup(50.0f, 25.0f);
    TEST_ASSERT_EQUAL_INT(0, (int)est.update_count);
}

static void test_init_v_measured_zero(void) {
    setup(50.0f, 25.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, est.V_measured);
}

/* ── Getter tests ─────────────────────────────────────────────────────────── */

static void test_get_soc_returns_aukf_soc(void) {
    setup(75.0f, 25.0f);
    TEST_ASSERT_EQUAL_FLOAT(est.SoC_AUKF, SoC_Estimator_GetSoC(&est));
}

static void test_get_soc_ccm_returns_ccm_soc(void) {
    setup(75.0f, 25.0f);
    TEST_ASSERT_EQUAL_FLOAT(est.SoC_CCM, SoC_Estimator_GetSoC_CCM(&est));
}

/* ── Update tests ─────────────────────────────────────────────────────────── */

static void test_update_returns_valid_range(void) {
    setup(50.0f, 25.0f);
    float soc = SoC_Estimator_Update(&est, 1.0f, 3.22f, 1.0f);
    TEST_ASSERT_FLOAT_BETWEEN(0.0f, 100.0f, soc);
}

static void test_update_increments_count(void) {
    setup(50.0f, 25.0f);
    SoC_Estimator_Update(&est, 1.0f, 3.22f, 1.0f);
    SoC_Estimator_Update(&est, 1.0f, 3.22f, 1.0f);
    TEST_ASSERT_EQUAL_INT(2, (int)est.update_count);
}

static void test_update_stores_measured_voltage(void) {
    setup(50.0f, 25.0f);
    SoC_Estimator_Update(&est, 1.0f, 3.35f, 1.0f);
    TEST_ASSERT_EQUAL_FLOAT(3.35f, est.V_measured);
}

static void test_update_returns_same_as_get_soc(void) {
    setup(50.0f, 25.0f);
    float returned = SoC_Estimator_Update(&est, 1.0f, 3.22f, 1.0f);
    TEST_ASSERT_EQUAL_FLOAT(returned, SoC_Estimator_GetSoC(&est));
}

static void test_update_ccm_soc_decreases_on_discharge(void) {
    /* Large dt so CCM change is visible */
    setup(80.0f, 25.0f);
    SoC_Estimator_Update(&est, 1.0f, 3.22f, 360.0f);
    TEST_ASSERT_TRUE(est.SoC_CCM < 80.0f);
}

static void test_update_soc_stays_in_range_long_run(void) {
    setup(80.0f, 25.0f);
    float v = 3.4f, soc_ccm = 80.0f;
    for (int i = 0; i < 30; i++) {
        soc_ccm -= 1.0f;
        if (soc_ccm < 0.0f) soc_ccm = 0.0f;
        float soc = SoC_Estimator_Update(&est, 1.0f, v, 1.0f);
        TEST_ASSERT_FLOAT_BETWEEN(0.0f, 100.0f, soc);
    }
}

static void test_update_zero_current_soc_unchanged_ccm(void) {
    setup(50.0f, 25.0f);
    float ccm_before = est.SoC_CCM;
    SoC_Estimator_Update(&est, 0.0f, 3.22f, 1.0f);
    TEST_ASSERT_EQUAL_FLOAT(ccm_before, est.SoC_CCM);
}

/* ── SetTemperature tests ─────────────────────────────────────────────────── */

static void test_set_temperature_updates_estimator(void) {
    setup(50.0f, 25.0f);
    SoC_Estimator_SetTemperature(&est, 40.0f);
    TEST_ASSERT_EQUAL_FLOAT(40.0f, est.temperature);
}

static void test_set_temperature_propagates_to_aukf(void) {
    setup(50.0f, 25.0f);
    SoC_Estimator_SetTemperature(&est, 40.0f);
    TEST_ASSERT_EQUAL_FLOAT(40.0f, est.aukf.temperature);
}

/* ── Reset tests ──────────────────────────────────────────────────────────── */

static void test_reset_sets_new_soc_ccm(void) {
    setup(50.0f, 25.0f);
    SoC_Estimator_Update(&est, 1.0f, 3.22f, 1.0f);
    SoC_Estimator_Reset(&est, 90.0f);
    TEST_ASSERT_EQUAL_FLOAT(90.0f, est.SoC_CCM);
}

static void test_reset_sets_new_soc_aukf(void) {
    setup(50.0f, 25.0f);
    SoC_Estimator_Update(&est, 1.0f, 3.22f, 1.0f);
    SoC_Estimator_Reset(&est, 90.0f);
    TEST_ASSERT_EQUAL_FLOAT(90.0f, est.SoC_AUKF);
}

static void test_reset_clears_update_count(void) {
    setup(50.0f, 25.0f);
    SoC_Estimator_Update(&est, 1.0f, 3.22f, 1.0f);
    SoC_Estimator_Update(&est, 1.0f, 3.22f, 1.0f);
    SoC_Estimator_Reset(&est, 50.0f);
    TEST_ASSERT_EQUAL_INT(0, (int)est.update_count);
}

static void test_reset_then_update_returns_valid(void) {
    setup(50.0f, 25.0f);
    SoC_Estimator_Reset(&est, 70.0f);
    float soc = SoC_Estimator_Update(&est, 1.0f, 3.3f, 1.0f);
    TEST_ASSERT_FLOAT_BETWEEN(0.0f, 100.0f, soc);
}

/* ── Main ─────────────────────────────────────────────────────────────────── */

int main(void) {
    TEST_SUITE_BEGIN("SoC_Estimator");

    RUN_TEST(test_init_soc_ccm_set);
    RUN_TEST(test_init_soc_aukf_set);
    RUN_TEST(test_init_temperature_stored);
    RUN_TEST(test_init_update_count_zero);
    RUN_TEST(test_init_v_measured_zero);

    RUN_TEST(test_get_soc_returns_aukf_soc);
    RUN_TEST(test_get_soc_ccm_returns_ccm_soc);

    RUN_TEST(test_update_returns_valid_range);
    RUN_TEST(test_update_increments_count);
    RUN_TEST(test_update_stores_measured_voltage);
    RUN_TEST(test_update_returns_same_as_get_soc);
    RUN_TEST(test_update_ccm_soc_decreases_on_discharge);
    RUN_TEST(test_update_soc_stays_in_range_long_run);
    RUN_TEST(test_update_zero_current_soc_unchanged_ccm);

    RUN_TEST(test_set_temperature_updates_estimator);
    RUN_TEST(test_set_temperature_propagates_to_aukf);

    RUN_TEST(test_reset_sets_new_soc_ccm);
    RUN_TEST(test_reset_sets_new_soc_aukf);
    RUN_TEST(test_reset_clears_update_count);
    RUN_TEST(test_reset_then_update_returns_valid);

    TEST_SUITE_END();
    return 0;
}
