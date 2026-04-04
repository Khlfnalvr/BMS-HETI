/**
 * test_soc_estimator.c
 * Unit tests for SOCEstimator.h / SOCEstimator.c
 *
 * Covers:
 *  - SoC_Estimator_Init        : initial SoC stored, update_count = 0
 *  - SoC_Estimator_GetSoC      : returns AUKF SoC
 *  - SoC_Estimator_GetSoC_CCM  : returns CCM SoC after update
 *  - SoC_Estimator_Update      : SoC in valid range, update_count increments,
 *                                CCM and AUKF outputs populated, discharge trend
 *  - SoC_Estimator_SetTemperature : temperature propagated to AUKF
 *  - SoC_Estimator_Reset       : SoC reset, update_count cleared
 */

#include "test_framework.h"
#include "SOCEstimator.h"

/* ── Helper ───────────────────────────────────────────────────────── */

static SoC_Estimator_t make_estimator(float initial_soc, float temp) {
    SoC_Estimator_t est;
    SoC_Estimator_Init(&est, initial_soc, temp);
    return est;
}

/* ── SoC_Estimator_Init ───────────────────────────────────────────── */

static void test_init_soc_aukf_equals_initial(void) {
    SoC_Estimator_t est = make_estimator(70.0f, 25.0f);
    ASSERT_FLOAT_NEAR(70.0f, SoC_Estimator_GetSoC(&est), 1e-5f);
}

static void test_init_update_count_zero(void) {
    SoC_Estimator_t est = make_estimator(50.0f, 25.0f);
    ASSERT_INT_EQ(0, est.update_count);
}

static void test_init_temperature_stored(void) {
    SoC_Estimator_t est = make_estimator(50.0f, 40.0f);
    ASSERT_FLOAT_NEAR(40.0f, est.temperature, 1e-6f);
}

static void test_init_out_of_range_soc_corrected_after_update(void) {
    /* SoC_Estimator_Init does not clamp its SoC_AUKF/SoC_CCM fields directly;
     * the sub-modules (CC, AUKF) clamp internally.  After the first Update the
     * returned SoC must be within [0, 100] regardless of the initial value. */
    SoC_Estimator_t est = make_estimator(-5.0f, 25.0f);
    float soc = SoC_Estimator_Update(&est, 0.0f, 3.0f, 1.0f);
    ASSERT_TRUE(soc >= 0.0f && soc <= 100.0f);
}

static void test_init_above_100_soc_corrected_after_update(void) {
    SoC_Estimator_t est = make_estimator(105.0f, 25.0f);
    float soc = SoC_Estimator_Update(&est, 0.0f, 3.7f, 1.0f);
    ASSERT_TRUE(soc >= 0.0f && soc <= 100.0f);
}

/* ── SoC_Estimator_Update ─────────────────────────────────────────── */

static void test_update_increments_update_count(void) {
    SoC_Estimator_t est = make_estimator(60.0f, 25.0f);
    SoC_Estimator_Update(&est, 1.0f, 3.22f, 1.0f);
    SoC_Estimator_Update(&est, 1.0f, 3.22f, 1.0f);
    ASSERT_INT_EQ(2, est.update_count);
}

static void test_update_returns_soc_in_valid_range(void) {
    SoC_Estimator_t est = make_estimator(50.0f, 25.0f);
    for (int i = 0; i < 10; i++) {
        float soc = SoC_Estimator_Update(&est, 1.0f, 3.15f, 5.0f);
        ASSERT_TRUE(soc >= 0.0f && soc <= 100.0f);
    }
}

static void test_update_returns_same_as_get_soc(void) {
    SoC_Estimator_t est = make_estimator(50.0f, 25.0f);
    float returned = SoC_Estimator_Update(&est, 1.0f, 3.22f, 1.0f);
    ASSERT_FLOAT_NEAR(returned, SoC_Estimator_GetSoC(&est), 1e-6f);
}

static void test_update_populates_ccm_soc(void) {
    SoC_Estimator_t est = make_estimator(50.0f, 25.0f);
    /* After an update the CCM SoC field must be updated */
    SoC_Estimator_Update(&est, 1.0f, 3.22f, 1.0f);
    /* CCM should differ from initial 50 % (tiny but non-zero step) */
    float ccm = SoC_Estimator_GetSoC_CCM(&est);
    ASSERT_TRUE(ccm >= 0.0f && ccm <= 100.0f);
    ASSERT_TRUE(ccm < 50.0f); /* discharge → lower */
}

static void test_update_stores_measured_voltage(void) {
    SoC_Estimator_t est = make_estimator(50.0f, 25.0f);
    SoC_Estimator_Update(&est, 1.0f, 3.30f, 1.0f);
    ASSERT_FLOAT_NEAR(3.30f, est.V_measured, 1e-5f);
}

static void test_update_discharge_decreases_soc(void) {
    /* Run 60 steps of 10 s each at 1 A discharge → SoC should fall */
    SoC_Estimator_t est = make_estimator(80.0f, 25.0f);
    for (int i = 0; i < 60; i++) {
        SoC_Estimator_Update(&est, 1.0f, 3.15f, 10.0f);
    }
    ASSERT_TRUE(SoC_Estimator_GetSoC(&est) < 80.0f);
}

static void test_update_charge_increases_soc(void) {
    /* Charge from 30 % */
    SoC_Estimator_t est = make_estimator(30.0f, 25.0f);
    for (int i = 0; i < 60; i++) {
        SoC_Estimator_Update(&est, -1.0f, 3.5f, 10.0f);
    }
    ASSERT_TRUE(SoC_Estimator_GetSoC(&est) > 30.0f);
}

static void test_update_zero_current_soc_stable(void) {
    /* No current → CCM SoC stays put; AUKF may drift slightly but
     * the getter must remain within [0, 100]. */
    SoC_Estimator_t est = make_estimator(50.0f, 25.0f);
    SoC_Estimator_Update(&est, 0.0f, 3.22f, 1.0f);
    float ccm = SoC_Estimator_GetSoC_CCM(&est);
    ASSERT_FLOAT_NEAR(50.0f, ccm, 1e-5f);
}

/* ── SoC_Estimator_SetTemperature ─────────────────────────────────── */

static void test_set_temperature_updates_estimator(void) {
    SoC_Estimator_t est = make_estimator(50.0f, 25.0f);
    SoC_Estimator_SetTemperature(&est, 45.0f);
    ASSERT_FLOAT_NEAR(45.0f, est.temperature, 1e-6f);
}

static void test_set_temperature_propagates_to_aukf(void) {
    SoC_Estimator_t est = make_estimator(50.0f, 25.0f);
    SoC_Estimator_SetTemperature(&est, 45.0f);
    ASSERT_FLOAT_NEAR(45.0f, est.aukf.temperature, 1e-6f);
}

/* ── SoC_Estimator_Reset ──────────────────────────────────────────── */

static void test_reset_sets_new_soc(void) {
    SoC_Estimator_t est = make_estimator(80.0f, 25.0f);
    SoC_Estimator_Update(&est, 1.0f, 3.15f, 10.0f);
    SoC_Estimator_Reset(&est, 50.0f);
    ASSERT_FLOAT_NEAR(50.0f, SoC_Estimator_GetSoC(&est), 1e-5f);
}

static void test_reset_clears_update_count(void) {
    SoC_Estimator_t est = make_estimator(80.0f, 25.0f);
    SoC_Estimator_Update(&est, 1.0f, 3.15f, 10.0f);
    SoC_Estimator_Reset(&est, 50.0f);
    ASSERT_INT_EQ(0, est.update_count);
}

static void test_reset_soc_ccm_updated(void) {
    SoC_Estimator_t est = make_estimator(80.0f, 25.0f);
    SoC_Estimator_Update(&est, 1.0f, 3.15f, 10.0f);
    SoC_Estimator_Reset(&est, 55.0f);
    ASSERT_FLOAT_NEAR(55.0f, est.SoC_CCM, 1e-5f);
}

/* ── Suite entry point ────────────────────────────────────────────── */

TEST_SUITE_BEGIN(soc_estimator)
    RUN_TEST(test_init_soc_aukf_equals_initial);
    RUN_TEST(test_init_update_count_zero);
    RUN_TEST(test_init_temperature_stored);
    RUN_TEST(test_init_out_of_range_soc_corrected_after_update);
    RUN_TEST(test_init_above_100_soc_corrected_after_update);

    RUN_TEST(test_update_increments_update_count);
    RUN_TEST(test_update_returns_soc_in_valid_range);
    RUN_TEST(test_update_returns_same_as_get_soc);
    RUN_TEST(test_update_populates_ccm_soc);
    RUN_TEST(test_update_stores_measured_voltage);
    RUN_TEST(test_update_discharge_decreases_soc);
    RUN_TEST(test_update_charge_increases_soc);
    RUN_TEST(test_update_zero_current_soc_stable);

    RUN_TEST(test_set_temperature_updates_estimator);
    RUN_TEST(test_set_temperature_propagates_to_aukf);

    RUN_TEST(test_reset_sets_new_soc);
    RUN_TEST(test_reset_clears_update_count);
    RUN_TEST(test_reset_soc_ccm_updated);
TEST_SUITE_END()
