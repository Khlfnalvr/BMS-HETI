/**
 * Unit tests for the Adaptive Unscented Kalman Filter (AUKF) module
 */

#include "unity.h"
#include "../aukf.h"
#include "../Bateryparameter.h"

static AUKF_t aukf;
static BatteryParams_t battery;

static void setup(void) {
    BatteryParams_Init(&battery);
    AUKF_Init(&aukf, &battery, 50.0f, 25.0f);
}

/* ============================================================
 * Init tests
 * ============================================================ */

static void test_init_sets_soc(void) {
    setup();
    TEST_ASSERT_EQUAL_FLOAT(50.0f, AUKF_GetSoC(&aukf), 1e-4f);
}

static void test_init_sets_vtr_zero(void) {
    setup();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, AUKF_GetVtr(&aukf), 1e-6f);
}

static void test_init_clamps_soc_above_100(void) {
    BatteryParams_Init(&battery);
    AUKF_Init(&aukf, &battery, 120.0f, 25.0f);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, AUKF_GetSoC(&aukf), 1e-4f);
}

static void test_init_clamps_soc_below_0(void) {
    BatteryParams_Init(&battery);
    AUKF_Init(&aukf, &battery, -10.0f, 25.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, AUKF_GetSoC(&aukf), 1e-4f);
}

static void test_init_resets_update_count(void) {
    setup();
    TEST_ASSERT_EQUAL_INT(0, (int)aukf.update_count);
}

static void test_init_positive_covariance_diagonal(void) {
    setup();
    TEST_ASSERT(aukf.P[0][0] > 0.0f);
    TEST_ASSERT(aukf.P[1][1] > 0.0f);
}

static void test_init_positive_Q_diagonal(void) {
    setup();
    TEST_ASSERT(aukf.Q[0][0] > 0.0f);
    TEST_ASSERT(aukf.Q[1][1] > 0.0f);
}

static void test_init_positive_R(void) {
    setup();
    TEST_ASSERT(aukf.R > 0.0f);
}

/* ============================================================
 * Update output range tests
 * ============================================================ */

static void test_update_returns_soc_in_range(void) {
    setup();
    float soc = AUKF_Update(&aukf, 50.0f, 1.0f, 3.2f, 1.0f);
    TEST_ASSERT_FLOAT_RANGE(soc, 0.0f, 100.0f);
}

static void test_update_increments_count(void) {
    setup();
    AUKF_Update(&aukf, 50.0f, 1.0f, 3.2f, 1.0f);
    AUKF_Update(&aukf, 49.0f, 1.0f, 3.2f, 1.0f);
    TEST_ASSERT_EQUAL_INT(2, (int)aukf.update_count);
}

static void test_update_get_soc_matches_return(void) {
    setup();
    float ret = AUKF_Update(&aukf, 50.0f, 1.0f, 3.3f, 1.0f);
    TEST_ASSERT_EQUAL_FLOAT(ret, AUKF_GetSoC(&aukf), 1e-5f);
}

static void test_update_zero_current_soc_in_range(void) {
    setup();
    float soc = AUKF_Update(&aukf, 50.0f, 0.0f, 3.22f, 1.0f);
    TEST_ASSERT_FLOAT_RANGE(soc, 0.0f, 100.0f);
}

static void test_update_many_iterations_stay_in_range(void) {
    setup();
    float soc_pred = 80.0f;
    float soc = 0.0f;
    for (int i = 0; i < 50; i++) {
        soc = AUKF_Update(&aukf, soc_pred, 1.0f, 3.35f, 1.0f);
        soc_pred = soc - (100.0f * 1.0f * 1.0f) / (3600.0f * 2.6f);
        if (soc_pred < 0.0f) soc_pred = 0.0f;
    }
    TEST_ASSERT_FLOAT_RANGE(soc, 0.0f, 100.0f);
}

/* ============================================================
 * Innovation / correction behavior tests
 * ============================================================ */

static void test_update_innovation_is_nonzero_with_mismatch(void) {
    setup();
    /* Use a voltage far from the model prediction to force a large innovation */
    AUKF_Update(&aukf, 50.0f, 1.0f, 2.5f, 1.0f);
    /* Innovation should be non-zero when measured != predicted */
    float innov = AUKF_GetInnovation(&aukf);
    TEST_ASSERT(innov != 0.0f);
}

static void test_update_covariance_stays_positive_definite(void) {
    setup();
    for (int i = 0; i < 10; i++) {
        AUKF_Update(&aukf, 50.0f, 1.0f, 3.2f, 1.0f);
    }
    TEST_ASSERT(aukf.P[0][0] > 0.0f);
    TEST_ASSERT(aukf.P[1][1] > 0.0f);
}

static void test_update_with_exact_model_voltage_minimal_correction(void) {
    /*
     * When the measured voltage matches the OCV at the known SoC exactly,
     * the correction should be small.
     */
    BatteryParams_Init(&battery);
    AUKF_Init(&aukf, &battery, 50.0f, 25.0f);
    float ocv_50 = BatteryParams_GetOCV(&battery, 50.0f, 25.0f);
    /* With zero current: V_terminal ≈ OCV */
    float soc = AUKF_Update(&aukf, 50.0f, 0.0f, ocv_50, 1.0f);
    /* SoC should stay close to 50% */
    TEST_ASSERT_FLOAT_RANGE(soc, 40.0f, 60.0f);
}

/* ============================================================
 * Temperature variation
 * ============================================================ */

static void test_different_temperatures_give_different_innovations(void) {
    /*
     * OCV at 50% SoC differs significantly between 25°C (≈3.22 V) and
     * 45°C (≈3.43 V).  With the same measured voltage the innovations
     * (y_meas - y_pred) must therefore be different.
     */
    BatteryParams_Init(&battery);
    AUKF_Init(&aukf, &battery, 50.0f, 25.0f);
    AUKF_Update(&aukf, 50.0f, 1.0f, 3.2f, 1.0f);
    float innov_25 = AUKF_GetInnovation(&aukf);

    BatteryParams_Init(&battery);
    AUKF_Init(&aukf, &battery, 50.0f, 45.0f);
    AUKF_Update(&aukf, 50.0f, 1.0f, 3.2f, 1.0f);
    float innov_45 = AUKF_GetInnovation(&aukf);

    TEST_ASSERT(innov_25 != innov_45);
}

/* ============================================================
 * Test suite entry point
 * ============================================================ */

void test_aukf_run(void) {
    UNITY_BEGIN("Adaptive Unscented Kalman Filter (AUKF)");

    RUN_TEST(test_init_sets_soc);
    RUN_TEST(test_init_sets_vtr_zero);
    RUN_TEST(test_init_clamps_soc_above_100);
    RUN_TEST(test_init_clamps_soc_below_0);
    RUN_TEST(test_init_resets_update_count);
    RUN_TEST(test_init_positive_covariance_diagonal);
    RUN_TEST(test_init_positive_Q_diagonal);
    RUN_TEST(test_init_positive_R);

    RUN_TEST(test_update_returns_soc_in_range);
    RUN_TEST(test_update_increments_count);
    RUN_TEST(test_update_get_soc_matches_return);
    RUN_TEST(test_update_zero_current_soc_in_range);
    RUN_TEST(test_update_many_iterations_stay_in_range);

    RUN_TEST(test_update_innovation_is_nonzero_with_mismatch);
    RUN_TEST(test_update_covariance_stays_positive_definite);
    RUN_TEST(test_update_with_exact_model_voltage_minimal_correction);

    RUN_TEST(test_different_temperatures_give_different_innovations);
}
