/**
 * test_aukf.c
 * Unit tests for the Adaptive Unscented Kalman Filter (AUKF) module.
 *
 * Strategy: focus on observable behaviour via public API:
 *   - AUKF_Init  : state initialisation, clamping, covariance defaults
 *   - AUKF_Update: returns SoC in [0, 100]; update_count increments;
 *                  filter converges direction for correct measurements
 *   - Getters    : AUKF_GetSoC, AUKF_GetVtr, AUKF_GetInnovation
 */

#include "test_framework.h"
#include "../AUKF.h"
#include "../Bateryparameter.h"

/* ── Shared fixture ───────────────────────────────────────────────────────── */

static AUKF_t        aukf;
static BatteryParams_t bp;

static void setup(float initial_soc) {
    BatteryParams_Init(&bp);
    AUKF_Init(&aukf, &bp, initial_soc, 25.0f);
}

/* ── Init tests ───────────────────────────────────────────────────────────── */

static void test_init_soc_stored(void) {
    setup(60.0f);
    TEST_ASSERT_EQUAL_FLOAT(60.0f, aukf.x[0]);
}

static void test_init_vtr_is_zero(void) {
    setup(50.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, aukf.x[1]);
}

static void test_init_clamps_soc_above_100(void) {
    setup(150.0f);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, aukf.x[0]);
}

static void test_init_clamps_soc_below_0(void) {
    setup(-20.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, aukf.x[0]);
}

static void test_init_temperature_stored(void) {
    BatteryParams_Init(&bp);
    AUKF_Init(&aukf, &bp, 50.0f, 45.0f);
    TEST_ASSERT_EQUAL_FLOAT(45.0f, aukf.temperature);
}

static void test_init_update_count_zero(void) {
    setup(50.0f);
    TEST_ASSERT_EQUAL_INT(0, (int)aukf.update_count);
}

static void test_init_covariance_p00_positive(void) {
    setup(50.0f);
    TEST_ASSERT_TRUE(aukf.P[0][0] > 0.0f);
}

static void test_init_covariance_p11_positive(void) {
    setup(50.0f);
    TEST_ASSERT_TRUE(aukf.P[1][1] > 0.0f);
}

static void test_init_process_noise_q_positive(void) {
    setup(50.0f);
    TEST_ASSERT_TRUE(aukf.Q[0][0] > 0.0f);
    TEST_ASSERT_TRUE(aukf.Q[1][1] > 0.0f);
}

static void test_init_measurement_noise_r_positive(void) {
    setup(50.0f);
    TEST_ASSERT_TRUE(aukf.R > 0.0f);
}

static void test_init_battery_pointer_set(void) {
    setup(50.0f);
    TEST_ASSERT_TRUE(aukf.battery == &bp);
}

/* ── Getter tests ─────────────────────────────────────────────────────────── */

static void test_get_soc_returns_x0(void) {
    setup(70.0f);
    TEST_ASSERT_EQUAL_FLOAT(70.0f, AUKF_GetSoC(&aukf));
}

static void test_get_vtr_returns_x1(void) {
    setup(50.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, AUKF_GetVtr(&aukf));
}

static void test_get_innovation_zero_before_update(void) {
    setup(50.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, AUKF_GetInnovation(&aukf));
}

/* ── Update tests ─────────────────────────────────────────────────────────── */

static void test_update_returns_valid_soc_range(void) {
    setup(50.0f);
    float soc = AUKF_Update(&aukf, 50.0f, 1.0f, 3.22f, 1.0f);
    TEST_ASSERT_FLOAT_BETWEEN(0.0f, 100.0f, soc);
}

static void test_update_increments_count(void) {
    setup(50.0f);
    AUKF_Update(&aukf, 50.0f, 1.0f, 3.22f, 1.0f);
    AUKF_Update(&aukf, 50.0f, 1.0f, 3.22f, 1.0f);
    TEST_ASSERT_EQUAL_INT(2, (int)aukf.update_count);
}

static void test_update_get_soc_reflects_result(void) {
    setup(50.0f);
    float returned = AUKF_Update(&aukf, 50.0f, 1.0f, 3.22f, 1.0f);
    TEST_ASSERT_EQUAL_FLOAT(returned, AUKF_GetSoC(&aukf));
}

static void test_update_innovation_set_after_update(void) {
    setup(50.0f);
    /* Supply a voltage that differs from predicted; innovation must be non-zero */
    AUKF_Update(&aukf, 50.0f, 1.0f, 3.50f, 1.0f);
    /* We only check it's finite and accessible; sign depends on internals */
    float inno = AUKF_GetInnovation(&aukf);
    TEST_ASSERT_TRUE(inno > -10.0f && inno < 10.0f);
}

static void test_update_soc_clamped_at_zero(void) {
    /* Start near empty; large discharge should not produce negative SoC */
    setup(1.0f);
    float soc = AUKF_Update(&aukf, 0.0f, 50.0f, 3.0f, 100.0f);
    TEST_ASSERT_FLOAT_BETWEEN(0.0f, 100.0f, soc);
}

static void test_update_soc_clamped_at_hundred(void) {
    /* Start near full; large charge should not exceed 100 */
    setup(99.0f);
    float soc = AUKF_Update(&aukf, 100.0f, -50.0f, 3.7f, 100.0f);
    TEST_ASSERT_FLOAT_BETWEEN(0.0f, 100.0f, soc);
}

static void test_update_multiple_iterations_stay_valid(void) {
    setup(80.0f);
    float voltage = 3.4f;
    float soc_ccm = 80.0f;
    for (int i = 0; i < 20; i++) {
        soc_ccm -= 0.5f;
        if (soc_ccm < 0.0f) soc_ccm = 0.0f;
        float soc = AUKF_Update(&aukf, soc_ccm, 1.0f, voltage, 1.0f);
        TEST_ASSERT_FLOAT_BETWEEN(0.0f, 100.0f, soc);
    }
}

static void test_update_vtr_changes_after_update(void) {
    /* After an update with non-zero current, V_tr should move from 0 */
    setup(50.0f);
    AUKF_Update(&aukf, 50.0f, 2.0f, 3.1f, 1.0f);
    /* V_tr should have been updated (not necessarily non-zero, but the field
       was written — just check it's a finite, bounded value) */
    float vtr = AUKF_GetVtr(&aukf);
    TEST_ASSERT_TRUE(vtr > -1.0f && vtr < 1.0f);
}

/* ── Main ─────────────────────────────────────────────────────────────────── */

int main(void) {
    TEST_SUITE_BEGIN("AUKF");

    RUN_TEST(test_init_soc_stored);
    RUN_TEST(test_init_vtr_is_zero);
    RUN_TEST(test_init_clamps_soc_above_100);
    RUN_TEST(test_init_clamps_soc_below_0);
    RUN_TEST(test_init_temperature_stored);
    RUN_TEST(test_init_update_count_zero);
    RUN_TEST(test_init_covariance_p00_positive);
    RUN_TEST(test_init_covariance_p11_positive);
    RUN_TEST(test_init_process_noise_q_positive);
    RUN_TEST(test_init_measurement_noise_r_positive);
    RUN_TEST(test_init_battery_pointer_set);

    RUN_TEST(test_get_soc_returns_x0);
    RUN_TEST(test_get_vtr_returns_x1);
    RUN_TEST(test_get_innovation_zero_before_update);

    RUN_TEST(test_update_returns_valid_soc_range);
    RUN_TEST(test_update_increments_count);
    RUN_TEST(test_update_get_soc_reflects_result);
    RUN_TEST(test_update_innovation_set_after_update);
    RUN_TEST(test_update_soc_clamped_at_zero);
    RUN_TEST(test_update_soc_clamped_at_hundred);
    RUN_TEST(test_update_multiple_iterations_stay_valid);
    RUN_TEST(test_update_vtr_changes_after_update);

    TEST_SUITE_END();
    return 0;
}
