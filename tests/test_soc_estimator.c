/**
 * test_soc_estimator.c - Integration tests for SoC_Estimator (SOCEstimator.c)
 *
 * Covers:
 *  - SoC_Estimator_Init
 *  - SoC_Estimator_Update  (return value, CCM vs AUKF fields, count)
 *  - SoC_Estimator_GetSoC / GetSoC_CCM
 *  - SoC_Estimator_SetTemperature
 *  - SoC_Estimator_Reset
 */

#include "unity.h"
#include "../SOCEstimator.h"

/* ── shared fixture ──────────────────────────────────────────────────── */

static SoC_Estimator_t est;

static void setup(float initial_soc, float temperature) {
    SoC_Estimator_Init(&est, initial_soc, temperature);
}

/* ── SoC_Estimator_Init ──────────────────────────────────────────────── */

TEST(init_ccm_soc_set) {
    setup(70.0f, 25.0f);
    ASSERT_FLOAT_EQ(est.SoC_CCM, 70.0f, 1e-5f);
}

TEST(init_aukf_soc_set) {
    setup(70.0f, 25.0f);
    ASSERT_FLOAT_EQ(est.SoC_AUKF, 70.0f, 1e-5f);
}

TEST(init_update_count_zero) {
    setup(70.0f, 25.0f);
    ASSERT_INT_EQ(est.update_count, 0);
}

TEST(init_temperature_stored) {
    setup(50.0f, 35.0f);
    ASSERT_FLOAT_EQ(est.temperature, 35.0f, 1e-5f);
}

TEST(init_battery_nominal_capacity_set) {
    setup(50.0f, 25.0f);
    ASSERT_FLOAT_EQ(est.battery.Q_nominal, BATTERY_NOMINAL_CAPACITY, 1e-5f);
}

/* ── SoC_Estimator_Update ────────────────────────────────────────────── */

TEST(update_returns_soc_in_valid_range) {
    setup(50.0f, 25.0f);
    float soc = SoC_Estimator_Update(&est, 1.0f, 3.3f, 1.0f);
    ASSERT_FLOAT_GE(soc, 0.0f);
    ASSERT_FLOAT_LE(soc, 100.0f);
}

TEST(update_increments_count) {
    setup(50.0f, 25.0f);
    SoC_Estimator_Update(&est, 1.0f, 3.3f, 1.0f);
    SoC_Estimator_Update(&est, 1.0f, 3.3f, 1.0f);
    ASSERT_INT_EQ(est.update_count, 2);
}

TEST(update_stores_measured_voltage) {
    setup(50.0f, 25.0f);
    SoC_Estimator_Update(&est, 1.0f, 3.45f, 1.0f);
    ASSERT_FLOAT_EQ(est.V_measured, 3.45f, 1e-5f);
}

TEST(update_ccm_soc_decreases_on_discharge) {
    setup(80.0f, 25.0f);
    SoC_Estimator_Update(&est, 2.6f, 3.3f, 3600.0f);   /* full capacity discharged */
    ASSERT_TRUE(est.SoC_CCM < 80.0f);
}

TEST(update_returns_aukf_soc_not_ccm) {
    /*
     * With a voltage that is far from CCM prediction, the AUKF will
     * correct the SoC.  The returned value should equal SoC_AUKF, not
     * SoC_CCM.
     */
    setup(50.0f, 25.0f);
    float ret = SoC_Estimator_Update(&est, 0.0f, 3.7f, 1.0f);
    ASSERT_FLOAT_EQ(ret, est.SoC_AUKF, 1e-5f);
}

TEST(update_get_soc_matches_update_return) {
    setup(60.0f, 25.0f);
    float ret = SoC_Estimator_Update(&est, 1.0f, 3.3f, 1.0f);
    ASSERT_FLOAT_EQ(SoC_Estimator_GetSoC(&est), ret, 1e-5f);
}

TEST(update_get_soc_ccm_matches_field) {
    setup(60.0f, 25.0f);
    SoC_Estimator_Update(&est, 1.0f, 3.3f, 1.0f);
    ASSERT_FLOAT_EQ(SoC_Estimator_GetSoC_CCM(&est), est.SoC_CCM, 1e-5f);
}

/* ── Consistency: zero current, matching voltage ─────────────────────── */

TEST(update_no_current_correct_voltage_stays_near_initial) {
    /*
     * At SoC=50%, zero current, feed the exact OCV for 50%.
     * After one step both CCM and AUKF should remain close to 50%.
     */
    setup(50.0f, 25.0f);
    float ocv = BatteryParams_GetOCV(&est.battery, 50.0f, 25.0f);
    float soc = SoC_Estimator_Update(&est, 0.0f, ocv, 1.0f);
    ASSERT_FLOAT_GE(soc, 45.0f);
    ASSERT_FLOAT_LE(soc, 55.0f);
}

/* ── SoC_Estimator_SetTemperature ────────────────────────────────────── */

TEST(set_temperature_updates_estimator_temp) {
    setup(50.0f, 25.0f);
    SoC_Estimator_SetTemperature(&est, 40.0f);
    ASSERT_FLOAT_EQ(est.temperature, 40.0f, 1e-5f);
}

TEST(set_temperature_updates_aukf_temp) {
    setup(50.0f, 25.0f);
    SoC_Estimator_SetTemperature(&est, 40.0f);
    ASSERT_FLOAT_EQ(est.aukf.temperature, 40.0f, 1e-5f);
}

/* ── SoC_Estimator_Reset ─────────────────────────────────────────────── */

TEST(reset_sets_ccm_soc) {
    setup(50.0f, 25.0f);
    SoC_Estimator_Update(&est, 1.0f, 3.3f, 10.0f);
    SoC_Estimator_Reset(&est, 90.0f);
    ASSERT_FLOAT_EQ(est.SoC_CCM, 90.0f, 1e-5f);
}

TEST(reset_sets_aukf_soc) {
    setup(50.0f, 25.0f);
    SoC_Estimator_Update(&est, 1.0f, 3.3f, 10.0f);
    SoC_Estimator_Reset(&est, 90.0f);
    ASSERT_FLOAT_EQ(est.SoC_AUKF, 90.0f, 1e-5f);
}

TEST(reset_zeroes_update_count) {
    setup(50.0f, 25.0f);
    SoC_Estimator_Update(&est, 1.0f, 3.3f, 1.0f);
    SoC_Estimator_Update(&est, 1.0f, 3.3f, 1.0f);
    SoC_Estimator_Reset(&est, 50.0f);
    ASSERT_INT_EQ(est.update_count, 0);
}

TEST(reset_then_update_works_correctly) {
    setup(50.0f, 25.0f);
    SoC_Estimator_Reset(&est, 80.0f);
    float soc = SoC_Estimator_Update(&est, 0.0f, 3.42f, 1.0f);
    ASSERT_FLOAT_GE(soc, 0.0f);
    ASSERT_FLOAT_LE(soc, 100.0f);
    ASSERT_INT_EQ(est.update_count, 1);
}

/* ── Multi-step scenario ─────────────────────────────────────────────── */

TEST(multi_step_ccm_always_clamps_at_zero) {
    /*
     * Start at 5% SoC and discharge hard enough to hit the 0% floor.
     * Both CCM and AUKF should be ≥ 0 at all times.
     */
    setup(5.0f, 25.0f);
    for (int i = 0; i < 10; i++) {
        float soc = SoC_Estimator_Update(&est, 5.0f, 3.0f, 100.0f);
        ASSERT_FLOAT_GE(soc, 0.0f);
        ASSERT_FLOAT_GE(est.SoC_CCM, 0.0f);
    }
}

TEST(multi_step_ccm_always_clamps_at_100) {
    setup(95.0f, 25.0f);
    for (int i = 0; i < 10; i++) {
        /* negative current = charging */
        float soc = SoC_Estimator_Update(&est, -5.0f, 3.7f, 100.0f);
        ASSERT_FLOAT_LE(soc, 100.0f);
        ASSERT_FLOAT_LE(est.SoC_CCM, 100.0f);
    }
}

/* ── entry point ─────────────────────────────────────────────────────── */

int main(void) {
    printf("=== SoC_Estimator Integration Tests ===\n");

    RUN_TEST(init_ccm_soc_set);
    RUN_TEST(init_aukf_soc_set);
    RUN_TEST(init_update_count_zero);
    RUN_TEST(init_temperature_stored);
    RUN_TEST(init_battery_nominal_capacity_set);

    RUN_TEST(update_returns_soc_in_valid_range);
    RUN_TEST(update_increments_count);
    RUN_TEST(update_stores_measured_voltage);
    RUN_TEST(update_ccm_soc_decreases_on_discharge);
    RUN_TEST(update_returns_aukf_soc_not_ccm);
    RUN_TEST(update_get_soc_matches_update_return);
    RUN_TEST(update_get_soc_ccm_matches_field);
    RUN_TEST(update_no_current_correct_voltage_stays_near_initial);

    RUN_TEST(set_temperature_updates_estimator_temp);
    RUN_TEST(set_temperature_updates_aukf_temp);

    RUN_TEST(reset_sets_ccm_soc);
    RUN_TEST(reset_sets_aukf_soc);
    RUN_TEST(reset_zeroes_update_count);
    RUN_TEST(reset_then_update_works_correctly);

    RUN_TEST(multi_step_ccm_always_clamps_at_zero);
    RUN_TEST(multi_step_ccm_always_clamps_at_100);

    PRINT_RESULTS();
    return RESULTS_OK() ? 0 : 1;
}
