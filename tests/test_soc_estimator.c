/**
 * Unit tests for SoC Estimator (SOCEstimator.c)
 */

#include "test_framework.h"
#include "../SOCEstimator.h"

#define TOL   1e-4f
#define LOOSE 5.0f   /* loose tolerance for filter corrections */

/* ---- Init tests ---- */

static void test_init_soc_ccm_set(void) {
    SoC_Estimator_t est;
    SoC_Estimator_Init(&est, 75.0f, 25.0f);
    ASSERT_FLOAT_EQ(est.SoC_CCM, 75.0f, TOL);
}

static void test_init_soc_aukf_set(void) {
    SoC_Estimator_t est;
    SoC_Estimator_Init(&est, 75.0f, 25.0f);
    ASSERT_FLOAT_EQ(est.SoC_AUKF, 75.0f, TOL);
}

static void test_init_temperature_stored(void) {
    SoC_Estimator_t est;
    SoC_Estimator_Init(&est, 50.0f, 35.0f);
    ASSERT_FLOAT_EQ(est.temperature, 35.0f, TOL);
}

static void test_init_update_count_zero(void) {
    SoC_Estimator_t est;
    SoC_Estimator_Init(&est, 50.0f, 25.0f);
    ASSERT_INT_EQ(est.update_count, 0);
}

static void test_init_v_predicted_zero(void) {
    SoC_Estimator_t est;
    SoC_Estimator_Init(&est, 50.0f, 25.0f);
    ASSERT_FLOAT_EQ(est.V_predicted, 0.0f, TOL);
}

static void test_init_v_measured_zero(void) {
    SoC_Estimator_t est;
    SoC_Estimator_Init(&est, 50.0f, 25.0f);
    ASSERT_FLOAT_EQ(est.V_measured, 0.0f, TOL);
}

static void test_init_battery_capacity_set(void) {
    SoC_Estimator_t est;
    SoC_Estimator_Init(&est, 50.0f, 25.0f);
    /* Q_nominal should be the configured battery capacity */
    ASSERT_FLOAT_EQ(est.battery.Q_nominal, 2.6f, TOL);
}

/* ---- GetSoC / GetSoC_CCM after init ---- */

static void test_getsoc_returns_initial_soc(void) {
    SoC_Estimator_t est;
    SoC_Estimator_Init(&est, 80.0f, 25.0f);
    ASSERT_FLOAT_EQ(SoC_Estimator_GetSoC(&est), 80.0f, TOL);
}

static void test_getsoc_ccm_returns_initial_soc(void) {
    SoC_Estimator_t est;
    SoC_Estimator_Init(&est, 80.0f, 25.0f);
    ASSERT_FLOAT_EQ(SoC_Estimator_GetSoC_CCM(&est), 80.0f, TOL);
}

/* ---- Update tests ---- */

static void test_update_returns_soc_in_range(void) {
    SoC_Estimator_t est;
    SoC_Estimator_Init(&est, 80.0f, 25.0f);
    float soc = SoC_Estimator_Update(&est, 1.0f, 3.45f, 1.0f);
    ASSERT_FLOAT_GE(soc, 0.0f);
    ASSERT_FLOAT_LE(soc, 100.0f);
}

static void test_update_records_measured_voltage(void) {
    SoC_Estimator_t est;
    SoC_Estimator_Init(&est, 80.0f, 25.0f);
    SoC_Estimator_Update(&est, 1.0f, 3.45f, 1.0f);
    ASSERT_FLOAT_EQ(est.V_measured, 3.45f, TOL);
}

static void test_update_increments_count(void) {
    SoC_Estimator_t est;
    SoC_Estimator_Init(&est, 80.0f, 25.0f);
    SoC_Estimator_Update(&est, 1.0f, 3.45f, 1.0f);
    SoC_Estimator_Update(&est, 1.0f, 3.44f, 1.0f);
    ASSERT_INT_EQ(est.update_count, 2);
}

static void test_update_ccm_decreases_on_discharge(void) {
    SoC_Estimator_t est;
    SoC_Estimator_Init(&est, 80.0f, 25.0f);
    SoC_Estimator_Update(&est, 2.6f, 3.45f, 3600.0f);
    /* After 1C discharge for 1 hour → CCM SoC should be 0% */
    ASSERT_FLOAT_LE(est.SoC_CCM, 80.0f);
}

static void test_update_multiple_calls_stable(void) {
    SoC_Estimator_t est;
    SoC_Estimator_Init(&est, 80.0f, 25.0f);
    float soc = 80.0f;
    for (int i = 0; i < 30; i++) {
        soc = SoC_Estimator_Update(&est, 0.5f, 3.4f, 1.0f);
        ASSERT_FLOAT_GE(soc, 0.0f);
        ASSERT_FLOAT_LE(soc, 100.0f);
    }
}

static void test_update_with_charge_current(void) {
    SoC_Estimator_t est;
    SoC_Estimator_Init(&est, 40.0f, 25.0f);
    /* Negative current = charging */
    float soc = SoC_Estimator_Update(&est, -1.0f, 3.5f, 1.0f);
    ASSERT_FLOAT_GE(soc, 0.0f);
    ASSERT_FLOAT_LE(soc, 100.0f);
}

static void test_update_at_high_temperature(void) {
    SoC_Estimator_t est;
    SoC_Estimator_Init(&est, 70.0f, 45.0f);
    float soc = SoC_Estimator_Update(&est, 1.0f, 3.6f, 1.0f);
    ASSERT_FLOAT_GE(soc, 0.0f);
    ASSERT_FLOAT_LE(soc, 100.0f);
}

/* ---- SetTemperature tests ---- */

static void test_set_temperature_updates_estimator(void) {
    SoC_Estimator_t est;
    SoC_Estimator_Init(&est, 50.0f, 25.0f);
    SoC_Estimator_SetTemperature(&est, 45.0f);
    ASSERT_FLOAT_EQ(est.temperature, 45.0f, TOL);
}

static void test_set_temperature_updates_aukf(void) {
    SoC_Estimator_t est;
    SoC_Estimator_Init(&est, 50.0f, 25.0f);
    SoC_Estimator_SetTemperature(&est, 45.0f);
    ASSERT_FLOAT_EQ(est.aukf.temperature, 45.0f, TOL);
}

/* ---- Reset tests ---- */

static void test_reset_changes_soc(void) {
    SoC_Estimator_t est;
    SoC_Estimator_Init(&est, 80.0f, 25.0f);
    SoC_Estimator_Update(&est, 1.0f, 3.4f, 10.0f);
    SoC_Estimator_Reset(&est, 50.0f);
    ASSERT_FLOAT_EQ(est.SoC_CCM, 50.0f, TOL);
    ASSERT_FLOAT_EQ(est.SoC_AUKF, 50.0f, TOL);
}

static void test_reset_clears_update_count(void) {
    SoC_Estimator_t est;
    SoC_Estimator_Init(&est, 80.0f, 25.0f);
    SoC_Estimator_Update(&est, 1.0f, 3.4f, 10.0f);
    SoC_Estimator_Reset(&est, 50.0f);
    ASSERT_INT_EQ(est.update_count, 0);
}

static void test_reset_aukf_soc_matches(void) {
    SoC_Estimator_t est;
    SoC_Estimator_Init(&est, 80.0f, 25.0f);
    SoC_Estimator_Reset(&est, 30.0f);
    ASSERT_FLOAT_EQ(AUKF_GetSoC(&est.aukf), 30.0f, TOL);
}

static void test_getsoc_matches_soc_aukf_field(void) {
    SoC_Estimator_t est;
    SoC_Estimator_Init(&est, 60.0f, 25.0f);
    SoC_Estimator_Update(&est, 1.0f, 3.35f, 1.0f);
    ASSERT_FLOAT_EQ(SoC_Estimator_GetSoC(&est), est.SoC_AUKF, TOL);
}

static void test_getsoc_ccm_matches_soc_ccm_field(void) {
    SoC_Estimator_t est;
    SoC_Estimator_Init(&est, 60.0f, 25.0f);
    SoC_Estimator_Update(&est, 1.0f, 3.35f, 1.0f);
    ASSERT_FLOAT_EQ(SoC_Estimator_GetSoC_CCM(&est), est.SoC_CCM, TOL);
}

/* ---- Main ---- */

int main(void) {
    printf("========================================\n");
    printf("SoC Estimator Unit Tests\n");
    printf("========================================\n");

    RUN_TEST(test_init_soc_ccm_set);
    RUN_TEST(test_init_soc_aukf_set);
    RUN_TEST(test_init_temperature_stored);
    RUN_TEST(test_init_update_count_zero);
    RUN_TEST(test_init_v_predicted_zero);
    RUN_TEST(test_init_v_measured_zero);
    RUN_TEST(test_init_battery_capacity_set);

    RUN_TEST(test_getsoc_returns_initial_soc);
    RUN_TEST(test_getsoc_ccm_returns_initial_soc);

    RUN_TEST(test_update_returns_soc_in_range);
    RUN_TEST(test_update_records_measured_voltage);
    RUN_TEST(test_update_increments_count);
    RUN_TEST(test_update_ccm_decreases_on_discharge);
    RUN_TEST(test_update_multiple_calls_stable);
    RUN_TEST(test_update_with_charge_current);
    RUN_TEST(test_update_at_high_temperature);

    RUN_TEST(test_set_temperature_updates_estimator);
    RUN_TEST(test_set_temperature_updates_aukf);

    RUN_TEST(test_reset_changes_soc);
    RUN_TEST(test_reset_clears_update_count);
    RUN_TEST(test_reset_aukf_soc_matches);
    RUN_TEST(test_getsoc_matches_soc_aukf_field);
    RUN_TEST(test_getsoc_ccm_matches_soc_ccm_field);

    PRINT_RESULTS();
    return RETURN_TEST_STATUS();
}
