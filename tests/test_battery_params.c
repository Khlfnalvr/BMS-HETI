/**
 * Unit tests for BatteryParams module (Bateryparameter.h / bateryparamter.c)
 */

#include "unity.h"
#include "../Bateryparameter.h"

static BatteryParams_t params;

/* ============================================================
 * Init tests
 * ============================================================ */

static void test_init_nominal_capacity(void) {
    BatteryParams_Init(&params);
    TEST_ASSERT_EQUAL_FLOAT(2.6f, params.Q_nominal, 1e-5f);
}

static void test_init_nominal_voltage(void) {
    BatteryParams_Init(&params);
    TEST_ASSERT_EQUAL_FLOAT(3.7f, params.V_nominal, 1e-5f);
}

static void test_init_soc_points_ascending(void) {
    BatteryParams_Init(&params);
    for (int i = 0; i < NUM_SOC_POINTS - 1; i++) {
        TEST_ASSERT(params.SoC_points[i] < params.SoC_points[i + 1]);
    }
}

static void test_init_soc_points_bounds(void) {
    BatteryParams_Init(&params);
    TEST_ASSERT_EQUAL_FLOAT(0.0f,   params.SoC_points[0],               1e-4f);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, params.SoC_points[NUM_SOC_POINTS-1], 1e-4f);
}

static void test_init_ocv_25c_ascending(void) {
    BatteryParams_Init(&params);
    for (int i = 0; i < NUM_SOC_POINTS - 1; i++) {
        TEST_ASSERT(params.OCV_25C[i] < params.OCV_25C[i + 1]);
    }
}

/* ============================================================
 * Linear_Interpolate tests
 * ============================================================ */

static void test_interpolate_exact_point(void) {
    float x[] = {0.0f, 50.0f, 100.0f};
    float y[] = {1.0f,  5.0f,   9.0f};
    TEST_ASSERT_EQUAL_FLOAT(5.0f, Linear_Interpolate(x, y, 3, 50.0f), 1e-5f);
}

static void test_interpolate_midpoint(void) {
    float x[] = {0.0f, 100.0f};
    float y[] = {0.0f,  10.0f};
    TEST_ASSERT_EQUAL_FLOAT(5.0f, Linear_Interpolate(x, y, 2, 50.0f), 1e-5f);
}

static void test_interpolate_below_range_clamps(void) {
    float x[] = {10.0f, 20.0f, 30.0f};
    float y[] = { 1.0f,  2.0f,  3.0f};
    TEST_ASSERT_EQUAL_FLOAT(1.0f, Linear_Interpolate(x, y, 3, -5.0f), 1e-5f);
}

static void test_interpolate_above_range_clamps(void) {
    float x[] = {10.0f, 20.0f, 30.0f};
    float y[] = { 1.0f,  2.0f,  3.0f};
    TEST_ASSERT_EQUAL_FLOAT(3.0f, Linear_Interpolate(x, y, 3, 100.0f), 1e-5f);
}

static void test_interpolate_first_point(void) {
    float x[] = {0.0f, 50.0f, 100.0f};
    float y[] = {1.0f,  5.0f,   9.0f};
    TEST_ASSERT_EQUAL_FLOAT(1.0f, Linear_Interpolate(x, y, 3, 0.0f), 1e-5f);
}

static void test_interpolate_last_point(void) {
    float x[] = {0.0f, 50.0f, 100.0f};
    float y[] = {1.0f,  5.0f,   9.0f};
    TEST_ASSERT_EQUAL_FLOAT(9.0f, Linear_Interpolate(x, y, 3, 100.0f), 1e-5f);
}

/* ============================================================
 * GetOCV tests
 * ============================================================ */

static void test_ocv_at_soc0_temp25(void) {
    BatteryParams_Init(&params);
    float ocv = BatteryParams_GetOCV(&params, 0.0f, 25.0f);
    TEST_ASSERT_EQUAL_FLOAT(params.OCV_25C[0], ocv, 1e-4f);
}

static void test_ocv_at_soc100_temp25(void) {
    BatteryParams_Init(&params);
    float ocv = BatteryParams_GetOCV(&params, 100.0f, 25.0f);
    TEST_ASSERT_EQUAL_FLOAT(params.OCV_25C[NUM_SOC_POINTS-1], ocv, 1e-4f);
}

static void test_ocv_at_soc0_temp45(void) {
    BatteryParams_Init(&params);
    float ocv = BatteryParams_GetOCV(&params, 0.0f, 45.0f);
    TEST_ASSERT_EQUAL_FLOAT(params.OCV_45C[0], ocv, 1e-4f);
}

static void test_ocv_at_soc100_temp45(void) {
    BatteryParams_Init(&params);
    float ocv = BatteryParams_GetOCV(&params, 100.0f, 45.0f);
    TEST_ASSERT_EQUAL_FLOAT(params.OCV_45C[NUM_SOC_POINTS-1], ocv, 1e-4f);
}

static void test_ocv_uses_25c_table_at_temp_below_35(void) {
    BatteryParams_Init(&params);
    float ocv_25  = BatteryParams_GetOCV(&params, 50.0f, 25.0f);
    float ocv_35  = BatteryParams_GetOCV(&params, 50.0f, 35.0f);
    float ocv_45  = BatteryParams_GetOCV(&params, 50.0f, 45.0f);
    /* 25°C and 35°C both use the 25°C table */
    TEST_ASSERT_EQUAL_FLOAT(ocv_25, ocv_35, 1e-4f);
    /* 45°C uses the 45°C table — should differ */
    TEST_ASSERT(ocv_45 != ocv_25);
}

static void test_ocv_clamps_soc_below_0(void) {
    BatteryParams_Init(&params);
    float ocv_neg = BatteryParams_GetOCV(&params, -50.0f, 25.0f);
    float ocv_0   = BatteryParams_GetOCV(&params,   0.0f, 25.0f);
    TEST_ASSERT_EQUAL_FLOAT(ocv_0, ocv_neg, 1e-4f);
}

static void test_ocv_clamps_soc_above_100(void) {
    BatteryParams_Init(&params);
    float ocv_200 = BatteryParams_GetOCV(&params, 200.0f, 25.0f);
    float ocv_100 = BatteryParams_GetOCV(&params, 100.0f, 25.0f);
    TEST_ASSERT_EQUAL_FLOAT(ocv_100, ocv_200, 1e-4f);
}

static void test_ocv_monotonically_increasing_with_soc(void) {
    BatteryParams_Init(&params);
    float prev = BatteryParams_GetOCV(&params, 0.0f, 25.0f);
    float soc_steps[] = {10.0f, 25.0f, 50.0f, 75.0f, 90.0f, 100.0f};
    for (int i = 0; i < 6; i++) {
        float curr = BatteryParams_GetOCV(&params, soc_steps[i], 25.0f);
        TEST_ASSERT(curr >= prev);
        prev = curr;
    }
}

/* ============================================================
 * GetRo tests
 * ============================================================ */

static void test_ro_at_soc0_temp25(void) {
    BatteryParams_Init(&params);
    float ro = BatteryParams_GetRo(&params, 0.0f, 25.0f);
    TEST_ASSERT_EQUAL_FLOAT(params.Ro_25C[0], ro, 1e-5f);
}

static void test_ro_positive_value(void) {
    BatteryParams_Init(&params);
    float ro = BatteryParams_GetRo(&params, 50.0f, 25.0f);
    TEST_ASSERT(ro > 0.0f);
}

static void test_ro_uses_45c_table_at_high_temp(void) {
    BatteryParams_Init(&params);
    float ro_25 = BatteryParams_GetRo(&params, 50.0f, 25.0f);
    float ro_45 = BatteryParams_GetRo(&params, 50.0f, 45.0f);
    TEST_ASSERT(ro_25 != ro_45);
}

/* ============================================================
 * GetRtr tests
 * ============================================================ */

static void test_rtr_positive_value(void) {
    BatteryParams_Init(&params);
    float rtr = BatteryParams_GetRtr(&params, 50.0f, 25.0f);
    TEST_ASSERT(rtr > 0.0f);
}

static void test_rtr_at_boundary_soc(void) {
    BatteryParams_Init(&params);
    float rtr0   = BatteryParams_GetRtr(&params,   0.0f, 25.0f);
    float rtr100 = BatteryParams_GetRtr(&params, 100.0f, 25.0f);
    TEST_ASSERT_EQUAL_FLOAT(params.Rtr_25C[0],             rtr0,   1e-5f);
    TEST_ASSERT_EQUAL_FLOAT(params.Rtr_25C[NUM_SOC_POINTS-1], rtr100, 1e-5f);
}

/* ============================================================
 * GetTau tests
 * ============================================================ */

static void test_tau_positive_value(void) {
    BatteryParams_Init(&params);
    float tau = BatteryParams_GetTau(&params, 50.0f, 25.0f);
    TEST_ASSERT(tau > 0.0f);
}

static void test_tau_at_boundary_soc(void) {
    BatteryParams_Init(&params);
    float tau0   = BatteryParams_GetTau(&params,   0.0f, 25.0f);
    float tau100 = BatteryParams_GetTau(&params, 100.0f, 25.0f);
    TEST_ASSERT_EQUAL_FLOAT(params.tau_25C[0],             tau0,   1e-3f);
    TEST_ASSERT_EQUAL_FLOAT(params.tau_25C[NUM_SOC_POINTS-1], tau100, 1e-3f);
}

/* ============================================================
 * Test suite entry point
 * ============================================================ */

void test_battery_params_run(void) {
    UNITY_BEGIN("Battery Parameters");

    RUN_TEST(test_init_nominal_capacity);
    RUN_TEST(test_init_nominal_voltage);
    RUN_TEST(test_init_soc_points_ascending);
    RUN_TEST(test_init_soc_points_bounds);
    RUN_TEST(test_init_ocv_25c_ascending);

    RUN_TEST(test_interpolate_exact_point);
    RUN_TEST(test_interpolate_midpoint);
    RUN_TEST(test_interpolate_below_range_clamps);
    RUN_TEST(test_interpolate_above_range_clamps);
    RUN_TEST(test_interpolate_first_point);
    RUN_TEST(test_interpolate_last_point);

    RUN_TEST(test_ocv_at_soc0_temp25);
    RUN_TEST(test_ocv_at_soc100_temp25);
    RUN_TEST(test_ocv_at_soc0_temp45);
    RUN_TEST(test_ocv_at_soc100_temp45);
    RUN_TEST(test_ocv_uses_25c_table_at_temp_below_35);
    RUN_TEST(test_ocv_clamps_soc_below_0);
    RUN_TEST(test_ocv_clamps_soc_above_100);
    RUN_TEST(test_ocv_monotonically_increasing_with_soc);

    RUN_TEST(test_ro_at_soc0_temp25);
    RUN_TEST(test_ro_positive_value);
    RUN_TEST(test_ro_uses_45c_table_at_high_temp);

    RUN_TEST(test_rtr_positive_value);
    RUN_TEST(test_rtr_at_boundary_soc);

    RUN_TEST(test_tau_positive_value);
    RUN_TEST(test_tau_at_boundary_soc);
}
