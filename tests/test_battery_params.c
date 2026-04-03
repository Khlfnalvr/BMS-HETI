/**
 * test_battery_params.c - Unit tests for BatteryParams (bateryparamter.c)
 *
 * Covers:
 *  - BatteryParams_Init  (nominal values)
 *  - Linear_Interpolate  (boundary and mid-point)
 *  - BatteryParams_GetOCV  (25°C table, 45°C table, clamping)
 *  - BatteryParams_GetRo / GetRtr / GetTau (sanity ranges)
 */

#include "unity.h"
#include "../Bateryparameter.h"

/* ── shared fixture ──────────────────────────────────────────────────── */

static BatteryParams_t bp;

static void setup(void) {
    BatteryParams_Init(&bp);
}

/* ── BatteryParams_Init ──────────────────────────────────────────────── */

TEST(init_nominal_capacity) {
    setup();
    ASSERT_FLOAT_EQ(bp.Q_nominal, BATTERY_NOMINAL_CAPACITY, 1e-5f);
}

TEST(init_nominal_voltage) {
    setup();
    ASSERT_FLOAT_EQ(bp.V_nominal, BATTERY_NOMINAL_VOLTAGE, 1e-5f);
}

TEST(init_soc_points_start_at_0) {
    setup();
    ASSERT_FLOAT_EQ(bp.SoC_points[0], 0.0f, 1e-5f);
}

TEST(init_soc_points_end_at_100) {
    setup();
    ASSERT_FLOAT_EQ(bp.SoC_points[NUM_SOC_POINTS - 1], 100.0f, 1e-5f);
}

TEST(init_ocv_25c_monotone_increasing) {
    setup();
    for (int i = 1; i < NUM_SOC_POINTS; i++) {
        ASSERT_TRUE(bp.OCV_25C[i] > bp.OCV_25C[i - 1]);
    }
}

TEST(init_ocv_45c_monotone_increasing) {
    setup();
    for (int i = 1; i < NUM_SOC_POINTS; i++) {
        ASSERT_TRUE(bp.OCV_45C[i] > bp.OCV_45C[i - 1]);
    }
}

/* ── Linear_Interpolate ──────────────────────────────────────────────── */

TEST(interpolate_at_lower_bound) {
    float x[] = {0.0f, 10.0f, 20.0f};
    float y[] = {1.0f,  2.0f,  3.0f};
    /* query == x[0] → should return y[0] */
    float result = Linear_Interpolate(x, y, 3, 0.0f);
    ASSERT_FLOAT_EQ(result, 1.0f, 1e-5f);
}

TEST(interpolate_at_upper_bound) {
    float x[] = {0.0f, 10.0f, 20.0f};
    float y[] = {1.0f,  2.0f,  3.0f};
    float result = Linear_Interpolate(x, y, 3, 20.0f);
    ASSERT_FLOAT_EQ(result, 3.0f, 1e-5f);
}

TEST(interpolate_below_lower_bound_clamps) {
    float x[] = {0.0f, 10.0f, 20.0f};
    float y[] = {1.0f,  2.0f,  3.0f};
    float result = Linear_Interpolate(x, y, 3, -5.0f);
    ASSERT_FLOAT_EQ(result, 1.0f, 1e-5f);
}

TEST(interpolate_above_upper_bound_clamps) {
    float x[] = {0.0f, 10.0f, 20.0f};
    float y[] = {1.0f,  2.0f,  3.0f};
    float result = Linear_Interpolate(x, y, 3, 30.0f);
    ASSERT_FLOAT_EQ(result, 3.0f, 1e-5f);
}

TEST(interpolate_midpoint) {
    float x[] = {0.0f, 10.0f, 20.0f};
    float y[] = {0.0f, 10.0f, 20.0f};   /* identity: y == x */
    float result = Linear_Interpolate(x, y, 3, 5.0f);
    ASSERT_FLOAT_EQ(result, 5.0f, 1e-4f);
}

TEST(interpolate_quarter_point) {
    float x[] = {0.0f, 100.0f};
    float y[] = {3.0f,   4.0f};
    /* at x=25 → y = 3.0 + 0.25*(4.0-3.0) = 3.25 */
    float result = Linear_Interpolate(x, y, 2, 25.0f);
    ASSERT_FLOAT_EQ(result, 3.25f, 1e-5f);
}

/* ── BatteryParams_GetOCV ────────────────────────────────────────────── */

TEST(get_ocv_soc0_at_25c) {
    setup();
    float ocv = BatteryParams_GetOCV(&bp, 0.0f, 25.0f);
    ASSERT_FLOAT_EQ(ocv, 3.0f, 1e-4f);
}

TEST(get_ocv_soc100_at_25c) {
    setup();
    float ocv = BatteryParams_GetOCV(&bp, 100.0f, 25.0f);
    ASSERT_FLOAT_EQ(ocv, 3.7f, 1e-4f);
}

TEST(get_ocv_soc50_at_25c_interpolated) {
    setup();
    /* SoC=50 is an exact table point → OCV_25C[3] = 3.22 V */
    float ocv = BatteryParams_GetOCV(&bp, 50.0f, 25.0f);
    ASSERT_FLOAT_EQ(ocv, 3.22f, 1e-4f);
}

TEST(get_ocv_soc0_at_45c) {
    setup();
    float ocv = BatteryParams_GetOCV(&bp, 0.0f, 45.0f);
    ASSERT_FLOAT_EQ(ocv, 3.2109f, 1e-4f);
}

TEST(get_ocv_soc100_at_45c) {
    setup();
    float ocv = BatteryParams_GetOCV(&bp, 100.0f, 45.0f);
    ASSERT_FLOAT_EQ(ocv, 3.9009f, 1e-4f);
}

TEST(get_ocv_switches_table_at_35c_boundary) {
    setup();
    float ocv_low  = BatteryParams_GetOCV(&bp, 100.0f, 35.0f);   /* 25C table */
    float ocv_high = BatteryParams_GetOCV(&bp, 100.0f, 36.0f);   /* 45C table */
    ASSERT_FLOAT_EQ(ocv_low,  3.7f,    1e-4f);
    ASSERT_FLOAT_EQ(ocv_high, 3.9009f, 1e-4f);
}

TEST(get_ocv_clamps_negative_soc) {
    setup();
    float ocv_neg = BatteryParams_GetOCV(&bp, -10.0f, 25.0f);
    float ocv_0   = BatteryParams_GetOCV(&bp,   0.0f, 25.0f);
    ASSERT_FLOAT_EQ(ocv_neg, ocv_0, 1e-5f);
}

TEST(get_ocv_clamps_soc_above_100) {
    setup();
    float ocv_over = BatteryParams_GetOCV(&bp, 110.0f, 25.0f);
    float ocv_100  = BatteryParams_GetOCV(&bp, 100.0f, 25.0f);
    ASSERT_FLOAT_EQ(ocv_over, ocv_100, 1e-5f);
}

TEST(get_ocv_midpoint_between_breakpoints) {
    setup();
    /* Between SoC=25 (3.13V) and SoC=50 (3.22V) → SoC=37.5 → 3.175V */
    float expected = 3.13f + (37.5f - 25.0f) / (50.0f - 25.0f) * (3.22f - 3.13f);
    float ocv = BatteryParams_GetOCV(&bp, 37.5f, 25.0f);
    ASSERT_FLOAT_EQ(ocv, expected, 1e-4f);
}

/* ── BatteryParams_GetRo ─────────────────────────────────────────────── */

TEST(get_ro_at_25c_reasonable_range) {
    setup();
    for (int i = 0; i < NUM_SOC_POINTS; i++) {
        float ro = BatteryParams_GetRo(&bp, bp.SoC_points[i], 25.0f);
        ASSERT_FLOAT_GE(ro, 0.04f);
        ASSERT_FLOAT_LE(ro, 0.10f);
    }
}

TEST(get_ro_at_45c_reasonable_range) {
    setup();
    for (int i = 0; i < NUM_SOC_POINTS; i++) {
        float ro = BatteryParams_GetRo(&bp, bp.SoC_points[i], 45.0f);
        ASSERT_FLOAT_GE(ro, 0.07f);
        ASSERT_FLOAT_LE(ro, 0.10f);
    }
}

/* ── BatteryParams_GetRtr ────────────────────────────────────────────── */

TEST(get_rtr_at_25c_reasonable_range) {
    setup();
    for (int i = 0; i < NUM_SOC_POINTS; i++) {
        float rtr = BatteryParams_GetRtr(&bp, bp.SoC_points[i], 25.0f);
        ASSERT_FLOAT_GE(rtr, 0.005f);
        ASSERT_FLOAT_LE(rtr, 0.02f);
    }
}

/* ── BatteryParams_GetTau ────────────────────────────────────────────── */

TEST(get_tau_at_25c_positive) {
    setup();
    for (int i = 0; i < NUM_SOC_POINTS; i++) {
        float tau = BatteryParams_GetTau(&bp, bp.SoC_points[i], 25.0f);
        ASSERT_FLOAT_GE(tau, 50.0f);
        ASSERT_FLOAT_LE(tau, 300.0f);
    }
}

TEST(get_tau_at_45c_positive) {
    setup();
    for (int i = 0; i < NUM_SOC_POINTS; i++) {
        float tau = BatteryParams_GetTau(&bp, bp.SoC_points[i], 45.0f);
        ASSERT_FLOAT_GE(tau, 50.0f);
        ASSERT_FLOAT_LE(tau, 300.0f);
    }
}

/* ── entry point ─────────────────────────────────────────────────────── */

int main(void) {
    printf("=== BatteryParams Tests ===\n");

    RUN_TEST(init_nominal_capacity);
    RUN_TEST(init_nominal_voltage);
    RUN_TEST(init_soc_points_start_at_0);
    RUN_TEST(init_soc_points_end_at_100);
    RUN_TEST(init_ocv_25c_monotone_increasing);
    RUN_TEST(init_ocv_45c_monotone_increasing);

    RUN_TEST(interpolate_at_lower_bound);
    RUN_TEST(interpolate_at_upper_bound);
    RUN_TEST(interpolate_below_lower_bound_clamps);
    RUN_TEST(interpolate_above_upper_bound_clamps);
    RUN_TEST(interpolate_midpoint);
    RUN_TEST(interpolate_quarter_point);

    RUN_TEST(get_ocv_soc0_at_25c);
    RUN_TEST(get_ocv_soc100_at_25c);
    RUN_TEST(get_ocv_soc50_at_25c_interpolated);
    RUN_TEST(get_ocv_soc0_at_45c);
    RUN_TEST(get_ocv_soc100_at_45c);
    RUN_TEST(get_ocv_switches_table_at_35c_boundary);
    RUN_TEST(get_ocv_clamps_negative_soc);
    RUN_TEST(get_ocv_clamps_soc_above_100);
    RUN_TEST(get_ocv_midpoint_between_breakpoints);

    RUN_TEST(get_ro_at_25c_reasonable_range);
    RUN_TEST(get_ro_at_45c_reasonable_range);
    RUN_TEST(get_rtr_at_25c_reasonable_range);
    RUN_TEST(get_tau_at_25c_positive);
    RUN_TEST(get_tau_at_45c_positive);

    PRINT_RESULTS();
    return RESULTS_OK() ? 0 : 1;
}
