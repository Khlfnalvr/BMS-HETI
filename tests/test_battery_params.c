/**
 * test_battery_params.c
 * Unit tests for Bateryparameter.h / bateryparamter.c
 *
 * Covers:
 *  - BatteryParams_Init  : nominal values and lookup tables populated
 *  - Linear_Interpolate  : exact points, midpoint, clamping below/above range
 *  - BatteryParams_GetOCV: known breakpoints, temperature table selection,
 *                          SoC clamping
 *  - BatteryParams_GetRo : temperature table selection
 *  - BatteryParams_GetRtr: temperature table selection
 *  - BatteryParams_GetTau: temperature table selection
 */

#include "test_framework.h"
#include "Bateryparameter.h"

/* ── Helpers ──────────────────────────────────────────────────────── */

static BatteryParams_t make_params(void) {
    BatteryParams_t p;
    BatteryParams_Init(&p);
    return p;
}

/* ── BatteryParams_Init ───────────────────────────────────────────── */

static void test_init_nominal_capacity(void) {
    BatteryParams_t p = make_params();
    ASSERT_FLOAT_NEAR(2.6f, p.Q_nominal, 1e-6f);
}

static void test_init_nominal_voltage(void) {
    BatteryParams_t p = make_params();
    ASSERT_FLOAT_NEAR(3.7f, p.V_nominal, 1e-6f);
}

static void test_init_soc_points_boundaries(void) {
    BatteryParams_t p = make_params();
    /* First and last breakpoints */
    ASSERT_FLOAT_NEAR(0.0f,   p.SoC_points[0], 1e-6f);
    ASSERT_FLOAT_NEAR(100.0f, p.SoC_points[NUM_SOC_POINTS - 1], 1e-6f);
}

static void test_init_ocv25_boundaries(void) {
    BatteryParams_t p = make_params();
    ASSERT_FLOAT_NEAR(3.0f, p.OCV_25C[0], 1e-6f);   /* SoC = 0 % */
    ASSERT_FLOAT_NEAR(3.7f, p.OCV_25C[NUM_SOC_POINTS - 1], 1e-6f); /* SoC = 100 % */
}

static void test_init_ocv45_boundaries(void) {
    BatteryParams_t p = make_params();
    ASSERT_FLOAT_NEAR(3.2109f, p.OCV_45C[0], 1e-4f);
    ASSERT_FLOAT_NEAR(3.9009f, p.OCV_45C[NUM_SOC_POINTS - 1], 1e-4f);
}

/* ── Linear_Interpolate ───────────────────────────────────────────── */

static void test_interp_exact_first_point(void) {
    float x[] = {0.0f, 10.0f, 25.0f, 50.0f, 75.0f, 90.0f, 100.0f};
    float y[] = {3.0f, 3.06f, 3.13f, 3.22f, 3.42f, 3.58f, 3.7f};
    float result = Linear_Interpolate(x, y, 7, 0.0f);
    ASSERT_FLOAT_NEAR(3.0f, result, 1e-6f);
}

static void test_interp_exact_last_point(void) {
    float x[] = {0.0f, 10.0f, 25.0f, 50.0f, 75.0f, 90.0f, 100.0f};
    float y[] = {3.0f, 3.06f, 3.13f, 3.22f, 3.42f, 3.58f, 3.7f};
    float result = Linear_Interpolate(x, y, 7, 100.0f);
    ASSERT_FLOAT_NEAR(3.7f, result, 1e-6f);
}

static void test_interp_exact_midpoint(void) {
    /* SoC = 50 is an exact breakpoint, expect 3.22 */
    float x[] = {0.0f, 10.0f, 25.0f, 50.0f, 75.0f, 90.0f, 100.0f};
    float y[] = {3.0f, 3.06f, 3.13f, 3.22f, 3.42f, 3.58f, 3.7f};
    float result = Linear_Interpolate(x, y, 7, 50.0f);
    ASSERT_FLOAT_NEAR(3.22f, result, 1e-5f);
}

static void test_interp_between_points(void) {
    /* Midpoint between SoC 50 (3.22) and SoC 75 (3.42) → 3.32 */
    float x[] = {0.0f, 10.0f, 25.0f, 50.0f, 75.0f, 90.0f, 100.0f};
    float y[] = {3.0f, 3.06f, 3.13f, 3.22f, 3.42f, 3.58f, 3.7f};
    float result = Linear_Interpolate(x, y, 7, 62.5f);
    ASSERT_FLOAT_NEAR(3.32f, result, 1e-5f);
}

static void test_interp_clamp_below_range(void) {
    float x[] = {0.0f, 50.0f, 100.0f};
    float y[] = {1.0f,  2.0f,   3.0f};
    /* Query below minimum → should return y[0] */
    float result = Linear_Interpolate(x, y, 3, -10.0f);
    ASSERT_FLOAT_NEAR(1.0f, result, 1e-6f);
}

static void test_interp_clamp_above_range(void) {
    float x[] = {0.0f, 50.0f, 100.0f};
    float y[] = {1.0f,  2.0f,   3.0f};
    /* Query above maximum → should return y[n-1] */
    float result = Linear_Interpolate(x, y, 3, 110.0f);
    ASSERT_FLOAT_NEAR(3.0f, result, 1e-6f);
}

/* ── BatteryParams_GetOCV ─────────────────────────────────────────── */

static void test_ocv_at_soc0_25C(void) {
    BatteryParams_t p = make_params();
    float ocv = BatteryParams_GetOCV(&p, 0.0f, 25.0f);
    ASSERT_FLOAT_NEAR(3.0f, ocv, 1e-5f);
}

static void test_ocv_at_soc100_25C(void) {
    BatteryParams_t p = make_params();
    float ocv = BatteryParams_GetOCV(&p, 100.0f, 25.0f);
    ASSERT_FLOAT_NEAR(3.7f, ocv, 1e-5f);
}

static void test_ocv_at_soc50_25C(void) {
    BatteryParams_t p = make_params();
    float ocv = BatteryParams_GetOCV(&p, 50.0f, 25.0f);
    ASSERT_FLOAT_NEAR(3.22f, ocv, 1e-5f);
}

static void test_ocv_uses_25C_table_at_temp_25(void) {
    BatteryParams_t p = make_params();
    float ocv_25 = BatteryParams_GetOCV(&p, 0.0f, 25.0f);
    ASSERT_FLOAT_NEAR(p.OCV_25C[0], ocv_25, 1e-5f);
}

static void test_ocv_uses_25C_table_at_temp_35(void) {
    /* Temperature ≤ 35 → 25 °C table */
    BatteryParams_t p = make_params();
    float ocv = BatteryParams_GetOCV(&p, 0.0f, 35.0f);
    ASSERT_FLOAT_NEAR(p.OCV_25C[0], ocv, 1e-5f);
}

static void test_ocv_uses_45C_table_at_temp_36(void) {
    /* Temperature > 35 → 45 °C table */
    BatteryParams_t p = make_params();
    float ocv = BatteryParams_GetOCV(&p, 0.0f, 36.0f);
    ASSERT_FLOAT_NEAR(p.OCV_45C[0], ocv, 1e-4f);
}

static void test_ocv_uses_45C_table_at_temp_45(void) {
    BatteryParams_t p = make_params();
    float ocv = BatteryParams_GetOCV(&p, 100.0f, 45.0f);
    ASSERT_FLOAT_NEAR(p.OCV_45C[NUM_SOC_POINTS - 1], ocv, 1e-4f);
}

static void test_ocv_clamps_negative_soc(void) {
    BatteryParams_t p = make_params();
    float ocv_at_0   = BatteryParams_GetOCV(&p, 0.0f,   25.0f);
    float ocv_at_neg = BatteryParams_GetOCV(&p, -50.0f, 25.0f);
    ASSERT_FLOAT_NEAR(ocv_at_0, ocv_at_neg, 1e-6f);
}

static void test_ocv_clamps_soc_above_100(void) {
    BatteryParams_t p = make_params();
    float ocv_at_100  = BatteryParams_GetOCV(&p, 100.0f, 25.0f);
    float ocv_at_150  = BatteryParams_GetOCV(&p, 150.0f, 25.0f);
    ASSERT_FLOAT_NEAR(ocv_at_100, ocv_at_150, 1e-6f);
}

/* ── BatteryParams_GetRo ──────────────────────────────────────────── */

static void test_ro_at_soc0_25C(void) {
    BatteryParams_t p = make_params();
    float ro = BatteryParams_GetRo(&p, 0.0f, 25.0f);
    ASSERT_FLOAT_NEAR(0.0515f, ro, 1e-4f);
}

static void test_ro_uses_45C_table_when_hot(void) {
    BatteryParams_t p = make_params();
    float ro = BatteryParams_GetRo(&p, 0.0f, 45.0f);
    ASSERT_FLOAT_NEAR(p.Ro_45C[0], ro, 1e-4f);
}

/* ── BatteryParams_GetRtr ─────────────────────────────────────────── */

static void test_rtr_at_soc0_25C(void) {
    BatteryParams_t p = make_params();
    float rtr = BatteryParams_GetRtr(&p, 0.0f, 25.0f);
    ASSERT_FLOAT_NEAR(0.0137f, rtr, 1e-4f);
}

static void test_rtr_uses_45C_table_when_hot(void) {
    BatteryParams_t p = make_params();
    float rtr = BatteryParams_GetRtr(&p, 0.0f, 45.0f);
    ASSERT_FLOAT_NEAR(p.Rtr_45C[0], rtr, 1e-4f);
}

/* ── BatteryParams_GetTau ─────────────────────────────────────────── */

static void test_tau_at_soc0_25C(void) {
    BatteryParams_t p = make_params();
    float tau = BatteryParams_GetTau(&p, 0.0f, 25.0f);
    ASSERT_FLOAT_NEAR(104.65f, tau, 1e-3f);
}

static void test_tau_uses_45C_table_when_hot(void) {
    BatteryParams_t p = make_params();
    float tau = BatteryParams_GetTau(&p, 0.0f, 45.0f);
    ASSERT_FLOAT_NEAR(p.tau_45C[0], tau, 1e-3f);
}

/* ── Suite entry point ────────────────────────────────────────────── */

TEST_SUITE_BEGIN(battery_params)
    /* Init */
    RUN_TEST(test_init_nominal_capacity);
    RUN_TEST(test_init_nominal_voltage);
    RUN_TEST(test_init_soc_points_boundaries);
    RUN_TEST(test_init_ocv25_boundaries);
    RUN_TEST(test_init_ocv45_boundaries);

    /* Linear_Interpolate */
    RUN_TEST(test_interp_exact_first_point);
    RUN_TEST(test_interp_exact_last_point);
    RUN_TEST(test_interp_exact_midpoint);
    RUN_TEST(test_interp_between_points);
    RUN_TEST(test_interp_clamp_below_range);
    RUN_TEST(test_interp_clamp_above_range);

    /* GetOCV */
    RUN_TEST(test_ocv_at_soc0_25C);
    RUN_TEST(test_ocv_at_soc100_25C);
    RUN_TEST(test_ocv_at_soc50_25C);
    RUN_TEST(test_ocv_uses_25C_table_at_temp_25);
    RUN_TEST(test_ocv_uses_25C_table_at_temp_35);
    RUN_TEST(test_ocv_uses_45C_table_at_temp_36);
    RUN_TEST(test_ocv_uses_45C_table_at_temp_45);
    RUN_TEST(test_ocv_clamps_negative_soc);
    RUN_TEST(test_ocv_clamps_soc_above_100);

    /* GetRo */
    RUN_TEST(test_ro_at_soc0_25C);
    RUN_TEST(test_ro_uses_45C_table_when_hot);

    /* GetRtr */
    RUN_TEST(test_rtr_at_soc0_25C);
    RUN_TEST(test_rtr_uses_45C_table_when_hot);

    /* GetTau */
    RUN_TEST(test_tau_at_soc0_25C);
    RUN_TEST(test_tau_uses_45C_table_when_hot);
TEST_SUITE_END()
