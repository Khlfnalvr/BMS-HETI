/**
 * test_battery_params.c
 * Unit tests for the BatteryParams module (bateryparamter.c / Bateryparameter.h).
 *
 * Covers:
 *   - BatteryParams_Init
 *   - Linear_Interpolate
 *   - BatteryParams_GetOCV  (both temperature tables, clamping, interpolation)
 *   - BatteryParams_GetRo
 *   - BatteryParams_GetRtr
 *   - BatteryParams_GetTau
 */

#include "test_framework.h"
#include "../Bateryparameter.h"

/* ── Shared fixture ───────────────────────────────────────────────────────── */

static BatteryParams_t bp;

static void setup(void) {
    BatteryParams_Init(&bp);
}

/* ── Init tests ───────────────────────────────────────────────────────────── */

static void test_init_nominal_capacity(void) {
    setup();
    TEST_ASSERT_EQUAL_FLOAT(BATTERY_NOMINAL_CAPACITY, bp.Q_nominal);
}

static void test_init_nominal_voltage(void) {
    setup();
    TEST_ASSERT_EQUAL_FLOAT(BATTERY_NOMINAL_VOLTAGE, bp.V_nominal);
}

static void test_init_soc_breakpoints(void) {
    setup();
    TEST_ASSERT_EQUAL_FLOAT(0.0f,   bp.SoC_points[0]);
    TEST_ASSERT_EQUAL_FLOAT(10.0f,  bp.SoC_points[1]);
    TEST_ASSERT_EQUAL_FLOAT(25.0f,  bp.SoC_points[2]);
    TEST_ASSERT_EQUAL_FLOAT(50.0f,  bp.SoC_points[3]);
    TEST_ASSERT_EQUAL_FLOAT(75.0f,  bp.SoC_points[4]);
    TEST_ASSERT_EQUAL_FLOAT(90.0f,  bp.SoC_points[5]);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, bp.SoC_points[6]);
}

static void test_init_ocv_25c_boundaries(void) {
    setup();
    TEST_ASSERT_EQUAL_FLOAT(3.0f, bp.OCV_25C[0]);   /* SoC = 0   */
    TEST_ASSERT_EQUAL_FLOAT(3.7f, bp.OCV_25C[6]);   /* SoC = 100 */
}

static void test_init_ocv_45c_boundaries(void) {
    setup();
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 3.2109f, bp.OCV_45C[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 3.9009f, bp.OCV_45C[6]);
}

/* ── Linear_Interpolate tests ─────────────────────────────────────────────── */

static void test_linear_interp_below_range(void) {
    float x[] = {0.0f, 10.0f, 20.0f};
    float y[] = {1.0f,  2.0f,  3.0f};
    /* query below x[0] → return y[0] */
    float result = Linear_Interpolate(x, y, 3, -5.0f);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, result);
}

static void test_linear_interp_above_range(void) {
    float x[] = {0.0f, 10.0f, 20.0f};
    float y[] = {1.0f,  2.0f,  3.0f};
    /* query above x[n-1] → return y[n-1] */
    float result = Linear_Interpolate(x, y, 3, 30.0f);
    TEST_ASSERT_EQUAL_FLOAT(3.0f, result);
}

static void test_linear_interp_exact_lower_bound(void) {
    float x[] = {0.0f, 10.0f, 20.0f};
    float y[] = {1.0f,  2.0f,  3.0f};
    TEST_ASSERT_EQUAL_FLOAT(1.0f, Linear_Interpolate(x, y, 3, 0.0f));
}

static void test_linear_interp_exact_upper_bound(void) {
    float x[] = {0.0f, 10.0f, 20.0f};
    float y[] = {1.0f,  2.0f,  3.0f};
    TEST_ASSERT_EQUAL_FLOAT(3.0f, Linear_Interpolate(x, y, 3, 20.0f));
}

static void test_linear_interp_exact_interior_point(void) {
    float x[] = {0.0f, 10.0f, 20.0f};
    float y[] = {1.0f,  2.0f,  3.0f};
    TEST_ASSERT_EQUAL_FLOAT(2.0f, Linear_Interpolate(x, y, 3, 10.0f));
}

static void test_linear_interp_midpoint(void) {
    float x[] = {0.0f, 10.0f};
    float y[] = {0.0f, 10.0f};
    /* midpoint (5) should give 5 */
    TEST_ASSERT_EQUAL_FLOAT(5.0f, Linear_Interpolate(x, y, 2, 5.0f));
}

static void test_linear_interp_quarter_point(void) {
    float x[] = {0.0f, 100.0f};
    float y[] = {3.0f,   4.0f};
    /* at x=25 → slope = 0.01, y = 3.0 + 0.01*25 = 3.25 */
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 3.25f, Linear_Interpolate(x, y, 2, 25.0f));
}

/* ── BatteryParams_GetOCV tests ───────────────────────────────────────────── */

static void test_get_ocv_soc0_25c(void) {
    setup();
    float ocv = BatteryParams_GetOCV(&bp, 0.0f, 25.0f);
    TEST_ASSERT_EQUAL_FLOAT(3.0f, ocv);
}

static void test_get_ocv_soc100_25c(void) {
    setup();
    float ocv = BatteryParams_GetOCV(&bp, 100.0f, 25.0f);
    TEST_ASSERT_EQUAL_FLOAT(3.7f, ocv);
}

static void test_get_ocv_soc50_25c_exact_breakpoint(void) {
    setup();
    /* SoC=50 is an exact breakpoint → OCV_25C[3] = 3.22 */
    float ocv = BatteryParams_GetOCV(&bp, 50.0f, 25.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 3.22f, ocv);
}

static void test_get_ocv_uses_45c_table_above_35c(void) {
    setup();
    float ocv_25 = BatteryParams_GetOCV(&bp, 0.0f, 25.0f);
    float ocv_45 = BatteryParams_GetOCV(&bp, 0.0f, 45.0f);
    /* 45°C table has different (higher) values at SoC=0 */
    TEST_ASSERT_TRUE(ocv_45 > ocv_25);
}

static void test_get_ocv_temp_boundary_35c_uses_25c_table(void) {
    setup();
    float ocv_at_35 = BatteryParams_GetOCV(&bp, 0.0f, 35.0f);
    TEST_ASSERT_EQUAL_FLOAT(3.0f, ocv_at_35);  /* 25°C table value */
}

static void test_get_ocv_temp_boundary_36c_uses_45c_table(void) {
    setup();
    float ocv_at_36 = BatteryParams_GetOCV(&bp, 0.0f, 36.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 3.2109f, ocv_at_36);  /* 45°C table value */
}

static void test_get_ocv_clamps_soc_above_100(void) {
    setup();
    float ocv_100  = BatteryParams_GetOCV(&bp, 100.0f, 25.0f);
    float ocv_over = BatteryParams_GetOCV(&bp, 150.0f, 25.0f);
    TEST_ASSERT_EQUAL_FLOAT(ocv_100, ocv_over);
}

static void test_get_ocv_clamps_soc_below_0(void) {
    setup();
    float ocv_0   = BatteryParams_GetOCV(&bp, 0.0f,   25.0f);
    float ocv_neg = BatteryParams_GetOCV(&bp, -10.0f, 25.0f);
    TEST_ASSERT_EQUAL_FLOAT(ocv_0, ocv_neg);
}

static void test_get_ocv_monotone_with_soc_25c(void) {
    setup();
    /* OCV should increase with SoC (monotone) */
    float prev = BatteryParams_GetOCV(&bp, 0.0f, 25.0f);
    for (int s = 10; s <= 100; s += 10) {
        float curr = BatteryParams_GetOCV(&bp, (float)s, 25.0f);
        TEST_ASSERT_TRUE(curr >= prev);
        prev = curr;
    }
}

/* ── BatteryParams_GetRo tests ────────────────────────────────────────────── */

static void test_get_ro_soc0_25c(void) {
    setup();
    float ro = BatteryParams_GetRo(&bp, 0.0f, 25.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0515f, ro);
}

static void test_get_ro_soc100_25c(void) {
    setup();
    float ro = BatteryParams_GetRo(&bp, 100.0f, 25.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0528f, ro);
}

static void test_get_ro_uses_45c_table(void) {
    setup();
    float ro_25 = BatteryParams_GetRo(&bp, 0.0f, 25.0f);
    float ro_45 = BatteryParams_GetRo(&bp, 0.0f, 45.0f);
    /* 45°C Ro is higher for this battery */
    TEST_ASSERT_TRUE(ro_45 > ro_25);
}

static void test_get_ro_returns_positive(void) {
    setup();
    for (int s = 0; s <= 100; s += 25) {
        float ro = BatteryParams_GetRo(&bp, (float)s, 25.0f);
        TEST_ASSERT_TRUE(ro > 0.0f);
    }
}

/* ── BatteryParams_GetRtr tests ───────────────────────────────────────────── */

static void test_get_rtr_soc0_25c(void) {
    setup();
    float rtr = BatteryParams_GetRtr(&bp, 0.0f, 25.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0137f, rtr);
}

static void test_get_rtr_soc100_25c(void) {
    setup();
    float rtr = BatteryParams_GetRtr(&bp, 100.0f, 25.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0056f, rtr);
}

static void test_get_rtr_returns_positive(void) {
    setup();
    for (int s = 0; s <= 100; s += 25) {
        float rtr = BatteryParams_GetRtr(&bp, (float)s, 25.0f);
        TEST_ASSERT_TRUE(rtr > 0.0f);
    }
}

/* ── BatteryParams_GetTau tests ───────────────────────────────────────────── */

static void test_get_tau_soc0_25c(void) {
    setup();
    float tau = BatteryParams_GetTau(&bp, 0.0f, 25.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 104.65f, tau);
}

static void test_get_tau_soc100_25c(void) {
    setup();
    float tau = BatteryParams_GetTau(&bp, 100.0f, 25.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 109.71f, tau);
}

static void test_get_tau_returns_positive(void) {
    setup();
    for (int s = 0; s <= 100; s += 25) {
        float tau = BatteryParams_GetTau(&bp, (float)s, 25.0f);
        TEST_ASSERT_TRUE(tau > 0.0f);
    }
}

/* ── Main ─────────────────────────────────────────────────────────────────── */

int main(void) {
    TEST_SUITE_BEGIN("BatteryParams");

    RUN_TEST(test_init_nominal_capacity);
    RUN_TEST(test_init_nominal_voltage);
    RUN_TEST(test_init_soc_breakpoints);
    RUN_TEST(test_init_ocv_25c_boundaries);
    RUN_TEST(test_init_ocv_45c_boundaries);

    RUN_TEST(test_linear_interp_below_range);
    RUN_TEST(test_linear_interp_above_range);
    RUN_TEST(test_linear_interp_exact_lower_bound);
    RUN_TEST(test_linear_interp_exact_upper_bound);
    RUN_TEST(test_linear_interp_exact_interior_point);
    RUN_TEST(test_linear_interp_midpoint);
    RUN_TEST(test_linear_interp_quarter_point);

    RUN_TEST(test_get_ocv_soc0_25c);
    RUN_TEST(test_get_ocv_soc100_25c);
    RUN_TEST(test_get_ocv_soc50_25c_exact_breakpoint);
    RUN_TEST(test_get_ocv_uses_45c_table_above_35c);
    RUN_TEST(test_get_ocv_temp_boundary_35c_uses_25c_table);
    RUN_TEST(test_get_ocv_temp_boundary_36c_uses_45c_table);
    RUN_TEST(test_get_ocv_clamps_soc_above_100);
    RUN_TEST(test_get_ocv_clamps_soc_below_0);
    RUN_TEST(test_get_ocv_monotone_with_soc_25c);

    RUN_TEST(test_get_ro_soc0_25c);
    RUN_TEST(test_get_ro_soc100_25c);
    RUN_TEST(test_get_ro_uses_45c_table);
    RUN_TEST(test_get_ro_returns_positive);

    RUN_TEST(test_get_rtr_soc0_25c);
    RUN_TEST(test_get_rtr_soc100_25c);
    RUN_TEST(test_get_rtr_returns_positive);

    RUN_TEST(test_get_tau_soc0_25c);
    RUN_TEST(test_get_tau_soc100_25c);
    RUN_TEST(test_get_tau_returns_positive);

    TEST_SUITE_END();
    return 0;
}
