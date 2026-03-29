/**
 * Unit tests for Battery Parameters (bateryparamter.c)
 */

#include "test_framework.h"
#include "../Bateryparameter.h"

#define TOL  1e-4f
#define FTOL 1e-3f

/* ---- BatteryParams_Init tests ---- */

static void test_init_nominal_capacity(void) {
    BatteryParams_t p;
    BatteryParams_Init(&p);
    ASSERT_FLOAT_EQ(p.Q_nominal, 2.6f, TOL);
}

static void test_init_nominal_voltage(void) {
    BatteryParams_t p;
    BatteryParams_Init(&p);
    ASSERT_FLOAT_EQ(p.V_nominal, 3.7f, TOL);
}

static void test_init_soc_points_first(void) {
    BatteryParams_t p;
    BatteryParams_Init(&p);
    ASSERT_FLOAT_EQ(p.SoC_points[0], 0.0f, TOL);
}

static void test_init_soc_points_last(void) {
    BatteryParams_t p;
    BatteryParams_Init(&p);
    ASSERT_FLOAT_EQ(p.SoC_points[6], 100.0f, TOL);
}

static void test_init_ocv_25c_at_0pct(void) {
    BatteryParams_t p;
    BatteryParams_Init(&p);
    ASSERT_FLOAT_EQ(p.OCV_25C[0], 3.0f, TOL);
}

static void test_init_ocv_25c_at_100pct(void) {
    BatteryParams_t p;
    BatteryParams_Init(&p);
    ASSERT_FLOAT_EQ(p.OCV_25C[6], 3.7f, TOL);
}

static void test_init_ocv_45c_at_0pct(void) {
    BatteryParams_t p;
    BatteryParams_Init(&p);
    ASSERT_FLOAT_EQ(p.OCV_45C[0], 3.2109f, FTOL);
}

static void test_init_ocv_45c_at_100pct(void) {
    BatteryParams_t p;
    BatteryParams_Init(&p);
    ASSERT_FLOAT_EQ(p.OCV_45C[6], 3.9009f, FTOL);
}

/* ---- Linear_Interpolate tests ---- */

static void test_interp_below_range_returns_first(void) {
    float x[] = {0.0f, 50.0f, 100.0f};
    float y[] = {1.0f, 5.0f, 10.0f};
    float result = Linear_Interpolate(x, y, 3, -10.0f);
    ASSERT_FLOAT_EQ(result, 1.0f, TOL);
}

static void test_interp_above_range_returns_last(void) {
    float x[] = {0.0f, 50.0f, 100.0f};
    float y[] = {1.0f, 5.0f, 10.0f};
    float result = Linear_Interpolate(x, y, 3, 200.0f);
    ASSERT_FLOAT_EQ(result, 10.0f, TOL);
}

static void test_interp_exact_first_point(void) {
    float x[] = {0.0f, 50.0f, 100.0f};
    float y[] = {1.0f, 5.0f, 10.0f};
    float result = Linear_Interpolate(x, y, 3, 0.0f);
    ASSERT_FLOAT_EQ(result, 1.0f, TOL);
}

static void test_interp_exact_last_point(void) {
    float x[] = {0.0f, 50.0f, 100.0f};
    float y[] = {1.0f, 5.0f, 10.0f};
    float result = Linear_Interpolate(x, y, 3, 100.0f);
    ASSERT_FLOAT_EQ(result, 10.0f, TOL);
}

static void test_interp_exact_middle_point(void) {
    float x[] = {0.0f, 50.0f, 100.0f};
    float y[] = {1.0f, 5.0f, 10.0f};
    float result = Linear_Interpolate(x, y, 3, 50.0f);
    ASSERT_FLOAT_EQ(result, 5.0f, TOL);
}

static void test_interp_midpoint_first_interval(void) {
    float x[] = {0.0f, 50.0f, 100.0f};
    float y[] = {0.0f, 10.0f, 20.0f};
    /* midpoint of [0,50] → should be 5.0 */
    float result = Linear_Interpolate(x, y, 3, 25.0f);
    ASSERT_FLOAT_EQ(result, 5.0f, TOL);
}

static void test_interp_midpoint_second_interval(void) {
    float x[] = {0.0f, 50.0f, 100.0f};
    float y[] = {0.0f, 10.0f, 20.0f};
    /* midpoint of [50,100] → should be 15.0 */
    float result = Linear_Interpolate(x, y, 3, 75.0f);
    ASSERT_FLOAT_EQ(result, 15.0f, TOL);
}

static void test_interp_uniform_spacing(void) {
    float x[] = {0.0f, 25.0f, 50.0f, 75.0f, 100.0f};
    float y[] = {0.0f, 25.0f, 50.0f, 75.0f, 100.0f};
    /* identity interpolation */
    ASSERT_FLOAT_EQ(Linear_Interpolate(x, y, 5, 37.5f), 37.5f, TOL);
}

/* ---- BatteryParams_GetOCV tests ---- */

static void test_getocv_soc0_temp25(void) {
    BatteryParams_t p;
    BatteryParams_Init(&p);
    float ocv = BatteryParams_GetOCV(&p, 0.0f, 25.0f);
    ASSERT_FLOAT_EQ(ocv, 3.0f, TOL);
}

static void test_getocv_soc100_temp25(void) {
    BatteryParams_t p;
    BatteryParams_Init(&p);
    float ocv = BatteryParams_GetOCV(&p, 100.0f, 25.0f);
    ASSERT_FLOAT_EQ(ocv, 3.7f, TOL);
}

static void test_getocv_soc0_temp45(void) {
    BatteryParams_t p;
    BatteryParams_Init(&p);
    float ocv = BatteryParams_GetOCV(&p, 0.0f, 45.0f);
    ASSERT_FLOAT_EQ(ocv, 3.2109f, FTOL);
}

static void test_getocv_soc100_temp45(void) {
    BatteryParams_t p;
    BatteryParams_Init(&p);
    float ocv = BatteryParams_GetOCV(&p, 100.0f, 45.0f);
    ASSERT_FLOAT_EQ(ocv, 3.9009f, FTOL);
}

static void test_getocv_uses_25c_table_at_boundary(void) {
    BatteryParams_t p;
    BatteryParams_Init(&p);
    /* temperature <= 35 uses 25C table */
    float ocv_35 = BatteryParams_GetOCV(&p, 0.0f, 35.0f);
    float ocv_25 = BatteryParams_GetOCV(&p, 0.0f, 25.0f);
    ASSERT_FLOAT_EQ(ocv_35, ocv_25, TOL);
}

static void test_getocv_uses_45c_table_above_boundary(void) {
    BatteryParams_t p;
    BatteryParams_Init(&p);
    /* temperature > 35 uses 45C table */
    float ocv_36 = BatteryParams_GetOCV(&p, 0.0f, 36.0f);
    float ocv_45 = BatteryParams_GetOCV(&p, 0.0f, 45.0f);
    ASSERT_FLOAT_EQ(ocv_36, ocv_45, TOL);
}

static void test_getocv_clamps_negative_soc(void) {
    BatteryParams_t p;
    BatteryParams_Init(&p);
    float ocv_neg = BatteryParams_GetOCV(&p, -10.0f, 25.0f);
    float ocv_0   = BatteryParams_GetOCV(&p, 0.0f, 25.0f);
    ASSERT_FLOAT_EQ(ocv_neg, ocv_0, TOL);
}

static void test_getocv_clamps_soc_above_100(void) {
    BatteryParams_t p;
    BatteryParams_Init(&p);
    float ocv_over = BatteryParams_GetOCV(&p, 110.0f, 25.0f);
    float ocv_100  = BatteryParams_GetOCV(&p, 100.0f, 25.0f);
    ASSERT_FLOAT_EQ(ocv_over, ocv_100, TOL);
}

static void test_getocv_monotonically_increases_at_25c(void) {
    BatteryParams_t p;
    BatteryParams_Init(&p);
    /* OCV should increase with SoC */
    ASSERT_TRUE(BatteryParams_GetOCV(&p, 50.0f, 25.0f) >
                BatteryParams_GetOCV(&p, 10.0f, 25.0f));
    ASSERT_TRUE(BatteryParams_GetOCV(&p, 90.0f, 25.0f) >
                BatteryParams_GetOCV(&p, 50.0f, 25.0f));
}

/* ---- BatteryParams_GetRo tests ---- */

static void test_getro_soc0_temp25(void) {
    BatteryParams_t p;
    BatteryParams_Init(&p);
    float ro = BatteryParams_GetRo(&p, 0.0f, 25.0f);
    ASSERT_FLOAT_EQ(ro, 0.0515f, FTOL);
}

static void test_getro_soc0_temp45(void) {
    BatteryParams_t p;
    BatteryParams_Init(&p);
    float ro = BatteryParams_GetRo(&p, 0.0f, 45.0f);
    ASSERT_FLOAT_EQ(ro, 0.0801f, FTOL);
}

static void test_getro_returns_positive_value(void) {
    BatteryParams_t p;
    BatteryParams_Init(&p);
    float ro = BatteryParams_GetRo(&p, 50.0f, 25.0f);
    ASSERT_FLOAT_GE(ro, 0.0f);
}

/* ---- BatteryParams_GetRtr tests ---- */

static void test_getrtr_soc0_temp25(void) {
    BatteryParams_t p;
    BatteryParams_Init(&p);
    float rtr = BatteryParams_GetRtr(&p, 0.0f, 25.0f);
    ASSERT_FLOAT_EQ(rtr, 0.0137f, FTOL);
}

static void test_getrtr_returns_positive_value(void) {
    BatteryParams_t p;
    BatteryParams_Init(&p);
    float rtr = BatteryParams_GetRtr(&p, 50.0f, 25.0f);
    ASSERT_FLOAT_GE(rtr, 0.0f);
}

/* ---- BatteryParams_GetTau tests ---- */

static void test_gettau_soc0_temp25(void) {
    BatteryParams_t p;
    BatteryParams_Init(&p);
    float tau = BatteryParams_GetTau(&p, 0.0f, 25.0f);
    ASSERT_FLOAT_EQ(tau, 104.65f, 0.1f);
}

static void test_gettau_returns_positive_value(void) {
    BatteryParams_t p;
    BatteryParams_Init(&p);
    float tau = BatteryParams_GetTau(&p, 50.0f, 25.0f);
    ASSERT_FLOAT_GE(tau, 0.0f);
}

/* ---- Main ---- */

int main(void) {
    printf("========================================\n");
    printf("Battery Parameters Unit Tests\n");
    printf("========================================\n");

    RUN_TEST(test_init_nominal_capacity);
    RUN_TEST(test_init_nominal_voltage);
    RUN_TEST(test_init_soc_points_first);
    RUN_TEST(test_init_soc_points_last);
    RUN_TEST(test_init_ocv_25c_at_0pct);
    RUN_TEST(test_init_ocv_25c_at_100pct);
    RUN_TEST(test_init_ocv_45c_at_0pct);
    RUN_TEST(test_init_ocv_45c_at_100pct);

    RUN_TEST(test_interp_below_range_returns_first);
    RUN_TEST(test_interp_above_range_returns_last);
    RUN_TEST(test_interp_exact_first_point);
    RUN_TEST(test_interp_exact_last_point);
    RUN_TEST(test_interp_exact_middle_point);
    RUN_TEST(test_interp_midpoint_first_interval);
    RUN_TEST(test_interp_midpoint_second_interval);
    RUN_TEST(test_interp_uniform_spacing);

    RUN_TEST(test_getocv_soc0_temp25);
    RUN_TEST(test_getocv_soc100_temp25);
    RUN_TEST(test_getocv_soc0_temp45);
    RUN_TEST(test_getocv_soc100_temp45);
    RUN_TEST(test_getocv_uses_25c_table_at_boundary);
    RUN_TEST(test_getocv_uses_45c_table_above_boundary);
    RUN_TEST(test_getocv_clamps_negative_soc);
    RUN_TEST(test_getocv_clamps_soc_above_100);
    RUN_TEST(test_getocv_monotonically_increases_at_25c);

    RUN_TEST(test_getro_soc0_temp25);
    RUN_TEST(test_getro_soc0_temp45);
    RUN_TEST(test_getro_returns_positive_value);

    RUN_TEST(test_getrtr_soc0_temp25);
    RUN_TEST(test_getrtr_returns_positive_value);

    RUN_TEST(test_gettau_soc0_temp25);
    RUN_TEST(test_gettau_returns_positive_value);

    PRINT_RESULTS();
    return RETURN_TEST_STATUS();
}
