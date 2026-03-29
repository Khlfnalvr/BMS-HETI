/**
 * Unit tests for Adaptive Unscented Kalman Filter (AUKF.c)
 */

#include "test_framework.h"
#include "../AUKF.h"
#include "../Bateryparameter.h"

#define TOL   1e-4f
#define LOOSE 1.0f   /* loose tolerance for filter outputs */

/* Shared battery params */
static BatteryParams_t g_battery;

static void setup_battery(void) {
    BatteryParams_Init(&g_battery);
}

/* ---- Init tests ---- */

static void test_init_soc_set(void) {
    setup_battery();
    AUKF_t aukf;
    AUKF_Init(&aukf, &g_battery, 80.0f, 25.0f);
    ASSERT_FLOAT_EQ(aukf.x[0], 80.0f, TOL);
}

static void test_init_vtr_zero(void) {
    setup_battery();
    AUKF_t aukf;
    AUKF_Init(&aukf, &g_battery, 80.0f, 25.0f);
    ASSERT_FLOAT_EQ(aukf.x[1], 0.0f, TOL);
}

static void test_init_clamps_soc_below_zero(void) {
    setup_battery();
    AUKF_t aukf;
    AUKF_Init(&aukf, &g_battery, -20.0f, 25.0f);
    ASSERT_FLOAT_EQ(aukf.x[0], 0.0f, TOL);
}

static void test_init_clamps_soc_above_hundred(void) {
    setup_battery();
    AUKF_t aukf;
    AUKF_Init(&aukf, &g_battery, 120.0f, 25.0f);
    ASSERT_FLOAT_EQ(aukf.x[0], 100.0f, TOL);
}

static void test_init_temperature_stored(void) {
    setup_battery();
    AUKF_t aukf;
    AUKF_Init(&aukf, &g_battery, 50.0f, 35.0f);
    ASSERT_FLOAT_EQ(aukf.temperature, 35.0f, TOL);
}

static void test_init_battery_pointer_set(void) {
    setup_battery();
    AUKF_t aukf;
    AUKF_Init(&aukf, &g_battery, 50.0f, 25.0f);
    ASSERT_TRUE(aukf.battery == &g_battery);
}

static void test_init_update_count_zero(void) {
    setup_battery();
    AUKF_t aukf;
    AUKF_Init(&aukf, &g_battery, 50.0f, 25.0f);
    ASSERT_INT_EQ(aukf.update_count, 0);
}

static void test_init_covariance_positive(void) {
    setup_battery();
    AUKF_t aukf;
    AUKF_Init(&aukf, &g_battery, 50.0f, 25.0f);
    /* P must be positive definite: diagonal entries > 0 */
    ASSERT_TRUE(aukf.P[0][0] > 0.0f);
    ASSERT_TRUE(aukf.P[1][1] > 0.0f);
}

static void test_init_process_noise_positive(void) {
    setup_battery();
    AUKF_t aukf;
    AUKF_Init(&aukf, &g_battery, 50.0f, 25.0f);
    ASSERT_TRUE(aukf.Q[0][0] > 0.0f);
    ASSERT_TRUE(aukf.Q[1][1] > 0.0f);
}

static void test_init_measurement_noise_positive(void) {
    setup_battery();
    AUKF_t aukf;
    AUKF_Init(&aukf, &g_battery, 50.0f, 25.0f);
    ASSERT_TRUE(aukf.R > 0.0f);
}

/* ---- GetSoC / GetVtr tests ---- */

static void test_getsoc_returns_initial_soc(void) {
    setup_battery();
    AUKF_t aukf;
    AUKF_Init(&aukf, &g_battery, 65.0f, 25.0f);
    ASSERT_FLOAT_EQ(AUKF_GetSoC(&aukf), 65.0f, TOL);
}

static void test_getvtr_returns_zero_after_init(void) {
    setup_battery();
    AUKF_t aukf;
    AUKF_Init(&aukf, &g_battery, 65.0f, 25.0f);
    ASSERT_FLOAT_EQ(AUKF_GetVtr(&aukf), 0.0f, TOL);
}

/* ---- Update tests ---- */

static void test_update_returns_soc_in_range(void) {
    setup_battery();
    AUKF_t aukf;
    AUKF_Init(&aukf, &g_battery, 80.0f, 25.0f);
    /* Typical discharge: 1A, measured voltage ≈ OCV at 80% */
    float soc = AUKF_Update(&aukf, 80.0f, 1.0f, 3.5f, 1.0f);
    ASSERT_FLOAT_GE(soc, 0.0f);
    ASSERT_FLOAT_LE(soc, 100.0f);
}

static void test_update_increments_count(void) {
    setup_battery();
    AUKF_t aukf;
    AUKF_Init(&aukf, &g_battery, 80.0f, 25.0f);
    AUKF_Update(&aukf, 80.0f, 1.0f, 3.5f, 1.0f);
    AUKF_Update(&aukf, 79.5f, 1.0f, 3.5f, 1.0f);
    ASSERT_INT_EQ(aukf.update_count, 2);
}

static void test_update_multiple_calls_stable(void) {
    setup_battery();
    AUKF_t aukf;
    AUKF_Init(&aukf, &g_battery, 80.0f, 25.0f);
    float soc = 80.0f;
    for (int i = 0; i < 20; i++) {
        soc = AUKF_Update(&aukf, soc, 0.5f, 3.45f, 1.0f);
        ASSERT_FLOAT_GE(soc, 0.0f);
        ASSERT_FLOAT_LE(soc, 100.0f);
    }
}

static void test_update_with_zero_current_stable(void) {
    setup_battery();
    AUKF_t aukf;
    AUKF_Init(&aukf, &g_battery, 50.0f, 25.0f);
    float soc = AUKF_Update(&aukf, 50.0f, 0.0f, 3.22f, 1.0f);
    ASSERT_FLOAT_GE(soc, 0.0f);
    ASSERT_FLOAT_LE(soc, 100.0f);
}

static void test_update_at_high_temperature(void) {
    setup_battery();
    AUKF_t aukf;
    AUKF_Init(&aukf, &g_battery, 70.0f, 45.0f);
    float soc = AUKF_Update(&aukf, 70.0f, 1.0f, 3.6f, 1.0f);
    ASSERT_FLOAT_GE(soc, 0.0f);
    ASSERT_FLOAT_LE(soc, 100.0f);
}

static void test_update_near_full_soc(void) {
    setup_battery();
    AUKF_t aukf;
    AUKF_Init(&aukf, &g_battery, 99.0f, 25.0f);
    float soc = AUKF_Update(&aukf, 99.0f, -0.5f, 3.68f, 1.0f);
    ASSERT_FLOAT_GE(soc, 0.0f);
    ASSERT_FLOAT_LE(soc, 100.0f);
}

static void test_update_near_empty_soc(void) {
    setup_battery();
    AUKF_t aukf;
    AUKF_Init(&aukf, &g_battery, 2.0f, 25.0f);
    float soc = AUKF_Update(&aukf, 2.0f, 0.5f, 3.01f, 1.0f);
    ASSERT_FLOAT_GE(soc, 0.0f);
    ASSERT_FLOAT_LE(soc, 100.0f);
}

/* ---- GetInnovation tests ---- */

static void test_getinnovation_zero_before_update(void) {
    setup_battery();
    AUKF_t aukf;
    AUKF_Init(&aukf, &g_battery, 50.0f, 25.0f);
    /* last_innovation initialized to 0 */
    ASSERT_FLOAT_EQ(AUKF_GetInnovation(&aukf), 0.0f, TOL);
}

/* ---- Main ---- */

int main(void) {
    printf("========================================\n");
    printf("AUKF Unit Tests\n");
    printf("========================================\n");

    RUN_TEST(test_init_soc_set);
    RUN_TEST(test_init_vtr_zero);
    RUN_TEST(test_init_clamps_soc_below_zero);
    RUN_TEST(test_init_clamps_soc_above_hundred);
    RUN_TEST(test_init_temperature_stored);
    RUN_TEST(test_init_battery_pointer_set);
    RUN_TEST(test_init_update_count_zero);
    RUN_TEST(test_init_covariance_positive);
    RUN_TEST(test_init_process_noise_positive);
    RUN_TEST(test_init_measurement_noise_positive);

    RUN_TEST(test_getsoc_returns_initial_soc);
    RUN_TEST(test_getvtr_returns_zero_after_init);

    RUN_TEST(test_update_returns_soc_in_range);
    RUN_TEST(test_update_increments_count);
    RUN_TEST(test_update_multiple_calls_stable);
    RUN_TEST(test_update_with_zero_current_stable);
    RUN_TEST(test_update_at_high_temperature);
    RUN_TEST(test_update_near_full_soc);
    RUN_TEST(test_update_near_empty_soc);

    RUN_TEST(test_getinnovation_zero_before_update);

    PRINT_RESULTS();
    return RETURN_TEST_STATUS();
}
