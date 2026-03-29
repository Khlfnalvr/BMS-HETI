/**
 * Unit tests for Coulomb Counter (CC.c)
 */

#include "test_framework.h"
#include "../CC.h"

/* Tolerance for float comparisons */
#define TOL 1e-4f

/* Battery capacity used in tests */
#define TEST_CAPACITY 2.6f
#define TEST_ETA      1.0f

/* ---- Init tests ---- */

static void test_init_sets_soc(void) {
    CoulombCounter_t cc;
    CoulombCounter_Init(&cc, 80.0f, TEST_CAPACITY, TEST_ETA);
    ASSERT_FLOAT_EQ(cc.SoC, 80.0f, TOL);
}

static void test_init_sets_capacity(void) {
    CoulombCounter_t cc;
    CoulombCounter_Init(&cc, 50.0f, TEST_CAPACITY, TEST_ETA);
    ASSERT_FLOAT_EQ(cc.capacity, TEST_CAPACITY, TOL);
}

static void test_init_sets_eta(void) {
    CoulombCounter_t cc;
    CoulombCounter_Init(&cc, 50.0f, TEST_CAPACITY, 0.98f);
    ASSERT_FLOAT_EQ(cc.eta, 0.98f, TOL);
}

static void test_init_resets_statistics(void) {
    CoulombCounter_t cc;
    CoulombCounter_Init(&cc, 50.0f, TEST_CAPACITY, TEST_ETA);
    ASSERT_INT_EQ(cc.update_count, 0);
    ASSERT_FLOAT_EQ(cc.total_Ah, 0.0f, TOL);
}

static void test_init_clamps_soc_below_zero(void) {
    CoulombCounter_t cc;
    CoulombCounter_Init(&cc, -10.0f, TEST_CAPACITY, TEST_ETA);
    ASSERT_FLOAT_EQ(cc.SoC, 0.0f, TOL);
}

static void test_init_clamps_soc_above_hundred(void) {
    CoulombCounter_t cc;
    CoulombCounter_Init(&cc, 150.0f, TEST_CAPACITY, TEST_ETA);
    ASSERT_FLOAT_EQ(cc.SoC, 100.0f, TOL);
}

static void test_init_accepts_zero_soc(void) {
    CoulombCounter_t cc;
    CoulombCounter_Init(&cc, 0.0f, TEST_CAPACITY, TEST_ETA);
    ASSERT_FLOAT_EQ(cc.SoC, 0.0f, TOL);
}

static void test_init_accepts_hundred_soc(void) {
    CoulombCounter_t cc;
    CoulombCounter_Init(&cc, 100.0f, TEST_CAPACITY, TEST_ETA);
    ASSERT_FLOAT_EQ(cc.SoC, 100.0f, TOL);
}

/* ---- Update tests ---- */

static void test_update_zero_current_no_change(void) {
    CoulombCounter_t cc;
    CoulombCounter_Init(&cc, 60.0f, TEST_CAPACITY, TEST_ETA);
    float soc = CoulombCounter_Update(&cc, 0.0f, 1.0f);
    ASSERT_FLOAT_EQ(soc, 60.0f, TOL);
}

static void test_update_positive_current_decreases_soc(void) {
    CoulombCounter_t cc;
    CoulombCounter_Init(&cc, 60.0f, TEST_CAPACITY, TEST_ETA);
    float soc = CoulombCounter_Update(&cc, 1.0f, 3600.0f);
    /*
     * delta_SoC = 100 * 1.0 * 1.0 * 3600 / (3600 * 2.6)
     *           = 100 / 2.6 ≈ 38.46 %
     * expected SoC ≈ 60 - 38.46 = 21.54 %
     */
    float expected = 60.0f - (100.0f * TEST_ETA * 1.0f * 3600.0f) / (3600.0f * TEST_CAPACITY);
    ASSERT_FLOAT_EQ(soc, expected, TOL);
}

static void test_update_negative_current_increases_soc(void) {
    CoulombCounter_t cc;
    CoulombCounter_Init(&cc, 40.0f, TEST_CAPACITY, TEST_ETA);
    float soc = CoulombCounter_Update(&cc, -1.0f, 3600.0f);
    float expected = 40.0f + (100.0f * TEST_ETA * 1.0f * 3600.0f) / (3600.0f * TEST_CAPACITY);
    ASSERT_FLOAT_EQ(soc, expected, TOL);
}

static void test_update_clamps_soc_at_zero(void) {
    CoulombCounter_t cc;
    CoulombCounter_Init(&cc, 5.0f, TEST_CAPACITY, TEST_ETA);
    /* Large discharge drives SoC well below 0 */
    float soc = CoulombCounter_Update(&cc, 10.0f, 3600.0f);
    ASSERT_FLOAT_EQ(soc, 0.0f, TOL);
}

static void test_update_clamps_soc_at_hundred(void) {
    CoulombCounter_t cc;
    CoulombCounter_Init(&cc, 95.0f, TEST_CAPACITY, TEST_ETA);
    /* Large charge drives SoC well above 100 */
    float soc = CoulombCounter_Update(&cc, -10.0f, 3600.0f);
    ASSERT_FLOAT_EQ(soc, 100.0f, TOL);
}

static void test_update_increments_update_count(void) {
    CoulombCounter_t cc;
    CoulombCounter_Init(&cc, 50.0f, TEST_CAPACITY, TEST_ETA);
    CoulombCounter_Update(&cc, 0.5f, 1.0f);
    CoulombCounter_Update(&cc, 0.5f, 1.0f);
    ASSERT_INT_EQ(cc.update_count, 2);
}

static void test_update_accumulates_total_ah(void) {
    CoulombCounter_t cc;
    CoulombCounter_Init(&cc, 50.0f, TEST_CAPACITY, TEST_ETA);
    /* current=1A, dt=3600s → 1 Ah */
    CoulombCounter_Update(&cc, 1.0f, 3600.0f);
    ASSERT_FLOAT_EQ(cc.total_Ah, 1.0f, TOL);
}

static void test_update_small_timestep(void) {
    CoulombCounter_t cc;
    CoulombCounter_Init(&cc, 80.0f, TEST_CAPACITY, TEST_ETA);
    float soc = CoulombCounter_Update(&cc, 1.0f, 1.0f);
    float expected = 80.0f - (100.0f * 1.0f * 1.0f * 1.0f) / (3600.0f * TEST_CAPACITY);
    ASSERT_FLOAT_EQ(soc, expected, 1e-3f);
}

/* ---- Reset tests ---- */

static void test_reset_changes_soc(void) {
    CoulombCounter_t cc;
    CoulombCounter_Init(&cc, 50.0f, TEST_CAPACITY, TEST_ETA);
    CoulombCounter_Update(&cc, 1.0f, 10.0f);
    CoulombCounter_Reset(&cc, 90.0f);
    ASSERT_FLOAT_EQ(cc.SoC, 90.0f, TOL);
}

static void test_reset_clears_statistics(void) {
    CoulombCounter_t cc;
    CoulombCounter_Init(&cc, 50.0f, TEST_CAPACITY, TEST_ETA);
    CoulombCounter_Update(&cc, 1.0f, 10.0f);
    CoulombCounter_Reset(&cc, 90.0f);
    ASSERT_INT_EQ(cc.update_count, 0);
    ASSERT_FLOAT_EQ(cc.total_Ah, 0.0f, TOL);
}

static void test_reset_clamps_soc_below_zero(void) {
    CoulombCounter_t cc;
    CoulombCounter_Init(&cc, 50.0f, TEST_CAPACITY, TEST_ETA);
    CoulombCounter_Reset(&cc, -5.0f);
    ASSERT_FLOAT_EQ(cc.SoC, 0.0f, TOL);
}

static void test_reset_clamps_soc_above_hundred(void) {
    CoulombCounter_t cc;
    CoulombCounter_Init(&cc, 50.0f, TEST_CAPACITY, TEST_ETA);
    CoulombCounter_Reset(&cc, 110.0f);
    ASSERT_FLOAT_EQ(cc.SoC, 100.0f, TOL);
}

/* ---- GetSoC tests ---- */

static void test_getsoc_returns_current_soc(void) {
    CoulombCounter_t cc;
    CoulombCounter_Init(&cc, 75.0f, TEST_CAPACITY, TEST_ETA);
    ASSERT_FLOAT_EQ(CoulombCounter_GetSoC(&cc), 75.0f, TOL);
}

static void test_getsoc_reflects_update(void) {
    CoulombCounter_t cc;
    CoulombCounter_Init(&cc, 60.0f, TEST_CAPACITY, TEST_ETA);
    CoulombCounter_Update(&cc, 1.0f, 3600.0f);
    float expected = 60.0f - (100.0f / TEST_CAPACITY);
    ASSERT_FLOAT_EQ(CoulombCounter_GetSoC(&cc), expected, TOL);
}

/* ---- Main ---- */

int main(void) {
    printf("========================================\n");
    printf("Coulomb Counter Unit Tests\n");
    printf("========================================\n");

    RUN_TEST(test_init_sets_soc);
    RUN_TEST(test_init_sets_capacity);
    RUN_TEST(test_init_sets_eta);
    RUN_TEST(test_init_resets_statistics);
    RUN_TEST(test_init_clamps_soc_below_zero);
    RUN_TEST(test_init_clamps_soc_above_hundred);
    RUN_TEST(test_init_accepts_zero_soc);
    RUN_TEST(test_init_accepts_hundred_soc);

    RUN_TEST(test_update_zero_current_no_change);
    RUN_TEST(test_update_positive_current_decreases_soc);
    RUN_TEST(test_update_negative_current_increases_soc);
    RUN_TEST(test_update_clamps_soc_at_zero);
    RUN_TEST(test_update_clamps_soc_at_hundred);
    RUN_TEST(test_update_increments_update_count);
    RUN_TEST(test_update_accumulates_total_ah);
    RUN_TEST(test_update_small_timestep);

    RUN_TEST(test_reset_changes_soc);
    RUN_TEST(test_reset_clears_statistics);
    RUN_TEST(test_reset_clamps_soc_below_zero);
    RUN_TEST(test_reset_clamps_soc_above_hundred);

    RUN_TEST(test_getsoc_returns_current_soc);
    RUN_TEST(test_getsoc_reflects_update);

    PRINT_RESULTS();
    return RETURN_TEST_STATUS();
}
