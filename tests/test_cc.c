/**
 * Unit tests for Coulomb Counting (CC) module
 */

#include "unity.h"
#include "../cc.h"

/* ============================================================
 * Helpers
 * ============================================================ */

static CoulombCounter_t cc;

/* ============================================================
 * Init tests
 * ============================================================ */

static void test_init_sets_soc(void) {
    CoulombCounter_Init(&cc, 80.0f, 2.6f, 1.0f);
    TEST_ASSERT_EQUAL_FLOAT(80.0f, CoulombCounter_GetSoC(&cc), 1e-4f);
}

static void test_init_clamps_soc_above_100(void) {
    CoulombCounter_Init(&cc, 120.0f, 2.6f, 1.0f);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, CoulombCounter_GetSoC(&cc), 1e-4f);
}

static void test_init_clamps_soc_below_0(void) {
    CoulombCounter_Init(&cc, -10.0f, 2.6f, 1.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, CoulombCounter_GetSoC(&cc), 1e-4f);
}

static void test_init_sets_capacity(void) {
    CoulombCounter_Init(&cc, 50.0f, 3.0f, 1.0f);
    TEST_ASSERT_EQUAL_FLOAT(3.0f, cc.capacity, 1e-5f);
}

static void test_init_sets_eta(void) {
    CoulombCounter_Init(&cc, 50.0f, 2.6f, 0.98f);
    TEST_ASSERT_EQUAL_FLOAT(0.98f, cc.eta, 1e-5f);
}

static void test_init_resets_update_count(void) {
    CoulombCounter_Init(&cc, 50.0f, 2.6f, 1.0f);
    TEST_ASSERT_EQUAL_INT(0, (int)cc.update_count);
}

static void test_init_resets_total_ah(void) {
    CoulombCounter_Init(&cc, 50.0f, 2.6f, 1.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, cc.total_Ah, 1e-6f);
}

/* ============================================================
 * Update tests
 * ============================================================ */

static void test_update_zero_current_no_change(void) {
    CoulombCounter_Init(&cc, 50.0f, 2.6f, 1.0f);
    float soc = CoulombCounter_Update(&cc, 0.0f, 1.0f);
    TEST_ASSERT_EQUAL_FLOAT(50.0f, soc, 1e-4f);
}

static void test_update_positive_current_decreases_soc(void) {
    /* Discharge: positive current should lower SoC */
    CoulombCounter_Init(&cc, 50.0f, 2.6f, 1.0f);
    float soc = CoulombCounter_Update(&cc, 1.0f, 1.0f);
    TEST_ASSERT(soc < 50.0f);
}

static void test_update_negative_current_increases_soc(void) {
    /* Charge: negative current should raise SoC */
    CoulombCounter_Init(&cc, 50.0f, 2.6f, 1.0f);
    float soc = CoulombCounter_Update(&cc, -1.0f, 1.0f);
    TEST_ASSERT(soc > 50.0f);
}

static void test_update_formula_correctness(void) {
    /*
     * delta_SoC = (100 * eta * I * dt) / (3600 * Q)
     *           = (100 * 1.0 * 1.0 * 3600) / (3600 * 2.6)
     *           = 100 / 2.6 ≈ 38.4615 %
     * New SoC   = 50 - 38.4615 ≈ 11.5385 %
     */
    CoulombCounter_Init(&cc, 50.0f, 2.6f, 1.0f);
    float soc = CoulombCounter_Update(&cc, 1.0f, 3600.0f);
    float expected = 50.0f - (100.0f * 1.0f * 1.0f * 3600.0f) / (3600.0f * 2.6f);
    TEST_ASSERT_EQUAL_FLOAT(expected, soc, 1e-3f);
}

static void test_update_eta_scales_delta(void) {
    /* eta=0.5 halves the capacity loss */
    CoulombCounter_Init(&cc, 50.0f, 2.6f, 0.5f);
    float soc_half = CoulombCounter_Update(&cc, 1.0f, 3600.0f);

    CoulombCounter_Init(&cc, 50.0f, 2.6f, 1.0f);
    float soc_full = CoulombCounter_Update(&cc, 1.0f, 3600.0f);

    /* delta at eta=0.5 should be half that at eta=1.0 */
    float delta_half = 50.0f - soc_half;
    float delta_full = 50.0f - soc_full;
    TEST_ASSERT_EQUAL_FLOAT(delta_full / 2.0f, delta_half, 1e-3f);
}

static void test_update_clamps_soc_at_zero(void) {
    CoulombCounter_Init(&cc, 1.0f, 2.6f, 1.0f);
    /* Large discharge */
    float soc = CoulombCounter_Update(&cc, 100.0f, 3600.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, soc, 1e-4f);
}

static void test_update_clamps_soc_at_100(void) {
    CoulombCounter_Init(&cc, 99.0f, 2.6f, 1.0f);
    /* Large charge (negative current) */
    float soc = CoulombCounter_Update(&cc, -100.0f, 3600.0f);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, soc, 1e-4f);
}

static void test_update_increments_update_count(void) {
    CoulombCounter_Init(&cc, 50.0f, 2.6f, 1.0f);
    CoulombCounter_Update(&cc, 0.0f, 1.0f);
    CoulombCounter_Update(&cc, 0.0f, 1.0f);
    TEST_ASSERT_EQUAL_INT(2, (int)cc.update_count);
}

static void test_update_accumulates_total_ah(void) {
    CoulombCounter_Init(&cc, 50.0f, 2.6f, 1.0f);
    CoulombCounter_Update(&cc, 2.0f, 3600.0f);  /* 2 A * 1 h = 2 Ah */
    TEST_ASSERT_EQUAL_FLOAT(2.0f, cc.total_Ah, 1e-3f);
}

/* ============================================================
 * Reset tests
 * ============================================================ */

static void test_reset_sets_new_soc(void) {
    CoulombCounter_Init(&cc, 50.0f, 2.6f, 1.0f);
    CoulombCounter_Update(&cc, 1.0f, 1.0f);
    CoulombCounter_Reset(&cc, 75.0f);
    TEST_ASSERT_EQUAL_FLOAT(75.0f, CoulombCounter_GetSoC(&cc), 1e-4f);
}

static void test_reset_clears_total_ah(void) {
    CoulombCounter_Init(&cc, 50.0f, 2.6f, 1.0f);
    CoulombCounter_Update(&cc, 1.0f, 3600.0f);
    CoulombCounter_Reset(&cc, 50.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, cc.total_Ah, 1e-6f);
}

static void test_reset_clears_update_count(void) {
    CoulombCounter_Init(&cc, 50.0f, 2.6f, 1.0f);
    CoulombCounter_Update(&cc, 1.0f, 1.0f);
    CoulombCounter_Reset(&cc, 50.0f);
    TEST_ASSERT_EQUAL_INT(0, (int)cc.update_count);
}

static void test_reset_clamps_above_100(void) {
    CoulombCounter_Init(&cc, 50.0f, 2.6f, 1.0f);
    CoulombCounter_Reset(&cc, 150.0f);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, CoulombCounter_GetSoC(&cc), 1e-4f);
}

/* ============================================================
 * Test suite entry point
 * ============================================================ */

void test_cc_run(void) {
    UNITY_BEGIN("Coulomb Counting (CC)");

    RUN_TEST(test_init_sets_soc);
    RUN_TEST(test_init_clamps_soc_above_100);
    RUN_TEST(test_init_clamps_soc_below_0);
    RUN_TEST(test_init_sets_capacity);
    RUN_TEST(test_init_sets_eta);
    RUN_TEST(test_init_resets_update_count);
    RUN_TEST(test_init_resets_total_ah);

    RUN_TEST(test_update_zero_current_no_change);
    RUN_TEST(test_update_positive_current_decreases_soc);
    RUN_TEST(test_update_negative_current_increases_soc);
    RUN_TEST(test_update_formula_correctness);
    RUN_TEST(test_update_eta_scales_delta);
    RUN_TEST(test_update_clamps_soc_at_zero);
    RUN_TEST(test_update_clamps_soc_at_100);
    RUN_TEST(test_update_increments_update_count);
    RUN_TEST(test_update_accumulates_total_ah);

    RUN_TEST(test_reset_sets_new_soc);
    RUN_TEST(test_reset_clears_total_ah);
    RUN_TEST(test_reset_clears_update_count);
    RUN_TEST(test_reset_clamps_above_100);
}
