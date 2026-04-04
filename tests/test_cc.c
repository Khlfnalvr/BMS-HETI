/**
 * test_cc.c
 * Unit tests for CC.h / CC.c (CoulombCounter)
 *
 * Covers:
 *  - CoulombCounter_Init  : initial SoC, capacity, eta, clamping
 *  - CoulombCounter_Update: formula correctness, discharge/charge direction,
 *                           SoC clamping at 0 and 100, stat tracking
 *  - CoulombCounter_Reset : SoC value, stats cleared
 *  - CoulombCounter_GetSoC: returns current SoC
 */

#include "test_framework.h"
#include "CC.h"

/* ── CoulombCounter_Init ──────────────────────────────────────────── */

static void test_init_sets_soc(void) {
    CoulombCounter_t cc;
    CoulombCounter_Init(&cc, 80.0f, 2.6f, 1.0f);
    ASSERT_FLOAT_NEAR(80.0f, cc.SoC, 1e-5f);
}

static void test_init_sets_capacity(void) {
    CoulombCounter_t cc;
    CoulombCounter_Init(&cc, 50.0f, 2.6f, 1.0f);
    ASSERT_FLOAT_NEAR(2.6f, cc.capacity, 1e-6f);
}

static void test_init_sets_eta(void) {
    CoulombCounter_t cc;
    CoulombCounter_Init(&cc, 50.0f, 2.6f, 0.98f);
    ASSERT_FLOAT_NEAR(0.98f, cc.eta, 1e-6f);
}

static void test_init_clamps_negative_soc_to_zero(void) {
    CoulombCounter_t cc;
    CoulombCounter_Init(&cc, -10.0f, 2.6f, 1.0f);
    ASSERT_FLOAT_NEAR(0.0f, cc.SoC, 1e-6f);
}

static void test_init_clamps_soc_above_100(void) {
    CoulombCounter_t cc;
    CoulombCounter_Init(&cc, 120.0f, 2.6f, 1.0f);
    ASSERT_FLOAT_NEAR(100.0f, cc.SoC, 1e-6f);
}

static void test_init_clears_stats(void) {
    CoulombCounter_t cc;
    CoulombCounter_Init(&cc, 50.0f, 2.6f, 1.0f);
    ASSERT_INT_EQ(0, cc.update_count);
    ASSERT_FLOAT_NEAR(0.0f, cc.total_Ah, 1e-6f);
}

/* ── CoulombCounter_Update ────────────────────────────────────────── */

/*
 * Formula: delta_SoC = (100 * eta * I * dt) / (3600 * Q)
 *          SoC_new   = SoC_prev - delta_SoC
 *
 * Exact scenario: I = 2.6 A, dt = 360 s, Q = 2.6 Ah, eta = 1.0
 *   delta_SoC = (100 * 1.0 * 2.6 * 360) / (3600 * 2.6) = 10.0 %
 */

static void test_update_discharge_reduces_soc(void) {
    CoulombCounter_t cc;
    CoulombCounter_Init(&cc, 50.0f, 2.6f, 1.0f);
    float soc = CoulombCounter_Update(&cc, 2.6f, 360.0f); /* discharge */
    ASSERT_FLOAT_NEAR(40.0f, soc, 1e-4f);
}

static void test_update_charge_increases_soc(void) {
    CoulombCounter_t cc;
    CoulombCounter_Init(&cc, 50.0f, 2.6f, 1.0f);
    float soc = CoulombCounter_Update(&cc, -2.6f, 360.0f); /* charge */
    ASSERT_FLOAT_NEAR(60.0f, soc, 1e-4f);
}

static void test_update_formula_with_eta(void) {
    /* eta = 0.5 halves the effective Ah */
    CoulombCounter_t cc;
    CoulombCounter_Init(&cc, 50.0f, 2.6f, 0.5f);
    /* delta = 100 * 0.5 * 2.6 * 360 / (3600 * 2.6) = 5.0 % */
    float soc = CoulombCounter_Update(&cc, 2.6f, 360.0f);
    ASSERT_FLOAT_NEAR(45.0f, soc, 1e-4f);
}

static void test_update_clamps_to_zero_on_over_discharge(void) {
    CoulombCounter_t cc;
    CoulombCounter_Init(&cc, 5.0f, 2.6f, 1.0f);
    /* Pull 100 % worth of charge – should clamp at 0 */
    float soc = CoulombCounter_Update(&cc, 2.6f, 3600.0f);
    ASSERT_FLOAT_NEAR(0.0f, soc, 1e-6f);
}

static void test_update_clamps_to_100_on_over_charge(void) {
    CoulombCounter_t cc;
    CoulombCounter_Init(&cc, 95.0f, 2.6f, 1.0f);
    /* Push 100 % worth of charge – should clamp at 100 */
    float soc = CoulombCounter_Update(&cc, -2.6f, 3600.0f);
    ASSERT_FLOAT_NEAR(100.0f, soc, 1e-6f);
}

static void test_update_increments_update_count(void) {
    CoulombCounter_t cc;
    CoulombCounter_Init(&cc, 50.0f, 2.6f, 1.0f);
    CoulombCounter_Update(&cc, 1.0f, 1.0f);
    CoulombCounter_Update(&cc, 1.0f, 1.0f);
    ASSERT_INT_EQ(2, cc.update_count);
}

static void test_update_accumulates_total_ah(void) {
    /* total_Ah += (I * dt) / 3600 */
    CoulombCounter_t cc;
    CoulombCounter_Init(&cc, 80.0f, 2.6f, 1.0f);
    CoulombCounter_Update(&cc, 2.6f, 3600.0f); /* 2.6 Ah discharged */
    ASSERT_FLOAT_NEAR(2.6f, cc.total_Ah, 1e-4f);
}

static void test_update_returns_same_as_getter(void) {
    CoulombCounter_t cc;
    CoulombCounter_Init(&cc, 70.0f, 2.6f, 1.0f);
    float returned = CoulombCounter_Update(&cc, 1.0f, 10.0f);
    ASSERT_FLOAT_NEAR(returned, CoulombCounter_GetSoC(&cc), 1e-6f);
}

static void test_update_zero_current_no_change(void) {
    CoulombCounter_t cc;
    CoulombCounter_Init(&cc, 60.0f, 2.6f, 1.0f);
    float soc = CoulombCounter_Update(&cc, 0.0f, 1.0f);
    ASSERT_FLOAT_NEAR(60.0f, soc, 1e-6f);
}

/* ── CoulombCounter_Reset ─────────────────────────────────────────── */

static void test_reset_sets_new_soc(void) {
    CoulombCounter_t cc;
    CoulombCounter_Init(&cc, 80.0f, 2.6f, 1.0f);
    CoulombCounter_Update(&cc, 1.0f, 100.0f);
    CoulombCounter_Reset(&cc, 50.0f);
    ASSERT_FLOAT_NEAR(50.0f, cc.SoC, 1e-6f);
}

static void test_reset_clears_update_count(void) {
    CoulombCounter_t cc;
    CoulombCounter_Init(&cc, 80.0f, 2.6f, 1.0f);
    CoulombCounter_Update(&cc, 1.0f, 100.0f);
    CoulombCounter_Reset(&cc, 50.0f);
    ASSERT_INT_EQ(0, cc.update_count);
}

static void test_reset_clears_total_ah(void) {
    CoulombCounter_t cc;
    CoulombCounter_Init(&cc, 80.0f, 2.6f, 1.0f);
    CoulombCounter_Update(&cc, 2.6f, 3600.0f);
    CoulombCounter_Reset(&cc, 50.0f);
    ASSERT_FLOAT_NEAR(0.0f, cc.total_Ah, 1e-6f);
}

static void test_reset_clamps_negative_soc(void) {
    CoulombCounter_t cc;
    CoulombCounter_Init(&cc, 50.0f, 2.6f, 1.0f);
    CoulombCounter_Reset(&cc, -5.0f);
    ASSERT_FLOAT_NEAR(0.0f, cc.SoC, 1e-6f);
}

static void test_reset_clamps_soc_above_100(void) {
    CoulombCounter_t cc;
    CoulombCounter_Init(&cc, 50.0f, 2.6f, 1.0f);
    CoulombCounter_Reset(&cc, 110.0f);
    ASSERT_FLOAT_NEAR(100.0f, cc.SoC, 1e-6f);
}

/* ── CoulombCounter_GetSoC ────────────────────────────────────────── */

static void test_get_soc_returns_initial(void) {
    CoulombCounter_t cc;
    CoulombCounter_Init(&cc, 75.0f, 2.6f, 1.0f);
    ASSERT_FLOAT_NEAR(75.0f, CoulombCounter_GetSoC(&cc), 1e-6f);
}

/* ── Suite entry point ────────────────────────────────────────────── */

TEST_SUITE_BEGIN(cc)
    RUN_TEST(test_init_sets_soc);
    RUN_TEST(test_init_sets_capacity);
    RUN_TEST(test_init_sets_eta);
    RUN_TEST(test_init_clamps_negative_soc_to_zero);
    RUN_TEST(test_init_clamps_soc_above_100);
    RUN_TEST(test_init_clears_stats);

    RUN_TEST(test_update_discharge_reduces_soc);
    RUN_TEST(test_update_charge_increases_soc);
    RUN_TEST(test_update_formula_with_eta);
    RUN_TEST(test_update_clamps_to_zero_on_over_discharge);
    RUN_TEST(test_update_clamps_to_100_on_over_charge);
    RUN_TEST(test_update_increments_update_count);
    RUN_TEST(test_update_accumulates_total_ah);
    RUN_TEST(test_update_returns_same_as_getter);
    RUN_TEST(test_update_zero_current_no_change);

    RUN_TEST(test_reset_sets_new_soc);
    RUN_TEST(test_reset_clears_update_count);
    RUN_TEST(test_reset_clears_total_ah);
    RUN_TEST(test_reset_clamps_negative_soc);
    RUN_TEST(test_reset_clamps_soc_above_100);

    RUN_TEST(test_get_soc_returns_initial);
TEST_SUITE_END()
