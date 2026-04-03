/**
 * test_cc.c - Unit tests for CoulombCounter (CC.c / CC.h)
 *
 * Formula under test:
 *   SoC_new = SoC_old - (100 * eta * I * dt) / (3600 * Q)
 *
 * Positive current  = discharge  → SoC decreases
 * Negative current  = charge     → SoC increases
 */

#include "unity.h"
#include "../CC.h"

/* ── helpers ─────────────────────────────────────────────────────────── */

/* Capacity 2.6 Ah, efficiency 1.0 */
static CoulombCounter_t make_cc(float initial_soc) {
    CoulombCounter_t cc;
    CoulombCounter_Init(&cc, initial_soc, 2.6f, 1.0f);
    return cc;
}

/* ── init tests ──────────────────────────────────────────────────────── */

TEST(init_stores_soc) {
    CoulombCounter_t cc = make_cc(80.0f);
    ASSERT_FLOAT_EQ(cc.SoC, 80.0f, 1e-5f);
}

TEST(init_stores_capacity) {
    CoulombCounter_t cc = make_cc(50.0f);
    ASSERT_FLOAT_EQ(cc.capacity, 2.6f, 1e-5f);
}

TEST(init_stores_eta) {
    CoulombCounter_t cc = make_cc(50.0f);
    ASSERT_FLOAT_EQ(cc.eta, 1.0f, 1e-5f);
}

TEST(init_zeroes_statistics) {
    CoulombCounter_t cc = make_cc(50.0f);
    ASSERT_INT_EQ(cc.update_count, 0);
    ASSERT_FLOAT_EQ(cc.total_Ah, 0.0f, 1e-5f);
}

TEST(init_clamps_soc_above_100) {
    CoulombCounter_t cc = make_cc(120.0f);
    ASSERT_FLOAT_EQ(cc.SoC, 100.0f, 1e-5f);
}

TEST(init_clamps_soc_below_0) {
    CoulombCounter_t cc = make_cc(-10.0f);
    ASSERT_FLOAT_EQ(cc.SoC, 0.0f, 1e-5f);
}

/* ── update: basic formula ───────────────────────────────────────────── */

TEST(update_discharge_decreases_soc) {
    CoulombCounter_t cc = make_cc(50.0f);
    /* 1 A for 1 s → delta = (100 * 1 * 1 * 1) / (3600 * 2.6) ≈ 0.01068 % */
    float soc = CoulombCounter_Update(&cc, 1.0f, 1.0f);
    ASSERT_TRUE(soc < 50.0f);
}

TEST(update_charge_increases_soc) {
    CoulombCounter_t cc = make_cc(50.0f);
    /* negative current = charge */
    float soc = CoulombCounter_Update(&cc, -1.0f, 1.0f);
    ASSERT_TRUE(soc > 50.0f);
}

TEST(update_formula_exact) {
    CoulombCounter_t cc = make_cc(50.0f);
    /* delta = (100 * 1.0 * 2.0 * 10.0) / (3600 * 2.6) = 2000/9360 ≈ 0.21368 % */
    float expected_delta = (100.0f * 1.0f * 2.0f * 10.0f) / (3600.0f * 2.6f);
    float soc = CoulombCounter_Update(&cc, 2.0f, 10.0f);
    ASSERT_FLOAT_EQ(soc, 50.0f - expected_delta, 1e-4f);
}

TEST(update_zero_current_no_change) {
    CoulombCounter_t cc = make_cc(75.0f);
    float soc = CoulombCounter_Update(&cc, 0.0f, 1.0f);
    ASSERT_FLOAT_EQ(soc, 75.0f, 1e-5f);
}

TEST(update_zero_dt_no_change) {
    CoulombCounter_t cc = make_cc(75.0f);
    float soc = CoulombCounter_Update(&cc, 5.0f, 0.0f);
    ASSERT_FLOAT_EQ(soc, 75.0f, 1e-5f);
}

/* ── update: clamping ────────────────────────────────────────────────── */

TEST(update_clamps_at_zero_on_overdischarge) {
    CoulombCounter_t cc = make_cc(0.5f);
    /* huge current for long time → would go very negative */
    float soc = CoulombCounter_Update(&cc, 100.0f, 3600.0f);
    ASSERT_FLOAT_EQ(soc, 0.0f, 1e-5f);
}

TEST(update_clamps_at_100_on_overcharge) {
    CoulombCounter_t cc = make_cc(99.5f);
    float soc = CoulombCounter_Update(&cc, -100.0f, 3600.0f);
    ASSERT_FLOAT_EQ(soc, 100.0f, 1e-5f);
}

/* ── update: statistics ──────────────────────────────────────────────── */

TEST(update_increments_update_count) {
    CoulombCounter_t cc = make_cc(50.0f);
    CoulombCounter_Update(&cc, 1.0f, 1.0f);
    CoulombCounter_Update(&cc, 1.0f, 1.0f);
    ASSERT_INT_EQ(cc.update_count, 2);
}

TEST(update_accumulates_total_ah) {
    CoulombCounter_t cc = make_cc(50.0f);
    /* 2 A for 1800 s = 2 * 1800 / 3600 = 1.0 Ah */
    CoulombCounter_Update(&cc, 2.0f, 1800.0f);
    ASSERT_FLOAT_EQ(cc.total_Ah, 1.0f, 1e-4f);
}

TEST(update_accumulates_total_ah_multiple_steps) {
    CoulombCounter_t cc = make_cc(50.0f);
    CoulombCounter_Update(&cc, 1.0f, 3600.0f);   /* 1 Ah */
    CoulombCounter_Update(&cc, 2.0f, 1800.0f);   /* 1 Ah */
    ASSERT_FLOAT_EQ(cc.total_Ah, 2.0f, 1e-4f);
}

/* ── reset ───────────────────────────────────────────────────────────── */

TEST(reset_sets_new_soc) {
    CoulombCounter_t cc = make_cc(50.0f);
    CoulombCounter_Update(&cc, 1.0f, 100.0f);
    CoulombCounter_Reset(&cc, 90.0f);
    ASSERT_FLOAT_EQ(cc.SoC, 90.0f, 1e-5f);
}

TEST(reset_clamps_soc) {
    CoulombCounter_t cc = make_cc(50.0f);
    CoulombCounter_Reset(&cc, 110.0f);
    ASSERT_FLOAT_EQ(cc.SoC, 100.0f, 1e-5f);
}

TEST(reset_zeroes_statistics) {
    CoulombCounter_t cc = make_cc(50.0f);
    CoulombCounter_Update(&cc, 1.0f, 100.0f);
    CoulombCounter_Reset(&cc, 50.0f);
    ASSERT_INT_EQ(cc.update_count, 0);
    ASSERT_FLOAT_EQ(cc.total_Ah, 0.0f, 1e-5f);
}

/* ── getter ──────────────────────────────────────────────────────────── */

TEST(get_soc_returns_current_soc) {
    CoulombCounter_t cc = make_cc(60.0f);
    ASSERT_FLOAT_EQ(CoulombCounter_GetSoC(&cc), 60.0f, 1e-5f);
    CoulombCounter_Update(&cc, 1.0f, 1.0f);
    ASSERT_FLOAT_EQ(CoulombCounter_GetSoC(&cc), cc.SoC, 1e-5f);
}

/* ── cumulative discharge test ───────────────────────────────────────── */

TEST(full_discharge_from_100_reaches_0) {
    CoulombCounter_t cc = make_cc(100.0f);
    /* Discharge at 2.6 A for 3600 s = exactly 2.6 Ah = 100 % */
    CoulombCounter_Update(&cc, 2.6f, 3600.0f);
    ASSERT_FLOAT_EQ(CoulombCounter_GetSoC(&cc), 0.0f, 1e-3f);
}

/* ── entry point ─────────────────────────────────────────────────────── */

int main(void) {
    printf("=== CoulombCounter Tests ===\n");

    RUN_TEST(init_stores_soc);
    RUN_TEST(init_stores_capacity);
    RUN_TEST(init_stores_eta);
    RUN_TEST(init_zeroes_statistics);
    RUN_TEST(init_clamps_soc_above_100);
    RUN_TEST(init_clamps_soc_below_0);

    RUN_TEST(update_discharge_decreases_soc);
    RUN_TEST(update_charge_increases_soc);
    RUN_TEST(update_formula_exact);
    RUN_TEST(update_zero_current_no_change);
    RUN_TEST(update_zero_dt_no_change);
    RUN_TEST(update_clamps_at_zero_on_overdischarge);
    RUN_TEST(update_clamps_at_100_on_overcharge);
    RUN_TEST(update_increments_update_count);
    RUN_TEST(update_accumulates_total_ah);
    RUN_TEST(update_accumulates_total_ah_multiple_steps);

    RUN_TEST(reset_sets_new_soc);
    RUN_TEST(reset_clamps_soc);
    RUN_TEST(reset_zeroes_statistics);

    RUN_TEST(get_soc_returns_current_soc);
    RUN_TEST(full_discharge_from_100_reaches_0);

    PRINT_RESULTS();
    return RESULTS_OK() ? 0 : 1;
}
