/**
 * test_cc.c
 * Unit tests for the Coulomb Counting (CC) module.
 *
 * Formula under test:
 *   delta_SoC = (100 * eta * I * dt) / (3600 * Q)
 *   SoC_new   = SoC_prev - delta_SoC   (clamped to [0, 100])
 */

#include "test_framework.h"
#include "../CC.h"

/* ── helpers ──────────────────────────────────────────────────────────────── */

static void make_cc(CoulombCounter_t *cc, float soc, float cap, float eta) {
    CoulombCounter_Init(cc, soc, cap, eta);
}

/* ── Init tests ───────────────────────────────────────────────────────────── */

static void test_init_normal(void) {
    CoulombCounter_t cc;
    make_cc(&cc, 50.0f, 2.6f, 1.0f);

    TEST_ASSERT_EQUAL_FLOAT(50.0f, cc.SoC);
    TEST_ASSERT_EQUAL_FLOAT(2.6f,  cc.capacity);
    TEST_ASSERT_EQUAL_FLOAT(1.0f,  cc.eta);
    TEST_ASSERT_EQUAL_INT(0, (int)cc.update_count);
    TEST_ASSERT_EQUAL_FLOAT(0.0f,  cc.total_Ah);
}

static void test_init_clamps_soc_above_100(void) {
    CoulombCounter_t cc;
    make_cc(&cc, 120.0f, 2.6f, 1.0f);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, cc.SoC);
}

static void test_init_clamps_soc_below_0(void) {
    CoulombCounter_t cc;
    make_cc(&cc, -10.0f, 2.6f, 1.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, cc.SoC);
}

static void test_init_soc_at_boundary_0(void) {
    CoulombCounter_t cc;
    make_cc(&cc, 0.0f, 2.6f, 1.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, cc.SoC);
}

static void test_init_soc_at_boundary_100(void) {
    CoulombCounter_t cc;
    make_cc(&cc, 100.0f, 2.6f, 1.0f);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, cc.SoC);
}

/* ── Update tests ─────────────────────────────────────────────────────────── */

static void test_update_discharge_decreases_soc(void) {
    CoulombCounter_t cc;
    make_cc(&cc, 50.0f, 2.6f, 1.0f);

    float soc = CoulombCounter_Update(&cc, 1.0f, 1.0f);

    /* delta = (100 * 1.0 * 1.0 * 1.0) / (3600 * 2.6) = 0.010684 */
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 49.9893f, soc);
    TEST_ASSERT_TRUE(soc < 50.0f);
}

static void test_update_charge_increases_soc(void) {
    CoulombCounter_t cc;
    make_cc(&cc, 50.0f, 2.6f, 1.0f);

    float soc = CoulombCounter_Update(&cc, -1.0f, 1.0f);

    /* negative current → charging → SoC increases */
    TEST_ASSERT_TRUE(soc > 50.0f);
}

static void test_update_formula_accuracy(void) {
    CoulombCounter_t cc;
    make_cc(&cc, 80.0f, 2.6f, 1.0f);

    /* 10 A discharge for 360 s → delta = (100*1*10*360)/(3600*2.6) = 38.46% */
    float soc = CoulombCounter_Update(&cc, 10.0f, 360.0f);
    float expected = 80.0f - (100.0f * 1.0f * 10.0f * 360.0f) / (3600.0f * 2.6f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, expected, soc);
}

static void test_update_efficiency_applied(void) {
    CoulombCounter_t cc;
    make_cc(&cc, 50.0f, 2.6f, 0.9f);

    /* With eta=0.9: delta = (100*0.9*1.0*1.0)/(3600*2.6) = 0.009615 */
    CoulombCounter_Update(&cc, 1.0f, 1.0f);
    float expected = 50.0f - (100.0f * 0.9f * 1.0f * 1.0f) / (3600.0f * 2.6f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, expected, cc.SoC);
}

static void test_update_clamps_at_zero(void) {
    CoulombCounter_t cc;
    make_cc(&cc, 1.0f, 2.6f, 1.0f);

    /* Huge discharge: current=100A, dt=3600s → SoC would go deeply negative */
    float soc = CoulombCounter_Update(&cc, 100.0f, 3600.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, soc);
}

static void test_update_clamps_at_hundred(void) {
    CoulombCounter_t cc;
    make_cc(&cc, 99.0f, 2.6f, 1.0f);

    /* Charging with large current: SoC would exceed 100 */
    float soc = CoulombCounter_Update(&cc, -100.0f, 3600.0f);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, soc);
}

static void test_update_zero_current_no_change(void) {
    CoulombCounter_t cc;
    make_cc(&cc, 60.0f, 2.6f, 1.0f);

    float soc = CoulombCounter_Update(&cc, 0.0f, 1.0f);
    TEST_ASSERT_EQUAL_FLOAT(60.0f, soc);
}

static void test_update_increments_count(void) {
    CoulombCounter_t cc;
    make_cc(&cc, 50.0f, 2.6f, 1.0f);

    CoulombCounter_Update(&cc, 1.0f, 1.0f);
    CoulombCounter_Update(&cc, 1.0f, 1.0f);
    CoulombCounter_Update(&cc, 1.0f, 1.0f);

    TEST_ASSERT_EQUAL_INT(3, (int)cc.update_count);
}

static void test_update_accumulates_total_ah(void) {
    CoulombCounter_t cc;
    make_cc(&cc, 50.0f, 2.6f, 1.0f);

    /* 2 A for 3600 s → 2 Ah */
    CoulombCounter_Update(&cc, 2.0f, 3600.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 2.0f, cc.total_Ah);
}

static void test_update_accumulates_total_ah_multiple(void) {
    CoulombCounter_t cc;
    make_cc(&cc, 100.0f, 100.0f, 1.0f);

    CoulombCounter_Update(&cc, 1.0f, 1800.0f);  /* 0.5 Ah */
    CoulombCounter_Update(&cc, 1.0f, 1800.0f);  /* 0.5 Ah */
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.0f, cc.total_Ah);
}

/* ── Reset tests ──────────────────────────────────────────────────────────── */

static void test_reset_sets_new_soc(void) {
    CoulombCounter_t cc;
    make_cc(&cc, 50.0f, 2.6f, 1.0f);
    CoulombCounter_Update(&cc, 1.0f, 1.0f);

    CoulombCounter_Reset(&cc, 80.0f);
    TEST_ASSERT_EQUAL_FLOAT(80.0f, cc.SoC);
}

static void test_reset_clears_stats(void) {
    CoulombCounter_t cc;
    make_cc(&cc, 50.0f, 2.6f, 1.0f);
    CoulombCounter_Update(&cc, 1.0f, 1.0f);
    CoulombCounter_Update(&cc, 1.0f, 1.0f);

    CoulombCounter_Reset(&cc, 50.0f);
    TEST_ASSERT_EQUAL_INT(0, (int)cc.update_count);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, cc.total_Ah);
}

static void test_reset_clamps_soc(void) {
    CoulombCounter_t cc;
    make_cc(&cc, 50.0f, 2.6f, 1.0f);
    CoulombCounter_Reset(&cc, 150.0f);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, cc.SoC);
}

/* ── Getter test ──────────────────────────────────────────────────────────── */

static void test_get_soc_returns_current_soc(void) {
    CoulombCounter_t cc;
    make_cc(&cc, 75.0f, 2.6f, 1.0f);
    TEST_ASSERT_EQUAL_FLOAT(75.0f, CoulombCounter_GetSoC(&cc));

    CoulombCounter_Update(&cc, 1.0f, 1.0f);
    TEST_ASSERT_EQUAL_FLOAT(cc.SoC, CoulombCounter_GetSoC(&cc));
}

/* ── Main ─────────────────────────────────────────────────────────────────── */

int main(void) {
    TEST_SUITE_BEGIN("CoulombCounter");

    RUN_TEST(test_init_normal);
    RUN_TEST(test_init_clamps_soc_above_100);
    RUN_TEST(test_init_clamps_soc_below_0);
    RUN_TEST(test_init_soc_at_boundary_0);
    RUN_TEST(test_init_soc_at_boundary_100);

    RUN_TEST(test_update_discharge_decreases_soc);
    RUN_TEST(test_update_charge_increases_soc);
    RUN_TEST(test_update_formula_accuracy);
    RUN_TEST(test_update_efficiency_applied);
    RUN_TEST(test_update_clamps_at_zero);
    RUN_TEST(test_update_clamps_at_hundred);
    RUN_TEST(test_update_zero_current_no_change);
    RUN_TEST(test_update_increments_count);
    RUN_TEST(test_update_accumulates_total_ah);
    RUN_TEST(test_update_accumulates_total_ah_multiple);

    RUN_TEST(test_reset_sets_new_soc);
    RUN_TEST(test_reset_clears_stats);
    RUN_TEST(test_reset_clamps_soc);

    RUN_TEST(test_get_soc_returns_current_soc);

    TEST_SUITE_END();
    return 0;
}
