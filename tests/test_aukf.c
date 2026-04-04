/**
 * test_aukf.c
 * Unit tests for AUKF.h / AUKF.c (Adaptive Unscented Kalman Filter)
 *
 * Covers:
 *  - AUKF_Init    : initial state, SoC clamping, V_tr = 0, covariance setup
 *  - AUKF_GetSoC  : returns current state SoC
 *  - AUKF_GetVtr  : returns current V_tr
 *  - AUKF_Update  : SoC stays in [0, 100], update_count increments,
 *                   discharge scenario trends SoC down,
 *                   innovation accessible via AUKF_GetInnovation
 */

#include "test_framework.h"
#include "AUKF.h"
#include "Bateryparameter.h"

/* ── Helper ───────────────────────────────────────────────────────── */

static void make_aukf(AUKF_t *aukf, float initial_soc, float temp) {
    static BatteryParams_t params; /* static so pointer stays valid */
    BatteryParams_Init(&params);
    AUKF_Init(aukf, &params, initial_soc, temp);
}

/* ── AUKF_Init ────────────────────────────────────────────────────── */

static void test_init_sets_soc(void) {
    AUKF_t aukf;
    make_aukf(&aukf, 80.0f, 25.0f);
    ASSERT_FLOAT_NEAR(80.0f, aukf.x[0], 1e-5f);
}

static void test_init_vtr_is_zero(void) {
    AUKF_t aukf;
    make_aukf(&aukf, 80.0f, 25.0f);
    ASSERT_FLOAT_NEAR(0.0f, aukf.x[1], 1e-6f);
}

static void test_init_clamps_soc_below_zero(void) {
    AUKF_t aukf;
    make_aukf(&aukf, -20.0f, 25.0f);
    ASSERT_FLOAT_NEAR(0.0f, aukf.x[0], 1e-6f);
}

static void test_init_clamps_soc_above_100(void) {
    AUKF_t aukf;
    make_aukf(&aukf, 110.0f, 25.0f);
    ASSERT_FLOAT_NEAR(100.0f, aukf.x[0], 1e-6f);
}

static void test_init_update_count_zero(void) {
    AUKF_t aukf;
    make_aukf(&aukf, 50.0f, 25.0f);
    ASSERT_INT_EQ(0, aukf.update_count);
}

static void test_init_P_diagonal_positive(void) {
    AUKF_t aukf;
    make_aukf(&aukf, 50.0f, 25.0f);
    ASSERT_TRUE(aukf.P[0][0] > 0.0f);
    ASSERT_TRUE(aukf.P[1][1] > 0.0f);
}

static void test_init_R_positive(void) {
    AUKF_t aukf;
    make_aukf(&aukf, 50.0f, 25.0f);
    ASSERT_TRUE(aukf.R > 0.0f);
}

static void test_init_stores_temperature(void) {
    AUKF_t aukf;
    make_aukf(&aukf, 50.0f, 35.0f);
    ASSERT_FLOAT_NEAR(35.0f, aukf.temperature, 1e-6f);
}

/* ── AUKF_GetSoC / AUKF_GetVtr ────────────────────────────────────── */

static void test_get_soc_returns_init_value(void) {
    AUKF_t aukf;
    make_aukf(&aukf, 65.0f, 25.0f);
    ASSERT_FLOAT_NEAR(65.0f, AUKF_GetSoC(&aukf), 1e-5f);
}

static void test_get_vtr_returns_zero_after_init(void) {
    AUKF_t aukf;
    make_aukf(&aukf, 65.0f, 25.0f);
    ASSERT_FLOAT_NEAR(0.0f, AUKF_GetVtr(&aukf), 1e-6f);
}

/* ── AUKF_Update ──────────────────────────────────────────────────── */

/*
 * Use a typical voltage ≈ OCV(50%) at 25°C = 3.22 V.
 * Current = 1 A, dt = 1 s → tiny SoC change.
 */

static void test_update_increments_update_count(void) {
    AUKF_t aukf;
    make_aukf(&aukf, 50.0f, 25.0f);
    AUKF_Update(&aukf, 50.0f, 1.0f, 3.22f, 1.0f);
    AUKF_Update(&aukf, 49.9f, 1.0f, 3.22f, 1.0f);
    ASSERT_INT_EQ(2, aukf.update_count);
}

static void test_update_soc_stays_in_valid_range(void) {
    AUKF_t aukf;
    make_aukf(&aukf, 50.0f, 25.0f);
    for (int i = 0; i < 20; i++) {
        float soc_pred = 50.0f - i * 0.5f;
        AUKF_Update(&aukf, soc_pred, 1.0f, 3.22f, 1.0f);
    }
    float soc = AUKF_GetSoC(&aukf);
    ASSERT_TRUE(soc >= 0.0f);
    ASSERT_TRUE(soc <= 100.0f);
}

static void test_update_discharge_trends_soc_downward(void) {
    /* Feed consistently decreasing CCM SoC with matching voltage.
     * After many steps the AUKF estimate should also decrease. */
    AUKF_t aukf;
    make_aukf(&aukf, 80.0f, 25.0f);
    float soc_ccm = 80.0f;
    float dt = 10.0f;
    float current = 1.0f;

    for (int i = 0; i < 30; i++) {
        /* delta = 100 * 1 * 1 * 10 / (3600 * 2.6) ≈ 0.107 % per step */
        soc_ccm -= (100.0f * current * dt) / (3600.0f * 2.6f);
        if (soc_ccm < 0.0f) soc_ccm = 0.0f;
        AUKF_Update(&aukf, soc_ccm, current, 3.15f, dt);
    }

    ASSERT_TRUE(AUKF_GetSoC(&aukf) < 80.0f);
}

static void test_update_charge_trends_soc_upward(void) {
    AUKF_t aukf;
    make_aukf(&aukf, 30.0f, 25.0f);
    float soc_ccm = 30.0f;
    float dt = 10.0f;
    float current = -1.0f; /* charging */

    for (int i = 0; i < 30; i++) {
        soc_ccm -= (100.0f * current * dt) / (3600.0f * 2.6f);
        if (soc_ccm > 100.0f) soc_ccm = 100.0f;
        AUKF_Update(&aukf, soc_ccm, current, 3.5f, dt);
    }

    ASSERT_TRUE(AUKF_GetSoC(&aukf) > 30.0f);
}

static void test_update_covariance_stays_positive(void) {
    AUKF_t aukf;
    make_aukf(&aukf, 50.0f, 25.0f);
    for (int i = 0; i < 15; i++) {
        AUKF_Update(&aukf, 50.0f, 1.0f, 3.22f, 1.0f);
    }
    ASSERT_TRUE(aukf.P[0][0] > 0.0f);
    ASSERT_TRUE(aukf.P[1][1] > 0.0f);
}

static void test_update_innovation_is_finite(void) {
    AUKF_t aukf;
    make_aukf(&aukf, 50.0f, 25.0f);
    AUKF_Update(&aukf, 50.0f, 1.0f, 3.22f, 1.0f);
    float innov = AUKF_GetInnovation(&aukf);
    /* Should not be NaN or Inf */
    ASSERT_TRUE(innov == innov);              /* NaN check */
    ASSERT_TRUE(innov > -100.0f && innov < 100.0f);
}

static void test_update_returns_soc_matching_getter(void) {
    AUKF_t aukf;
    make_aukf(&aukf, 50.0f, 25.0f);
    float returned = AUKF_Update(&aukf, 50.0f, 1.0f, 3.22f, 1.0f);
    ASSERT_FLOAT_NEAR(returned, AUKF_GetSoC(&aukf), 1e-6f);
}

/* ── Suite entry point ────────────────────────────────────────────── */

TEST_SUITE_BEGIN(aukf)
    RUN_TEST(test_init_sets_soc);
    RUN_TEST(test_init_vtr_is_zero);
    RUN_TEST(test_init_clamps_soc_below_zero);
    RUN_TEST(test_init_clamps_soc_above_100);
    RUN_TEST(test_init_update_count_zero);
    RUN_TEST(test_init_P_diagonal_positive);
    RUN_TEST(test_init_R_positive);
    RUN_TEST(test_init_stores_temperature);

    RUN_TEST(test_get_soc_returns_init_value);
    RUN_TEST(test_get_vtr_returns_zero_after_init);

    RUN_TEST(test_update_increments_update_count);
    RUN_TEST(test_update_soc_stays_in_valid_range);
    RUN_TEST(test_update_discharge_trends_soc_downward);
    RUN_TEST(test_update_charge_trends_soc_upward);
    RUN_TEST(test_update_covariance_stays_positive);
    RUN_TEST(test_update_innovation_is_finite);
    RUN_TEST(test_update_returns_soc_matching_getter);
TEST_SUITE_END()
