/**
 * test_aukf.c - Unit tests for AUKF (AUKF.c / AUKF.h)
 *
 * Covers:
 *  - AUKF_Init  (initial state, covariance, clamping)
 *  - AUKF_GetSoC / AUKF_GetVtr / AUKF_GetInnovation
 *  - AUKF_Update  (output range, count, convergence direction)
 */

#include "unity.h"
#include "../AUKF.h"
#include "../Bateryparameter.h"

/* ── shared fixture ──────────────────────────────────────────────────── */

static BatteryParams_t bp;
static AUKF_t aukf;

static void setup(float initial_soc, float temperature) {
    BatteryParams_Init(&bp);
    AUKF_Init(&aukf, &bp, initial_soc, temperature);
}

/* ── AUKF_Init ───────────────────────────────────────────────────────── */

TEST(init_soc_state_set) {
    setup(80.0f, 25.0f);
    ASSERT_FLOAT_EQ(aukf.x[0], 80.0f, 1e-5f);
}

TEST(init_vtr_starts_at_zero) {
    setup(80.0f, 25.0f);
    ASSERT_FLOAT_EQ(aukf.x[1], 0.0f, 1e-5f);
}

TEST(init_soc_clamped_above_100) {
    setup(150.0f, 25.0f);
    ASSERT_FLOAT_EQ(aukf.x[0], 100.0f, 1e-5f);
}

TEST(init_soc_clamped_below_0) {
    setup(-20.0f, 25.0f);
    ASSERT_FLOAT_EQ(aukf.x[0], 0.0f, 1e-5f);
}

TEST(init_diagonal_covariance_positive) {
    setup(50.0f, 25.0f);
    ASSERT_TRUE(aukf.P[0][0] > 0.0f);
    ASSERT_TRUE(aukf.P[1][1] > 0.0f);
}

TEST(init_off_diagonal_covariance_zero) {
    setup(50.0f, 25.0f);
    ASSERT_FLOAT_EQ(aukf.P[0][1], 0.0f, 1e-5f);
    ASSERT_FLOAT_EQ(aukf.P[1][0], 0.0f, 1e-5f);
}

TEST(init_process_noise_positive) {
    setup(50.0f, 25.0f);
    ASSERT_TRUE(aukf.Q[0][0] > 0.0f);
    ASSERT_TRUE(aukf.Q[1][1] > 0.0f);
}

TEST(init_measurement_noise_positive) {
    setup(50.0f, 25.0f);
    ASSERT_TRUE(aukf.R > 0.0f);
}

TEST(init_update_count_zero) {
    setup(50.0f, 25.0f);
    ASSERT_INT_EQ(aukf.update_count, 0);
}

TEST(init_battery_pointer_set) {
    setup(50.0f, 25.0f);
    ASSERT_TRUE(aukf.battery == &bp);
}

/* ── Getters ─────────────────────────────────────────────────────────── */

TEST(get_soc_returns_state_x0) {
    setup(65.0f, 25.0f);
    ASSERT_FLOAT_EQ(AUKF_GetSoC(&aukf), 65.0f, 1e-5f);
}

TEST(get_vtr_returns_state_x1) {
    setup(65.0f, 25.0f);
    ASSERT_FLOAT_EQ(AUKF_GetVtr(&aukf), 0.0f, 1e-5f);
}

TEST(get_innovation_initially_zero) {
    setup(65.0f, 25.0f);
    ASSERT_FLOAT_EQ(AUKF_GetInnovation(&aukf), 0.0f, 1e-5f);
}

/* ── AUKF_Update: output validity ────────────────────────────────────── */

TEST(update_returns_soc_in_valid_range) {
    setup(50.0f, 25.0f);
    float soc = AUKF_Update(&aukf, 50.0f, 1.0f, 3.3f, 1.0f);
    ASSERT_FLOAT_GE(soc, 0.0f);
    ASSERT_FLOAT_LE(soc, 100.0f);
}

TEST(update_increments_update_count) {
    setup(50.0f, 25.0f);
    AUKF_Update(&aukf, 50.0f, 1.0f, 3.3f, 1.0f);
    AUKF_Update(&aukf, 49.9f, 1.0f, 3.3f, 1.0f);
    ASSERT_INT_EQ(aukf.update_count, 2);
}

TEST(update_covariance_remains_positive_definite) {
    setup(50.0f, 25.0f);
    AUKF_Update(&aukf, 50.0f, 1.0f, 3.3f, 1.0f);
    ASSERT_TRUE(aukf.P[0][0] > 0.0f);
    ASSERT_TRUE(aukf.P[1][1] > 0.0f);
}

TEST(update_with_zero_current_soc_stays_in_range) {
    setup(50.0f, 25.0f);
    float ocv_at_50 = BatteryParams_GetOCV(&bp, 50.0f, 25.0f);
    float soc = AUKF_Update(&aukf, 50.0f, 0.0f, ocv_at_50, 1.0f);
    ASSERT_FLOAT_GE(soc, 0.0f);
    ASSERT_FLOAT_LE(soc, 100.0f);
}

/* ── AUKF_Update: correction direction ──────────────────────────────── */

TEST(update_high_voltage_pushes_soc_up) {
    /*
     * Init at SoC=30% but repeatedly feed the voltage consistent with
     * SoC=80% (OCV ≈ 3.42 V at 25°C, zero current).
     * The initial P[0][0] is tiny (1e-6), so the gain starts near zero
     * and grows over time.  After many consistent steps the filter should
     * have moved above the initial prediction of 30%.
     */
    setup(30.0f, 25.0f);
    float high_voltage = BatteryParams_GetOCV(&bp, 80.0f, 25.0f);
    float soc = 30.0f;
    for (int i = 0; i < 50; i++) {
        soc = AUKF_Update(&aukf, soc, 0.0f, high_voltage, 1.0f);
    }
    ASSERT_TRUE(soc > 30.0f);
}

TEST(update_low_voltage_pushes_soc_down) {
    /*
     * Init at SoC=80%, repeatedly feed voltage consistent with SoC=30%.
     * Same reasoning: after enough steps the correction accumulates and
     * SoC should fall below the initial 80%.
     */
    setup(80.0f, 25.0f);
    float low_voltage = BatteryParams_GetOCV(&bp, 30.0f, 25.0f);
    float soc = 80.0f;
    for (int i = 0; i < 50; i++) {
        soc = AUKF_Update(&aukf, soc, 0.0f, low_voltage, 1.0f);
    }
    ASSERT_TRUE(soc < 80.0f);
}

/* ── AUKF_Update: repeated updates converge ─────────────────────────── */

TEST(update_repeated_converges_toward_true_soc) {
    /*
     * Over many steps at zero current with the correct voltage for
     * SoC=50%, the filter should stay near 50%.
     */
    setup(50.0f, 25.0f);
    float true_ocv = BatteryParams_GetOCV(&bp, 50.0f, 25.0f);

    float soc = 50.0f;
    for (int i = 0; i < 20; i++) {
        soc = AUKF_Update(&aukf, soc, 0.0f, true_ocv, 1.0f);
    }
    /* After 20 consistent steps should remain within ±5 % of 50 */
    ASSERT_FLOAT_GE(soc, 45.0f);
    ASSERT_FLOAT_LE(soc, 55.0f);
}

/* ── Temperature effect ──────────────────────────────────────────────── */

TEST(update_at_45c_stays_in_range) {
    setup(50.0f, 45.0f);
    float ocv = BatteryParams_GetOCV(&bp, 50.0f, 45.0f);
    float soc = AUKF_Update(&aukf, 50.0f, 0.0f, ocv, 1.0f);
    ASSERT_FLOAT_GE(soc, 0.0f);
    ASSERT_FLOAT_LE(soc, 100.0f);
}

/* ── entry point ─────────────────────────────────────────────────────── */

int main(void) {
    printf("=== AUKF Tests ===\n");

    RUN_TEST(init_soc_state_set);
    RUN_TEST(init_vtr_starts_at_zero);
    RUN_TEST(init_soc_clamped_above_100);
    RUN_TEST(init_soc_clamped_below_0);
    RUN_TEST(init_diagonal_covariance_positive);
    RUN_TEST(init_off_diagonal_covariance_zero);
    RUN_TEST(init_process_noise_positive);
    RUN_TEST(init_measurement_noise_positive);
    RUN_TEST(init_update_count_zero);
    RUN_TEST(init_battery_pointer_set);

    RUN_TEST(get_soc_returns_state_x0);
    RUN_TEST(get_vtr_returns_state_x1);
    RUN_TEST(get_innovation_initially_zero);

    RUN_TEST(update_returns_soc_in_valid_range);
    RUN_TEST(update_increments_update_count);
    RUN_TEST(update_covariance_remains_positive_definite);
    RUN_TEST(update_with_zero_current_soc_stays_in_range);

    RUN_TEST(update_high_voltage_pushes_soc_up);
    RUN_TEST(update_low_voltage_pushes_soc_down);
    RUN_TEST(update_repeated_converges_toward_true_soc);
    RUN_TEST(update_at_45c_stays_in_range);

    PRINT_RESULTS();
    return RESULTS_OK() ? 0 : 1;
}
