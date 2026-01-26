/**
 ******************************************************************************
 * @file    aukf.c
 * @brief   Adaptive Unscented Kalman Filter implementation
 * @note    This module USES OCV for voltage prediction
 ******************************************************************************
 */

#include "aukf.h"
#include <string.h>
#include <math.h>

/* ============================================
 * PRIVATE MACROS
 * ============================================ */

#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
#define EPSILON 1e-9f

/* ============================================
 * PRIVATE FUNCTION PROTOTYPES
 * ============================================ */

static void AUKF_InitUKFParams(UKFParams_t *params);
static void AUKF_GenerateSigmaPoints(const AUKF_t *aukf, 
                                     float sigma_points[AUKF_NUM_SIGMA][AUKF_STATE_DIM]);
static void AUKF_StateTransition(const AUKF_t *aukf, const float x_in[AUKF_STATE_DIM],
                                float x_out[AUKF_STATE_DIM], float current, float dt);
static float AUKF_MeasurementFunction(const AUKF_t *aukf, 
                                     const float x[AUKF_STATE_DIM], float current);
static void AUKF_Prediction(AUKF_t *aukf, float SoC_ccm, float current, float dt,
                           float *x_pred, float P_pred[AUKF_STATE_DIM][AUKF_STATE_DIM],
                           float sigma_pred[AUKF_NUM_SIGMA][AUKF_STATE_DIM],
                           float *y_pred, float y_sigma[AUKF_NUM_SIGMA]);
static void AUKF_Correction(AUKF_t *aukf, const float x_pred[AUKF_STATE_DIM],
                           const float P_pred[AUKF_STATE_DIM][AUKF_STATE_DIM],
                           const float sigma_pred[AUKF_NUM_SIGMA][AUKF_STATE_DIM],
                           float y_pred, const float y_sigma[AUKF_NUM_SIGMA],
                           float y_meas, float x_corr[AUKF_STATE_DIM],
                           float P_corr[AUKF_STATE_DIM][AUKF_STATE_DIM]);
static void AUKF_AdaptiveNoiseUpdate(AUKF_t *aukf, float innovation, float residual);

/* ============================================
 * MATH HELPER FUNCTIONS
 * ============================================ */

static void matrix_copy_2x2(float dest[2][2], const float src[2][2]) {
    memcpy(dest, src, 4 * sizeof(float));
}

static void matrix_add_2x2(float result[2][2], const float A[2][2], const float B[2][2]) {
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            result[i][j] = A[i][j] + B[i][j];
        }
    }
}

static float safe_sqrt(float x) {
    return (x > 0.0f) ? sqrtf(x) : 0.0f;
}

/* ============================================
 * PUBLIC FUNCTIONS
 * ============================================ */

void AUKF_Init(AUKF_t *aukf, const BatteryParams_t *battery, 
               float initial_SoC, float temperature) {
    // Initialize state [SoC, V_tr]
    aukf->x[0] = CLAMP(initial_SoC, 0.0f, 100.0f);
    aukf->x[1] = 0.0f;  // V_tr starts at 0
    
    // Initialize state covariance P
    aukf->P[0][0] = 1e-6f;
    aukf->P[0][1] = 0.0f;
    aukf->P[1][0] = 0.0f;
    aukf->P[1][1] = 1.0f;
    
    // Initialize process noise covariance Q
    aukf->Q[0][0] = 0.1f;
    aukf->Q[0][1] = 0.0f;
    aukf->Q[1][0] = 0.0f;
    aukf->Q[1][1] = 0.1f;
    
    // Initialize measurement noise covariance R
    aukf->R = 0.25f;
    
    // Initialize UKF parameters
    AUKF_InitUKFParams(&aukf->ukf_params);
    
    // Initialize adaptive noise
    memset(&aukf->adaptive, 0, sizeof(AdaptiveNoise_t));
    
    // Set battery parameters
    aukf->battery = battery;
    aukf->temperature = temperature;
    
    // Initialize Kalman gain
    aukf->K_prev[0] = 0.0f;
    aukf->K_prev[1] = 0.0f;
    
    // Statistics
    aukf->update_count = 0;
    aukf->last_innovation = 0.0f;
    aukf->last_residual = 0.0f;
}

float AUKF_Update(AUKF_t *aukf, float SoC_predicted, float current, 
                  float voltage_measured, float dt) {
    /**
     * AUKF UPDATE - USES OCV FOR VOLTAGE PREDICTION
     * 
     * Algorithm:
     * 1. Prediction: Use SoC from CCM to predict voltage (uses OCV!)
     * 2. Correction: Compare predicted vs measured voltage
     * 3. Update: Correct SoC based on voltage error
     * 4. Adaptive: Adjust Q and R
     */
    
    float x_pred[AUKF_STATE_DIM];
    float P_pred[AUKF_STATE_DIM][AUKF_STATE_DIM];
    float sigma_pred[AUKF_NUM_SIGMA][AUKF_STATE_DIM];
    float y_pred;
    float y_sigma[AUKF_NUM_SIGMA];
    
    float x_corr[AUKF_STATE_DIM];
    float P_corr[AUKF_STATE_DIM][AUKF_STATE_DIM];
    
    // ════════════════════════════════════════════════════════════
    // PHASE 1: PREDICTION
    // ════════════════════════════════════════════════════════════
    
    AUKF_Prediction(aukf, SoC_predicted, current, dt,
                   x_pred, P_pred, sigma_pred, &y_pred, y_sigma);
    
    // ════════════════════════════════════════════════════════════
    // PHASE 2: CORRECTION
    // ════════════════════════════════════════════════════════════
    
    AUKF_Correction(aukf, x_pred, P_pred, sigma_pred,
                   y_pred, y_sigma, voltage_measured, x_corr, P_corr);
    
    // ════════════════════════════════════════════════════════════
    // PHASE 3: ADAPTIVE NOISE UPDATE
    // ════════════════════════════════════════════════════════════
    
    // Calculate residual
    float y_estimated = AUKF_MeasurementFunction(aukf, x_corr, current);
    float residual = voltage_measured - y_estimated;
    
    // Update adaptive noise
    AUKF_AdaptiveNoiseUpdate(aukf, aukf->last_innovation, residual);
    
    // ════════════════════════════════════════════════════════════
    // UPDATE STATE
    // ════════════════════════════════════════════════════════════
    
    aukf->x[0] = x_corr[0];
    aukf->x[1] = x_corr[1];
    matrix_copy_2x2(aukf->P, P_corr);
    
    aukf->last_residual = residual;
    aukf->update_count++;
    
    return aukf->x[0];  // Return corrected SoC
}

static void AUKF_InitUKFParams(UKFParams_t *params) {
    // UKF parameters
    params->alpha = 1.0f;
    params->beta = 1.0f;
    params->kappa = 0.0f;
    
    // Calculate lambda
    params->lambda = params->alpha * params->alpha * 
                     (AUKF_STATE_DIM + params->kappa) - AUKF_STATE_DIM;
    
    // Calculate weights for mean
    params->Wm[0] = params->lambda / (AUKF_STATE_DIM + params->lambda);
    for (int i = 1; i < AUKF_NUM_SIGMA; i++) {
        params->Wm[i] = 1.0f / (2.0f * (AUKF_STATE_DIM + params->lambda));
    }
    
    // Calculate weights for covariance
    params->Wc[0] = params->lambda / (AUKF_STATE_DIM + params->lambda) + 
                    (1.0f - params->alpha * params->alpha + params->beta);
    for (int i = 1; i < AUKF_NUM_SIGMA; i++) {
        params->Wc[i] = 1.0f / (2.0f * (AUKF_STATE_DIM + params->lambda));
    }
}

static void AUKF_GenerateSigmaPoints(const AUKF_t *aukf, 
                                     float sigma_points[AUKF_NUM_SIGMA][AUKF_STATE_DIM]) {
    /**
     * Generate 2L+1 sigma points
     * 
     * χ_0 = x
     * χ_i = x + sqrt((L+λ)P)_i     for i = 1,...,L
     * χ_i = x - sqrt((L+λ)P)_{i-L} for i = L+1,...,2L
     */
    
    float lambda = aukf->ukf_params.lambda;
    float scale = AUKF_STATE_DIM + lambda;
    
    // Calculate sqrt((L+λ)P) using Cholesky-like decomposition
    float sqrt_P[AUKF_STATE_DIM][AUKF_STATE_DIM];
    
    // Simple sqrt for 2x2 diagonal-dominant matrix
    sqrt_P[0][0] = safe_sqrt(scale * aukf->P[0][0]);
    sqrt_P[0][1] = 0.0f;
    sqrt_P[1][0] = 0.0f;
    sqrt_P[1][1] = safe_sqrt(scale * aukf->P[1][1]);
    
    // Central point
    sigma_points[0][0] = aukf->x[0];
    sigma_points[0][1] = aukf->x[1];
    
    // Positive spread
    for (int i = 0; i < AUKF_STATE_DIM; i++) {
        sigma_points[i+1][0] = aukf->x[0] + sqrt_P[0][i];
        sigma_points[i+1][1] = aukf->x[1] + sqrt_P[1][i];
    }
    
    // Negative spread
    for (int i = 0; i < AUKF_STATE_DIM; i++) {
        sigma_points[AUKF_STATE_DIM+i+1][0] = aukf->x[0] - sqrt_P[0][i];
        sigma_points[AUKF_STATE_DIM+i+1][1] = aukf->x[1] - sqrt_P[1][i];
    }
}

static void AUKF_StateTransition(const AUKF_t *aukf, const float x_in[AUKF_STATE_DIM],
                                float x_out[AUKF_STATE_DIM], float current, float dt) {
    /**
     * State transition function (CCM-based)
     * 
     * SoC_new = SoC - (100 × I × dt) / (3600 × Q)
     * V_tr_new = V_tr × exp(-dt/τ) + R_tr × (1 - exp(-dt/τ)) × I
     * 
     * NO OCV USED HERE - Only dynamics!
     */
    
    float SoC = x_in[0];
    float V_tr = x_in[1];
    
    // SoC update using Coulomb Counting
    float delta_SoC = (100.0f * current * dt) / (3600.0f * aukf->battery->Q_nominal);
    float SoC_new = SoC - delta_SoC;
    SoC_new = CLAMP(SoC_new, 0.0f, 100.0f);
    
    // V_tr update using RC dynamics
    float Rtr = BatteryParams_GetRtr(aukf->battery, SoC, aukf->temperature);
    float tau = BatteryParams_GetTau(aukf->battery, SoC, aukf->temperature);
    
    float exp_factor = expf(-dt / tau);
    float V_tr_new = V_tr * exp_factor + Rtr * (1.0f - exp_factor) * current;
    
    x_out[0] = SoC_new;
    x_out[1] = V_tr_new;
}

static float AUKF_MeasurementFunction(const AUKF_t *aukf, 
                                     const float x[AUKF_STATE_DIM], float current) {
    /**
     * ★★★ MEASUREMENT FUNCTION - OCV USED HERE! ★★★
     * 
     * V_terminal = V_ocv(SoC) - V_tr - R_o × I
     * 
     * This is where OCV lookup happens!
     */
    
    float SoC = x[0];
    float V_tr = x[1];
    
    // ★★★ LOOKUP OCV - OCV DIGUNAKAN! ★★★
    float V_ocv = BatteryParams_GetOCV(aukf->battery, SoC, aukf->temperature);
    
    // Get series resistance
    float Ro = BatteryParams_GetRo(aukf->battery, SoC, aukf->temperature);
    
    // ★★★ CALCULATE TERMINAL VOLTAGE ★★★
    float V_terminal = V_ocv - V_tr - Ro * current;
    
    return V_terminal;
}

static void AUKF_Prediction(AUKF_t *aukf, float SoC_ccm, float current, float dt,
                           float *x_pred, float P_pred[AUKF_STATE_DIM][AUKF_STATE_DIM],
                           float sigma_pred[AUKF_NUM_SIGMA][AUKF_STATE_DIM],
                           float *y_pred, float y_sigma[AUKF_NUM_SIGMA]) {
    /**
     * PREDICTION STEP
     * 1. Generate sigma points
     * 2. Propagate through state transition
     * 3. Calculate predicted state
     * 4. Propagate through measurement (uses OCV!)
     * 5. Calculate predicted measurement
     */
    
    float sigma_points[AUKF_NUM_SIGMA][AUKF_STATE_DIM];
    
    // Override current state SoC with CCM prediction
    float x_temp[AUKF_STATE_DIM];
    x_temp[0] = SoC_ccm;
    x_temp[1] = aukf->x[1];
    
    // Temporarily update state for sigma point generation
    float x_backup[AUKF_STATE_DIM];
    x_backup[0] = aukf->x[0];
    x_backup[1] = aukf->x[1];
    
    aukf->x[0] = SoC_ccm;
    
    // Step 1: Generate sigma points
    AUKF_GenerateSigmaPoints(aukf, sigma_points);
    
    // Restore state
    aukf->x[0] = x_backup[0];
    aukf->x[1] = x_backup[1];
    
    // Step 2: Propagate sigma points through state transition
    for (int i = 0; i < AUKF_NUM_SIGMA; i++) {
        AUKF_StateTransition(aukf, sigma_points[i], sigma_pred[i], current, dt);
    }
    
    // Step 3: Calculate predicted state (weighted mean)
    x_pred[0] = 0.0f;
    x_pred[1] = 0.0f;
    
    for (int i = 0; i < AUKF_NUM_SIGMA; i++) {
        x_pred[0] += aukf->ukf_params.Wm[i] * sigma_pred[i][0];
        x_pred[1] += aukf->ukf_params.Wm[i] * sigma_pred[i][1];
    }
    
    // Step 4: Calculate predicted covariance
    float P_temp[AUKF_STATE_DIM][AUKF_STATE_DIM] = {{0}};
    
    for (int i = 0; i < AUKF_NUM_SIGMA; i++) {
        float diff[AUKF_STATE_DIM];
        diff[0] = sigma_pred[i][0] - x_pred[0];
        diff[1] = sigma_pred[i][1] - x_pred[1];
        
        P_temp[0][0] += aukf->ukf_params.Wc[i] * diff[0] * diff[0];
        P_temp[0][1] += aukf->ukf_params.Wc[i] * diff[0] * diff[1];
        P_temp[1][0] += aukf->ukf_params.Wc[i] * diff[1] * diff[0];
        P_temp[1][1] += aukf->ukf_params.Wc[i] * diff[1] * diff[1];
    }
    
    // Add process noise
    matrix_add_2x2(P_pred, P_temp, aukf->Q);
    
    // Step 5: Propagate through measurement function (USES OCV!)
    for (int i = 0; i < AUKF_NUM_SIGMA; i++) {
        y_sigma[i] = AUKF_MeasurementFunction(aukf, sigma_pred[i], current);
    }
    
    // Step 6: Calculate predicted measurement
    *y_pred = 0.0f;
    for (int i = 0; i < AUKF_NUM_SIGMA; i++) {
        *y_pred += aukf->ukf_params.Wm[i] * y_sigma[i];
    }
}

static void AUKF_Correction(AUKF_t *aukf, const float x_pred[AUKF_STATE_DIM],
                           const float P_pred[AUKF_STATE_DIM][AUKF_STATE_DIM],
                           const float sigma_pred[AUKF_NUM_SIGMA][AUKF_STATE_DIM],
                           float y_pred, const float y_sigma[AUKF_NUM_SIGMA],
                           float y_meas, float x_corr[AUKF_STATE_DIM],
                           float P_corr[AUKF_STATE_DIM][AUKF_STATE_DIM]) {
    /**
     * CORRECTION STEP
     * 1. Calculate innovation covariance Pyy
     * 2. Calculate cross-covariance Pxy
     * 3. Calculate Kalman gain
     * 4. Correct state
     * 5. Update covariance
     */
    
    // Step 1: Innovation covariance
    float Pyy = aukf->R;
    for (int i = 0; i < AUKF_NUM_SIGMA; i++) {
        float diff_y = y_sigma[i] - y_pred;
        Pyy += aukf->ukf_params.Wc[i] * diff_y * diff_y;
    }
    
    // Step 2: Cross-covariance
    float Pxy[AUKF_STATE_DIM] = {0};
    for (int i = 0; i < AUKF_NUM_SIGMA; i++) {
        float diff_x[AUKF_STATE_DIM];
        diff_x[0] = sigma_pred[i][0] - x_pred[0];
        diff_x[1] = sigma_pred[i][1] - x_pred[1];
        
        float diff_y = y_sigma[i] - y_pred;
        
        Pxy[0] += aukf->ukf_params.Wc[i] * diff_x[0] * diff_y;
        Pxy[1] += aukf->ukf_params.Wc[i] * diff_x[1] * diff_y;
    }
    
    // Step 3: Kalman gain
    float K[AUKF_STATE_DIM];
    K[0] = Pxy[0] / (Pyy + EPSILON);
    K[1] = Pxy[1] / (Pyy + EPSILON);
    
    // Save for adaptive Q update
    aukf->K_prev[0] = K[0];
    aukf->K_prev[1] = K[1];
    
    // Step 4: Innovation
    float innovation = y_meas - y_pred;
    aukf->last_innovation = innovation;
    
    // Step 5: State correction
    x_corr[0] = x_pred[0] + K[0] * innovation;
    x_corr[1] = x_pred[1] + K[1] * innovation;
    
    // Clamp SoC
    x_corr[0] = CLAMP(x_corr[0], 0.0f, 100.0f);
    
    // Step 6: Covariance correction
    P_corr[0][0] = P_pred[0][0] - K[0] * Pyy * K[0];
    P_corr[0][1] = P_pred[0][1] - K[0] * Pyy * K[1];
    P_corr[1][0] = P_pred[1][0] - K[1] * Pyy * K[0];
    P_corr[1][1] = P_pred[1][1] - K[1] * Pyy * K[1];
    
    // Ensure positive definite
    if (P_corr[0][0] < 1e-6f) P_corr[0][0] = 1e-6f;
    if (P_corr[1][1] < 1e-6f) P_corr[1][1] = 1e-6f;
}

static void AUKF_AdaptiveNoiseUpdate(AUKF_t *aukf, float innovation, float residual) {
    /**
     * ADAPTIVE NOISE ESTIMATION
     * 
     * Q update: based on innovation
     * R update: based on residual
     */
    
    AdaptiveNoise_t *adapt = &aukf->adaptive;
    
    // Add to history
    adapt->innovation_history[adapt->history_index] = innovation;
    adapt->residual_history[adapt->history_index] = residual;
    
    adapt->history_index = (adapt->history_index + 1) % AUKF_WINDOW_SIZE;
    if (adapt->history_count < AUKF_WINDOW_SIZE) {
        adapt->history_count++;
    }
    
    // Need at least 2 samples
    if (adapt->history_count < 2) {
        return;
    }
    
    // Calculate innovation variance (Cd)
    float innovation_mean = 0.0f;
    for (int i = 0; i < adapt->history_count; i++) {
        innovation_mean += adapt->innovation_history[i];
    }
    innovation_mean /= adapt->history_count;
    
    float Cd = 0.0f;
    for (int i = 0; i < adapt->history_count; i++) {
        float diff = adapt->innovation_history[i] - innovation_mean;
        Cd += diff * diff;
    }
    Cd /= adapt->history_count;
    
    // Update Q: Q = K × Cd × K^T
    aukf->Q[0][0] = aukf->K_prev[0] * Cd * aukf->K_prev[0];
    aukf->Q[0][1] = aukf->K_prev[0] * Cd * aukf->K_prev[1];
    aukf->Q[1][0] = aukf->K_prev[1] * Cd * aukf->K_prev[0];
    aukf->Q[1][1] = aukf->K_prev[1] * Cd * aukf->K_prev[1];
    
    // Clamp Q to reasonable bounds
    aukf->Q[0][0] = CLAMP(aukf->Q[0][0], 1e-6f, 1.0f);
    aukf->Q[1][1] = CLAMP(aukf->Q[1][1], 1e-6f, 1.0f);
    
    // Calculate residual variance (Cr)
    float residual_mean = 0.0f;
    for (int i = 0; i < adapt->history_count; i++) {
        residual_mean += adapt->residual_history[i];
    }
    residual_mean /= adapt->history_count;
    
    float Cr = 0.0f;
    for (int i = 0; i < adapt->history_count; i++) {
        float diff = adapt->residual_history[i] - residual_mean;
        Cr += diff * diff;
    }
    Cr /= adapt->history_count;
    
    // Update R: R = Cr + H × P × H^T
    // H = [-1, 0] for this system
    aukf->R = Cr + aukf->P[0][0];
    
    // Clamp R to reasonable bounds
    aukf->R = CLAMP(aukf->R, 0.01f, 1.0f);
}

float AUKF_GetSoC(const AUKF_t *aukf) {
    return aukf->x[0];
}

float AUKF_GetVtr(const AUKF_t *aukf) {
    return aukf->x[1];
}

float AUKF_GetInnovation(const AUKF_t *aukf) {
    return aukf->last_innovation;
}