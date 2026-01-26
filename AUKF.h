/**
 ******************************************************************************
 * @file    aukf.h
 * @brief   Adaptive Unscented Kalman Filter for SoC estimation
 * @author  Your Name
 * @date    2024
 ******************************************************************************
 */

#ifndef AUKF_H
#define AUKF_H

#include <stdint.h>
#include <stdbool.h>
#include "Bateryparameter.h"

/* ============================================
 * AUKF CONFIGURATION
 * ============================================ */

#define AUKF_STATE_DIM          2       // State dimension [SoC, V_tr]
#define AUKF_NUM_SIGMA          5       // 2*L+1 sigma points
#define AUKF_WINDOW_SIZE        10      // Moving window for adaptation

/* ============================================
 * AUKF STRUCTURES
 * ============================================ */

/* UKF Parameters */
typedef struct {
    float alpha;            // Spread of sigma points (0.001 to 1)
    float beta;             // Prior knowledge (2 for Gaussian)
    float kappa;            // Secondary scaling parameter
    float lambda;           // Calculated scaling parameter
    
    float Wm[AUKF_NUM_SIGMA];   // Weights for mean
    float Wc[AUKF_NUM_SIGMA];   // Weights for covariance
    
} UKFParams_t;

/* Adaptive Noise Estimator */
typedef struct {
    float innovation_history[AUKF_WINDOW_SIZE];
    float residual_history[AUKF_WINDOW_SIZE];
    uint8_t history_index;
    uint8_t history_count;
    
} AdaptiveNoise_t;

/* Main AUKF Structure */
typedef struct {
    // State vector [SoC, V_tr]
    float x[AUKF_STATE_DIM];
    
    // State covariance matrix (2x2)
    float P[AUKF_STATE_DIM][AUKF_STATE_DIM];
    
    // Process noise covariance (2x2)
    float Q[AUKF_STATE_DIM][AUKF_STATE_DIM];
    
    // Measurement noise covariance (scalar)
    float R;
    
    // UKF parameters
    UKFParams_t ukf_params;
    
    // Adaptive noise estimator
    AdaptiveNoise_t adaptive;
    
    // Battery parameters
    const BatteryParams_t *battery;
    
    // Temperature
    float temperature;
    
    // Previous Kalman gain (for adaptive Q)
    float K_prev[AUKF_STATE_DIM];
    
    // Statistics
    uint32_t update_count;
    float last_innovation;
    float last_residual;
    
} AUKF_t;

/* ============================================
 * FUNCTION PROTOTYPES
 * ============================================ */

/**
 * @brief Initialize AUKF
 * @param aukf Pointer to AUKF structure
 * @param battery Pointer to battery parameters
 * @param initial_SoC Initial SoC estimate (%)
 * @param temperature Operating temperature (°C)
 */
void AUKF_Init(AUKF_t *aukf, const BatteryParams_t *battery, 
               float initial_SoC, float temperature);

/**
 * @brief AUKF update (prediction + correction)
 * @note This function USES OCV for voltage prediction
 * 
 * @param aukf Pointer to AUKF structure
 * @param SoC_predicted SoC prediction from Coulomb Counting (%)
 * @param current Battery current (A)
 * @param voltage_measured Measured terminal voltage (V)
 * @param dt Time step (seconds)
 * @return Corrected SoC estimate (%)
 */
float AUKF_Update(AUKF_t *aukf, float SoC_predicted, float current, 
                  float voltage_measured, float dt);

/**
 * @brief Get current SoC estimate
 * @param aukf Pointer to AUKF structure
 * @return Current SoC (%)
 */
float AUKF_GetSoC(const AUKF_t *aukf);

/**
 * @brief Get internal state V_tr
 * @param aukf Pointer to AUKF structure
 * @return V_tr (V)
 */
float AUKF_GetVtr(const AUKF_t *aukf);

/**
 * @brief Get last innovation (for diagnostics)
 * @param aukf Pointer to AUKF structure
 * @return Innovation (V)
 */
float AUKF_GetInnovation(const AUKF_t *aukf);

#endif /* AUKF_H */