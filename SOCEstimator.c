/**
 ******************************************************************************
 * @file    soc_estimator.c
 * @brief   Main SoC Estimator implementation
 ******************************************************************************
 */

#include "socestimator.h"
#include <string.h>

/* ============================================
 * PUBLIC FUNCTIONS
 * ============================================ */

void SoC_Estimator_Init(SoC_Estimator_t *estimator, float initial_SoC, 
                        float temperature) {
    // Initialize battery parameters
    BatteryParams_Init(&estimator->battery);
    
    // Initialize Coulomb Counter
    CoulombCounter_Init(&estimator->cc, initial_SoC, 
                       estimator->battery.Q_nominal, 1.0f);
    
    // Initialize AUKF
    AUKF_Init(&estimator->aukf, &estimator->battery, initial_SoC, temperature);
    
    // Set temperature
    estimator->temperature = temperature;
    
    // Initialize results
    estimator->SoC_CCM = initial_SoC;
    estimator->SoC_AUKF = initial_SoC;
    estimator->V_predicted = 0.0f;
    estimator->V_measured = 0.0f;
    
    estimator->update_count = 0;
}

float SoC_Estimator_Update(SoC_Estimator_t *estimator, float current, 
                           float voltage, float dt) {
    /**
     * ═══════════════════════════════════════════════════════════════
     * HYBRID CCM + AUKF ALGORITHM
     * ═══════════════════════════════════════════════════════════════
     * 
     * STEP 1: Coulomb Counting (NO OCV)
     *         - Predicts SoC from current integration only
     *         - Fast but accumulates error
     * 
     * STEP 2: AUKF Correction (USES OCV)
     *         - Uses OCV to predict voltage from SoC
     *         - Compares predicted vs measured voltage
     *         - Corrects SoC based on voltage error
     * 
     * STEP 3: Adaptive Noise Update
     *         - Adjusts Q and R automatically
     */
    
    // ════════════════════════════════════════════════════════════════
    // STEP 1: COULOMB COUNTING PREDICTION (NO OCV!)
    // ════════════════════════════════════════════════════════════════
    
    float SoC_CCM = CoulombCounter_Update(&estimator->cc, current, dt);
    
    // ════════════════════════════════════════════════════════════════
    // STEP 2: AUKF CORRECTION (USES OCV!)
    // ════════════════════════════════════════════════════════════════
    
    float SoC_AUKF = AUKF_Update(&estimator->aukf, SoC_CCM, current, voltage, dt);
    
    // ════════════════════════════════════════════════════════════════
    // UPDATE RESULTS
    // ════════════════════════════════════════════════════════════════
    
    estimator->SoC_CCM = SoC_CCM;
    estimator->SoC_AUKF = SoC_AUKF;
    estimator->V_measured = voltage;
    estimator->update_count++;
    
    // Return final corrected SoC
    return SoC_AUKF;
}

float SoC_Estimator_GetSoC(const SoC_Estimator_t *estimator) {
    return estimator->SoC_AUKF;
}

float SoC_Estimator_GetSoC_CCM(const SoC_Estimator_t *estimator) {
    return estimator->SoC_CCM;
}

void SoC_Estimator_SetTemperature(SoC_Estimator_t *estimator, float temperature) {
    estimator->temperature = temperature;
    estimator->aukf.temperature = temperature;
}

void SoC_Estimator_Reset(SoC_Estimator_t *estimator, float new_SoC) {
    CoulombCounter_Reset(&estimator->cc, new_SoC);
    AUKF_Init(&estimator->aukf, &estimator->battery, new_SoC, estimator->temperature);
    estimator->SoC_CCM = new_SoC;
    estimator->SoC_AUKF = new_SoC;
    estimator->update_count = 0;
}