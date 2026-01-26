/**
 ******************************************************************************
 * @file    soc_estimator.h
 * @brief   Main SoC Estimator combining CCM and AUKF
 * @author  Your Name
 * @date    2024
 ******************************************************************************
 */

#ifndef SOC_ESTIMATOR_H
#define SOC_ESTIMATOR_H

#include <stdint.h>
#include <stdbool.h>
#include "bateryparameter.h"
#include "cc.h"
#include "aukf.h"

/* ============================================
 * SOC ESTIMATOR STRUCTURE
 * ============================================ */

typedef struct {
    // Battery parameters
    BatteryParams_t battery;
    
    // Coulomb Counter (for prediction)
    CoulombCounter_t cc;
    
    // AUKF (for correction)
    AUKF_t aukf;
    
    // Current temperature
    float temperature;
    
    // Estimation results
    float SoC_CCM;          // SoC from CCM only
    float SoC_AUKF;         // SoC from AUKF (final estimate)
    float V_predicted;      // Predicted voltage
    float V_measured;       // Measured voltage
    
    // Statistics
    uint32_t update_count;
    
} SoC_Estimator_t;

/* ============================================
 * FUNCTION PROTOTYPES
 * ============================================ */

/**
 * @brief Initialize SoC Estimator
 * @param estimator Pointer to SoC_Estimator structure
 * @param initial_SoC Initial SoC estimate (%)
 * @param temperature Operating temperature (°C)
 */
void SoC_Estimator_Init(SoC_Estimator_t *estimator, float initial_SoC, 
                        float temperature);

/**
 * @brief Update SoC estimation
 * @note Call this function every sampling period
 * 
 * Algorithm:
 * 1. CCM predicts SoC (NO OCV)
 * 2. AUKF corrects SoC using voltage measurement (USES OCV)
 * 
 * @param estimator Pointer to SoC_Estimator structure
 * @param current Measured current (A) - positive for discharge
 * @param voltage Measured terminal voltage (V)
 * @param dt Time step (seconds)
 * @return Final estimated SoC (%)
 */
float SoC_Estimator_Update(SoC_Estimator_t *estimator, float current, 
                           float voltage, float dt);

/**
 * @brief Get final SoC estimate
 * @param estimator Pointer to SoC_Estimator structure
 * @return SoC estimate (%)
 */
float SoC_Estimator_GetSoC(const SoC_Estimator_t *estimator);

/**
 * @brief Get CCM-only SoC (for comparison)
 * @param estimator Pointer to SoC_Estimator structure
 * @return CCM SoC (%)
 */
float SoC_Estimator_GetSoC_CCM(const SoC_Estimator_t *estimator);

/**
 * @brief Update operating temperature
 * @param estimator Pointer to SoC_Estimator structure
 * @param temperature New temperature (°C)
 */
void SoC_Estimator_SetTemperature(SoC_Estimator_t *estimator, float temperature);

/**
 * @brief Reset estimator with new SoC
 * @param estimator Pointer to SoC_Estimator structure
 * @param new_SoC New SoC value (%)
 */
void SoC_Estimator_Reset(SoC_Estimator_t *estimator, float new_SoC);

#endif /* SOC_ESTIMATOR_H */