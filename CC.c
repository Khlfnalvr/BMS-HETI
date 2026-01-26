/**
 ******************************************************************************
 * @file    coulomb_counting.c
 * @brief   Coulomb Counting Method implementation
 * @note    This module DOES NOT use OCV - only current integration
 ******************************************************************************
 */

#include "cc.h"
#include <string.h>

/* ============================================
 * PRIVATE MACROS
 * ============================================ */

#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

/* ============================================
 * PUBLIC FUNCTIONS
 * ============================================ */

void CoulombCounter_Init(CoulombCounter_t *cc, float initial_SoC, 
                         float capacity, float eta) {
    cc->SoC = CLAMP(initial_SoC, 0.0f, 100.0f);
    cc->capacity = capacity;
    cc->eta = eta;
    cc->update_count = 0;
    cc->total_Ah = 0.0f;
}

float CoulombCounter_Update(CoulombCounter_t *cc, float current, float dt) {
    /**
     * COULOMB COUNTING - NO OCV USED!
     * 
     * Formula: SoC = SoC_prev - (100 × η × I × Δt) / (3600 × Q)
     * 
     * Where:
     * - SoC: State of Charge (%)
     * - η: Coulomb efficiency
     * - I: Current (A) - positive for discharge
     * - Δt: Time step (s)
     * - Q: Battery capacity (Ah)
     * - 3600: Conversion factor (seconds to hours)
     */
    
    // Calculate change in SoC
    float delta_SoC = (100.0f * cc->eta * current * dt) / (3600.0f * cc->capacity);
    
    // Update SoC (subtract because positive current = discharge)
    cc->SoC -= delta_SoC;
    
    // Clamp to valid range [0, 100]
    cc->SoC = CLAMP(cc->SoC, 0.0f, 100.0f);
    
    // Update statistics
    cc->total_Ah += (current * dt) / 3600.0f;
    cc->update_count++;
    
    return cc->SoC;
}

void CoulombCounter_Reset(CoulombCounter_t *cc, float new_SoC) {
    cc->SoC = CLAMP(new_SoC, 0.0f, 100.0f);
    cc->total_Ah = 0.0f;
    cc->update_count = 0;
}

float CoulombCounter_GetSoC(const CoulombCounter_t *cc) {
    return cc->SoC;
}