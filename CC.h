/**
 ******************************************************************************
 * @file    coulomb_counting.h
 * @brief   Coulomb Counting Method (CCM) for SoC estimation
 * @author  Your Name
 * @date    2024
 ******************************************************************************
 */

#ifndef COULOMB_COUNTING_H
#define COULOMB_COUNTING_H

#include <stdint.h>
#include <stdbool.h>

/* ============================================
 * CCM STRUCTURE
 * ============================================ */

typedef struct {
    float SoC;              // Current State of Charge (%)
    float capacity;         // Battery capacity (Ah)
    float eta;              // Coulomb efficiency (typically 1.0)
    
    // Statistics
    uint32_t update_count;  // Number of updates
    float total_Ah;         // Total Ah discharged/charged
    
} CoulombCounter_t;

/* ============================================
 * FUNCTION PROTOTYPES
 * ============================================ */

/**
 * @brief Initialize Coulomb Counter
 * @param cc Pointer to CoulombCounter structure
 * @param initial_SoC Initial SoC estimate (%)
 * @param capacity Battery capacity (Ah)
 * @param eta Coulomb efficiency (default 1.0)
 */
void CoulombCounter_Init(CoulombCounter_t *cc, float initial_SoC, 
                         float capacity, float eta);

/**
 * @brief Update SoC using Coulomb Counting
 * @note This function DOES NOT use OCV - only current integration
 * 
 * @param cc Pointer to CoulombCounter structure
 * @param current Battery current (A) - positive for discharge
 * @param dt Time step (seconds)
 * @return Updated SoC (%)
 */
float CoulombCounter_Update(CoulombCounter_t *cc, float current, float dt);

/**
 * @brief Reset Coulomb Counter
 * @param cc Pointer to CoulombCounter structure
 * @param new_SoC New SoC value (%)
 */
void CoulombCounter_Reset(CoulombCounter_t *cc, float new_SoC);

/**
 * @brief Get current SoC estimate
 * @param cc Pointer to CoulombCounter structure
 * @return Current SoC (%)
 */
float CoulombCounter_GetSoC(const CoulombCounter_t *cc);

#endif /* COULOMB_COUNTING_H */