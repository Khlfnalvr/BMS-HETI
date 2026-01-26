/**
 ******************************************************************************
 * @file    battery_params.h
 * @brief   Battery parameters and OCV lookup tables
 * @author  Your Name
 * @date    2024
 ******************************************************************************
 */

#ifndef BATTERY_PARAMS_H
#define BATTERY_PARAMS_H

#include <stdint.h>
#include <stdbool.h>

/* ============================================
 * BATTERY SPECIFICATIONS
 * ============================================ */

#define BATTERY_NOMINAL_CAPACITY    2.6f    // Ah
#define BATTERY_NOMINAL_VOLTAGE     3.7f    // V
#define NUM_SOC_POINTS              7       // Number of lookup table points

/* Battery Parameter Structure */
typedef struct {
    float Q_nominal;                        // Nominal capacity (Ah)
    float V_nominal;                        // Nominal voltage (V)
    
    // OCV-SoC lookup tables
    float SoC_points[NUM_SOC_POINTS];      // SoC breakpoints (%)
    float OCV_25C[NUM_SOC_POINTS];         // OCV at 25°C (V)
    float OCV_45C[NUM_SOC_POINTS];         // OCV at 45°C (V)
    
    // Resistance lookup tables
    float Ro_25C[NUM_SOC_POINTS];          // Series resistance at 25°C (Ω)
    float Ro_45C[NUM_SOC_POINTS];          // Series resistance at 45°C (Ω)
    
    float Rtr_25C[NUM_SOC_POINTS];         // Transient resistance at 25°C (Ω)
    float Rtr_45C[NUM_SOC_POINTS];         // Transient resistance at 45°C (Ω)
    
    // Time constant lookup tables
    float tau_25C[NUM_SOC_POINTS];         // Time constant at 25°C (s)
    float tau_45C[NUM_SOC_POINTS];         // Time constant at 45°C (s)
    
} BatteryParams_t;

/* ============================================
 * FUNCTION PROTOTYPES
 * ============================================ */

/**
 * @brief Initialize battery parameters
 * @param params Pointer to battery parameters structure
 */
void BatteryParams_Init(BatteryParams_t *params);

/**
 * @brief Lookup OCV based on SoC and temperature
 * @param params Pointer to battery parameters
 * @param SoC State of Charge (%)
 * @param temperature Temperature (°C)
 * @return OCV (V)
 */
float BatteryParams_GetOCV(const BatteryParams_t *params, float SoC, float temperature);

/**
 * @brief Lookup Ro based on SoC and temperature
 * @param params Pointer to battery parameters
 * @param SoC State of Charge (%)
 * @param temperature Temperature (°C)
 * @return Ro (Ω)
 */
float BatteryParams_GetRo(const BatteryParams_t *params, float SoC, float temperature);

/**
 * @brief Lookup Rtr based on SoC and temperature
 * @param params Pointer to battery parameters
 * @param SoC State of Charge (%)
 * @param temperature Temperature (°C)
 * @return Rtr (Ω)
 */
float BatteryParams_GetRtr(const BatteryParams_t *params, float SoC, float temperature);

/**
 * @brief Lookup tau based on SoC and temperature
 * @param params Pointer to battery parameters
 * @param SoC State of Charge (%)
 * @param temperature Temperature (°C)
 * @return tau (s)
 */
float BatteryParams_GetTau(const BatteryParams_t *params, float SoC, float temperature);

/**
 * @brief Linear interpolation helper function
 * @param x Array of x values
 * @param y Array of y values
 * @param n Number of points
 * @param x_query Query point
 * @return Interpolated y value
 */
float Linear_Interpolate(const float *x, const float *y, uint8_t n, float x_query);

#endif /* BATTERY_PARAMS_H */