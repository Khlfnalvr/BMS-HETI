/**
 ******************************************************************************
 * @file    battery_params.c
 * @brief   Battery parameters implementation
 ******************************************************************************
 */

#include "Bateryparameter.h"
#include <string.h>
#include <math.h>

/* ============================================
 * PRIVATE FUNCTIONS
 * ============================================ */

/**
 * @brief Clamp value between min and max
 */
static inline float clamp(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/* ============================================
 * PUBLIC FUNCTIONS
 * ============================================ */

void BatteryParams_Init(BatteryParams_t *params) {
    // Basic specifications
    params->Q_nominal = BATTERY_NOMINAL_CAPACITY;
    params->V_nominal = BATTERY_NOMINAL_VOLTAGE;
    
    // SoC breakpoints
    params->SoC_points[0] = 0.0f;
    params->SoC_points[1] = 10.0f;
    params->SoC_points[2] = 25.0f;
    params->SoC_points[3] = 50.0f;
    params->SoC_points[4] = 75.0f;
    params->SoC_points[5] = 90.0f;
    params->SoC_points[6] = 100.0f;
    
    // OCV lookup table at 25°C (from AVOA optimization)
    params->OCV_25C[0] = 3.0f;
    params->OCV_25C[1] = 3.06f;
    params->OCV_25C[2] = 3.13f;
    params->OCV_25C[3] = 3.22f;
    params->OCV_25C[4] = 3.42f;
    params->OCV_25C[5] = 3.58f;
    params->OCV_25C[6] = 3.7f;
    
    // OCV lookup table at 45°C
    params->OCV_45C[0] = 3.2109f;
    params->OCV_45C[1] = 3.2812f;
    params->OCV_45C[2] = 3.3409f;
    params->OCV_45C[3] = 3.4269f;
    params->OCV_45C[4] = 3.6409f;
    params->OCV_45C[5] = 3.7954f;
    params->OCV_45C[6] = 3.9009f;
    
    // Ro at 25°C
    params->Ro_25C[0] = 0.0515f;
    params->Ro_25C[1] = 0.0508f;
    params->Ro_25C[2] = 0.0512f;
    params->Ro_25C[3] = 0.0519f;
    params->Ro_25C[4] = 0.0519f;
    params->Ro_25C[5] = 0.0525f;
    params->Ro_25C[6] = 0.0528f;
    
    // Ro at 45°C
    params->Ro_45C[0] = 0.0801f;
    params->Ro_45C[1] = 0.0801f;
    params->Ro_45C[2] = 0.0803f;
    params->Ro_45C[3] = 0.0798f;
    params->Ro_45C[4] = 0.0799f;
    params->Ro_45C[5] = 0.0800f;
    params->Ro_45C[6] = 0.0800f;
    
    // Rtr at 25°C
    params->Rtr_25C[0] = 0.0137f;
    params->Rtr_25C[1] = 0.0097f;
    params->Rtr_25C[2] = 0.0075f;
    params->Rtr_25C[3] = 0.0062f;
    params->Rtr_25C[4] = 0.0061f;
    params->Rtr_25C[5] = 0.0061f;
    params->Rtr_25C[6] = 0.0056f;
    
    // Rtr at 45°C
    params->Rtr_45C[0] = 0.0083f;
    params->Rtr_45C[1] = 0.0078f;
    params->Rtr_45C[2] = 0.0080f;
    params->Rtr_45C[3] = 0.0079f;
    params->Rtr_45C[4] = 0.0086f;
    params->Rtr_45C[5] = 0.0081f;
    params->Rtr_45C[6] = 0.0080f;
    
    // tau at 25°C
    params->tau_25C[0] = 104.65f;
    params->tau_25C[1] = 115.36f;
    params->tau_25C[2] = 193.45f;
    params->tau_25C[3] = 120.54f;
    params->tau_25C[4] = 143.71f;
    params->tau_25C[5] = 124.36f;
    params->tau_25C[6] = 109.71f;
    
    // tau at 45°C
    params->tau_45C[0] = 157.0f;
    params->tau_45C[1] = 166.0f;
    params->tau_45C[2] = 226.0f;
    params->tau_45C[3] = 150.0f;
    params->tau_45C[4] = 198.0f;
    params->tau_45C[5] = 154.0f;
    params->tau_45C[6] = 160.0f;
}

float Linear_Interpolate(const float *x, const float *y, uint8_t n, float x_query) {
    // Clamp x_query to valid range
    if (x_query <= x[0]) return y[0];
    if (x_query >= x[n-1]) return y[n-1];
    
    // Find the interval
    for (uint8_t i = 0; i < n-1; i++) {
        if (x_query >= x[i] && x_query <= x[i+1]) {
            // Linear interpolation
            float slope = (y[i+1] - y[i]) / (x[i+1] - x[i]);
            return y[i] + slope * (x_query - x[i]);
        }
    }
    
    return y[n-1]; // Fallback
}

float BatteryParams_GetOCV(const BatteryParams_t *params, float SoC, float temperature) {
    // Clamp SoC to valid range
    SoC = clamp(SoC, 0.0f, 100.0f);
    
    // Select table based on temperature
    const float *ocv_table;
    if (temperature <= 35.0f) {
        ocv_table = params->OCV_25C;
    } else {
        ocv_table = params->OCV_45C;
    }
    
    // Interpolate
    return Linear_Interpolate(params->SoC_points, ocv_table, NUM_SOC_POINTS, SoC);
}

float BatteryParams_GetRo(const BatteryParams_t *params, float SoC, float temperature) {
    SoC = clamp(SoC, 0.0f, 100.0f);
    
    const float *ro_table;
    if (temperature <= 35.0f) {
        ro_table = params->Ro_25C;
    } else {
        ro_table = params->Ro_45C;
    }
    
    return Linear_Interpolate(params->SoC_points, ro_table, NUM_SOC_POINTS, SoC);
}

float BatteryParams_GetRtr(const BatteryParams_t *params, float SoC, float temperature) {
    SoC = clamp(SoC, 0.0f, 100.0f);
    
    const float *rtr_table;
    if (temperature <= 35.0f) {
        rtr_table = params->Rtr_25C;
    } else {
        rtr_table = params->Rtr_45C;
    }
    
    return Linear_Interpolate(params->SoC_points, rtr_table, NUM_SOC_POINTS, SoC);
}

float BatteryParams_GetTau(const BatteryParams_t *params, float SoC, float temperature) {
    SoC = clamp(SoC, 0.0f, 100.0f);
    
    const float *tau_table;
    if (temperature <= 35.0f) {
        tau_table = params->tau_25C;
    } else {
        tau_table = params->tau_45C;
    }
    
    return Linear_Interpolate(params->SoC_points, tau_table, NUM_SOC_POINTS, SoC);
}