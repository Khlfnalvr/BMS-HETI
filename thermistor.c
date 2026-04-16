#include "thermistor.h"

/**
 * Steinhart-Hart equation:
 * 1/T = A + B*ln(R) + C*(ln(R))^3
 * where T is absolute temperature in Kelvin
 *       R is resistance in ohms
 *       A, B, C are Steinhart-Hart coefficients
 */

float thermistor_adc_to_resistance(uint16_t adc_value) {
    /* Voltage divider: Vout = Vin * Rtherm / (Rs + Rtherm)
       Solving for Rtherm: Rtherm = Rs * Vout / (Vin - Vout) */

    float voltage_out = (adc_value / (float)ADC_MAX_VALUE) * ADC_REFERENCE_VOLTAGE;

    if (voltage_out >= ADC_REFERENCE_VOLTAGE) {
        return 1e6f;  /* Very high resistance (open circuit) */
    }

    if (voltage_out <= 0.01f) {
        return 1.0f;  /* Very low resistance (short circuit) */
    }

    float voltage_in = ADC_REFERENCE_VOLTAGE;
    float resistance = VOLTAGE_DIVIDER_RESISTANCE * voltage_out / (voltage_in - voltage_out);

    return resistance;
}

float thermistor_resistance_to_temperature(float resistance_ohms) {
    if (resistance_ohms <= 0) {
        return -273.15f;  /* Invalid */
    }

    /* Steinhart-Hart equation */
    float ln_r = logf(resistance_ohms);
    float ln_r_cubed = ln_r * ln_r * ln_r;

    float inv_t = SH_A + (SH_B * ln_r) + (SH_C * ln_r_cubed);

    if (inv_t <= 0) {
        return -273.15f;  /* Invalid */
    }

    float temperature_k = 1.0f / inv_t;
    float temperature_c = temperature_k - 273.15f;

    return temperature_c;
}

float thermistor_adc_to_temperature(uint16_t adc_value) {
    float resistance = thermistor_adc_to_resistance(adc_value);
    return thermistor_resistance_to_temperature(resistance);
}
