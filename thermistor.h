#ifndef THERMISTOR_H
#define THERMISTOR_H

#include <stdint.h>
#include <math.h>

/* Steinhart-Hart coefficients for 10k NTC thermistor */
#define SH_A   9.9594112270e-4f
#define SH_B   2.8977286903e-4f
#define SH_C  -3.6500315815e-7f

/* Thermistor circuit parameters */
#define THERMISTOR_NOMINAL_RESISTANCE  10000.0f  /* 10k ohms at 25°C */
#define VOLTAGE_DIVIDER_RESISTANCE     10000.0f  /* Series resistor (adjust as needed) */
#define ADC_MAX_VALUE                  4095      /* 12-bit ADC */
#define ADC_REFERENCE_VOLTAGE          3.3f      /* 3.3V reference */

/**
 * Convert ADC reading to thermistor resistance
 * Assumes: ADC -> Thermistor -> GND, with series resistor to VCC
 */
float thermistor_adc_to_resistance(uint16_t adc_value);

/**
 * Calculate temperature from thermistor resistance using Steinhart-Hart equation
 * Returns temperature in Celsius
 */
float thermistor_resistance_to_temperature(float resistance_ohms);

/**
 * Combined function: ADC value directly to temperature in Celsius
 */
float thermistor_adc_to_temperature(uint16_t adc_value);

#endif /* THERMISTOR_H */
