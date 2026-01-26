/**
 ******************************************************************************
 * @file    main.c
 * @brief   Main application - Example usage for STM32
 * @author  Your Name
 * @date    2024
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "socestimator.h"
#include <stdio.h>
#include <math.h>

/* Private variables ---------------------------------------------------------*/
SoC_Estimator_t soc_estimator;

// Simulated measurements (replace with actual ADC readings)
float battery_current = 0.0f;    // A
float battery_voltage = 0.0f;    // V
float battery_temperature = 25.0f; // °C

// Timing
uint32_t last_update_tick = 0;
float dt = 1.0f; // 1 second update rate

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Simulate_Battery_Measurements(void);
void Print_Results(void);

/**
 * @brief  Main program
 * @retval int
 */
int main(void)
{
    /* MCU Configuration--------------------------------------------------------*/
    
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();
    
    /* Configure the system clock */
    SystemClock_Config();
    
    /* Initialize all configured peripherals */
    // ... Initialize UART, ADC, Timers, etc.
    
    /* ════════════════════════════════════════════════════════════════
     * INITIALIZE SOC ESTIMATOR
     * ════════════════════════════════════════════════════════════════ */
    
    printf("\r\n");
    printf("════════════════════════════════════════════════════════\r\n");
    printf("  HYBRID CCM + AUKF STATE OF CHARGE ESTIMATOR\r\n");
    printf("════════════════════════════════════════════════════════\r\n");
    printf("\r\n");
    
    // Initialize with 80% SoC at 25°C
    float initial_SoC = 80.0f;
    SoC_Estimator_Init(&soc_estimator, initial_SoC, battery_temperature);
    
    printf("Initialized:\r\n");
    printf("  Initial SoC: %.2f%%\r\n", initial_SoC);
    printf("  Temperature: %.1f°C\r\n", battery_temperature);
    printf("  Capacity: %.2f Ah\r\n", soc_estimator.battery.Q_nominal);
    printf("\r\n");
    
    last_update_tick = HAL_GetTick();
    
    /* Infinite loop */
    while (1)
    {
        uint32_t current_tick = HAL_GetTick();
        
        // Update every 1 second
        if ((current_tick - last_update_tick) >= 1000) {
            
            dt = (current_tick - last_update_tick) / 1000.0f;
            last_update_tick = current_tick;
            
            /* ════════════════════════════════════════════════════════
             * READ MEASUREMENTS
             * ════════════════════════════════════════════════════════ */
            
            // TODO: Replace with actual ADC readings
            // battery_current = Read_Current_ADC();
            // battery_voltage = Read_Voltage_ADC();
            // battery_temperature = Read_Temperature_Sensor();
            
            // For now, simulate measurements
            Simulate_Battery_Measurements();
            
            /* ════════════════════════════════════════════════════════
             * UPDATE SOC ESTIMATION
             * ════════════════════════════════════════════════════════ */
            
            float estimated_SoC = SoC_Estimator_Update(&soc_estimator, 
                                                       battery_current,
                                                       battery_voltage,
                                                       dt);
            
            /* ════════════════════════════════════════════════════════
             * PRINT RESULTS
             * ════════════════════════════════════════════════════════ */
            
            Print_Results();
            
            /* ════════════════════════════════════════════════════════
             * CHECK FOR WARNINGS
             * ════════════════════════════════════════════════════════ */
            
            if (estimated_SoC < 20.0f) {
                printf("⚠️  WARNING: Low battery! SoC = %.2f%%\r\n", estimated_SoC);
            }
            
            if (estimated_SoC < 10.0f) {
                printf("🔴 CRITICAL: Very low battery! SoC = %.2f%%\r\n", estimated_SoC);
            }
        }
        
        // Other tasks...
        HAL_Delay(10);
    }
}

/**
 * @brief Simulate battery measurements (for testing)
 * @note Replace this with actual ADC readings in production
 */
void Simulate_Battery_Measurements(void)
{
    static uint32_t time_counter = 0;
    
    // Simulate discharge profile
    if (time_counter < 1800) {
        // First 30 minutes: 0.5C discharge
        battery_current = 1.3f; // 0.5C = 1.3A for 2.6Ah battery
    }
    else if (time_counter < 2400) {
        // Next 10 minutes: rest
        battery_current = 0.0f;
    }
    else if (time_counter < 3600) {
        // Next 20 minutes: 1C discharge
        battery_current = 2.6f; // 1C = 2.6A
    }
    else {
        // After 1 hour: stop
        battery_current = 0.0f;
    }
    
    // Simulate voltage based on current SoC
    float SoC = SoC_Estimator_GetSoC(&soc_estimator);
    float V_ocv = BatteryParams_GetOCV(&soc_estimator.battery, SoC, battery_temperature);
    float Ro = BatteryParams_GetRo(&soc_estimator.battery, SoC, battery_temperature);
    
    // Simple voltage model with noise
    battery_voltage = V_ocv - Ro * battery_current;
    
    // Add measurement noise (±10mV)
    float noise = ((float)(rand() % 200) - 100.0f) / 10000.0f;
    battery_voltage += noise;
    
    time_counter++;
}

/**
 * @brief Print estimation results to UART
 */
void Print_Results(void)
{
    static uint32_t print_counter = 0;
    
    // Print header every 20 lines
    if (print_counter % 20 == 0) {
        printf("\r\n");
        printf("Time(s) | Current(A) | Voltage(V) | SoC_CCM(%%) | SoC_AUKF(%%) | Error(%%)\r\n");
        printf("--------|------------|------------|------------|-------------|----------\r\n");
    }
    
    float SoC_CCM = SoC_Estimator_GetSoC_CCM(&soc_estimator);
    float SoC_AUKF = SoC_Estimator_GetSoC(&soc_estimator);
    float error = fabs(SoC_CCM - SoC_AUKF);
    
    printf("%7lu | %10.3f | %10.3f | %10.2f | %11.2f | %8.4f\r\n",
           print_counter,
           battery_current,
           battery_voltage,
           SoC_CCM,
           SoC_AUKF,
           error);
    
    print_counter++;
}

/**
 * @brief System Clock Configuration
 */
void SystemClock_Config(void)
{
    // TODO: Configure system clock based on your MCU
    // This is just a placeholder
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1) {
        HAL_IncTick();
    }
}

/**
 * @brief  This function is executed in case of error occurrence.
 */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
        // Error loop
    }
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    printf("Wrong parameters value: file %s on line %lu\r\n", file, line);
}
#endif /* USE_FULL_ASSERT */