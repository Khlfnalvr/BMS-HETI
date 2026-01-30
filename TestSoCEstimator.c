/**
 * @file TestSoCEstimator.c
 * @brief Test program for SoC Estimation algorithms with dummy data
 *
 * This program tests:
 * 1. Coulomb Counting algorithm
 * 2. Open Circuit Voltage lookup
 * 3. Adaptive Unscented Kalman Filter (AUKF)
 *
 * It reads test data from CSV and outputs results to a separate file
 * without modifying any existing source files.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "SOCEstimator.h"
#include "Bateryparameter.h"
#include "CC.h"
#include "AUKF.h"

#define MAX_LINE_LENGTH 256
#define INPUT_FILE "test_data/battery_test_data.csv"
#define OUTPUT_FILE "test_data/test_results.csv"

// Structure to hold test data point
typedef struct {
    float time_sec;
    float current_A;
    float voltage_V;
    float temperature_C;
    float true_SoC;
} TestDataPoint_t;

/**
 * @brief Read one line of CSV test data
 */
int read_csv_line(FILE *fp, TestDataPoint_t *data, int skip_header) {
    char line[MAX_LINE_LENGTH];

    if (fgets(line, sizeof(line), fp) == NULL) {
        return 0; // End of file
    }

    if (skip_header) {
        return 1; // Skip header line
    }

    // Parse CSV line
    int parsed = sscanf(line, "%f,%f,%f,%f,%f",
                       &data->time_sec,
                       &data->current_A,
                       &data->voltage_V,
                       &data->temperature_C,
                       &data->true_SoC);

    return (parsed == 5) ? 1 : 0;
}

/**
 * @brief Calculate statistics
 */
void calculate_statistics(float *errors, int count, float *mean, float *std, float *max_error) {
    float sum = 0.0f;
    float sum_sq = 0.0f;
    *max_error = 0.0f;

    for (int i = 0; i < count; i++) {
        sum += errors[i];
        sum_sq += errors[i] * errors[i];

        float abs_error = fabsf(errors[i]);
        if (abs_error > *max_error) {
            *max_error = abs_error;
        }
    }

    *mean = sum / count;
    *std = sqrtf((sum_sq / count) - (*mean * *mean));
}

/**
 * @brief Main test function
 */
int main(int argc, char *argv[]) {
    FILE *input_fp, *output_fp;
    TestDataPoint_t test_data;
    SoC_Estimator_t estimator;
    float prev_time = 0.0f;
    int data_count = 0;

    printf("=====================================\n");
    printf("  BMS SoC Estimator Test Program\n");
    printf("=====================================\n\n");

    // Open input file
    printf("Opening test data file: %s\n", INPUT_FILE);
    input_fp = fopen(INPUT_FILE, "r");
    if (!input_fp) {
        fprintf(stderr, "Error: Cannot open input file '%s'\n", INPUT_FILE);
        fprintf(stderr, "Make sure the file exists in the test_data/ directory\n");
        return 1;
    }

    // Open output file
    printf("Creating output file: %s\n\n", OUTPUT_FILE);
    output_fp = fopen(OUTPUT_FILE, "w");
    if (!output_fp) {
        fprintf(stderr, "Error: Cannot create output file '%s'\n", OUTPUT_FILE);
        fclose(input_fp);
        return 1;
    }

    // Write output CSV header
    fprintf(output_fp, "Time_sec,Current_A,Voltage_V,Temperature_C,True_SoC,");
    fprintf(output_fp, "SoC_CC,SoC_AUKF,Error_CC,Error_AUKF,");
    fprintf(output_fp, "V_predicted,Innovation,Vtr\n");

    // Arrays to store errors for statistics
    float *errors_cc = (float*)malloc(10000 * sizeof(float));
    float *errors_aukf = (float*)malloc(10000 * sizeof(float));

    // Skip header line
    read_csv_line(input_fp, &test_data, 1);

    // Read first data point to initialize
    if (!read_csv_line(input_fp, &test_data, 0)) {
        fprintf(stderr, "Error: Cannot read first data point\n");
        fclose(input_fp);
        fclose(output_fp);
        free(errors_cc);
        free(errors_aukf);
        return 1;
    }

    // Initialize SoC Estimator with first data point
    printf("Initializing SoC Estimator:\n");
    printf("  Initial SoC: %.2f%%\n", test_data.true_SoC);
    printf("  Initial Temperature: %.1f°C\n", test_data.temperature_C);
    printf("  Initial Voltage: %.3fV\n\n", test_data.voltage_V);

    SoC_Estimator_Init(&estimator, test_data.true_SoC, test_data.temperature_C);
    prev_time = test_data.time_sec;

    // Write first data point to output
    fprintf(output_fp, "%.1f,%.3f,%.3f,%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f\n",
            test_data.time_sec, test_data.current_A, test_data.voltage_V,
            test_data.temperature_C, test_data.true_SoC,
            estimator.SoC_CCM, estimator.SoC_AUKF,
            0.0f, 0.0f,  // Initial errors are zero
            estimator.V_predicted,
            estimator.aukf.last_innovation,
            estimator.aukf.x[1]);

    errors_cc[data_count] = 0.0f;
    errors_aukf[data_count] = 0.0f;
    data_count++;

    printf("Processing test data...\n");
    printf("Time(s)  Current(A)  Voltage(V)  Temp(°C)  True_SoC  SoC_CC  SoC_AUKF  Error_AUKF\n");
    printf("--------------------------------------------------------------------------------------\n");

    // Process all test data
    while (read_csv_line(input_fp, &test_data, 0)) {
        float dt = test_data.time_sec - prev_time;

        // Update temperature if it changed
        if (test_data.temperature_C != estimator.temperature) {
            SoC_Estimator_SetTemperature(&estimator, test_data.temperature_C);
        }

        // Update estimator
        SoC_Estimator_Update(&estimator,
                            test_data.current_A,
                            test_data.voltage_V,
                            dt);

        // Calculate errors
        float error_cc = estimator.SoC_CCM - test_data.true_SoC;
        float error_aukf = estimator.SoC_AUKF - test_data.true_SoC;

        // Store errors for statistics
        errors_cc[data_count] = error_cc;
        errors_aukf[data_count] = error_aukf;

        // Write to output file
        fprintf(output_fp, "%.1f,%.3f,%.3f,%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f\n",
                test_data.time_sec, test_data.current_A, test_data.voltage_V,
                test_data.temperature_C, test_data.true_SoC,
                estimator.SoC_CCM, estimator.SoC_AUKF,
                error_cc, error_aukf,
                estimator.V_predicted,
                estimator.aukf.last_innovation,
                estimator.aukf.x[1]);

        // Print progress every 10 minutes (600 seconds)
        if (((int)test_data.time_sec % 600) == 0 || test_data.time_sec < 60) {
            printf("%-8.0f %-11.3f %-11.3f %-9.1f %-9.2f %-7.2f %-9.2f %-10.3f\n",
                   test_data.time_sec, test_data.current_A, test_data.voltage_V,
                   test_data.temperature_C, test_data.true_SoC,
                   estimator.SoC_CCM, estimator.SoC_AUKF, error_aukf);
        }

        prev_time = test_data.time_sec;
        data_count++;
    }

    printf("\n");

    // Calculate and display statistics
    float mean_cc, std_cc, max_cc;
    float mean_aukf, std_aukf, max_aukf;

    calculate_statistics(errors_cc, data_count, &mean_cc, &std_cc, &max_cc);
    calculate_statistics(errors_aukf, data_count, &mean_aukf, &std_aukf, &max_aukf);

    printf("=====================================\n");
    printf("  Test Results Summary\n");
    printf("=====================================\n\n");

    printf("Total data points processed: %d\n", data_count);
    printf("Total time: %.1f seconds (%.1f minutes)\n\n", prev_time, prev_time/60.0f);

    printf("Coulomb Counting (CC) Performance:\n");
    printf("  Mean Error:     %+.3f%%\n", mean_cc);
    printf("  Std Deviation:   %.3f%%\n", std_cc);
    printf("  Max Error:       %.3f%%\n\n", max_cc);

    printf("AUKF Performance:\n");
    printf("  Mean Error:     %+.3f%%\n", mean_aukf);
    printf("  Std Deviation:   %.3f%%\n", std_aukf);
    printf("  Max Error:       %.3f%%\n\n", max_aukf);

    printf("Improvement (CC vs AUKF):\n");
    printf("  Mean Error Reduction:    %.3f%%\n", mean_cc - mean_aukf);
    printf("  Std Deviation Reduction: %.3f%%\n", std_cc - std_aukf);
    printf("  Max Error Reduction:     %.3f%%\n\n", max_cc - max_aukf);

    // Test OCV function directly
    printf("=====================================\n");
    printf("  OCV Function Test\n");
    printf("=====================================\n\n");

    printf("SoC(%%)  OCV@25°C(V)  OCV@45°C(V)\n");
    printf("-------------------------------------\n");
    float test_socs[] = {0.0f, 10.0f, 25.0f, 50.0f, 75.0f, 90.0f, 100.0f};
    for (int i = 0; i < 7; i++) {
        float ocv_25 = BatteryParams_GetOCV(&estimator.battery, test_socs[i], 25.0f);
        float ocv_45 = BatteryParams_GetOCV(&estimator.battery, test_socs[i], 45.0f);
        printf("%-6.0f  %-12.3f %-12.3f\n", test_socs[i], ocv_25, ocv_45);
    }
    printf("\n");

    // Test Coulomb Counting directly
    printf("=====================================\n");
    printf("  Coulomb Counting Direct Test\n");
    printf("=====================================\n\n");

    CoulombCounter_t cc_test;
    CoulombCounter_Init(&cc_test, 80.0f, 2.6f, 1.0f);

    printf("Initial SoC: %.2f%%\n", CoulombCounter_GetSoC(&cc_test));
    printf("Testing 1C discharge (2.6A) for 1 hour:\n");

    for (int i = 0; i < 3600; i++) {
        CoulombCounter_Update(&cc_test, 2.6f, 1.0f);  // Positive = discharge
    }

    printf("SoC after 1 hour: %.2f%% (Expected: 0.0%%)\n", CoulombCounter_GetSoC(&cc_test));
    printf("Total Ah processed: %.3f Ah\n\n", cc_test.total_Ah);

    printf("Test completed successfully!\n");
    printf("Results saved to: %s\n", OUTPUT_FILE);
    printf("\nYou can analyze the results using spreadsheet software or plotting tools.\n");

    // Cleanup
    fclose(input_fp);
    fclose(output_fp);
    free(errors_cc);
    free(errors_aukf);

    return 0;
}
