%% ==========================================================================
%  GENERATE SYNTHETIC BATTERY DATA FOR AUKF VERIFICATION
%  ==========================================================================
%  Script ini membuat data sintetis baterai dengan parameter yang DIKETAHUI
%  untuk memverifikasi bahwa algoritma CCM+AUKF bekerja dengan benar.
%
%  Output:
%    - Synthetic_battery_data.csv (Time, Current, Voltage, Temperature)
%    - Synthetic_OCV_curve.csv (SOC, V0)
%    - Synthetic_true_SOC.csv (Time, True_SOC) untuk validasi
%
%  Dengan data ini, kita bisa membandingkan:
%    - SOC estimated vs SOC true (yang kita ketahui)
%    - Voltage predicted vs Voltage simulated
%
%  Author: Generated for BMS-HETI project
%  ==========================================================================

clear; clc; close all;

fprintf('=== SYNTHETIC BATTERY DATA GENERATOR ===\n\n');

%% ==========================================================================
%  PARAMETER BATERAI (YANG DIKETAHUI - GROUND TRUTH)
%  ==========================================================================

% --- Parameter Kapasitas ---
Q_nominal = 2.6;        % Kapasitas nominal (Ah) - baterai kecil untuk demo
Q_As = Q_nominal * 3600; % Kapasitas dalam As

% --- Parameter Model RC (1-RC Thevenin) ---
Rs = 0.020;             % Serial/Ohmic resistance [Ohm]
R1 = 0.015;             % Polarization resistance [Ohm]
C1 = 1000;              % Polarization capacitance [F]
tau = R1 * C1;          % Time constant [s] = 15 seconds

% --- Parameter OCV (Open Circuit Voltage) ---
% OCV = f(SOC) - menggunakan model polynomial yang realistis untuk Li-ion
% Berdasarkan karakteristik tipikal NMC/LCO

% SOC dari 0 sampai 1 (0% sampai 100%)
SOC_points = (0:0.01:1)';  % 101 points

% OCV curve (V) - polynomial fit untuk Li-ion tipikal
% V = a0 + a1*SOC + a2*SOC^2 + a3*SOC^3 + a4*log(SOC) + a5*log(1-SOC)
% Simplified version:
OCV_curve = 3.0 + ...                           % Base voltage
            0.8 * SOC_points + ...              % Linear term
            0.3 * SOC_points.^2 - ...           % Quadratic term
            0.15 * SOC_points.^3 + ...          % Cubic term
            0.05 * log(SOC_points + 0.01) - ... % Low SOC nonlinearity
            0.02 * log(1 - SOC_points + 0.01);  % High SOC nonlinearity

% Clamp OCV ke range realistis
OCV_curve = max(2.8, min(4.2, OCV_curve));

% --- Noise Parameters ---
voltage_noise_std = 0.002;  % 2 mV standard deviation
current_noise_std = 0.01;   % 10 mA standard deviation

% --- Temperature (konstan untuk simplifikasi) ---
temperature = 25;  % Celsius

%% ==========================================================================
%  PROFIL ARUS (CURRENT PROFILE)
%  ==========================================================================

fprintf('Generating current profile...\n');

% Total simulation time
dt = 1;                 % Time step [s]
t_total = 10000;        % Total time [s] (~2.8 hours)
time = (0:dt:t_total)';
N = length(time);

% Initialize current array
current = zeros(N, 1);

% --- Buat profil arus yang realistis ---
% Skenario: Charge -> Rest -> Discharge -> Rest -> Pulse test -> Rest

% Phase 1: Initial rest (0-100s)
t1_start = 1; t1_end = 100;

% Phase 2: Constant current charge at 1C (100-4000s)
t2_start = 101; t2_end = 4000;
I_charge = -Q_nominal;  % Negative = charging (konvensi: positive = discharge)
current(t2_start:t2_end) = I_charge;

% Phase 3: Rest period (4000-4500s)
t3_start = 4001; t3_end = 4500;

% Phase 4: Constant current discharge at 0.5C (4500-8500s)
t4_start = 4501; t4_end = 8500;
I_discharge = Q_nominal * 0.5;  % Positive = discharging
current(t4_start:t4_end) = I_discharge;

% Phase 5: Rest period (8500-9000s)
t5_start = 8501; t5_end = 9000;

% Phase 6: Pulse discharge test (9000-10000s)
t6_start = 9001; t6_end = 10001;
for i = t6_start:t6_end
    t_in_phase = i - t6_start;
    % Pulse pattern: 10s discharge, 10s rest, repeat
    if mod(floor(t_in_phase / 10), 2) == 0
        current(i) = Q_nominal * 2;  % 2C pulse discharge
    else
        current(i) = 0;  % Rest
    end
end

%% ==========================================================================
%  SIMULASI BATERAI (GROUND TRUTH)
%  ==========================================================================

fprintf('Simulating battery response...\n');

% Initialize state variables
SOC_true = zeros(N, 1);
V_terminal = zeros(N, 1);
V_OCV = zeros(N, 1);
V_R1 = zeros(N, 1);  % Voltage across RC element

% Initial conditions
SOC_true(1) = 0.2;  % Start at 20% SOC
V_R1(1) = 0;        % No initial polarization

% Lookup function for OCV
get_OCV = @(soc) interp1(SOC_points, OCV_curve, soc, 'linear', 'extrap');

% Main simulation loop
for k = 2:N
    I_k = current(k);

    % --- Coulomb Counting (True SOC) ---
    % dSOC/dt = -I / Q
    delta_SOC = (I_k * dt) / Q_As;
    SOC_true(k) = SOC_true(k-1) - delta_SOC;

    % Clamp SOC to [0, 1]
    SOC_true(k) = max(0, min(1, SOC_true(k)));

    % --- RC Element Dynamics ---
    % dV_R1/dt = (I*R1 - V_R1) / tau
    % Using discrete approximation:
    exp_factor = exp(-dt / tau);
    V_R1(k) = V_R1(k-1) * exp_factor + R1 * (1 - exp_factor) * I_k;

    % --- OCV ---
    V_OCV(k) = get_OCV(SOC_true(k));

    % --- Terminal Voltage ---
    % V_terminal = OCV - Rs*I - V_R1
    V_terminal(k) = V_OCV(k) - Rs * I_k - V_R1(k);
end

% First sample
V_OCV(1) = get_OCV(SOC_true(1));
V_terminal(1) = V_OCV(1);

%% ==========================================================================
%  TAMBAH NOISE (REALISTIC MEASUREMENT)
%  ==========================================================================

fprintf('Adding measurement noise...\n');

% Add noise to measurements
voltage_measured = V_terminal + voltage_noise_std * randn(N, 1);
current_measured = current + current_noise_std * randn(N, 1);

% Temperature with small variation
temperature_measured = temperature + 0.5 * randn(N, 1);

%% ==========================================================================
%  SIMPAN DATA KE CSV
%  ==========================================================================

fprintf('Saving data to CSV files...\n');

% --- 1. Main experimental data ---
exp_data = [time, current_measured, voltage_measured, temperature_measured];
exp_table = array2table(exp_data, 'VariableNames', {'Time', 'Current', 'Voltage', 'Temperature'});
writetable(exp_table, 'Synthetic_battery_data.csv');
fprintf('  - Synthetic_battery_data.csv saved\n');

% --- 2. OCV curve ---
ocv_data = [SOC_points * 100, OCV_curve];  % SOC in %, V0 in V
ocv_table = array2table(ocv_data, 'VariableNames', {'SOC', 'V0'});
writetable(ocv_table, 'Synthetic_OCV_curve.csv');
fprintf('  - Synthetic_OCV_curve.csv saved\n');

% --- 3. True SOC (for validation) ---
true_soc_data = [time, SOC_true * 100];  % SOC in %
true_soc_table = array2table(true_soc_data, 'VariableNames', {'Time', 'True_SOC'});
writetable(true_soc_table, 'Synthetic_true_SOC.csv');
fprintf('  - Synthetic_true_SOC.csv saved\n');

% --- 4. Save parameters to file ---
fid = fopen('Synthetic_parameters.txt', 'w');
fprintf(fid, 'SYNTHETIC BATTERY PARAMETERS (GROUND TRUTH)\n');
fprintf(fid, '============================================\n\n');
fprintf(fid, 'Capacity:\n');
fprintf(fid, '  Q_nominal = %.2f Ah\n', Q_nominal);
fprintf(fid, '  Q_As = %.2f As\n\n', Q_As);
fprintf(fid, 'RC Model Parameters:\n');
fprintf(fid, '  Rs = %.4f Ohm (Serial resistance)\n', Rs);
fprintf(fid, '  R1 = %.4f Ohm (Polarization resistance)\n', R1);
fprintf(fid, '  C1 = %.2f F (Polarization capacitance)\n', C1);
fprintf(fid, '  tau = %.2f s (Time constant = R1*C1)\n\n', tau);
fprintf(fid, 'Noise Parameters:\n');
fprintf(fid, '  Voltage noise std = %.4f V\n', voltage_noise_std);
fprintf(fid, '  Current noise std = %.4f A\n\n', current_noise_std);
fprintf(fid, 'Initial Conditions:\n');
fprintf(fid, '  SOC_initial = %.1f %%\n', SOC_true(1) * 100);
fprintf(fid, '  Temperature = %.1f C\n', temperature);
fclose(fid);
fprintf('  - Synthetic_parameters.txt saved\n');

%% ==========================================================================
%  PLOT HASIL
%  ==========================================================================

fprintf('\nGenerating plots...\n');

% Figure 1: Current Profile
figure(1);
plot(time/3600, current, 'b-', 'LineWidth', 1.5);
xlabel('Time (hours)');
ylabel('Current (A)');
title('Synthetic Data: Current Profile');
grid on;
legend('Current (+ = discharge, - = charge)');

% Figure 2: True SOC
figure(2);
plot(time/3600, SOC_true * 100, 'r-', 'LineWidth', 1.5);
xlabel('Time (hours)');
ylabel('SOC (%)');
title('Synthetic Data: True SOC (Ground Truth)');
grid on;
ylim([0 105]);

% Figure 3: Voltage
figure(3);
plot(time/3600, V_terminal, 'b-', 'LineWidth', 1.5);
hold on;
plot(time/3600, V_OCV, 'r--', 'LineWidth', 1);
hold off;
xlabel('Time (hours)');
ylabel('Voltage (V)');
title('Synthetic Data: Voltage Response');
legend('Terminal Voltage', 'OCV');
grid on;

% Figure 4: OCV Curve
figure(4);
plot(SOC_points * 100, OCV_curve, 'b-', 'LineWidth', 2);
xlabel('SOC (%)');
ylabel('OCV (V)');
title('Synthetic Data: OCV Curve');
grid on;

%% ==========================================================================
%  RINGKASAN
%  ==========================================================================

fprintf('\n=== SUMMARY ===\n');
fprintf('Data generated successfully!\n\n');
fprintf('Files created:\n');
fprintf('  1. Synthetic_battery_data.csv - Main data (Time, Current, Voltage, Temp)\n');
fprintf('  2. Synthetic_OCV_curve.csv    - OCV lookup table (SOC, V0)\n');
fprintf('  3. Synthetic_true_SOC.csv     - True SOC for validation\n');
fprintf('  4. Synthetic_parameters.txt   - Ground truth parameters\n\n');

fprintf('Battery Parameters (GROUND TRUTH):\n');
fprintf('  Q_nominal = %.2f Ah\n', Q_nominal);
fprintf('  Rs = %.4f Ohm\n', Rs);
fprintf('  R1 = %.4f Ohm\n', R1);
fprintf('  C1 = %.2f F\n', C1);
fprintf('  tau = %.2f s\n\n', tau);

fprintf('To test AUKF:\n');
fprintf('  1. Update kalman_filter_soc.m to use these files\n');
fprintf('  2. Set parameters to match ground truth:\n');
fprintf('     Q_nominal = %.2f\n', Q_nominal);
fprintf('     Rs = %.4f\n', Rs);
fprintf('     R1 = %.4f (use R1b, set R1a = 0)\n', R1);
fprintf('     tau = %.2f (or C1 = %.2f)\n', tau, C1);
fprintf('  3. Compare SOC_estimated with Synthetic_true_SOC.csv\n');
fprintf('  4. Voltage predicted should match voltage measured closely\n\n');

fprintf('Expected result: SOC error < 2%% if parameters are correct\n');
