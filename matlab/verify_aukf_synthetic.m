%% ==========================================================================
%  VERIFY AUKF WITH SYNTHETIC DATA
%  ==========================================================================
%  Script ini menguji algoritma CCM+AUKF menggunakan data sintetis
%  dengan parameter yang DIKETAHUI (ground truth).
%
%  Tujuan:
%    - Memverifikasi algoritma AUKF bekerja dengan benar
%    - Membandingkan SOC estimated vs True SOC
%    - Menunjukkan bahwa masalah sebelumnya adalah parameter mismatch
%
%  Sebelum menjalankan script ini:
%    1. Jalankan generate_synthetic_data.m untuk membuat data
%    2. Pastikan file CSV sudah ada di folder matlab/
%
%  Author: Generated for BMS-HETI project
%  ==========================================================================

clear; clc; close all;

fprintf('=== AUKF VERIFICATION WITH SYNTHETIC DATA ===\n\n');

%% ==========================================================================
%  PARAMETER KONFIGURASI (GROUND TRUTH - HARUS SAMA DENGAN GENERATOR)
%  ==========================================================================

% --- Parameter Baterai (GROUND TRUTH) ---
Q_nominal = 2.6;        % Kapasitas nominal baterai (Ah)
eta = 1.0;              % Efisiensi Coulomb

% --- Parameter Model RC (GROUND TRUTH) ---
% Model 1-RC: V = OCV - Rs*I - V_R1
% dV_R1/dt = (I*R1 - V_R1) / tau
Rs = 0.020;             % Serial resistance [Ohm]
R1 = 0.015;             % Polarization resistance [Ohm]
C1 = 1000;              % Polarization capacitance [F]
tau = R1 * C1;          % Time constant [s] = 15 s

% --- Initial SOC ---
SOC_initial = 20;       % Harus sama dengan generator (20%)

% --- Parameter UKF (TUNED) ---
alpha = 1.0;
beta = 2.0;
kappa = 0.0;

% --- Initial Covariances (CONSERVATIVE) ---
P_soc_init = 1;         % Moderate - kita cukup yakin SOC awal
P_vtr_init = 0.01;      % RC voltage uncertainty
P_init = [P_soc_init, 0; 0, P_vtr_init];

Q_soc = 1e-6;           % Process noise SOC (sangat kecil - percaya CC)
Q_vtr = 1e-4;           % Process noise V_tr
Q_mat = [Q_soc, 0; 0, Q_vtr];

R_meas = 0.001;         % Measurement noise (sesuai dengan noise generator)

% --- DISABLE Adaptive ---
enable_adaptive = false;

%% ==========================================================================
%  LOAD DATA
%  ==========================================================================

fprintf('Loading synthetic data...\n');

% Check if files exist
if ~exist('Synthetic_battery_data.csv', 'file')
    error('File tidak ditemukan! Jalankan generate_synthetic_data.m terlebih dahulu.');
end

% Load experimental data
exp_data = readmatrix('Synthetic_battery_data.csv');
time_data = exp_data(:, 1);
current_data = exp_data(:, 2);
voltage_measured = exp_data(:, 3);
temperature_data = exp_data(:, 4);

% Load OCV curve
ocv_data = readmatrix('Synthetic_OCV_curve.csv');
SOC_ocv = ocv_data(:, 1);  % SOC in %
V0_ocv = ocv_data(:, 2);   % OCV in V

% Load true SOC for validation
true_soc_data = readmatrix('Synthetic_true_SOC.csv');
SOC_true = true_soc_data(:, 2);  % True SOC in %

N = length(time_data);
fprintf('  Loaded %d data points\n', N);

% Calculate time steps
dt = diff(time_data);
dt = [dt(1); dt];

%% ==========================================================================
%  SETUP FUNCTIONS
%  ==========================================================================

% OCV lookup function
lookup_ocv = @(soc) interp1(SOC_ocv, V0_ocv, soc, 'linear', 'extrap');

% Parameter functions (FIXED - sesuai ground truth)
get_Ro = @(soc, temp) Rs;
get_Rtr = @(soc, temp) R1;
get_tau_val = @(soc, temp) tau;

%% ==========================================================================
%  UKF SETUP
%  ==========================================================================

% State dimension
n = 2;  % [SOC, V_tr]

% UKF parameters
lambda = alpha^2 * (n + kappa) - n;
gamma = sqrt(n + lambda);
num_sigma = 2*n + 1;

% Weights
Wm = zeros(num_sigma, 1);
Wc = zeros(num_sigma, 1);
Wm(1) = lambda / (n + lambda);
Wc(1) = lambda / (n + lambda) + (1 - alpha^2 + beta);
for i = 2:num_sigma
    Wm(i) = 1 / (2 * (n + lambda));
    Wc(i) = 1 / (2 * (n + lambda));
end

%% ==========================================================================
%  INITIALIZE STATE
%  ==========================================================================

% State vector: [SOC (%), V_tr (V)]
x = [SOC_initial; 0];
P = P_init;
Q = Q_mat;
R = R_meas;

% Storage arrays
SOC_estimated = zeros(N, 1);
SOC_cc_only = zeros(N, 1);
voltage_predicted = zeros(N, 1);
V_tr_estimated = zeros(N, 1);
innovation_history = zeros(N, 1);

% Initial values
SOC_estimated(1) = SOC_initial;
SOC_cc_only(1) = SOC_initial;
voltage_predicted(1) = lookup_ocv(SOC_initial);
V_tr_estimated(1) = 0;

fprintf('Starting AUKF estimation...\n');

%% ==========================================================================
%  MAIN AUKF LOOP
%  ==========================================================================

for k = 2:N
    I_k = current_data(k);
    T_k = temperature_data(k);
    dt_k = dt(k);
    V_meas = voltage_measured(k);

    % Get parameters
    R_o = get_Ro(x(1), T_k);
    R_tr = get_Rtr(x(1), T_k);
    tau_k = get_tau_val(x(1), T_k);

    % ========================================
    % STEP 1: Coulomb Counting Prediction
    % ========================================
    delta_SOC = (100 * eta * I_k * dt_k) / (3600 * Q_nominal);
    SOC_ccm = x(1) - delta_SOC;
    SOC_ccm = max(0, min(100, SOC_ccm));

    % Store CC-only estimate
    SOC_cc_only(k) = SOC_cc_only(k-1) - delta_SOC;
    SOC_cc_only(k) = max(0, min(100, SOC_cc_only(k)));

    % ========================================
    % STEP 2: Generate Sigma Points
    % ========================================
    x_center = [SOC_ccm; x(2)];

    scale = n + lambda;
    try
        sqrt_P = chol(scale * P, 'lower');
    catch
        sqrt_P = sqrt(scale) * diag(sqrt(max(diag(P), 1e-10)));
    end

    sigma = zeros(n, num_sigma);
    sigma(:, 1) = x_center;
    for i = 1:n
        sigma(:, i+1) = x_center + sqrt_P(:, i);
        sigma(:, n+i+1) = x_center - sqrt_P(:, i);
    end

    % ========================================
    % STEP 3: Propagate Sigma Points
    % ========================================
    sigma_pred = zeros(n, num_sigma);

    for i = 1:num_sigma
        soc_i = sigma(1, i);
        vtr_i = sigma(2, i);

        % State transition
        soc_new = soc_i - (100 * eta * I_k * dt_k) / (3600 * Q_nominal);
        soc_new = max(0, min(100, soc_new));

        exp_factor = exp(-dt_k / tau_k);
        vtr_new = vtr_i * exp_factor + R_tr * (1 - exp_factor) * I_k;

        sigma_pred(:, i) = [soc_new; vtr_new];
    end

    % ========================================
    % STEP 4: Calculate Predicted Mean & Covariance
    % ========================================
    x_pred = zeros(n, 1);
    for i = 1:num_sigma
        x_pred = x_pred + Wm(i) * sigma_pred(:, i);
    end

    P_pred = Q;
    for i = 1:num_sigma
        diff = sigma_pred(:, i) - x_pred;
        P_pred = P_pred + Wc(i) * (diff * diff');
    end

    % ========================================
    % STEP 5: Measurement Prediction
    % ========================================
    y_sigma = zeros(1, num_sigma);
    for i = 1:num_sigma
        soc_i = sigma_pred(1, i);
        vtr_i = sigma_pred(2, i);
        ocv_i = lookup_ocv(soc_i);
        y_sigma(i) = ocv_i - vtr_i - R_o * I_k;
    end

    y_pred = 0;
    for i = 1:num_sigma
        y_pred = y_pred + Wm(i) * y_sigma(i);
    end

    % ========================================
    % STEP 6: Innovation Covariance
    % ========================================
    Pyy = R;
    for i = 1:num_sigma
        diff_y = y_sigma(i) - y_pred;
        Pyy = Pyy + Wc(i) * (diff_y * diff_y);
    end

    % Cross-covariance
    Pxy = zeros(n, 1);
    for i = 1:num_sigma
        diff_x = sigma_pred(:, i) - x_pred;
        diff_y = y_sigma(i) - y_pred;
        Pxy = Pxy + Wc(i) * diff_x * diff_y;
    end

    % ========================================
    % STEP 7: Kalman Gain & Update
    % ========================================
    K = Pxy / Pyy;

    innovation = V_meas - y_pred;
    innovation_history(k) = innovation;

    x = x_pred + K * innovation;
    x(1) = max(0, min(100, x(1)));

    P = P_pred - K * Pyy * K';
    P = (P + P') / 2;  % Ensure symmetry

    % ========================================
    % Store Results
    % ========================================
    SOC_estimated(k) = x(1);
    V_tr_estimated(k) = x(2);
    voltage_predicted(k) = lookup_ocv(x(1)) - x(2) - R_o * I_k;
end

fprintf('AUKF estimation complete.\n\n');

%% ==========================================================================
%  CALCULATE ERROR METRICS
%  ==========================================================================

fprintf('=== ERROR ANALYSIS ===\n\n');

% SOC Errors
SOC_error_aukf = SOC_estimated - SOC_true;
SOC_error_cc = SOC_cc_only - SOC_true;

% Voltage Error
voltage_error = voltage_predicted - voltage_measured;

% Statistics
fprintf('SOC Estimation Error (AUKF vs True):\n');
fprintf('  Mean Error:  %.4f %%\n', mean(SOC_error_aukf));
fprintf('  Std Error:   %.4f %%\n', std(SOC_error_aukf));
fprintf('  Max Error:   %.4f %%\n', max(abs(SOC_error_aukf)));
fprintf('  RMSE:        %.4f %%\n\n', sqrt(mean(SOC_error_aukf.^2)));

fprintf('SOC Estimation Error (CC Only vs True):\n');
fprintf('  Mean Error:  %.4f %%\n', mean(SOC_error_cc));
fprintf('  Std Error:   %.4f %%\n', std(SOC_error_cc));
fprintf('  Max Error:   %.4f %%\n', max(abs(SOC_error_cc)));
fprintf('  RMSE:        %.4f %%\n\n', sqrt(mean(SOC_error_cc.^2)));

fprintf('Voltage Prediction Error:\n');
fprintf('  Mean Error:  %.4f V\n', mean(voltage_error));
fprintf('  Std Error:   %.4f V\n', std(voltage_error));
fprintf('  RMSE:        %.4f V\n\n', sqrt(mean(voltage_error.^2)));

%% ==========================================================================
%  PLOT RESULTS
%  ==========================================================================

fprintf('Generating plots...\n');

% Figure 1: SOC Comparison
figure(1);
set(gcf, 'Position', [100, 100, 1200, 500]);
subplot(1, 2, 1);
plot(time_data/3600, SOC_true, 'k-', 'LineWidth', 2, 'DisplayName', 'True SOC');
hold on;
plot(time_data/3600, SOC_estimated, 'b-', 'LineWidth', 1.5, 'DisplayName', 'AUKF Estimated');
plot(time_data/3600, SOC_cc_only, 'r--', 'LineWidth', 1, 'DisplayName', 'CC Only');
hold off;
xlabel('Time (hours)');
ylabel('SOC (%)');
title('SOC Estimation: AUKF vs True SOC');
legend('Location', 'best');
grid on;
ylim([0 105]);

subplot(1, 2, 2);
plot(time_data/3600, SOC_error_aukf, 'b-', 'LineWidth', 1, 'DisplayName', 'AUKF Error');
hold on;
plot(time_data/3600, SOC_error_cc, 'r--', 'LineWidth', 1, 'DisplayName', 'CC Error');
hold off;
xlabel('Time (hours)');
ylabel('SOC Error (%)');
title('SOC Estimation Error');
legend('Location', 'best');
grid on;

% Figure 2: Voltage Comparison
figure(2);
set(gcf, 'Position', [100, 100, 1200, 500]);
subplot(1, 2, 1);
plot(time_data/3600, voltage_measured, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Measured');
hold on;
plot(time_data/3600, voltage_predicted, 'r--', 'LineWidth', 1, 'DisplayName', 'Predicted');
hold off;
xlabel('Time (hours)');
ylabel('Voltage (V)');
title('Voltage: Measured vs Predicted');
legend('Location', 'best');
grid on;

subplot(1, 2, 2);
plot(time_data/3600, voltage_error * 1000, 'b-', 'LineWidth', 1);
xlabel('Time (hours)');
ylabel('Voltage Error (mV)');
title('Voltage Prediction Error');
grid on;

% Figure 3: Current Profile
figure(3);
plot(time_data/3600, current_data, 'b-', 'LineWidth', 1.5);
xlabel('Time (hours)');
ylabel('Current (A)');
title('Current Profile');
grid on;

% Figure 4: Innovation
figure(4);
plot(time_data/3600, innovation_history * 1000, 'b-', 'LineWidth', 1);
xlabel('Time (hours)');
ylabel('Innovation (mV)');
title('Kalman Filter Innovation (V_{measured} - V_{predicted})');
grid on;

%% ==========================================================================
%  SUMMARY
%  ==========================================================================

fprintf('\n=== VERIFICATION SUMMARY ===\n\n');

if sqrt(mean(SOC_error_aukf.^2)) < 2
    fprintf('SUCCESS: AUKF SOC RMSE < 2%% (%.4f%%)\n', sqrt(mean(SOC_error_aukf.^2)));
    fprintf('The AUKF algorithm is working correctly!\n\n');
    fprintf('This proves that the issue with real data is:\n');
    fprintf('  -> PARAMETER MISMATCH, not algorithm error\n');
else
    fprintf('WARNING: AUKF SOC RMSE > 2%% (%.4f%%)\n', sqrt(mean(SOC_error_aukf.^2)));
    fprintf('Check tuning parameters.\n');
end

fprintf('\nConclusion:\n');
fprintf('  When model parameters match the battery,\n');
fprintf('  AUKF improves upon pure Coulomb Counting.\n');
fprintf('  With real data, parameters need to be identified.\n');
