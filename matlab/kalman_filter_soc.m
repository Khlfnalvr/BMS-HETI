%% ==========================================================================
%  ADAPTIVE UNSCENTED KALMAN FILTER (AUKF) SOC ESTIMATION
%  ==========================================================================
%  Implementasi AUKF untuk estimasi State of Charge (SOC) dengan koreksi OCV
%
%  Alur Algoritma:
%    1. Model (Coulomb Counting) -> prediksi SOC
%    2. State Space Equations -> prediksi Voltage menggunakan OCV
%    3. Error = Voltage_terukur - Voltage_prediksi
%    4. Filter (AUKF) -> koreksi SOC berdasarkan error
%    5. Adaptive -> update Q dan R
%
%  State Vector: x = [SOC; V_tr]
%    - SOC: State of Charge (%)
%    - V_tr: Transient voltage dari model RC (V)
%
%  Measurement Function (MENGGUNAKAN OCV):
%    V_terminal = OCV(SOC) - V_tr - R_o * I
%
%  Input Files:
%    - OCV_vs_SOC_curve.csv (columns: SOC, V0)
%    - Experimental_data_fresh_cell.csv (columns: Time, Current, Voltage, Temperature)
%
%  Output:
%    - Grafik SOC, Voltage, Current, dan Innovation vs Time (figure terpisah)
%    - File kalman_filter_results.mat
%  ==========================================================================

clear; clc; close all;

%% ==========================================================================
%  PARAMETER KONFIGURASI (Ubah sesuai kebutuhan)
%  ==========================================================================

% --- Parameter Baterai ---
Q_nominal = 2.6;        % Kapasitas nominal baterai (Ah)
eta = 1.0;              % Efisiensi Coulomb (1.0 = 100%)
SOC_initial = 100;      % SOC awal (%) - sesuaikan dengan kondisi eksperimen

% --- Parameter Model RC (default dari kode C, untuk 25°C) ---
% Nilai ini akan digunakan jika tidak ada lookup table
R_o_default = 0.052;    % Series resistance (Ohm)
R_tr_default = 0.008;   % Transient resistance (Ohm)
tau_default = 130;      % Time constant (seconds)

% --- Parameter UKF ---
alpha = 1.0;            % Spread of sigma points (0.001 to 1)
beta = 1.0;             % Prior knowledge (2 for Gaussian)
kappa = 0.0;            % Secondary scaling parameter

% --- Initial Covariances ---
P_init = [1e-6, 0; 0, 1.0];     % State covariance: sangat yakin SOC, tidak yakin V_tr
Q_init = [0.1, 0; 0, 0.1];      % Process noise covariance
R_init = 0.25;                   % Measurement noise covariance (~0.5V std dev)

% --- Adaptive Noise Settings ---
adaptive_window = 10;   % Window size untuk adaptive noise estimation
enable_adaptive = true; % Enable/disable adaptive noise

% --- Path File Data ---
ocv_file = '../OCV_vs_SOC_curve.csv';
exp_file = '../Experimental_data_fresh_cell.csv';

% --- Konvensi Arus ---
current_positive_discharge = true;

%% ==========================================================================
%  LOAD DATA
%  ==========================================================================

fprintf('=== ADAPTIVE UNSCENTED KALMAN FILTER SOC ESTIMATION ===\n\n');

% --- Load OCV Curve ---
fprintf('Loading OCV curve dari: %s\n', ocv_file);
try
    ocv_data = readtable(ocv_file);
    SOC_ocv = ocv_data.SOC;
    V0_ocv = ocv_data.V0;
    fprintf('  -> OCV curve loaded: %d data points\n', length(SOC_ocv));
catch ME
    warning('Tidak dapat load OCV file: %s', ME.message);
    warning('Menggunakan OCV curve default dari kode C');
    SOC_ocv = [0; 10; 25; 50; 75; 90; 100];
    V0_ocv = [3.0; 3.06; 3.13; 3.22; 3.42; 3.58; 3.7];
end

% --- Load Experimental Data ---
fprintf('Loading experimental data dari: %s\n', exp_file);
try
    exp_data = readtable(exp_file);
    time_data = exp_data.Time;
    current_data = exp_data.Current;
    voltage_measured = exp_data.Voltage;
    temperature_data = exp_data.Temperature;
    fprintf('  -> Experimental data loaded: %d samples\n', length(time_data));
    fprintf('  -> Duration: %.2f seconds (%.2f minutes)\n', ...
            time_data(end) - time_data(1), (time_data(end) - time_data(1))/60);
catch ME
    error('Gagal load experimental data: %s', ME.message);
end

%% ==========================================================================
%  PRE-PROCESSING
%  ==========================================================================

N = length(time_data);
dt = zeros(N, 1);
dt(1) = 0;
for k = 2:N
    dt(k) = time_data(k) - time_data(k-1);
end

if ~current_positive_discharge
    current_data = -current_data;
end

fprintf('\nStatistik Data:\n');
fprintf('  -> Current range: %.3f A to %.3f A\n', min(current_data), max(current_data));
fprintf('  -> Voltage range: %.3f V to %.3f V\n', min(voltage_measured), max(voltage_measured));
fprintf('  -> Temperature range: %.1f C to %.1f C\n', min(temperature_data), max(temperature_data));
fprintf('  -> Average dt: %.3f seconds\n', mean(dt(2:end)));

%% ==========================================================================
%  HELPER FUNCTIONS
%  ==========================================================================

% Fungsi lookup OCV dari SOC
lookup_ocv = @(soc) interp1(SOC_ocv, V0_ocv, soc, 'linear', 'extrap');

% Fungsi lookup parameter RC (simplified - bisa diperluas dengan lookup table)
get_Ro = @(soc, temp) R_o_default;
get_Rtr = @(soc, temp) R_tr_default;
get_tau = @(soc, temp) tau_default;

%% ==========================================================================
%  INITIALIZE UKF PARAMETERS
%  ==========================================================================

n = 2;  % State dimension [SOC, V_tr]
num_sigma = 2*n + 1;  % Number of sigma points

% Calculate lambda
lambda = alpha^2 * (n + kappa) - n;

% Calculate weights
Wm = zeros(num_sigma, 1);
Wc = zeros(num_sigma, 1);

Wm(1) = lambda / (n + lambda);
Wc(1) = lambda / (n + lambda) + (1 - alpha^2 + beta);

for i = 2:num_sigma
    Wm(i) = 1 / (2 * (n + lambda));
    Wc(i) = 1 / (2 * (n + lambda));
end

fprintf('\nParameter UKF:\n');
fprintf('  -> alpha = %.3f, beta = %.3f, kappa = %.3f\n', alpha, beta, kappa);
fprintf('  -> lambda = %.3f\n', lambda);
fprintf('  -> Sigma points: %d\n', num_sigma);

%% ==========================================================================
%  AUKF ALGORITHM
%  ==========================================================================

fprintf('\n--- Menjalankan Adaptive UKF ---\n');

% Initialize state
x = [SOC_initial; 0];  % [SOC; V_tr]
P = P_init;
Q = Q_init;
R = R_init;

% Storage arrays
SOC_estimated = zeros(N, 1);
SOC_cc_only = zeros(N, 1);  % Coulomb counting saja untuk perbandingan
V_tr_estimated = zeros(N, 1);
voltage_predicted = zeros(N, 1);
innovation_history = zeros(N, 1);
residual_history = zeros(N, 1);
K_gain_soc = zeros(N, 1);

% Adaptive noise storage
innov_buffer = zeros(adaptive_window, 1);
resid_buffer = zeros(adaptive_window, 1);
buffer_idx = 1;
buffer_count = 0;
K_prev = [0; 0];

% Initial values
SOC_estimated(1) = SOC_initial;
SOC_cc_only(1) = SOC_initial;
V_tr_estimated(1) = 0;
voltage_predicted(1) = lookup_ocv(SOC_initial);

% Main AUKF loop
for k = 2:N
    I_k = current_data(k);
    T_k = temperature_data(k);
    dt_k = dt(k);
    V_meas = voltage_measured(k);

    % Get battery parameters
    R_o = get_Ro(x(1), T_k);
    R_tr = get_Rtr(x(1), T_k);
    tau = get_tau(x(1), T_k);

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
    % Use CCM prediction as the center
    x_center = [SOC_ccm; x(2)];

    % Calculate sqrt((n+lambda)*P) using Cholesky decomposition
    scale = n + lambda;
    try
        sqrt_P = chol(scale * P, 'lower');
    catch
        % If P is not positive definite, use simplified sqrt
        sqrt_P = sqrt(scale) * diag(sqrt(max(diag(P), 1e-10)));
    end

    % Generate sigma points
    sigma = zeros(n, num_sigma);
    sigma(:, 1) = x_center;

    for i = 1:n
        sigma(:, i+1) = x_center + sqrt_P(:, i);
        sigma(:, n+i+1) = x_center - sqrt_P(:, i);
    end

    % ========================================
    % STEP 3: Propagate Sigma Points (State Transition)
    % ========================================
    sigma_pred = zeros(n, num_sigma);

    for i = 1:num_sigma
        soc_i = sigma(1, i);
        vtr_i = sigma(2, i);

        % State transition (Coulomb Counting + RC dynamics)
        soc_new = soc_i - (100 * eta * I_k * dt_k) / (3600 * Q_nominal);
        soc_new = max(0, min(100, soc_new));

        exp_factor = exp(-dt_k / tau);
        vtr_new = vtr_i * exp_factor + R_tr * (1 - exp_factor) * I_k;

        sigma_pred(:, i) = [soc_new; vtr_new];
    end

    % ========================================
    % STEP 4: Calculate Predicted State Mean and Covariance
    % ========================================
    x_pred = zeros(n, 1);
    for i = 1:num_sigma
        x_pred = x_pred + Wm(i) * sigma_pred(:, i);
    end

    P_pred = Q;  % Start with process noise
    for i = 1:num_sigma
        diff = sigma_pred(:, i) - x_pred;
        P_pred = P_pred + Wc(i) * (diff * diff');
    end

    % ========================================
    % STEP 5: Propagate Through Measurement Function (USES OCV!)
    % ========================================
    y_sigma = zeros(1, num_sigma);

    for i = 1:num_sigma
        soc_i = sigma_pred(1, i);
        vtr_i = sigma_pred(2, i);

        % Measurement function: V = OCV(SOC) - V_tr - R_o * I
        ocv_i = lookup_ocv(soc_i);
        y_sigma(i) = ocv_i - vtr_i - R_o * I_k;
    end

    % Predicted measurement
    y_pred = 0;
    for i = 1:num_sigma
        y_pred = y_pred + Wm(i) * y_sigma(i);
    end

    % ========================================
    % STEP 6: Calculate Covariances
    % ========================================
    % Innovation covariance Pyy
    Pyy = R;
    for i = 1:num_sigma
        diff_y = y_sigma(i) - y_pred;
        Pyy = Pyy + Wc(i) * diff_y^2;
    end

    % Cross-covariance Pxy
    Pxy = zeros(n, 1);
    for i = 1:num_sigma
        diff_x = sigma_pred(:, i) - x_pred;
        diff_y = y_sigma(i) - y_pred;
        Pxy = Pxy + Wc(i) * diff_x * diff_y;
    end

    % ========================================
    % STEP 7: Kalman Gain and State Correction
    % ========================================
    K = Pxy / (Pyy + 1e-10);
    K_prev = K;

    % Innovation
    innovation = V_meas - y_pred;

    % State correction
    x_corr = x_pred + K * innovation;
    x_corr(1) = max(0, min(100, x_corr(1)));  % Clamp SOC

    % Covariance correction
    P_corr = P_pred - K * Pyy * K';

    % Ensure positive definite
    P_corr(1, 1) = max(P_corr(1, 1), 1e-10);
    P_corr(2, 2) = max(P_corr(2, 2), 1e-10);

    % ========================================
    % STEP 8: Calculate Residual
    % ========================================
    % Residual based on corrected state
    ocv_corr = lookup_ocv(x_corr(1));
    y_corr = ocv_corr - x_corr(2) - R_o * I_k;
    residual = V_meas - y_corr;

    % ========================================
    % STEP 9: Adaptive Noise Update
    % ========================================
    if enable_adaptive
        % Add to buffer
        innov_buffer(buffer_idx) = innovation;
        resid_buffer(buffer_idx) = residual;
        buffer_idx = mod(buffer_idx, adaptive_window) + 1;
        buffer_count = min(buffer_count + 1, adaptive_window);

        if buffer_count >= 2
            % Calculate innovation variance
            innov_mean = mean(innov_buffer(1:buffer_count));
            Cd = var(innov_buffer(1:buffer_count));

            % Update Q: Q = K * Cd * K'
            Q_new = K_prev * Cd * K_prev';
            Q_new(1, 1) = max(min(Q_new(1, 1), 1.0), 1e-6);
            Q_new(2, 2) = max(min(Q_new(2, 2), 1.0), 1e-6);
            Q = Q_new;

            % Calculate residual variance
            resid_mean = mean(resid_buffer(1:buffer_count));
            Cr = var(resid_buffer(1:buffer_count));

            % Update R: R = Cr + H*P*H' (simplified: H = [dV/dSOC, -1])
            R_new = Cr + P_corr(1, 1);
            R = max(min(R_new, 1.0), 0.01);
        end
    end

    % ========================================
    % STEP 10: Update State and Store
    % ========================================
    x = x_corr;
    P = P_corr;

    SOC_estimated(k) = x(1);
    V_tr_estimated(k) = x(2);
    voltage_predicted(k) = y_pred;
    innovation_history(k) = innovation;
    residual_history(k) = residual;
    K_gain_soc(k) = K(1);
end

fprintf('  -> SOC awal: %.2f%%\n', SOC_initial);
fprintf('  -> SOC akhir (AUKF): %.2f%%\n', SOC_estimated(end));
fprintf('  -> SOC akhir (CC only): %.2f%%\n', SOC_cc_only(end));

% Calculate errors
voltage_error = voltage_measured - voltage_predicted;
fprintf('  -> Voltage RMSE: %.4f V\n', sqrt(mean(voltage_error.^2)));

%% ==========================================================================
%  OUTPUT - GRAFIK (Figure Terpisah)
%  ==========================================================================

fprintf('\n--- Membuat Grafik ---\n');

% --- Figure 1: SOC vs Time ---
figure('Name', 'Kalman Filter - SOC vs Time', 'NumberTitle', 'off');
plot(time_data, SOC_estimated, 'b-', 'LineWidth', 1.5, 'DisplayName', 'SOC (AUKF)');
hold on;
plot(time_data, SOC_cc_only, 'r--', 'LineWidth', 1.0, 'DisplayName', 'SOC (CC Only)');
hold off;
xlabel('Time (s)');
ylabel('State of Charge (%)');
title('Kalman Filter: SOC Estimation vs Time');
grid on;
ylim([0 105]);
legend('Location', 'best');

% --- Figure 2: Voltage vs Time ---
figure('Name', 'Kalman Filter - Voltage vs Time', 'NumberTitle', 'off');
plot(time_data, voltage_measured, 'b-', 'LineWidth', 1.0, 'DisplayName', 'Voltage Measured');
hold on;
plot(time_data, voltage_predicted, 'r--', 'LineWidth', 1.0, 'DisplayName', 'Voltage Predicted');
hold off;
xlabel('Time (s)');
ylabel('Voltage (V)');
title('Kalman Filter: Voltage Comparison');
grid on;
legend('Location', 'best');

% --- Figure 3: Current vs Time ---
figure('Name', 'Kalman Filter - Current vs Time', 'NumberTitle', 'off');
plot(time_data, current_data, 'g-', 'LineWidth', 1.0);
xlabel('Time (s)');
ylabel('Current (A)');
title('Kalman Filter: Current Profile');
grid on;
legend('Current', 'Location', 'best');

% --- Figure 4: Innovation vs Time ---
figure('Name', 'Kalman Filter - Innovation vs Time', 'NumberTitle', 'off');
plot(time_data, innovation_history, 'm-', 'LineWidth', 1.0);
xlabel('Time (s)');
ylabel('Innovation (V)');
title('Kalman Filter: Innovation (Measurement Error)');
grid on;
legend('Innovation', 'Location', 'best');
hold on;
plot([time_data(1), time_data(end)], [0, 0], 'k--', 'LineWidth', 0.5);
hold off;

% --- Figure 5 (Tambahan): V_tr vs Time ---
figure('Name', 'Kalman Filter - Transient Voltage vs Time', 'NumberTitle', 'off');
plot(time_data, V_tr_estimated, 'c-', 'LineWidth', 1.0);
xlabel('Time (s)');
ylabel('V_{tr} (V)');
title('Kalman Filter: Transient Voltage (V_{tr})');
grid on;
legend('V_{tr}', 'Location', 'best');

% --- Figure 6 (Tambahan): Kalman Gain vs Time ---
figure('Name', 'Kalman Filter - Kalman Gain vs Time', 'NumberTitle', 'off');
plot(time_data, K_gain_soc, 'k-', 'LineWidth', 1.0);
xlabel('Time (s)');
ylabel('Kalman Gain (SOC)');
title('Kalman Filter: Kalman Gain for SOC');
grid on;
legend('K_{SOC}', 'Location', 'best');

%% ==========================================================================
%  SAVE RESULTS
%  ==========================================================================

fprintf('\n--- Menyimpan Hasil ---\n');

% Struktur hasil
results.algorithm = 'Adaptive Unscented Kalman Filter';
results.time_data = time_data;
results.SOC_estimated = SOC_estimated;
results.SOC_cc_only = SOC_cc_only;
results.V_tr_estimated = V_tr_estimated;
results.voltage_measured = voltage_measured;
results.voltage_predicted = voltage_predicted;
results.voltage_error = voltage_error;
results.innovation = innovation_history;
results.residual = residual_history;
results.kalman_gain_soc = K_gain_soc;
results.current_data = current_data;
results.temperature_data = temperature_data;

% Parameter yang digunakan
results.parameters.Q_nominal = Q_nominal;
results.parameters.eta = eta;
results.parameters.SOC_initial = SOC_initial;
results.parameters.R_o = R_o_default;
results.parameters.R_tr = R_tr_default;
results.parameters.tau = tau_default;
results.parameters.UKF.alpha = alpha;
results.parameters.UKF.beta = beta;
results.parameters.UKF.kappa = kappa;
results.parameters.P_init = P_init;
results.parameters.Q_init = Q_init;
results.parameters.R_init = R_init;

% Statistik
results.statistics.SOC_final_AUKF = SOC_estimated(end);
results.statistics.SOC_final_CC = SOC_cc_only(end);
results.statistics.voltage_RMSE = sqrt(mean(voltage_error.^2));
results.statistics.voltage_MAE = mean(abs(voltage_error));
results.statistics.innovation_mean = mean(innovation_history);
results.statistics.innovation_std = std(innovation_history);

% Save ke file .mat
output_file = 'kalman_filter_results.mat';
save(output_file, 'results');
fprintf('  -> Hasil disimpan ke: %s\n', output_file);

%% ==========================================================================
%  RINGKASAN
%  ==========================================================================

fprintf('\n========================================\n');
fprintf('RINGKASAN HASIL KALMAN FILTER (AUKF)\n');
fprintf('========================================\n');
fprintf('SOC Awal:           %.2f%%\n', SOC_initial);
fprintf('SOC Akhir (AUKF):   %.2f%%\n', SOC_estimated(end));
fprintf('SOC Akhir (CC):     %.2f%%\n', SOC_cc_only(end));
fprintf('Perbedaan:          %.2f%%\n', abs(SOC_estimated(end) - SOC_cc_only(end)));
fprintf('Voltage RMSE:       %.4f V\n', results.statistics.voltage_RMSE);
fprintf('Voltage MAE:        %.4f V\n', results.statistics.voltage_MAE);
fprintf('Innovation Mean:    %.4f V\n', results.statistics.innovation_mean);
fprintf('Innovation Std:     %.4f V\n', results.statistics.innovation_std);
fprintf('========================================\n');
fprintf('\nKalman Filter menggunakan OCV untuk koreksi\n');
fprintf('sehingga error tidak terakumulasi seperti\n');
fprintf('Coulomb Counting murni.\n');
fprintf('========================================\n');
