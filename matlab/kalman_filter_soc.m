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

% --- Parameter Baterai (sesuai paper Braun et al. 2022) ---
Q_nominal = 19.96;      % Kapasitas nominal baterai (Ah)
eta = 1.0;              % Efisiensi Coulomb (1.0 = 100%)
SOC_initial = 50;       % SOC awal (%) - sesuaikan atau set [] untuk auto

% --- Parameter Model RC (sesuai paper Braun et al. 2022) ---
% Model RC dinamis: R1 = R1a * |I| + R1b
Rs = 0.001;             % Serial resistance [Ohm]
R1a = 4.667e-05;        % RC resistance component [Ohm/A]
R1b = 0.001767;         % RC resistance base [Ohm]
C1 = 10.536;            % Kapasitansi RC [F]

% --- Parameter UKF ---
alpha = 1.0;            % Spread of sigma points (0.001 to 1)
beta = 2.0;             % Prior knowledge (2 for Gaussian - optimal)
kappa = 0.0;            % Secondary scaling parameter

% --- Initial Covariances ---
% TIPS untuk tuning:
%   - P_soc tinggi (misal 100): filter lebih percaya measurement, konvergensi cepat
%   - P_soc rendah (misal 1e-6): filter lebih percaya model, konvergensi lambat
%   - Q tinggi: filter lebih responsif tapi lebih noisy
%   - R tinggi: filter lebih smooth tapi lambat merespon
P_soc_init = 100;       % Initial SOC variance (tinggi = tidak yakin SOC awal)
P_vtr_init = 0.01;      % Initial V_tr variance
P_init = [P_soc_init, 0; 0, P_vtr_init];

Q_soc = 0.001;          % Process noise SOC (model uncertainty)
Q_vtr = 0.001;          % Process noise V_tr
Q_init = [Q_soc, 0; 0, Q_vtr];

R_init = 0.01;          % Measurement noise (voltage sensor noise ~0.1V std dev)

% --- Adaptive Noise Settings ---
adaptive_window = 10;   % Window size untuk adaptive noise estimation
enable_adaptive = true; % Enable/disable adaptive noise

% --- Warm-up Period ---
% Filter butuh waktu untuk konvergensi, skip data awal untuk statistik
warmup_samples = 100;   % Jumlah sampel awal yang di-skip untuk perhitungan RMSE
                        % Set ke 0 untuk tidak skip

% --- Path File Data ---
ocv_file = '../OCV_vs_SOC_curve.csv';
exp_file = '../Experimental_data_fresh_cell.csv';

% --- Konvensi Arus ---
current_positive_discharge = true;

% --- Time Range Selection ---
% Pilih range waktu data yang ingin digunakan (dalam detik)
% Set ke [] atau 'auto' untuk menggunakan seluruh data
% Contoh: time_start = 0; time_end = 3600; untuk 1 jam pertama
time_start = [];        % Waktu mulai (detik), [] = dari awal
time_end = [];          % Waktu akhir (detik), [] = sampai akhir

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
    time_data_full = exp_data.Time;
    current_data_full = exp_data.Current;
    voltage_measured_full = exp_data.Voltage;
    temperature_data_full = exp_data.Temperature;
    fprintf('  -> Experimental data loaded: %d samples\n', length(time_data_full));
    fprintf('  -> Full duration: %.2f seconds (%.2f hours)\n', ...
            time_data_full(end) - time_data_full(1), (time_data_full(end) - time_data_full(1))/3600);
catch ME
    error('Gagal load experimental data: %s', ME.message);
end

% --- Apply Time Range Filter ---
% Determine time range
if isempty(time_start)
    t_start = time_data_full(1);
else
    t_start = time_start;
end

if isempty(time_end)
    t_end = time_data_full(end);
else
    t_end = time_end;
end

% Find indices within time range
idx_range = (time_data_full >= t_start) & (time_data_full <= t_end);

% Extract data within time range
time_data = time_data_full(idx_range);
current_data = current_data_full(idx_range);
voltage_measured = voltage_measured_full(idx_range);
temperature_data = temperature_data_full(idx_range);

fprintf('\n--- Time Range Selection ---\n');
fprintf('  -> Requested range: %.2f s to %.2f s\n', t_start, t_end);
fprintf('  -> Actual range: %.2f s to %.2f s\n', time_data(1), time_data(end));
fprintf('  -> Selected samples: %d (dari %d total)\n', length(time_data), length(time_data_full));
fprintf('  -> Selected duration: %.2f seconds (%.2f hours)\n', ...
        time_data(end) - time_data(1), (time_data(end) - time_data(1))/3600);

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

% Fungsi INVERSE: lookup SOC dari OCV (untuk auto-estimasi SOC awal)
lookup_soc_from_ocv = @(ocv) interp1(V0_ocv, SOC_ocv, ocv, 'linear', 'extrap');

% Fungsi lookup parameter RC (model dinamis sesuai paper Braun et al. 2022)
% R1 = R1a * |I| + R1b (tergantung arus)
get_Ro = @(soc, temp) Rs;              % Serial resistance konstan
get_R1 = @(I) R1a * abs(I) + R1b;      % RC resistance tergantung arus
get_tau = @(I) get_R1(I) * C1;         % Time constant dinamis

%% ==========================================================================
%  AUTO-ESTIMATE INITIAL SOC (jika tidak ditentukan)
%  ==========================================================================

if isempty(SOC_initial)
    % Estimasi SOC awal dari voltage pertama menggunakan kurva OCV
    V_first = voltage_measured(1);
    I_first = current_data(1);

    % Kompensasi resistansi: OCV = V_terminal + R_o * I
    OCV_estimated = V_first + Rs * I_first;

    % Lookup SOC dari OCV
    SOC_initial = lookup_soc_from_ocv(OCV_estimated);
    SOC_initial = max(0, min(100, SOC_initial));

    fprintf('\n--- Auto-Estimasi SOC Awal ---\n');
    fprintf('  -> Voltage pertama: %.4f V\n', V_first);
    fprintf('  -> Current pertama: %.4f A\n', I_first);
    fprintf('  -> OCV terkoreksi: %.4f V\n', OCV_estimated);
    fprintf('  -> SOC awal (estimasi): %.2f%%\n', SOC_initial);
end

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

    % Get battery parameters (model dinamis sesuai paper)
    R_o = get_Ro(x(1), T_k);
    R1 = get_R1(I_k);              % RC resistance tergantung arus
    tau = get_tau(I_k);            % Time constant dinamis

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
        vtr_new = vtr_i * exp_factor + R1 * (1 - exp_factor) * I_k;

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

% Calculate RMSE (excluding warm-up period)
if warmup_samples > 0 && warmup_samples < N
    idx_valid = (warmup_samples+1):N;
    voltage_RMSE_warmup = sqrt(mean(voltage_error(idx_valid).^2));
    voltage_RMSE_full = sqrt(mean(voltage_error.^2));
    fprintf('  -> Voltage RMSE (full): %.4f V\n', voltage_RMSE_full);
    fprintf('  -> Voltage RMSE (setelah warmup %d samples): %.4f V\n', warmup_samples, voltage_RMSE_warmup);
else
    voltage_RMSE_full = sqrt(mean(voltage_error.^2));
    voltage_RMSE_warmup = voltage_RMSE_full;
    fprintf('  -> Voltage RMSE: %.4f V\n', voltage_RMSE_full);
end

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

% Parameter yang digunakan (sesuai paper Braun et al. 2022)
results.parameters.Q_nominal = Q_nominal;
results.parameters.eta = eta;
results.parameters.SOC_initial = SOC_initial;
results.parameters.Rs = Rs;
results.parameters.R1a = R1a;
results.parameters.R1b = R1b;
results.parameters.C1 = C1;
results.parameters.UKF.alpha = alpha;
results.parameters.UKF.beta = beta;
results.parameters.UKF.kappa = kappa;
results.parameters.P_init = P_init;
results.parameters.Q_init = Q_init;
results.parameters.R_init = R_init;
results.parameters.time_start = t_start;
results.parameters.time_end = t_end;

% Statistik
results.statistics.SOC_final_AUKF = SOC_estimated(end);
results.statistics.SOC_final_CC = SOC_cc_only(end);
results.statistics.voltage_RMSE_full = voltage_RMSE_full;
results.statistics.voltage_RMSE_after_warmup = voltage_RMSE_warmup;
results.statistics.warmup_samples = warmup_samples;

if warmup_samples > 0 && warmup_samples < N
    results.statistics.voltage_MAE_full = mean(abs(voltage_error));
    results.statistics.voltage_MAE_after_warmup = mean(abs(voltage_error(idx_valid)));
    results.statistics.innovation_mean = mean(innovation_history(idx_valid));
    results.statistics.innovation_std = std(innovation_history(idx_valid));
else
    results.statistics.voltage_MAE_full = mean(abs(voltage_error));
    results.statistics.voltage_MAE_after_warmup = mean(abs(voltage_error));
    results.statistics.innovation_mean = mean(innovation_history);
    results.statistics.innovation_std = std(innovation_history);
end

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
fprintf('Time Range:         %.2f s - %.2f s\n', t_start, t_end);
fprintf('Duration:           %.2f hours\n', (t_end - t_start)/3600);
fprintf('Data Points:        %d samples\n', length(time_data));
fprintf('Warmup Samples:     %d\n', warmup_samples);
fprintf('----------------------------------------\n');
fprintf('SOC Awal:           %.2f%%\n', SOC_initial);
fprintf('SOC Akhir (AUKF):   %.2f%%\n', SOC_estimated(end));
fprintf('SOC Akhir (CC):     %.2f%%\n', SOC_cc_only(end));
fprintf('Perbedaan:          %.2f%%\n', abs(SOC_estimated(end) - SOC_cc_only(end)));
fprintf('----------------------------------------\n');
fprintf('AKURASI (full data):\n');
fprintf('  Voltage RMSE:     %.4f V\n', voltage_RMSE_full);
fprintf('  Voltage MAE:      %.4f V\n', results.statistics.voltage_MAE_full);
if warmup_samples > 0 && warmup_samples < N
    fprintf('AKURASI (setelah warmup):\n');
    fprintf('  Voltage RMSE:     %.4f V\n', voltage_RMSE_warmup);
    fprintf('  Voltage MAE:      %.4f V\n', results.statistics.voltage_MAE_after_warmup);
end
fprintf('----------------------------------------\n');
fprintf('Innovation Mean:    %.4f V\n', results.statistics.innovation_mean);
fprintf('Innovation Std:     %.4f V\n', results.statistics.innovation_std);
fprintf('========================================\n');
fprintf('\nTIPS JIKA HASIL TIDAK AKURAT:\n');
fprintf('1. Sesuaikan kurva OCV dengan baterai Anda\n');
fprintf('2. Tuning parameter R_o, R_tr, tau\n');
fprintf('3. Naikkan P_soc_init untuk konvergensi cepat\n');
fprintf('4. Set SOC_initial = [] untuk auto-estimasi\n');
fprintf('========================================\n');
