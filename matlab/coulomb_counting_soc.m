%% ==========================================================================
%  COULOMB COUNTING SOC ESTIMATION
%  ==========================================================================
%  Implementasi metode Coulomb Counting untuk estimasi State of Charge (SOC)
%
%  Algoritma: SoC(k) = SoC(k-1) - (100 * eta * I * dt) / (3600 * Q)
%
%  CATATAN: Metode ini TIDAK menggunakan OCV untuk koreksi - hanya integrasi arus
%
%  Input Files:
%    - OCV_vs_SOC_curve.csv (columns: SOC, V0) - untuk referensi saja
%    - Experimental_data_fresh_cell.csv (columns: Time, Current, Voltage, Temperature)
%
%  Output:
%    - Grafik SOC, Voltage, Current, dan Error vs Time (figure terpisah)
%    - File coulomb_counting_results.mat
%  ==========================================================================

clear; clc; close all;

%% ==========================================================================
%  PARAMETER KONFIGURASI (Ubah sesuai kebutuhan)
%  ==========================================================================

% --- Parameter Baterai (sesuai paper Braun et al. 2022) ---
Q_nominal = 19.96;      % Kapasitas nominal baterai (Ah)
eta = 1.0;              % Efisiensi Coulomb (1.0 = 100%)
SOC_initial = 0;        % SOC awal (%) - eksperimen mulai dari empty

% --- Parameter Kalibrasi Voltage (untuk reset error akumulasi) ---
enable_voltage_calibration = true;  % Enable kalibrasi saat V boundary
V_upper_cal = 4.19;     % Voltage untuk kalibrasi SOC = 100%
V_lower_cal = 3.01;     % Voltage untuk kalibrasi SOC = 0%
I_threshold_cal = 4.0;  % Threshold arus untuk kalibrasi (A)

% --- Path File Data ---
% Sesuaikan path jika file CSV tidak di folder parent
ocv_file = '../OCV_vs_SOC_curve.csv';
exp_file = '../Experimental_data_fresh_cell.csv';

% --- Konvensi Arus ---
% true  = arus positif berarti discharge (default BMS)
% false = arus positif berarti charge
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

fprintf('=== COULOMB COUNTING SOC ESTIMATION ===\n\n');

% --- Load OCV Curve (untuk referensi) ---
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

% Hitung time step (dt) untuk setiap sampel
N = length(time_data);
dt = zeros(N, 1);
dt(1) = 0;
for k = 2:N
    dt(k) = time_data(k) - time_data(k-1);
end

% Sesuaikan tanda arus jika perlu
if ~current_positive_discharge
    current_data = -current_data;
end

fprintf('\nStatistik Data:\n');
fprintf('  -> Current range: %.3f A to %.3f A\n', min(current_data), max(current_data));
fprintf('  -> Voltage range: %.3f V to %.3f V\n', min(voltage_measured), max(voltage_measured));
fprintf('  -> Temperature range: %.1f C to %.1f C\n', min(temperature_data), max(temperature_data));
fprintf('  -> Average dt: %.3f seconds\n', mean(dt(2:end)));

%% ==========================================================================
%  COULOMB COUNTING ALGORITHM
%  ==========================================================================

fprintf('\n--- Menjalankan Coulomb Counting ---\n');

% Inisialisasi
SOC_estimated = zeros(N, 1);
SOC_estimated(1) = SOC_initial;
total_Ah = 0;

% Variabel untuk voltage prediction (menggunakan OCV lookup - untuk visualisasi saja)
voltage_predicted = zeros(N, 1);

% Main loop - Coulomb Counting
for k = 2:N
    % Hitung perubahan SOC
    % Formula: delta_SOC = (100 * eta * I * dt) / (3600 * Q)
    delta_SOC = (100 * eta * current_data(k) * dt(k)) / (3600 * Q_nominal);

    % Update SOC (kurangi karena arus positif = discharge)
    SOC_estimated(k) = SOC_estimated(k-1) - delta_SOC;

    % Kalibrasi saat voltage boundary (sesuai paper Braun et al. 2022)
    if enable_voltage_calibration
        if (voltage_measured(k) > V_upper_cal && abs(current_data(k)) < I_threshold_cal) || SOC_estimated(k) > 100
            SOC_estimated(k) = 100;
        elseif (voltage_measured(k) < V_lower_cal && abs(current_data(k)) < I_threshold_cal) || SOC_estimated(k) < 0
            SOC_estimated(k) = 0;
        end
    else
        % Clamp SOC ke range [0, 100]
        SOC_estimated(k) = max(0, min(100, SOC_estimated(k)));
    end

    % Hitung total Ah
    total_Ah = total_Ah + (current_data(k) * dt(k)) / 3600;
end

% Hitung voltage prediction berdasarkan SOC (untuk visualisasi)
for k = 1:N
    voltage_predicted(k) = interp1(SOC_ocv, V0_ocv, SOC_estimated(k), 'linear', 'extrap');
end

% Hitung error voltage
voltage_error = voltage_measured - voltage_predicted;

fprintf('  -> SOC awal: %.2f%%\n', SOC_initial);
fprintf('  -> SOC akhir: %.2f%%\n', SOC_estimated(end));
fprintf('  -> Total Ah: %.4f Ah\n', abs(total_Ah));
fprintf('  -> Voltage RMSE: %.4f V\n', sqrt(mean(voltage_error.^2)));

%% ==========================================================================
%  OUTPUT - GRAFIK (Figure Terpisah)
%  ==========================================================================

fprintf('\n--- Membuat Grafik ---\n');

% --- Figure 1: SOC vs Time ---
figure('Name', 'Coulomb Counting - SOC vs Time', 'NumberTitle', 'off');
plot(time_data, SOC_estimated, 'b-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('State of Charge (%)');
title('Coulomb Counting: SOC Estimation vs Time');
grid on;
ylim([0 105]);
legend('SOC Estimated (CC)', 'Location', 'best');

% --- Figure 2: Voltage vs Time ---
figure('Name', 'Coulomb Counting - Voltage vs Time', 'NumberTitle', 'off');
plot(time_data, voltage_measured, 'b-', 'LineWidth', 1.0, 'DisplayName', 'Voltage Measured');
hold on;
plot(time_data, voltage_predicted, 'r--', 'LineWidth', 1.0, 'DisplayName', 'Voltage Predicted (OCV)');
hold off;
xlabel('Time (s)');
ylabel('Voltage (V)');
title('Coulomb Counting: Voltage Comparison');
grid on;
legend('Location', 'best');

% --- Figure 3: Current vs Time ---
figure('Name', 'Coulomb Counting - Current vs Time', 'NumberTitle', 'off');
plot(time_data, current_data, 'g-', 'LineWidth', 1.0);
xlabel('Time (s)');
ylabel('Current (A)');
title('Coulomb Counting: Current Profile');
grid on;
legend('Current', 'Location', 'best');

% --- Figure 4: Voltage Error vs Time ---
figure('Name', 'Coulomb Counting - Voltage Error vs Time', 'NumberTitle', 'off');
plot(time_data, voltage_error, 'm-', 'LineWidth', 1.0);
xlabel('Time (s)');
ylabel('Voltage Error (V)');
title('Coulomb Counting: Voltage Error (Measured - Predicted)');
grid on;
legend('Voltage Error', 'Location', 'best');

% Tambahkan garis nol untuk referensi
hold on;
plot([time_data(1), time_data(end)], [0, 0], 'k--', 'LineWidth', 0.5);
hold off;

%% ==========================================================================
%  SAVE RESULTS
%  ==========================================================================

fprintf('\n--- Menyimpan Hasil ---\n');

% Struktur hasil
results.algorithm = 'Coulomb Counting';
results.time_data = time_data;
results.SOC_estimated = SOC_estimated;
results.voltage_measured = voltage_measured;
results.voltage_predicted = voltage_predicted;
results.voltage_error = voltage_error;
results.current_data = current_data;
results.temperature_data = temperature_data;

% Parameter yang digunakan
results.parameters.Q_nominal = Q_nominal;
results.parameters.eta = eta;
results.parameters.SOC_initial = SOC_initial;
results.parameters.time_start = t_start;
results.parameters.time_end = t_end;

% Statistik
results.statistics.SOC_final = SOC_estimated(end);
results.statistics.total_Ah = total_Ah;
results.statistics.voltage_RMSE = sqrt(mean(voltage_error.^2));
results.statistics.voltage_MAE = mean(abs(voltage_error));

% Save ke file .mat
output_file = 'coulomb_counting_results.mat';
save(output_file, 'results');
fprintf('  -> Hasil disimpan ke: %s\n', output_file);

%% ==========================================================================
%  RINGKASAN
%  ==========================================================================

fprintf('\n========================================\n');
fprintf('RINGKASAN HASIL COULOMB COUNTING\n');
fprintf('========================================\n');
fprintf('Time Range:      %.2f s - %.2f s\n', t_start, t_end);
fprintf('Duration:        %.2f hours\n', (t_end - t_start)/3600);
fprintf('Data Points:     %d samples\n', length(time_data));
fprintf('----------------------------------------\n');
fprintf('SOC Awal:        %.2f%%\n', SOC_initial);
fprintf('SOC Akhir:       %.2f%%\n', SOC_estimated(end));
fprintf('Perubahan SOC:   %.2f%%\n', SOC_initial - SOC_estimated(end));
fprintf('Total Ah:        %.4f Ah\n', abs(total_Ah));
fprintf('Voltage RMSE:    %.4f V\n', results.statistics.voltage_RMSE);
fprintf('Voltage MAE:     %.4f V\n', results.statistics.voltage_MAE);
fprintf('========================================\n');
fprintf('\nCATATAN: Coulomb Counting TIDAK menggunakan\n');
fprintf('koreksi berbasis OCV. Error akan terakumulasi\n');
fprintf('seiring waktu.\n');
fprintf('========================================\n');
