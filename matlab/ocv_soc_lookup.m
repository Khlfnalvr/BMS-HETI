%% ==========================================================================
%  OCV-BASED SOC LOOKUP
%  ==========================================================================
%  Menghitung SOC langsung dari voltage menggunakan kurva OCV
%
%  Algoritma:
%    OCV_estimated = V_terminal + R_o * I  (kompensasi resistansi)
%    SOC = lookup(OCV_estimated)           (dari kurva OCV)
%
%  CATATAN PENTING:
%  - Metode ini BUKAN estimasi SOC yang sebenarnya!
%  - Mengabaikan V_tr (transient voltage dari dinamika RC)
%  - Hanya akurat saat arus RENDAH atau NOL (rest period)
%  - Saat arus tinggi/dinamis, hasil akan sangat NOISY dan TIDAK AKURAT
%  - Gunakan sebagai REFERENSI KASAR untuk perbandingan dengan metode lain
%
%  Input Files:
%    - OCV_vs_SOC_curve.csv (columns: SOC, V0)
%    - Experimental_data_fresh_cell.csv (columns: Time, Current, Voltage, Temperature)
%
%  Output:
%    - Grafik SOC, Voltage, Current, dan Error vs Time (figure terpisah)
%    - File ocv_soc_lookup_results.mat
%  ==========================================================================

clear; clc; close all;

%% ==========================================================================
%  PARAMETER KONFIGURASI (Ubah sesuai kebutuhan)
%  ==========================================================================

% --- Parameter untuk Kompensasi Resistansi ---
% OCV = V_terminal + R_o * I
% Set ke 0 untuk tidak kompensasi (langsung pakai V_terminal sebagai OCV)
R_o_compensation = 0.05;  % Series resistance untuk kompensasi (Ohm)

% --- Path File Data ---
ocv_file = '../OCV_vs_SOC_curve.csv';
exp_file = '../Experimental_data_fresh_cell.csv';

% --- Konvensi Arus ---
% true  = arus positif berarti discharge (default BMS)
% false = arus positif berarti charge
current_positive_discharge = true;

% --- Time Range Selection ---
% Pilih range waktu data yang ingin digunakan (dalam detik)
% Set ke [] untuk menggunakan seluruh data
time_start = [];        % Waktu mulai (detik), [] = dari awal
time_end = [];          % Waktu akhir (detik), [] = sampai akhir

%% ==========================================================================
%  LOAD DATA
%  ==========================================================================

fprintf('=== OCV-BASED SOC LOOKUP ===\n\n');

% --- Load OCV Curve ---
fprintf('Loading OCV curve dari: %s\n', ocv_file);
try
    ocv_data = readtable(ocv_file);
    SOC_ocv = ocv_data.SOC;
    V0_ocv = ocv_data.V0;
    fprintf('  -> OCV curve loaded: %d data points\n', length(SOC_ocv));
    fprintf('  -> SOC range: %.1f%% to %.1f%%\n', min(SOC_ocv), max(SOC_ocv));
    fprintf('  -> OCV range: %.3f V to %.3f V\n', min(V0_ocv), max(V0_ocv));
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

idx_range = (time_data_full >= t_start) & (time_data_full <= t_end);

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

% Sesuaikan tanda arus jika perlu
if ~current_positive_discharge
    current_data = -current_data;
end

fprintf('\nStatistik Data:\n');
fprintf('  -> Current range: %.3f A to %.3f A\n', min(current_data), max(current_data));
fprintf('  -> Voltage range: %.3f V to %.3f V\n', min(voltage_measured), max(voltage_measured));
fprintf('  -> Temperature range: %.1f C to %.1f C\n', min(temperature_data), max(temperature_data));

%% ==========================================================================
%  OCV-BASED SOC LOOKUP
%  ==========================================================================

fprintf('\n--- Menghitung SOC dari OCV Lookup ---\n');

% Fungsi lookup SOC dari OCV (inverse lookup)
lookup_soc_from_ocv = @(ocv) interp1(V0_ocv, SOC_ocv, ocv, 'linear', 'extrap');

% Fungsi lookup OCV dari SOC (untuk voltage prediction)
lookup_ocv_from_soc = @(soc) interp1(SOC_ocv, V0_ocv, soc, 'linear', 'extrap');

% Hitung SOC untuk setiap titik waktu
SOC_estimated = zeros(N, 1);
OCV_estimated = zeros(N, 1);
voltage_predicted = zeros(N, 1);

for k = 1:N
    % Kompensasi resistansi: OCV = V_terminal + R_o * I (discharge: I positif)
    OCV_estimated(k) = voltage_measured(k) + R_o_compensation * current_data(k);

    % Lookup SOC dari OCV
    SOC_estimated(k) = lookup_soc_from_ocv(OCV_estimated(k));

    % Clamp ke range valid
    SOC_estimated(k) = max(0, min(100, SOC_estimated(k)));

    % Voltage predicted (OCV dari SOC yang di-lookup)
    voltage_predicted(k) = lookup_ocv_from_soc(SOC_estimated(k));
end

% Hitung voltage error
voltage_error = voltage_measured - voltage_predicted;

fprintf('  -> R_o kompensasi: %.4f Ohm\n', R_o_compensation);
fprintf('  -> SOC awal (dari OCV): %.2f%%\n', SOC_estimated(1));
fprintf('  -> SOC akhir (dari OCV): %.2f%%\n', SOC_estimated(end));
fprintf('  -> Voltage RMSE: %.4f V\n', sqrt(mean(voltage_error.^2)));

%% ==========================================================================
%  OUTPUT - GRAFIK (Figure Terpisah)
%  ==========================================================================

fprintf('\n--- Membuat Grafik ---\n');

% --- Figure 1: SOC vs Time ---
figure('Name', 'OCV Lookup - SOC vs Time', 'NumberTitle', 'off');
plot(time_data, SOC_estimated, 'b-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('State of Charge (%)');
title('OCV Lookup: SOC Estimation vs Time');
grid on;
ylim([0 105]);
legend('SOC (OCV Lookup)', 'Location', 'best');

% --- Figure 2: Voltage vs Time ---
figure('Name', 'OCV Lookup - Voltage vs Time', 'NumberTitle', 'off');
plot(time_data, voltage_measured, 'b-', 'LineWidth', 1.0, 'DisplayName', 'Voltage Measured');
hold on;
plot(time_data, OCV_estimated, 'r--', 'LineWidth', 1.0, 'DisplayName', 'OCV Estimated');
hold off;
xlabel('Time (s)');
ylabel('Voltage (V)');
title('OCV Lookup: Voltage Comparison');
grid on;
legend('Location', 'best');

% --- Figure 3: Current vs Time ---
figure('Name', 'OCV Lookup - Current vs Time', 'NumberTitle', 'off');
plot(time_data, current_data, 'g-', 'LineWidth', 1.0);
xlabel('Time (s)');
ylabel('Current (A)');
title('OCV Lookup: Current Profile');
grid on;
legend('Current', 'Location', 'best');

% --- Figure 4: Voltage Error vs Time ---
figure('Name', 'OCV Lookup - Voltage Error vs Time', 'NumberTitle', 'off');
plot(time_data, voltage_error, 'm-', 'LineWidth', 1.0);
xlabel('Time (s)');
ylabel('Voltage Error (V)');
title('OCV Lookup: Voltage Error (Measured - Predicted)');
grid on;
legend('Voltage Error', 'Location', 'best');
hold on;
plot([time_data(1), time_data(end)], [0, 0], 'k--', 'LineWidth', 0.5);
hold off;

%% ==========================================================================
%  SAVE RESULTS
%  ==========================================================================

fprintf('\n--- Menyimpan Hasil ---\n');

% Struktur hasil
results.algorithm = 'OCV Lookup';
results.time_data = time_data;
results.SOC_estimated = SOC_estimated;
results.OCV_estimated = OCV_estimated;
results.voltage_measured = voltage_measured;
results.voltage_predicted = voltage_predicted;
results.voltage_error = voltage_error;
results.current_data = current_data;
results.temperature_data = temperature_data;

% Parameter yang digunakan
results.parameters.R_o_compensation = R_o_compensation;
results.parameters.time_start = t_start;
results.parameters.time_end = t_end;

% Statistik
results.statistics.SOC_initial = SOC_estimated(1);
results.statistics.SOC_final = SOC_estimated(end);
results.statistics.SOC_change = SOC_estimated(1) - SOC_estimated(end);
results.statistics.voltage_RMSE = sqrt(mean(voltage_error.^2));
results.statistics.voltage_MAE = mean(abs(voltage_error));

% Save ke file .mat
output_file = 'ocv_soc_lookup_results.mat';
save(output_file, 'results');
fprintf('  -> Hasil disimpan ke: %s\n', output_file);

%% ==========================================================================
%  RINGKASAN
%  ==========================================================================

fprintf('\n========================================\n');
fprintf('RINGKASAN HASIL OCV LOOKUP\n');
fprintf('========================================\n');
fprintf('Time Range:      %.2f s - %.2f s\n', t_start, t_end);
fprintf('Duration:        %.2f hours\n', (t_end - t_start)/3600);
fprintf('Data Points:     %d samples\n', length(time_data));
fprintf('----------------------------------------\n');
fprintf('R_o Kompensasi:  %.4f Ohm\n', R_o_compensation);
fprintf('SOC Awal:        %.2f%%\n', SOC_estimated(1));
fprintf('SOC Akhir:       %.2f%%\n', SOC_estimated(end));
fprintf('Perubahan SOC:   %.2f%%\n', results.statistics.SOC_change);
fprintf('----------------------------------------\n');
fprintf('Voltage RMSE:    %.4f V\n', results.statistics.voltage_RMSE);
fprintf('Voltage MAE:     %.4f V\n', results.statistics.voltage_MAE);
fprintf('========================================\n');
fprintf('\n!!! PERINGATAN PENTING !!!\n');
fprintf('========================================\n');
fprintf('Metode OCV Lookup ini:\n');
fprintf('1. BUKAN estimasi SOC yang sebenarnya!\n');
fprintf('2. Mengabaikan V_tr (transient voltage)\n');
fprintf('3. HANYA AKURAT saat arus RENDAH/NOL\n');
fprintf('4. Saat arus tinggi, hasil SANGAT NOISY\n');
fprintf('5. Gunakan sebagai REFERENSI KASAR saja\n');
fprintf('========================================\n');
