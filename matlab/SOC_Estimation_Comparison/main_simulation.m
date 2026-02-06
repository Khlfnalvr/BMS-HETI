%% ==========================================================================
%  MAIN_SIMULATION - Perbandingan Metode Estimasi SOC Baterai Li-ion
% ==========================================================================
%  Script utama untuk membandingkan 3 metode estimasi SOC:
%    1. Pure Coulomb Counting (CCM)
%    2. Pure OCV Method
%    3. Enhanced Coulomb Counting (ECC) - Lee & Won, IEEE Access 2023
%
%  Referensi Paper:
%    "Enhanced Coulomb Counting Method for SoC and SoH Estimation
%     Based on Coulombic Efficiency"
%    Authors: Jeong Lee, Jehyuk Won
%    DOI: 10.1109/ACCESS.2023.3244801
%
%  Baterai: Li-ion NMC 21700 (konfigurasi 20S, 72V nominal)
%
%  INSTRUKSI:
%    1. Letakkan file CSV di folder yang sama dengan script ini
%    2. Sesuaikan nama file CSV di bagian KONFIGURASI di bawah
%    3. Sesuaikan parameter baterai jika diperlukan
%    4. Jalankan script ini: >> main_simulation
%
%  Komentar konsep dalam Bahasa Indonesia
%  Technical MATLAB syntax comments in English
% ==========================================================================

clear; clc; close all;
fprintf('============================================================\n');
fprintf('  SIMULASI PERBANDINGAN METODE ESTIMASI SOC\n');
fprintf('  CCM vs OCV vs ECC (Lee & Won, IEEE Access 2023)\n');
fprintf('============================================================\n\n');

%% ==========================================================================
%  KONFIGURASI - Ubah parameter di sini sesuai kebutuhan
%  ==========================================================================

% --- Path File CSV ---
% Ganti nama file di sini jika file CSV Anda berbeda
ocv_csv_file = '../OCV_vs_SOC_curve_p.csv';    % OCV-SOC lookup table
ts_csv_file  = '../Experimental_data_fresh_cell.csv';  % Time-series data

% --- Parameter Baterai ---
% Sesuaikan dengan spesifikasi sel 21700 yang dipakai
C_nom = 5.0;          % Kapasitas nominal (Ah) - default sel 21700
Rb_init = 0.025;      % Internal resistance awal (Ohm)
eta_init = 0.998;     % Coulombic efficiency awal (0-1)

% --- Parameter Simulasi ---
% CCM sengaja diberi initial error untuk menunjukkan kelemahannya
ccm_initial_error = 0.05;  % +5% error pada SOC awal CCM (fraksi)
ccm_current_bias = 0.02;   % +0.02 A sensor bias pada input CCM

% --- Konvensi Arus ---
% true  = arus positif berarti discharge (SOC turun)
% false = arus positif berarti charge (SOC naik)
current_positive_discharge = true;

% --- Deteksi Siklus ---
min_cycle_duration = 30;  % Durasi minimum transisi siklus (detik)

% --- Time Range Selection (opsional) ---
% Set ke [] untuk menggunakan seluruh data
% Contoh: time_start = 0; time_end = 36000; untuk 10 jam pertama
time_start = [];  % Waktu mulai (detik), [] = dari awal
time_end = [];    % Waktu akhir (detik), [] = sampai akhir

%% ==========================================================================
%  STEP 1: Load Data dari CSV
%  ==========================================================================
fprintf('STEP 1: Loading data dari CSV...\n');
fprintf('------------------------------------------------------------\n');

data = load_data(ocv_csv_file, ts_csv_file);

% Ekstrak data untuk kemudahan akses
time = data.timeseries.Time;
current = data.timeseries.Current;
voltage = data.timeseries.Voltage;
temperature = data.timeseries.Temperature;

% Sesuaikan konvensi arus jika perlu
if ~current_positive_discharge
    current = -current;
    fprintf('  -> Konvensi arus dibalik (positif = charge)\n');
end

% Apply time range filter jika ditentukan
if ~isempty(time_start) || ~isempty(time_end)
    if isempty(time_start), time_start = time(1); end
    if isempty(time_end), time_end = time(end); end

    idx_range = (time >= time_start) & (time <= time_end);
    time = time(idx_range);
    current = current(idx_range);
    voltage = voltage(idx_range);
    temperature = temperature(idx_range);

    fprintf('  -> Time range filter applied: %.0f s to %.0f s\n', time_start, time_end);
    fprintf('  -> Samples setelah filter: %d\n', length(time));
end

N = length(time);
fprintf('  -> Total data points: %d\n', N);
fprintf('  -> Duration: %.2f jam\n\n', (time(end) - time(1)) / 3600);

%% ==========================================================================
%  STEP 2: Tentukan SOC Awal dari OCV Lookup
%  ==========================================================================
fprintf('STEP 2: Menentukan SOC awal...\n');
fprintf('------------------------------------------------------------\n');

% Estimasi OCV awal: kompensasi drop tegangan karena arus
V_first = voltage(1);
I_first = current(1);

if I_first > 0
    % Discharge: V_terminal < OCV -> OCV = V + I * Rb
    OCV_initial = V_first + abs(I_first) * Rb_init;
elseif I_first < 0
    % Charge: V_terminal > OCV -> OCV = V - |I| * Rb
    OCV_initial = V_first - abs(I_first) * Rb_init;
else
    % Rest: V_terminal ≈ OCV
    OCV_initial = V_first;
end

% Lookup SOC dari OCV yang dikompensasi
SOC_initial_pct = ocv_soc_lookup(data.ocv_table, OCV_initial, 'ocv2soc');
SOC_0_true = SOC_initial_pct / 100;  % Konversi ke fraksi (0-1)
SOC_0_true = max(0, min(1, SOC_0_true));

fprintf('  -> Voltage pertama: %.4f V\n', V_first);
fprintf('  -> Arus pertama: %.4f A\n', I_first);
fprintf('  -> OCV estimasi (kompensasi Rb): %.4f V\n', OCV_initial);
fprintf('  -> SOC awal (true): %.2f%%\n', SOC_0_true * 100);

% SOC awal untuk CCM (dengan error +5%)
SOC_0_ccm = SOC_0_true + ccm_initial_error;
SOC_0_ccm = max(0, min(1, SOC_0_ccm));
fprintf('  -> SOC awal CCM (dengan +%.0f%% error): %.2f%%\n', ccm_initial_error*100, SOC_0_ccm*100);

% SOC awal untuk ECC (sama dengan CCM sebelum koreksi, akan dikoreksi oleh ECC)
SOC_0_ecc = SOC_0_ccm;  % ECC akan mengoreksi ini di awal siklus pertama
fprintf('  -> SOC awal ECC (sebelum koreksi): %.2f%%\n\n', SOC_0_ecc*100);

%% ==========================================================================
%  STEP 3: Hitung Ground Truth SOC
%  ==========================================================================
fprintf('STEP 3: Menghitung ground truth SOC...\n');
fprintf('------------------------------------------------------------\n');

SOC_true = calc_true_soc(time, current, SOC_0_true, C_nom);

fprintf('  -> SOC true awal: %.2f%%\n', SOC_true(1)*100);
fprintf('  -> SOC true akhir: %.2f%%\n', SOC_true(end)*100);
fprintf('  -> Delta SOC: %.2f%%\n\n', (SOC_true(end) - SOC_true(1))*100);

%% ==========================================================================
%  STEP 4: Deteksi Siklus Charge/Discharge (untuk ECC)
%  ==========================================================================
fprintf('STEP 4: Deteksi siklus charge/discharge...\n');
fprintf('------------------------------------------------------------\n');

cycles = detect_cycles(time, current, min_cycle_duration);

%% ==========================================================================
%  STEP 5: Jalankan Ketiga Metode Estimasi SOC
%  ==========================================================================
fprintf('STEP 5: Menjalankan estimasi SOC...\n');
fprintf('============================================================\n\n');

% --- 5a: Pure Coulomb Counting (CCM) ---
fprintf('--- [1/3] Pure Coulomb Counting (CCM) ---\n');
SOC_ccm = ccm_estimator(time, current, SOC_0_ccm, C_nom, ccm_current_bias);
fprintf('  -> SOC CCM awal: %.2f%%, akhir: %.2f%%\n\n', SOC_ccm(1)*100, SOC_ccm(end)*100);

% --- 5b: Pure OCV Method ---
fprintf('--- [2/3] Pure OCV Method ---\n');
SOC_ocv = ocv_estimator(voltage, data.ocv_table);
fprintf('  -> SOC OCV awal: %.2f%%, akhir: %.2f%%\n\n', SOC_ocv(1)*100, SOC_ocv(end)*100);

% --- 5c: Enhanced Coulomb Counting (ECC) ---
fprintf('--- [3/3] Enhanced Coulomb Counting (ECC - Lee & Won) ---\n');
[SOC_ecc, params_history] = ecc_estimator(time, current, voltage, ...
    data.ocv_table, cycles, SOC_0_ecc, C_nom, Rb_init, eta_init);
fprintf('  -> SOC ECC awal: %.2f%%, akhir: %.2f%%\n\n', SOC_ecc(1)*100, SOC_ecc(end)*100);

%% ==========================================================================
%  STEP 6: Hitung Metrik Perbandingan
%  ==========================================================================
fprintf('STEP 6: Menghitung metrik perbandingan...\n');
fprintf('============================================================\n\n');

metrics_ccm = calc_metrics(SOC_true, SOC_ccm, time, 'CCM');
metrics_ocv = calc_metrics(SOC_true, SOC_ocv, time, 'OCV');
metrics_ecc = calc_metrics(SOC_true, SOC_ecc, time, 'ECC');

%% ==========================================================================
%  STEP 7: Tampilkan Tabel Metrik di Command Window
%  ==========================================================================
fprintf('============================================================\n');
fprintf('  TABEL PERBANDINGAN METRIK ESTIMASI SOC\n');
fprintf('============================================================\n');
fprintf('%-25s | %12s | %12s | %12s\n', 'Metrik', 'CCM', 'OCV', 'ECC');
fprintf('%-25s-+-%12s-+-%12s-+-%12s\n', repmat('-',1,25), repmat('-',1,12), repmat('-',1,12), repmat('-',1,12));
fprintf('%-25s | %10.4f %% | %10.4f %% | %10.4f %%\n', 'MAE', metrics_ccm.MAE, metrics_ocv.MAE, metrics_ecc.MAE);
fprintf('%-25s | %10.4f %% | %10.4f %% | %10.4f %%\n', 'RMSE', metrics_ccm.RMSE, metrics_ocv.RMSE, metrics_ecc.RMSE);
fprintf('%-25s | %10.4f %% | %10.4f %% | %10.4f %%\n', 'Max Error', metrics_ccm.MaxError, metrics_ocv.MaxError, metrics_ecc.MaxError);
fprintf('%-25s | %10.4f %% | %10.4f %% | %10.4f %%\n', 'Initial Error (t=0)', metrics_ccm.InitialError, metrics_ocv.InitialError, metrics_ecc.InitialError);
fprintf('%-25s | %10.4f %% | %10.4f %% | %10.4f %%\n', 'Final Error (t_end)', metrics_ccm.FinalError, metrics_ocv.FinalError, metrics_ecc.FinalError);
fprintf('%-25s | %10.4f %% | %10.4f %% | %10.4f %%\n', 'Std Dev Error', metrics_ccm.StdDevError, metrics_ocv.StdDevError, metrics_ecc.StdDevError);
fprintf('%-25s | %10.4f   | %10.4f   | %10.4f  \n', 'Drift Rate (%%/jam)', metrics_ccm.DriftRate, metrics_ocv.DriftRate, metrics_ecc.DriftRate);
fprintf('============================================================\n\n');

% Tentukan metode terbaik berdasarkan RMSE
[~, best_idx] = min([metrics_ccm.RMSE, metrics_ocv.RMSE, metrics_ecc.RMSE]);
method_names = {'CCM', 'OCV', 'ECC'};
fprintf('>> Metode terbaik berdasarkan RMSE: %s (%.4f%%)\n\n', ...
    method_names{best_idx}, min([metrics_ccm.RMSE, metrics_ocv.RMSE, metrics_ecc.RMSE]));

%% ==========================================================================
%  STEP 8: Tampilkan Parameter ECC yang Di-Update
%  ==========================================================================
if ~isempty(params_history)
    fprintf('============================================================\n');
    fprintf('  PARAMETER ECC PER SIKLUS (Lee & Won)\n');
    fprintf('============================================================\n');
    fprintf('%-8s | %10s | %10s | %10s | %10s | %10s\n', ...
        'Siklus', 'eta', 'Cb (Ah)', 'Rb (Ohm)', 'Qc (Ah)', 'Qd (Ah)');
    fprintf('%-8s-+-%10s-+-%10s-+-%10s-+-%10s-+-%10s\n', ...
        repmat('-',1,8), repmat('-',1,10), repmat('-',1,10), ...
        repmat('-',1,10), repmat('-',1,10), repmat('-',1,10));
    for i = 1:length(params_history)
        fprintf('%-8d | %10.6f | %10.4f | %10.6f | %10.4f | %10.4f\n', ...
            params_history(i).cycle, params_history(i).eta, ...
            params_history(i).Cb, params_history(i).Rb, ...
            params_history(i).Qc, params_history(i).Qd);
    end
    fprintf('============================================================\n\n');
end

%% ==========================================================================
%  STEP 9: Generate Semua Plot
%  ==========================================================================
fprintf('STEP 9: Membuat plot...\n');
fprintf('------------------------------------------------------------\n');

plot_results(time, data, SOC_true, SOC_ccm, SOC_ocv, SOC_ecc, ...
    metrics_ccm, metrics_ocv, metrics_ecc);

fprintf('  -> 4 figure berhasil dibuat\n\n');

%% ==========================================================================
%  STEP 10: Simpan Hasil
%  ==========================================================================
fprintf('STEP 10: Menyimpan hasil...\n');
fprintf('------------------------------------------------------------\n');

results.SOC_true = SOC_true;
results.SOC_ccm = SOC_ccm;
results.SOC_ocv = SOC_ocv;
results.SOC_ecc = SOC_ecc;
results.time = time;
results.current = current;
results.voltage = voltage;
results.temperature = temperature;
results.metrics.ccm = metrics_ccm;
results.metrics.ocv = metrics_ocv;
results.metrics.ecc = metrics_ecc;
results.params_ecc = params_history;
results.config.C_nom = C_nom;
results.config.Rb_init = Rb_init;
results.config.eta_init = eta_init;
results.config.ccm_initial_error = ccm_initial_error;
results.config.ccm_current_bias = ccm_current_bias;

save('soc_comparison_results.mat', 'results');
fprintf('  -> Hasil disimpan ke: soc_comparison_results.mat\n');

%% ==========================================================================
%  RINGKASAN AKHIR
%  ==========================================================================
fprintf('\n============================================================\n');
fprintf('  RINGKASAN SIMULASI\n');
fprintf('============================================================\n');
fprintf('Data: %d sampel, %.2f jam\n', N, (time(end)-time(1))/3600);
fprintf('Baterai: C_nom=%.1f Ah, Rb=%.3f Ohm\n', C_nom, Rb_init);
fprintf('------------------------------------------------------------\n');
fprintf('CCM:  RMSE=%.4f%%, Drift=%.4f%%/jam\n', metrics_ccm.RMSE, metrics_ccm.DriftRate);
fprintf('      -> Kelemahan: initial error + bias = drift terakumulasi\n');
fprintf('OCV:  RMSE=%.4f%%, Drift=%.4f%%/jam\n', metrics_ocv.RMSE, metrics_ocv.DriftRate);
fprintf('      -> Kelemahan: inakurat saat arus tinggi (Vb != OCV)\n');
fprintf('ECC:  RMSE=%.4f%%, Drift=%.4f%%/jam\n', metrics_ecc.RMSE, metrics_ecc.DriftRate);
fprintf('      -> Kelebihan: koreksi OCV + update parameter per siklus\n');
fprintf('------------------------------------------------------------\n');
fprintf('>> Kesimpulan: %s memiliki akurasi terbaik (RMSE terendah)\n', method_names{best_idx});
fprintf('============================================================\n');
fprintf('\nSimulasi selesai.\n');
