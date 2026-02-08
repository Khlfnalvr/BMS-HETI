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
%    1. Set use_synthetic_data = true/false sesuai kebutuhan
%    2. Jika pakai CSV: letakkan file di folder parent, sesuaikan nama
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

% --- Pilih Sumber Data ---
% true  = gunakan data SINTETIK (rekomendasi untuk demonstrasi ECC)
%         Data sintetik memiliki rest periods antar siklus,
%         variasi C-rate, dan ground truth yang diketahui persis.
%
% false = gunakan data CSV EKSPERIMEN
%         PERINGATAN: Data eksperimen ini memiliki 78% waktu di arus >10A,
%         hanya 1 rest period dalam 100 jam, dan SOC clamped 56% waktu.
%         Kondisi ini TIDAK memungkinkan ECC menunjukkan keunggulannya.
use_synthetic_data = true;

% --- Path File CSV (hanya dipakai jika use_synthetic_data = false) ---
ocv_csv_file = '../OCV_vs_SOC_curve_p.csv';    % OCV-SOC lookup table
ts_csv_file  = '../Experimental_data_fresh_cell.csv';  % Time-series data

% --- Parameter Baterai ---
% Ini adalah parameter NOMINAL yang diketahui estimator.
% Untuk data sintetik, parameter SEBENARNYA sedikit berbeda
% (C_true=4.85 Ah, Rb_true=0.008 Ohm) untuk menguji adaptasi ECC.
C_nom = 5.0;          % Kapasitas nominal (Ah) - default sel 21700
Rb_init = 0.025;      % Internal resistance awal (Ohm) - sengaja lebih besar
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

% --- Time Range Selection (opsional, hanya untuk data CSV) ---
% Set ke [] untuk menggunakan seluruh data
time_start = [];  % Waktu mulai (detik), [] = dari awal
time_end = [];    % Waktu akhir (detik), [] = sampai akhir

%% ==========================================================================
%  STEP 1: Load Data
%  ==========================================================================
fprintf('STEP 1: Loading data...\n');
fprintf('------------------------------------------------------------\n');

if use_synthetic_data
    % ==================================================================
    %  MODE SINTETIK: Generate data dengan rest periods dan variasi C-rate
    % ==================================================================
    fprintf('  -> Mode: DATA SINTETIK (rekomendasi untuk demonstrasi ECC)\n\n');

    % Load OCV table dulu dari CSV (ini tetap dipakai)
    ocv_data_raw = load_data(ocv_csv_file, ts_csv_file);
    ocv_table = ocv_data_raw.ocv_table;

    % Parameter data sintetik
    synth_params.C_true = 4.85;      % Kapasitas sebenarnya (sedikit < C_nom)
    synth_params.Rb_true = 0.008;    % Rb sebenarnya (lebih kecil dari Rb_init)
    synth_params.SOC_start = 0.80;   % Mulai dari 80%
    synth_params.I_noise_std = 0.05; % Noise sensor arus (50mA std)
    synth_params.V_noise_std = 0.003;% Noise sensor tegangan (3mV std)

    % Generate data
    [data, ground_truth_synth] = generate_synthetic_data(ocv_table, synth_params);

    % Ekstrak data
    time = data.timeseries.Time;
    current = data.timeseries.Current;
    voltage = data.timeseries.Voltage;
    temperature = data.timeseries.Temperature;

    fprintf('  -> C_nom (estimator tahu): %.3f Ah\n', C_nom);
    fprintf('  -> C_true (sebenarnya):    %.3f Ah (perbedaan %.1f%%)\n', ...
        synth_params.C_true, abs(C_nom - synth_params.C_true)/C_nom*100);
    fprintf('  -> Rb_init (estimator):    %.4f Ohm\n', Rb_init);
    fprintf('  -> Rb_true (sebenarnya):   %.4f Ohm\n\n', synth_params.Rb_true);

else
    % ==================================================================
    %  MODE CSV: Load data dari file eksperimen
    % ==================================================================
    fprintf('  -> Mode: DATA CSV EKSPERIMEN\n');
    fprintf('  -> PERINGATAN: Data ini kurang ideal untuk demonstrasi ECC\n');
    fprintf('     (78%% waktu di arus tinggi, hanya 1 rest period)\n\n');

    data = load_data(ocv_csv_file, ts_csv_file);

    % Ekstrak data
    time = data.timeseries.Time;
    current = data.timeseries.Current;
    voltage = data.timeseries.Voltage;
    temperature = data.timeseries.Temperature;

    % Sesuaikan konvensi arus jika perlu
    if ~current_positive_discharge
        current = -current;
        fprintf('  -> Konvensi arus dibalik (positif = charge)\n');
    end

    % Apply time range filter
    if ~isempty(time_start) || ~isempty(time_end)
        if isempty(time_start), time_start = time(1); end
        if isempty(time_end), time_end = time(end); end

        idx_range = (time >= time_start) & (time <= time_end);
        time = time(idx_range);
        current = current(idx_range);
        voltage = voltage(idx_range);
        temperature = temperature(idx_range);

        fprintf('  -> Time range filter: %.0f s to %.0f s\n', time_start, time_end);
    end
end

N = length(time);
fprintf('  -> Total data points: %d\n', N);
fprintf('  -> Duration: %.2f jam\n\n', (time(end) - time(1)) / 3600);

%% ==========================================================================
%  STEP 2: Tentukan SOC Awal dan Ground Truth
%  ==========================================================================
fprintf('STEP 2: Menentukan SOC awal dan ground truth...\n');
fprintf('------------------------------------------------------------\n');

if use_synthetic_data
    % ============================================================
    %  Sintetik: ground truth diketahui persis
    % ============================================================
    SOC_true = ground_truth_synth.SOC_true;
    SOC_0_true = SOC_true(1);

    fprintf('  -> Ground truth: EXACT (dari model sintetik)\n');
    fprintf('  -> SOC awal (true): %.2f%%\n', SOC_0_true * 100);
else
    % ============================================================
    %  CSV: estimasi SOC awal dari OCV lookup, ground truth via CC ideal
    % ============================================================
    V_first = voltage(1);
    I_first = current(1);

    if I_first > 0
        OCV_initial = V_first + abs(I_first) * Rb_init;
    elseif I_first < 0
        OCV_initial = V_first - abs(I_first) * Rb_init;
    else
        OCV_initial = V_first;
    end

    SOC_initial_pct = ocv_soc_lookup(data.ocv_table, OCV_initial, 'ocv2soc');
    SOC_0_true = SOC_initial_pct / 100;
    SOC_0_true = max(0, min(1, SOC_0_true));

    fprintf('  -> Ground truth: Coulomb Counting ideal (dari data CSV)\n');
    fprintf('  -> Voltage pertama: %.4f V, Arus pertama: %.4f A\n', V_first, I_first);
    fprintf('  -> SOC awal (true): %.2f%%\n', SOC_0_true * 100);

    SOC_true = calc_true_soc(time, current, SOC_0_true, C_nom);
end

% SOC awal untuk CCM (dengan error +5%)
SOC_0_ccm = SOC_0_true + ccm_initial_error;
SOC_0_ccm = max(0, min(1, SOC_0_ccm));
fprintf('  -> SOC awal CCM (+%.0f%% error): %.2f%%\n', ccm_initial_error*100, SOC_0_ccm*100);

% SOC awal untuk ECC (sama error, akan dikoreksi oleh ECC)
SOC_0_ecc = SOC_0_ccm;
fprintf('  -> SOC awal ECC (sebelum koreksi): %.2f%%\n', SOC_0_ecc*100);
fprintf('  -> SOC true akhir: %.2f%%\n', SOC_true(end)*100);
fprintf('  -> Delta SOC: %.2f%%\n\n', (SOC_true(end) - SOC_true(1))*100);

%% ==========================================================================
%  STEP 3: Deteksi Siklus Charge/Discharge (untuk ECC)
%  ==========================================================================
fprintf('STEP 3: Deteksi siklus charge/discharge...\n');
fprintf('------------------------------------------------------------\n');

cycles = detect_cycles(time, current, min_cycle_duration);

%% ==========================================================================
%  STEP 4: Jalankan Ketiga Metode Estimasi SOC
%  ==========================================================================
fprintf('STEP 4: Menjalankan estimasi SOC...\n');
fprintf('============================================================\n\n');

% --- 4a: Pure Coulomb Counting (CCM) ---
fprintf('--- [1/3] Pure Coulomb Counting (CCM) ---\n');
SOC_ccm = ccm_estimator(time, current, SOC_0_ccm, C_nom, ccm_current_bias);
fprintf('  -> SOC CCM awal: %.2f%%, akhir: %.2f%%\n\n', SOC_ccm(1)*100, SOC_ccm(end)*100);

% --- 4b: Pure OCV Method ---
fprintf('--- [2/3] Pure OCV Method ---\n');
SOC_ocv = ocv_estimator(voltage, data.ocv_table);
fprintf('  -> SOC OCV awal: %.2f%%, akhir: %.2f%%\n\n', SOC_ocv(1)*100, SOC_ocv(end)*100);

% --- 4c: Enhanced Coulomb Counting (ECC) ---
fprintf('--- [3/3] Enhanced Coulomb Counting (ECC - Lee & Won) ---\n');
[SOC_ecc, params_history] = ecc_estimator(time, current, voltage, ...
    data.ocv_table, cycles, SOC_0_ecc, C_nom, Rb_init, eta_init);
fprintf('  -> SOC ECC awal: %.2f%%, akhir: %.2f%%\n\n', SOC_ecc(1)*100, SOC_ecc(end)*100);

%% ==========================================================================
%  STEP 5: Hitung Metrik Perbandingan
%  ==========================================================================
fprintf('STEP 5: Menghitung metrik perbandingan...\n');
fprintf('============================================================\n\n');

metrics_ccm = calc_metrics(SOC_true, SOC_ccm, time, 'CCM');
metrics_ocv = calc_metrics(SOC_true, SOC_ocv, time, 'OCV');
metrics_ecc = calc_metrics(SOC_true, SOC_ecc, time, 'ECC');

%% ==========================================================================
%  STEP 6: Tampilkan Tabel Metrik di Command Window
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
%  STEP 7: Tampilkan Parameter ECC yang Di-Update
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
%  STEP 8: Generate Semua Plot
%  ==========================================================================
fprintf('STEP 8: Membuat plot...\n');
fprintf('------------------------------------------------------------\n');

plot_results(time, data, SOC_true, SOC_ccm, SOC_ocv, SOC_ecc, ...
    metrics_ccm, metrics_ocv, metrics_ecc);

fprintf('  -> 4 figure berhasil dibuat\n\n');

%% ==========================================================================
%  STEP 9: Simpan Hasil
%  ==========================================================================
fprintf('STEP 9: Menyimpan hasil...\n');
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
results.config.use_synthetic_data = use_synthetic_data;

if use_synthetic_data
    results.ground_truth_params = ground_truth_synth;
end

save('soc_comparison_results.mat', 'results');
fprintf('  -> Hasil disimpan ke: soc_comparison_results.mat\n');

%% ==========================================================================
%  RINGKASAN AKHIR
%  ==========================================================================
fprintf('\n============================================================\n');
fprintf('  RINGKASAN SIMULASI\n');
fprintf('============================================================\n');
if use_synthetic_data
    fprintf('Data: SINTETIK, %d sampel, %.2f jam\n', N, (time(end)-time(1))/3600);
    fprintf('Baterai: C_nom=%.1f Ah (true=%.2f), Rb_init=%.3f (true=%.4f)\n', ...
        C_nom, synth_params.C_true, Rb_init, synth_params.Rb_true);
else
    fprintf('Data: CSV EKSPERIMEN, %d sampel, %.2f jam\n', N, (time(end)-time(1))/3600);
    fprintf('Baterai: C_nom=%.1f Ah, Rb=%.3f Ohm\n', C_nom, Rb_init);
end
fprintf('------------------------------------------------------------\n');
fprintf('CCM:  RMSE=%.4f%%, MAE=%.4f%%, Drift=%.4f%%/jam\n', metrics_ccm.RMSE, metrics_ccm.MAE, metrics_ccm.DriftRate);
fprintf('      -> Kelemahan: initial error + bias = drift terakumulasi\n');
fprintf('OCV:  RMSE=%.4f%%, MAE=%.4f%%, Drift=%.4f%%/jam\n', metrics_ocv.RMSE, metrics_ocv.MAE, metrics_ocv.DriftRate);
fprintf('      -> Kelemahan: inakurat saat arus tinggi (Vb != OCV)\n');
fprintf('ECC:  RMSE=%.4f%%, MAE=%.4f%%, Drift=%.4f%%/jam\n', metrics_ecc.RMSE, metrics_ecc.MAE, metrics_ecc.DriftRate);
fprintf('      -> Kelebihan: koreksi OCV saat rest + update parameter per siklus\n');
fprintf('------------------------------------------------------------\n');
fprintf('>> Kesimpulan: %s memiliki akurasi terbaik (RMSE terendah)\n', method_names{best_idx});
fprintf('============================================================\n');
fprintf('\nSimulasi selesai.\n');
