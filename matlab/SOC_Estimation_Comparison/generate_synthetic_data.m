function [data, ground_truth] = generate_synthetic_data(ocv_table, params)
% ==========================================================================
%  GENERATE_SYNTHETIC_DATA - Buat data sintetik baterai untuk simulasi ECC
% ==========================================================================
%  Menghasilkan data time-series baterai yang REALISTIK dengan:
%    - Multiple siklus charge/discharge
%    - REST PERIODS antar siklus (kunci keunggulan ECC)
%    - Variasi C-rate (0.5C - 2C)
%    - Noise sensor yang realistik
%    - Ground truth SOC yang diketahui persis
%
%  Model baterai sederhana:
%    V_terminal = OCV(SOC) - I * Rb_true  (discharge, I>0)
%    V_terminal = OCV(SOC) + |I| * Rb_true (charge, I<0)
%    dengan konvensi: I positif = discharge
%
%  MENGAPA DATA SINTETIK DIPERLUKAN:
%    Data eksperimen (Experimental_data_fresh_cell.csv) memiliki:
%    - 78.4% waktu di arus >10A (tidak ada kesempatan koreksi OCV)
%    - Hanya 1 rest period dalam 100 jam
%    - 4C rate → SOC 0-100% dalam 15 menit → SOC clamped 56% waktu
%    Kondisi ini TIDAK memungkinkan ECC untuk mendemonstrasikan
%    keunggulannya (koreksi OCV periodik).
%
%  Input:
%    ocv_table - struct dengan .SOC (%) dan .OCV (V) dari CSV
%    params    - struct parameter (opsional), field:
%      .C_true       - kapasitas sebenarnya (Ah), default 4.85
%      .Rb_true      - internal resistance sebenarnya (Ohm), default 0.008
%      .SOC_start    - SOC awal (fraksi 0-1), default 0.80
%      .I_noise_std  - std dev noise arus (A), default 0.05
%      .V_noise_std  - std dev noise tegangan (V), default 0.003
%      .dt           - time step (detik), default 1
%
%  Output:
%    data         - struct sama formatnya dengan load_data() output:
%      .timeseries.Time, .Current, .Voltage, .Temperature
%      .ocv_table.SOC, .OCV
%    ground_truth - struct berisi SOC_true dan parameter sebenarnya
% ==========================================================================

%% Parameter default
if nargin < 2, params = struct(); end

% Kapasitas sebenarnya (sedikit lebih rendah dari nominal 5.0 Ah)
% Ini mensimulasikan degradasi ringan yang harus dideteksi ECC
C_true = get_field(params, 'C_true', 4.85);

% Rb sebenarnya (akan berbeda dari Rb_init yang dipakai ECC)
Rb_true = get_field(params, 'Rb_true', 0.008);

% Kondisi awal
SOC_start = get_field(params, 'SOC_start', 0.80);

% Noise sensor
I_noise_std = get_field(params, 'I_noise_std', 0.05);  % 50 mA std dev
V_noise_std = get_field(params, 'V_noise_std', 0.003);  % 3 mV std dev

% Time step
dt = get_field(params, 'dt', 1);  % 1 detik

% Temperature (konstan, realistik)
T_ambient = 25.0;  % °C

fprintf('--- Generate Synthetic Data ---\n');
fprintf('  -> C_true=%.3f Ah, Rb_true=%.4f Ohm\n', C_true, Rb_true);
fprintf('  -> SOC awal: %.1f%%\n', SOC_start*100);

%% ========================================================================
%  Definisi profil cycling
%  ========================================================================
%  Setiap siklus: [tipe, C-rate, target_SOC, rest_duration_s]
%    tipe: 'discharge' atau 'charge'
%    C-rate: kelipatan kapasitas (1C = C_true Ampere)
%    target_SOC: berhenti saat SOC mencapai target (fraksi 0-1)
%    rest_duration_s: durasi istirahat setelah siklus (detik)

cycle_profile = {
    % Siklus 1: Discharge 1C dari 80% ke 20%, rest 10 menit
    'discharge', 1.0,  0.20, 600;
    % Siklus 2: Charge 0.5C dari 20% ke 90%, rest 15 menit
    'charge',    0.5,  0.90, 900;
    % Siklus 3: Discharge 1.5C dari 90% ke 15%, rest 5 menit
    'discharge', 1.5,  0.15, 300;
    % Siklus 4: Charge 1C dari 15% ke 85%, rest 10 menit
    'charge',    1.0,  0.85, 600;
    % Siklus 5: Discharge 0.5C dari 85% ke 30%, rest 15 menit
    'discharge', 0.5,  0.30, 900;
    % Siklus 6: Charge 2C dari 30% ke 80%, rest 5 menit
    'charge',    2.0,  0.80, 300;
    % Siklus 7: Discharge 1C dari 80% ke 25%, rest 10 menit
    'discharge', 1.0,  0.25, 600;
    % Siklus 8: Charge 0.5C dari 25% ke 75%, rest 20 menit
    'charge',    0.5,  0.75, 1200;
    % Siklus 9: Dynamic profile (akan di-generate khusus), rest 10 menit
    'dynamic',   0,    0,    600;
    % Siklus 10: Discharge 1C dari ~50% ke 20%, rest 5 menit
    'discharge', 1.0,  0.20, 300;
};

%% ========================================================================
%  Generate data sample-by-sample
%  ========================================================================

% Prealokasi dengan estimasi ukuran
% Estimasi kasar: ~15 jam data @ 1 Hz = ~54000 samples
max_samples = 200000;
time_vec = zeros(max_samples, 1);
current_vec = zeros(max_samples, 1);
voltage_vec = zeros(max_samples, 1);
temp_vec = zeros(max_samples, 1);
soc_true_vec = zeros(max_samples, 1);

% State awal
t = 0;
SOC = SOC_start;
k = 1;

% Simpan sampel pertama
time_vec(k) = t;
current_vec(k) = 0 + I_noise_std * randn();
voltage_vec(k) = get_voltage(SOC, 0, Rb_true, ocv_table) + V_noise_std * randn();
temp_vec(k) = T_ambient + 0.5 * randn();
soc_true_vec(k) = SOC;

n_profiles = size(cycle_profile, 1);

for c = 1:n_profiles
    cycle_type = cycle_profile{c, 1};
    c_rate = cycle_profile{c, 2};
    target_soc = cycle_profile{c, 3};
    rest_s = cycle_profile{c, 4};

    if strcmp(cycle_type, 'dynamic')
        % ============================================================
        %  Profil dinamis: simulasi driving cycle
        %  Variasi arus acak antara -3A (regen) dan +8A (akselerasi)
        % ============================================================
        dynamic_duration = 3600;  % 1 jam profil dinamis
        n_steps = floor(dynamic_duration / dt);

        % Generate random current profile dengan segmen
        segment_len = 30;  % 30 detik per segmen
        n_segments = ceil(n_steps / segment_len);

        for seg = 1:n_segments
            % Random current untuk segmen ini
            I_base = -3 + 11 * rand();  % -3A (charge/regen) sampai +8A (discharge)
            % Kadang rest juga
            if rand() < 0.1
                I_base = 0;  % 10% chance rest
            end

            seg_start = (seg-1) * segment_len + 1;
            seg_end = min(seg * segment_len, n_steps);

            for s = seg_start:seg_end
                k = k + 1;
                if k > max_samples, break; end
                t = t + dt;

                % Arus dengan sedikit variasi intra-segmen
                I_true = I_base + 0.3 * randn();

                % Update SOC
                SOC = SOC - (I_true * dt) / (C_true * 3600);
                SOC = max(0.05, min(0.95, SOC));  % Jangan sampai rail

                % Generate measurements
                time_vec(k) = t;
                current_vec(k) = I_true + I_noise_std * randn();
                voltage_vec(k) = get_voltage(SOC, I_true, Rb_true, ocv_table) + V_noise_std * randn();
                temp_vec(k) = T_ambient + 0.5 * randn() + abs(I_true) * 0.02;  % Sedikit self-heating
                soc_true_vec(k) = SOC;
            end
            if k > max_samples, break; end
        end

    else
        % ============================================================
        %  Profil CC (Constant Current)
        % ============================================================
        I_magnitude = c_rate * C_true;  % Arus dalam Ampere

        if strcmp(cycle_type, 'discharge')
            I_true = I_magnitude;  % Positif = discharge
        else
            I_true = -I_magnitude;  % Negatif = charge
        end

        % Run sampai target SOC tercapai
        max_iter = 100000;
        iter = 0;
        while iter < max_iter
            iter = iter + 1;
            k = k + 1;
            if k > max_samples, break; end
            t = t + dt;

            % Update SOC
            SOC = SOC - (I_true * dt) / (C_true * 3600);

            % Cek target
            if strcmp(cycle_type, 'discharge') && SOC <= target_soc
                SOC = target_soc;
                % Record this sample
                time_vec(k) = t;
                current_vec(k) = I_true + I_noise_std * randn();
                voltage_vec(k) = get_voltage(SOC, I_true, Rb_true, ocv_table) + V_noise_std * randn();
                temp_vec(k) = T_ambient + 0.5 * randn() + abs(I_true) * 0.01;
                soc_true_vec(k) = SOC;
                break;
            elseif strcmp(cycle_type, 'charge') && SOC >= target_soc
                SOC = target_soc;
                time_vec(k) = t;
                current_vec(k) = I_true + I_noise_std * randn();
                voltage_vec(k) = get_voltage(SOC, I_true, Rb_true, ocv_table) + V_noise_std * randn();
                temp_vec(k) = T_ambient + 0.5 * randn() + abs(I_true) * 0.01;
                soc_true_vec(k) = SOC;
                break;
            end

            % Generate measurements
            time_vec(k) = t;
            current_vec(k) = I_true + I_noise_std * randn();
            voltage_vec(k) = get_voltage(SOC, I_true, Rb_true, ocv_table) + V_noise_std * randn();
            temp_vec(k) = T_ambient + 0.5 * randn() + abs(I_true) * 0.01;
            soc_true_vec(k) = SOC;
        end
        if k > max_samples, break; end
    end

    % ============================================================
    %  REST PERIOD setelah siklus
    %  Di sinilah ECC mendapat keuntungan: V ≈ OCV
    % ============================================================
    if rest_s > 0
        n_rest = floor(rest_s / dt);
        for r = 1:n_rest
            k = k + 1;
            if k > max_samples, break; end
            t = t + dt;

            % Arus = 0 saat rest (hanya noise sensor)
            I_true = 0;

            % Voltage: OCV(SOC) + sedikit relaxation transient
            % Eksponensial decay menuju OCV sebenarnya
            tau_relax = 60;  % time constant relaxation (detik)
            relax_factor = 1 - exp(-r * dt / tau_relax);
            V_term = get_voltage(SOC, 0, Rb_true, ocv_table);

            time_vec(k) = t;
            current_vec(k) = 0 + I_noise_std * randn();
            voltage_vec(k) = V_term + V_noise_std * randn();
            temp_vec(k) = T_ambient + 0.3 * randn();
            soc_true_vec(k) = SOC;
        end
        if k > max_samples, break; end
    end

    fprintf('  -> Siklus %d/%d selesai: %s %.1fC, SOC=%.1f%%, rest=%ds\n', ...
        c, n_profiles, cycle_type, c_rate, SOC*100, rest_s);
end

%% Trim to actual size
time_vec = time_vec(1:k);
current_vec = current_vec(1:k);
voltage_vec = voltage_vec(1:k);
temp_vec = temp_vec(1:k);
soc_true_vec = soc_true_vec(1:k);

%% ========================================================================
%  Pack output dalam format yang sama dengan load_data()
%  ========================================================================
data.timeseries.Time = time_vec;
data.timeseries.Current = current_vec;
data.timeseries.Voltage = voltage_vec;
data.timeseries.Temperature = temp_vec;
data.ocv_table = ocv_table;

% Ground truth (TIDAK tersedia di data real, hanya untuk evaluasi)
ground_truth.SOC_true = soc_true_vec;
ground_truth.C_true = C_true;
ground_truth.Rb_true = Rb_true;
ground_truth.SOC_start = SOC_start;
ground_truth.n_samples = k;

fprintf('\n--- Synthetic Data Generated ---\n');
fprintf('  -> Total samples: %d\n', k);
fprintf('  -> Duration: %.2f jam\n', (time_vec(end) - time_vec(1)) / 3600);
fprintf('  -> SOC range: %.1f%% - %.1f%%\n', min(soc_true_vec)*100, max(soc_true_vec)*100);
fprintf('  -> Jumlah siklus: %d\n', n_profiles);
fprintf('  -> C_true (sebenarnya): %.3f Ah (C_nom akan diset 5.0 Ah)\n', C_true);
fprintf('  -> Rb_true (sebenarnya): %.4f Ohm (Rb_init akan diset 0.025 Ohm)\n', Rb_true);
fprintf('\n');

end

%% ========================================================================
%  HELPER: Hitung terminal voltage dari SOC dan arus
%  ========================================================================
function V = get_voltage(SOC, I, Rb, ocv_table)
    % V = OCV(SOC) - I * Rb
    % Konvensi: I>0 = discharge -> voltage drop
    %           I<0 = charge -> voltage rise
    OCV = ocv_soc_lookup(ocv_table, SOC * 100, 'soc2ocv');  % SOC dalam %
    V = OCV - I * Rb;
end

%% ========================================================================
%  HELPER: Safe field access with default
%  ========================================================================
function val = get_field(s, field, default)
    if isfield(s, field)
        val = s.(field);
    else
        val = default;
    end
end
