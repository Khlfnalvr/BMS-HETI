function [SOC_ecc, params_history] = ecc_estimator(time, current, voltage, ...
    ocv_table, cycles, SOC_0_ecc, C_nom, Rb_init, eta_init)
% ==========================================================================
%  ECC_ESTIMATOR - Enhanced Coulomb Counting (Lee & Won, IEEE Access 2023)
% ==========================================================================
%  Implementasi IMPROVED dari metode Enhanced Coulomb Counting.
%
%  Referensi paper:
%    "Enhanced Coulomb Counting Method for SoC and SoH Estimation
%     Based on Coulombic Efficiency"
%    DOI: 10.1109/ACCESS.2023.3244801
%
%  PERBAIKAN dari versi awal:
%    1. Estimasi Rb dari data (voltage step saat current berubah)
%       -> Menghindari Rb terlalu besar yang menyebabkan OCV error
%    2. Koreksi OCV HANYA saat rest period atau arus sangat rendah
%       -> Menghindari koreksi yang salah saat beban tinggi
%    3. Weighted blending (bukan hard reset)
%       -> SOC = w*SOC_ocv + (1-w)*SOC_cc, w tergantung confidence
%    4. Parameter update lebih konservatif per siklus
%
%  Prinsip utama:
%    - Coulomb counting sebagai BACKBONE (seperti CCM, tapi dengan eta)
%    - OCV correction HANYA membantu, TIDAK boleh memperburuk
%    - Koreksi hanya saat confidence OCV tinggi (rest/low current)
%
%  Input:
%    time      - vektor waktu (detik), Nx1
%    current   - vektor arus (Ampere), Nx1, positif = discharge
%    voltage   - vektor terminal voltage (Volt), Nx1
%    ocv_table - struct dengan field .SOC (%) dan .OCV (V)
%    cycles    - struct array dari detect_cycles()
%    SOC_0_ecc - SOC awal (fraksi 0-1), mengandung error awal
%    C_nom     - kapasitas nominal awal (Ah)
%    Rb_init   - internal resistance awal (Ohm)
%    eta_init  - Coulombic efficiency awal (0-1)
%
%  Output:
%    SOC_ecc       - vektor SOC estimasi ECC (fraksi 0-1), Nx1
%    params_history - struct array per siklus, parameter update
% ==========================================================================

N = length(time);
SOC_ecc = zeros(N, 1);

% Parameter aktif (akan di-update)
Cb = C_nom;
Rb = Rb_init;
eta = eta_init;

n_cycles = length(cycles);

fprintf('--- ECC Estimator (Improved - Lee & Won, IEEE Access 2023) ---\n');

%% ========================================================================
%  STEP 0: Estimasi Rb dari data (data-driven)
%  ========================================================================
%  Cari step besar pada arus dan hitung Rb = |dV|/|dI|
%  Ini jauh lebih akurat daripada Rb hardcoded
Rb_estimated = estimate_rb_from_steps(time, current, voltage);

if Rb_estimated > 0
    Rb = Rb_estimated;
    fprintf('  -> Rb diestimasi dari data: %.6f Ohm\n', Rb);
else
    fprintf('  -> Rb dari data tidak tersedia, gunakan default: %.6f Ohm\n', Rb);
end

fprintf('  -> Parameter awal: Cb=%.3f Ah, Rb=%.6f Ohm, eta=%.4f\n', Cb, Rb, eta);

%% ========================================================================
%  STEP 1: Deteksi rest periods dalam data
%  ========================================================================
%  Rest period = |I| < threshold selama > durasi minimum
%  Ini satu-satunya kondisi di mana V_terminal ≈ OCV
I_rest_threshold = 0.5;     % Arus di bawah ini dianggap rest (A)
min_rest_duration = 30;     % Minimum 30 detik rest untuk koreksi OCV
I_low_threshold = 3.0;      % Batas arus rendah untuk koreksi gentle (A)

% Track durasi rest secara kumulatif
rest_duration = zeros(N, 1);  % Berapa lama sudah rest (detik)
for k = 2:N
    if abs(current(k)) < I_rest_threshold
        rest_duration(k) = rest_duration(k-1) + (time(k) - time(k-1));
    else
        rest_duration(k) = 0;  % Reset saat arus mengalir
    end
end

n_rest_points = sum(rest_duration >= min_rest_duration);
fprintf('  -> Titik data dengan rest >= %ds: %d (%.1f%%)\n', ...
    min_rest_duration, n_rest_points, 100*n_rest_points/N);

%% ========================================================================
%  STEP 2: Koreksi SOC awal
%  ========================================================================
%  Eq. (3) dari paper Lee & Won - Initial value correction
%  Hanya lakukan koreksi penuh jika arus awal rendah

V_first = voltage(1);
I_first = current(1);

if abs(I_first) < I_low_threshold
    % Arus rendah: bisa estimasi OCV dengan kompensasi Rb ringan
    % Konvensi: discharge (I>0) -> OCV = V + I*Rb, charge (I<0) -> OCV = V + I*Rb
    OCV_est = V_first + I_first * Rb;
    SOC_ocv_init = ocv_soc_lookup(ocv_table, OCV_est, 'ocv2soc') / 100;
    SOC_ocv_init = max(0, min(1, SOC_ocv_init));

    % Blend berdasarkan magnitude arus
    % Semakin kecil arus, semakin percaya OCV
    w = max(0, 1 - abs(I_first) / I_low_threshold);  % 0-1
    w = w * 0.8;  % Maks 80% koreksi (selalu sisakan sedikit bobot untuk CC)
    SOC_ecc(1) = w * SOC_ocv_init + (1 - w) * SOC_0_ecc;

    fprintf('  -> SOC awal: OCV correction applied (w=%.2f)\n', w);
    fprintf('     SOC_input=%.2f%%, SOC_ocv=%.2f%%, SOC_corrected=%.2f%%\n', ...
        SOC_0_ecc*100, SOC_ocv_init*100, SOC_ecc(1)*100);
else
    % Arus tinggi: tidak koreksi, gunakan nilai awal apa adanya
    SOC_ecc(1) = SOC_0_ecc;
    fprintf('  -> SOC awal: arus terlalu tinggi (%.1fA), tidak dikoreksi (%.2f%%)\n', ...
        I_first, SOC_ecc(1)*100);
end

%% ========================================================================
%  STEP 3: Main loop - Coulomb Counting + OCV Correction
%  ========================================================================
%  Sample-by-sample processing:
%    1. Coulomb counting (backbone, selalu berjalan)
%    2. Cek apakah kondisi memungkinkan OCV correction
%    3. Apply weighted blend jika ya

Cb_coulomb = Cb * 3600;  % Konversi Ah ke As
n_corrections = 0;       % Counter koreksi yang diterapkan

% Track akumulasi charge/discharge per siklus (untuk parameter update)
Qc_accum = 0;  % Charge accumulator (As)
Qd_accum = 0;  % Discharge accumulator (As)
current_cycle = 1;

for k = 2:N
    dt = time(k) - time(k-1);
    I_k = current(k);

    % ==================================================================
    %  3a. Coulomb Counting (Eq. 1-2 dari paper Lee & Won)
    %  Ini adalah tracking utama, sama seperti CCM tapi dengan eta
    % ==================================================================
    if I_k < 0
        % Charging: arus negatif, SOC naik
        delta_soc = (abs(I_k) * eta * dt) / Cb_coulomb;
        SOC_ecc(k) = SOC_ecc(k-1) + delta_soc;
        Qc_accum = Qc_accum + abs(I_k) * dt;
    else
        % Discharging: arus positif, SOC turun
        delta_soc = (I_k * eta * dt) / Cb_coulomb;
        SOC_ecc(k) = SOC_ecc(k-1) - delta_soc;
        Qd_accum = Qd_accum + I_k * dt;
    end

    % ==================================================================
    %  3b. OCV Correction - HANYA saat kondisi yang reliable
    %  Ini adalah perbedaan utama ECC vs CCM
    % ==================================================================

    % --- Kondisi 1: Rest period (|I| < 0.5A selama > 30s) ---
    % Confidence TINGGI: V_terminal ≈ OCV
    if rest_duration(k) >= min_rest_duration
        OCV_est = voltage(k);  % Saat rest, terminal voltage ≈ OCV
        SOC_ocv = ocv_soc_lookup(ocv_table, OCV_est, 'ocv2soc') / 100;
        SOC_ocv = max(0, min(1, SOC_ocv));

        % Weight meningkat dengan durasi rest (lebih lama = OCV lebih settled)
        % Saturasi di 90% setelah 5 menit rest
        w_rest = min(0.9, rest_duration(k) / 300);

        SOC_before = SOC_ecc(k);
        SOC_ecc(k) = w_rest * SOC_ocv + (1 - w_rest) * SOC_ecc(k);
        n_corrections = n_corrections + 1;

    % --- Kondisi 2: Arus sangat rendah (<3A) tapi belum rest ---
    % Confidence RENDAH: koreksi sangat gentle
    elseif abs(I_k) < I_low_threshold && abs(I_k) >= I_rest_threshold
        % Kompensasi IR drop: OCV = V + I*Rb (I positif saat discharge)
        OCV_est = voltage(k) + I_k * Rb;
        SOC_ocv = ocv_soc_lookup(ocv_table, OCV_est, 'ocv2soc') / 100;
        SOC_ocv = max(0, min(1, SOC_ocv));

        % Weight sangat kecil, proporsional ke kedekatan dengan nol
        % Semakin kecil arus, semakin percaya OCV
        w_low = 0.005 * (1 - abs(I_k) / I_low_threshold);

        SOC_ecc(k) = w_low * SOC_ocv + (1 - w_low) * SOC_ecc(k);
    end

    % --- Kondisi 3: Arus tinggi (>= 3A) ---
    % TIDAK ada koreksi OCV - kompensasi Rb terlalu unreliable
    % Coulomb counting saja yang berjalan

    % ==================================================================
    %  3c. Clamp SOC ke range valid [0, 1]
    % ==================================================================
    SOC_ecc(k) = max(0, min(1, SOC_ecc(k)));

    % ==================================================================
    %  3d. Check cycle boundary - update parameters
    %  Eq. (4-7) dari paper Lee & Won
    % ==================================================================
    if current_cycle <= n_cycles && k >= cycles(current_cycle).end_idx
        % Akhir siklus: update parameter
        Qc_Ah = Qc_accum / 3600;
        Qd_Ah = Qd_accum / 3600;

        % --- Update Coulombic Efficiency (Eq. 4) ---
        if Qc_Ah > 0.5 && Qd_Ah > 0.5
            eta_new = Qd_Ah / Qc_Ah;
            eta_new = max(0.95, min(1.0, eta_new));  % Range ketat
            % Exponential moving average: hindari perubahan drastis
            eta = 0.7 * eta + 0.3 * eta_new;
        end

        % --- Update Kapasitas (Eq. 5-6) ---
        SOC_start_cycle = SOC_ecc(cycles(current_cycle).start_idx);
        SOC_end_cycle = SOC_ecc(k);
        delta_SOC = abs(SOC_end_cycle - SOC_start_cycle);

        if delta_SOC > 0.10  % Minimal 10% delta SOC
            Q_cycle = max(Qc_Ah, Qd_Ah);
            Cb_new = Q_cycle / delta_SOC;
            Cb_new = max(C_nom * 0.8, min(C_nom * 1.1, Cb_new));  % Range ketat
            Cb = 0.8 * Cb + 0.2 * Cb_new;  % EMA
            Cb_coulomb = Cb * 3600;
        end

        % Simpan history
        params_history(current_cycle).cycle = current_cycle;
        params_history(current_cycle).eta = eta;
        params_history(current_cycle).Cb = Cb;
        params_history(current_cycle).Rb = Rb;
        params_history(current_cycle).SOC_start = SOC_start_cycle;
        params_history(current_cycle).Qc = Qc_Ah;
        params_history(current_cycle).Qd = Qd_Ah;

        % Reset akumulator
        Qc_accum = 0;
        Qd_accum = 0;
        current_cycle = current_cycle + 1;
    end
end

% Jika masih ada siklus terakhir yang belum tercatat
if current_cycle <= n_cycles
    Qc_Ah = Qc_accum / 3600;
    Qd_Ah = Qd_accum / 3600;
    params_history(current_cycle).cycle = current_cycle;
    params_history(current_cycle).eta = eta;
    params_history(current_cycle).Cb = Cb;
    params_history(current_cycle).Rb = Rb;
    params_history(current_cycle).SOC_start = SOC_ecc(cycles(current_cycle).start_idx);
    params_history(current_cycle).Qc = Qc_Ah;
    params_history(current_cycle).Qd = Qd_Ah;
end

fprintf('  -> Total OCV corrections applied: %d\n', n_corrections);
fprintf('  -> Parameter akhir: Cb=%.3f Ah, Rb=%.6f Ohm, eta=%.4f\n', Cb, Rb, eta);
fprintf('--- ECC Estimator Selesai ---\n\n');

end

%% ========================================================================
%  HELPER: Estimasi Rb dari voltage steps saat arus berubah
%  ========================================================================
%  Metode: cari momen di mana |ΔI| besar (> 5A) dalam 1 time step,
%  hitung Rb = |ΔV| / |ΔI| (instantaneous resistance)
%  Rata-ratakan beberapa estimasi untuk robustness
function Rb_est = estimate_rb_from_steps(time, current, voltage)
    N = length(time);
    Rb_samples = [];
    min_dI = 5.0;  % Minimal step arus 5A untuk estimasi

    for k = 2:min(N, 50000)  % Scan hanya awal data (cukup beberapa step)
        dI = current(k) - current(k-1);
        dV = voltage(k) - voltage(k-1);
        dt = time(k) - time(k-1);

        % Hanya gunakan step yang cukup besar dan time step normal
        if abs(dI) > min_dI && dt > 0 && dt < 5
            Rb_sample = abs(dV) / abs(dI);
            % Filter: Rb harus dalam range yang masuk akal
            if Rb_sample > 0.0005 && Rb_sample < 0.2
                Rb_samples(end+1) = Rb_sample; %#ok<AGROW>
            end
        end
    end

    if length(Rb_samples) >= 3
        % Gunakan median (robust terhadap outlier)
        Rb_est = median(Rb_samples);
    elseif ~isempty(Rb_samples)
        Rb_est = mean(Rb_samples);
    else
        Rb_est = -1;  % Flag: tidak bisa estimasi
    end
end
