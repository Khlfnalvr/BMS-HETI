function [SOC_ecc, params_history] = ecc_estimator(time, current, voltage, ...
    ocv_table, cycles, SOC_0_ecc, C_nom, Rb_init, eta_init)
% ==========================================================================
%  ECC_ESTIMATOR - Enhanced Coulomb Counting (Lee & Won, IEEE Access 2023)
% ==========================================================================
%  Implementasi metode Enhanced Coulomb Counting berdasarkan paper:
%  "Enhanced Coulomb Counting Method for SoC and SoH Estimation
%   Based on Coulombic Efficiency"
%  DOI: 10.1109/ACCESS.2023.3244801
%
%  Perbedaan utama dengan CCM:
%    1. Koreksi initial value setiap awal siklus via OCV + Rb kompensasi
%    2. Memperhitungkan Coulombic Efficiency (eta) yang di-update per siklus
%    3. Update kapasitas (Cb) per siklus berdasarkan muatan terukur
%    4. Update internal resistance (Rb) per siklus
%
%  Algoritma (sesuai paper):
%    Step 3-1: Initial value correction (setiap awal siklus k)
%      SOC_0,k = lookup(Vocv ± Ib * Rb,k-1)
%
%    Step 3-2: Real-time tracking (selama siklus)
%      Charging:    SOC(t,k) = SOC_0,k + (1/Cb,k-1) * integral(Ic * eta) * dt
%      Discharging: SOC(t,k) = SOC_0,k - (1/Cb,k-1) * integral(Id * eta) * dt
%
%    Step 5-6: Update parameter akhir siklus
%      eta_b,k = |Qd| / Qc                    % Eq. (4) - Coulombic efficiency
%      Cb,k = Q_measured / delta_SOC           % Eq. (5-6) - Kapasitas update
%
%    Step 7: Internal resistance update
%      Rb,k = Vocv * (1 - eta_b,k) / Ib       % Eq. (7)
%
%  Input:
%    time      - vektor waktu (detik), Nx1
%    current   - vektor arus (Ampere), Nx1, positif = discharge
%    voltage   - vektor terminal voltage (Volt), Nx1
%    ocv_table - struct dengan field .SOC (%) dan .OCV (V)
%    cycles    - struct array dari detect_cycles()
%    SOC_0_ecc - SOC awal (fraksi 0-1), akan dikoreksi oleh ECC
%    C_nom     - kapasitas nominal awal (Ah)
%    Rb_init   - internal resistance awal (Ohm)
%    eta_init  - Coulombic efficiency awal (0-1)
%
%  Output:
%    SOC_ecc       - vektor SOC estimasi ECC (fraksi 0-1), Nx1
%    params_history - struct array per siklus, parameter yang di-update:
%      .cycle       - nomor siklus
%      .eta         - Coulombic efficiency
%      .Cb          - kapasitas (Ah)
%      .Rb          - internal resistance (Ohm)
%      .SOC_start   - SOC awal siklus (setelah koreksi)
%      .Qc          - muatan charge (Ah)
%      .Qd          - muatan discharge (Ah)
% ==========================================================================

N = length(time);
SOC_ecc = zeros(N, 1);

% Parameter saat ini (akan di-update per siklus)
Cb = C_nom;       % Kapasitas (Ah)
Rb = Rb_init;     % Internal resistance (Ohm)
eta = eta_init;   % Coulombic efficiency

n_cycles = length(cycles);

% Inisialisasi history parameter
params_history = struct('cycle', {}, 'eta', {}, 'Cb', {}, 'Rb', {}, ...
                        'SOC_start', {}, 'Qc', {}, 'Qd', {});

fprintf('--- ECC Estimator (Lee & Won, IEEE Access 2023) ---\n');
fprintf('  -> Parameter awal: Cb=%.3f Ah, Rb=%.4f Ohm, eta=%.4f\n', Cb, Rb, eta);
fprintf('  -> Jumlah siklus: %d\n', n_cycles);

% Konversi kapasitas ke Coulomb
Cb_coulomb = Cb * 3600;

for c = 1:n_cycles
    idx_start = cycles(c).start_idx;
    idx_end = cycles(c).end_idx;
    cycle_type = cycles(c).type;

    % ====================================================================
    %  Step 3-1: Initial Value Correction (Koreksi SOC awal siklus)
    %  Eq. (3) dari paper Lee & Won
    % ====================================================================
    V_start = voltage(idx_start);
    I_start = current(idx_start);

    if c == 1
        % Siklus pertama: gunakan SOC_0_ecc yang diberikan, tapi koreksi
        % menggunakan OCV + kompensasi Rb
        % Estimasi OCV dari terminal voltage
        if I_start > 0
            % Discharge: OCV = V_terminal + I * Rb (V_terminal < OCV)
            OCV_est = V_start + abs(I_start) * Rb;
        elseif I_start < 0
            % Charge: OCV = V_terminal - |I| * Rb (V_terminal > OCV)
            OCV_est = V_start - abs(I_start) * Rb;
        else
            % Rest: V_terminal ≈ OCV
            OCV_est = V_start;
        end

        % Lookup SOC dari OCV yang telah dikompensasi
        SOC_corrected = ocv_soc_lookup(ocv_table, OCV_est, 'ocv2soc') / 100;
        SOC_corrected = max(0, min(1, SOC_corrected));

        % Gunakan koreksi OCV sebagai initial value
        SOC_ecc(idx_start) = SOC_corrected;

        fprintf('  -> Siklus %d: SOC awal dikoreksi dari %.2f%% ke %.2f%% (OCV=%.4fV)\n', ...
            c, SOC_0_ecc*100, SOC_corrected*100, OCV_est);
    else
        % Siklus selanjutnya: koreksi dari OCV + Rb saat ini
        if I_start > 0
            OCV_est = V_start + abs(I_start) * Rb;
        elseif I_start < 0
            OCV_est = V_start - abs(I_start) * Rb;
        else
            OCV_est = V_start;
        end

        SOC_corrected = ocv_soc_lookup(ocv_table, OCV_est, 'ocv2soc') / 100;
        SOC_corrected = max(0, min(1, SOC_corrected));
        SOC_ecc(idx_start) = SOC_corrected;

        fprintf('  -> Siklus %d: SOC awal dikoreksi ke %.2f%% (OCV=%.4fV)\n', ...
            c, SOC_corrected*100, OCV_est);
    end

    % ====================================================================
    %  Step 3-2: Real-time SOC Tracking (selama siklus)
    %  Eq. (1-2) dari paper Lee & Won
    % ====================================================================
    Qc_cycle = 0;  % Akumulasi muatan charge (As)
    Qd_cycle = 0;  % Akumulasi muatan discharge (As)

    for k = (idx_start + 1):idx_end
        dt = time(k) - time(k-1);
        I_k = current(k);

        if I_k < 0
            % Charging: arus negatif (konvensi), SOC naik
            % SOC(t) = SOC_0 + (1/Cb) * integral(|Ic| * eta) * dt
            SOC_ecc(k) = SOC_ecc(k-1) + (abs(I_k) * eta * dt) / Cb_coulomb;
            Qc_cycle = Qc_cycle + abs(I_k) * dt;  % Akumulasi charge (As)
        else
            % Discharging: arus positif, SOC turun
            % SOC(t) = SOC_0 - (1/Cb) * integral(Id * eta) * dt
            SOC_ecc(k) = SOC_ecc(k-1) - (I_k * eta * dt) / Cb_coulomb;
            Qd_cycle = Qd_cycle + I_k * dt;  % Akumulasi discharge (As)
        end

        % Clamp SOC ke range [0, 1]
        SOC_ecc(k) = max(0, min(1, SOC_ecc(k)));
    end

    % Konversi muatan ke Ah
    Qc_Ah = Qc_cycle / 3600;
    Qd_Ah = Qd_cycle / 3600;

    % ====================================================================
    %  Step 5-6: Update Parameter Akhir Siklus
    % ====================================================================

    % --- Step 5: Update Coulombic Efficiency (Eq. (4) dari paper) ---
    % eta_b,k = |Qd| / Qc
    if Qc_Ah > 0.1 && Qd_Ah > 0.1
        % Hanya update jika ada charge DAN discharge signifikan
        eta_new = Qd_Ah / Qc_Ah;

        % Batasi range efisiensi yang masuk akal (0.90 - 1.0)
        eta_new = max(0.90, min(1.0, eta_new));
        eta = eta_new;

        fprintf('  -> Siklus %d: eta updated = %.4f (Qc=%.3f Ah, Qd=%.3f Ah)\n', ...
            c, eta, Qc_Ah, Qd_Ah);
    else
        fprintf('  -> Siklus %d: eta tidak di-update (Qc=%.3f, Qd=%.3f Ah - tidak cukup data)\n', ...
            c, Qc_Ah, Qd_Ah);
    end

    % --- Step 6: Update Kapasitas (Eq. (5-6) dari paper) ---
    % Cb,k = Q_measured / delta_SOC
    SOC_end_cycle = SOC_ecc(idx_end);
    delta_SOC = abs(SOC_end_cycle - SOC_corrected);

    if delta_SOC > 0.05  % Minimal 5% perubahan SOC untuk update kapasitas
        Q_total_Ah = max(Qc_Ah, Qd_Ah);  % Gunakan muatan terbesar
        Cb_new = Q_total_Ah / delta_SOC;

        % Batasi range kapasitas yang masuk akal (70% - 120% dari nominal)
        Cb_new = max(C_nom * 0.7, min(C_nom * 1.2, Cb_new));
        Cb = Cb_new;
        Cb_coulomb = Cb * 3600;

        fprintf('  -> Siklus %d: Cb updated = %.3f Ah\n', c, Cb);
    end

    % --- Step 7: Update Internal Resistance (Eq. (7) dari paper) ---
    % Rb,k = Vocv * (1 - eta) / Ib
    % Gunakan rata-rata arus absolut selama siklus
    I_avg = mean(abs(current(idx_start:idx_end)));
    V_end = voltage(idx_end);

    if I_avg > 0.5  % Minimal 0.5A rata-rata untuk estimasi Rb
        % Estimasi OCV di akhir siklus
        if current(idx_end) > 0
            OCV_end = V_end + abs(current(idx_end)) * Rb;
        elseif current(idx_end) < 0
            OCV_end = V_end - abs(current(idx_end)) * Rb;
        else
            OCV_end = V_end;
        end

        % Update Rb berdasarkan Eq. (7)
        if eta < 1.0 && OCV_end > 0
            Rb_new = OCV_end * (1 - eta) / I_avg;

            % Batasi range Rb (0.001 - 0.5 Ohm)
            Rb_new = max(0.001, min(0.5, Rb_new));
            Rb = Rb_new;

            fprintf('  -> Siklus %d: Rb updated = %.4f Ohm\n', c, Rb);
        end
    end

    % Simpan history parameter
    params_history(c).cycle = c;
    params_history(c).eta = eta;
    params_history(c).Cb = Cb;
    params_history(c).Rb = Rb;
    params_history(c).SOC_start = SOC_corrected;
    params_history(c).Qc = Qc_Ah;
    params_history(c).Qd = Qd_Ah;
end

fprintf('  -> Parameter akhir: Cb=%.3f Ah, Rb=%.4f Ohm, eta=%.4f\n', Cb, Rb, eta);
fprintf('--- ECC Estimator Selesai ---\n\n');

end
