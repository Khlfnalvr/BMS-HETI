function SOC_true = calc_true_soc(time, current, SOC_0, C_nom)
% ==========================================================================
%  CALC_TRUE_SOC - Hitung ground truth SOC via Coulomb Counting ideal
% ==========================================================================
%  Ground truth SOC dihitung menggunakan integrasi arus IDEAL:
%    - Tanpa bias sensor
%    - Tanpa error akumulasi buatan
%    - SOC awal yang benar (dari OCV lookup)
%
%  Formula:
%    SOC_true(t) = SOC_0 - (1/C_nom) * integral(I * dt)
%
%  Konvensi: Arus positif = discharge (SOC turun)
%
%  Input:
%    time    - vektor waktu (detik), Nx1
%    current - vektor arus (Ampere), Nx1, positif = discharge
%    SOC_0   - SOC awal (fraksi 0-1)
%    C_nom   - kapasitas nominal baterai (Ah)
%
%  Output:
%    SOC_true - vektor SOC ground truth (fraksi 0-1), Nx1
%
%  Catatan:
%    Ground truth ini bergantung pada akurasi sensor arus di CSV.
%    Ini adalah referensi terbaik yang tersedia tanpa equipment lab tambahan.
% ==========================================================================

N = length(time);
SOC_true = zeros(N, 1);
SOC_true(1) = SOC_0;

% Konversi kapasitas ke Coulomb: C_nom [Ah] * 3600 [s/h] = kapasitas [As]
C_coulomb = C_nom * 3600;

for k = 2:N
    dt = time(k) - time(k-1);

    % Integrasi arus: positif = discharge -> SOC turun
    % SOC berkurang sebesar (I * dt) / C_total
    SOC_true(k) = SOC_true(k-1) - (current(k) * dt) / C_coulomb;

    % Clamp ke range [0, 1]
    SOC_true(k) = max(0, min(1, SOC_true(k)));
end

end
