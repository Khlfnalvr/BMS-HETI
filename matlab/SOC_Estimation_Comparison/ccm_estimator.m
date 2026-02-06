function SOC_ccm = ccm_estimator(time, current, SOC_0_ccm, C_nom, I_bias)
% ==========================================================================
%  CCM_ESTIMATOR - Pure Coulomb Counting Method (CCM)
% ==========================================================================
%  Estimasi SOC menggunakan integrasi arus sederhana TANPA koreksi.
%
%  Formula:
%    SOC_ccm(t) = SOC_0 - (1/C_nom) * integral(I(t) * dt)
%
%  Karakteristik CCM:
%    - SOC awal mengandung error (sengaja untuk demonstrasi)
%    - TIDAK ada koreksi OCV selama operasi
%    - Efisiensi Coulombic diasumsikan = 1 (tidak diperhitungkan)
%    - Sensor bias (offset) ditambahkan untuk simulasi error realistik
%    - Error akan terakumulasi seiring waktu (drift)
%
%  Input:
%    time      - vektor waktu (detik), Nx1
%    current   - vektor arus (Ampere), Nx1, positif = discharge
%    SOC_0_ccm - SOC awal untuk CCM (fraksi 0-1), sudah termasuk error
%    C_nom     - kapasitas nominal baterai (Ah)
%    I_bias    - sensor current bias/offset (Ampere), default +0.02 A
%
%  Output:
%    SOC_ccm - vektor SOC estimasi CCM (fraksi 0-1), Nx1
%
%  Catatan:
%    Metode ini SENGAJA tidak sempurna untuk menunjukkan kelemahan CCM:
%    1. Initial value error -> offset awal
%    2. Current sensor bias -> drift linear
%    3. Tidak ada mekanisme koreksi -> error terakumulasi
% ==========================================================================

if nargin < 5
    I_bias = 0.02;  % Default sensor bias: +0.02 A
end

N = length(time);
SOC_ccm = zeros(N, 1);
SOC_ccm(1) = SOC_0_ccm;

% Konversi kapasitas ke Coulomb
C_coulomb = C_nom * 3600;  % Ah -> As

% Efisiensi Coulombic = 1.0 (CCM mengabaikan efisiensi)
eta = 1.0;

for k = 2:N
    dt = time(k) - time(k-1);

    % Arus dengan bias sensor (simulasi error sensor)
    I_measured = current(k) + I_bias;

    % Coulomb counting: positif = discharge -> SOC turun
    % eta = 1 untuk CCM (tidak memperhitungkan efisiensi)
    SOC_ccm(k) = SOC_ccm(k-1) - (eta * I_measured * dt) / C_coulomb;

    % Clamp ke range [0, 1]
    SOC_ccm(k) = max(0, min(1, SOC_ccm(k)));
end

end
