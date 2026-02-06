function SOC_ocv = ocv_estimator(voltage, ocv_table)
% ==========================================================================
%  OCV_ESTIMATOR - Pure OCV Method untuk estimasi SOC
% ==========================================================================
%  Estimasi SOC langsung dari terminal voltage menggunakan OCV-SOC lookup.
%
%  Formula:
%    SOC_ocv(t) = lookup_table_inverse(Vb(t))
%
%  Karakteristik:
%    - Langsung mapping terminal voltage ke SOC via OCV-SOC table
%    - MASALAH: Vb != OCV saat ada arus mengalir (ada drop Ib * Rb)
%    - Saat beban tinggi, estimasi akan sangat tidak akurat
%    - Saat rest (I ≈ 0), estimasi mendekati nilai sebenarnya
%    - Metode ini sengaja dibuat tanpa kompensasi Rb untuk menunjukkan
%      kelemahannya pada operasi dinamis
%
%  Input:
%    voltage   - vektor terminal voltage terukur (Volt), Nx1
%    ocv_table - struct dengan field .SOC (%) dan .OCV (V)
%
%  Output:
%    SOC_ocv - vektor SOC estimasi (fraksi 0-1), Nx1
%
%  Catatan:
%    OCV-SOC table harus memiliki SOC dalam persen (0-100).
%    Output SOC dikembalikan dalam fraksi (0-1).
% ==========================================================================

N = length(voltage);
SOC_ocv = zeros(N, 1);

for k = 1:N
    % Langsung lookup terminal voltage ke SOC (tanpa kompensasi Rb)
    % Ini adalah kelemahan utama metode OCV saat operasi dinamis
    soc_pct = ocv_soc_lookup(ocv_table, voltage(k), 'ocv2soc');

    % Konversi dari persen (0-100) ke fraksi (0-1)
    SOC_ocv(k) = soc_pct / 100;

    % Clamp ke range [0, 1]
    SOC_ocv(k) = max(0, min(1, SOC_ocv(k)));
end

end
