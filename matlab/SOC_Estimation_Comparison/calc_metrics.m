function metrics = calc_metrics(SOC_true, SOC_est, time, method_name)
% ==========================================================================
%  CALC_METRICS - Hitung metrik perbandingan akurasi estimasi SOC
% ==========================================================================
%  Menghitung metrik kuantitatif untuk evaluasi performa estimasi SOC.
%
%  Input:
%    SOC_true    - vektor SOC ground truth (fraksi 0-1), Nx1
%    SOC_est     - vektor SOC estimasi (fraksi 0-1), Nx1
%    time        - vektor waktu (detik), Nx1
%    method_name - nama metode (string) untuk label output
%
%  Output:
%    metrics - struct dengan field:
%      .name          - nama metode
%      .MAE           - Mean Absolute Error (%)
%      .RMSE          - Root Mean Square Error (%)
%      .MaxError      - Maximum Absolute Error (%)
%      .InitialError  - Error pada t=0 (%)
%      .FinalError    - Error pada t_end (%)
%      .StdDevError   - Standar deviasi error (%)
%      .DriftRate     - Error drift rate (%/jam)
%      .error         - vektor error lengkap (%) untuk plotting
% ==========================================================================

% Hitung error dalam persen (SOC_est - SOC_true) * 100
error_pct = (SOC_est - SOC_true) * 100;  % dalam %

% Hitung error absolut
abs_error = abs(error_pct);

% --- Mean Absolute Error (MAE) ---
metrics.MAE = mean(abs_error);

% --- Root Mean Square Error (RMSE) ---
metrics.RMSE = sqrt(mean(error_pct.^2));

% --- Maximum Absolute Error ---
metrics.MaxError = max(abs_error);

% --- Initial Error (pada t=0) ---
metrics.InitialError = error_pct(1);

% --- Final Error (pada t_end) ---
metrics.FinalError = error_pct(end);

% --- Standard Deviation of Error ---
metrics.StdDevError = std(error_pct);

% --- Error Drift Rate (%/jam) ---
% Hitung sebagai perubahan error per jam menggunakan regresi linear
total_time_hours = (time(end) - time(1)) / 3600;
if total_time_hours > 0
    % Linear fit: error = a*t + b, drift rate = a (konversi ke %/jam)
    t_hours = (time - time(1)) / 3600;
    p = polyfit(t_hours, error_pct, 1);
    metrics.DriftRate = p(1);  % slope = %/jam
else
    metrics.DriftRate = 0;
end

% Simpan vektor error untuk plotting
metrics.error = error_pct;
metrics.name = method_name;

end
