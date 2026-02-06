function plot_results(time, data, SOC_true, SOC_ccm, SOC_ocv, SOC_ecc, ...
    metrics_ccm, metrics_ocv, metrics_ecc)
% ==========================================================================
%  PLOT_RESULTS - Generate semua plot perbandingan metode estimasi SOC
% ==========================================================================
%  Membuat 4 figure untuk visualisasi hasil simulasi:
%    Plot 1: Perbandingan SOC vs Waktu (4 kurva)
%    Plot 2: Error SOC vs Waktu (3 kurva)
%    Plot 3: Data Mentah dari CSV (verifikasi input)
%    Plot 4: Bar Chart Perbandingan Metrik
%
%  Input:
%    time        - vektor waktu (detik), Nx1
%    data        - struct dari load_data() (untuk data mentah)
%    SOC_true    - vektor ground truth SOC (fraksi 0-1), Nx1
%    SOC_ccm     - vektor SOC CCM (fraksi 0-1), Nx1
%    SOC_ocv     - vektor SOC OCV (fraksi 0-1), Nx1
%    SOC_ecc     - vektor SOC ECC (fraksi 0-1), Nx1
%    metrics_ccm - struct metrik dari calc_metrics() untuk CCM
%    metrics_ocv - struct metrik dari calc_metrics() untuk OCV
%    metrics_ecc - struct metrik dari calc_metrics() untuk ECC
% ==========================================================================

% Konversi waktu ke menit untuk display yang lebih baik
time_min = (time - time(1)) / 60;
time_label = 'Waktu (menit)';

% Warna konsisten untuk semua plot
color_true = [0 0 0];           % Hitam - ground truth
color_ccm  = [0.85 0.33 0.10];  % Oranye - CCM
color_ocv  = [0.47 0.67 0.19];  % Hijau - OCV
color_ecc  = [0 0.45 0.74];     % Biru - ECC

lw_true = 2.0;   % Line width ground truth
lw_est  = 1.5;   % Line width estimasi

%% ========================================================================
%  PLOT 1: Perbandingan SOC vs Waktu
%  ========================================================================
figure('Name', 'Plot 1: Perbandingan SOC vs Waktu', 'NumberTitle', 'off', ...
    'Position', [50 400 900 500]);

plot(time_min, SOC_true * 100, '-', 'Color', color_true, 'LineWidth', lw_true, ...
    'DisplayName', 'Ground Truth SOC');
hold on;
plot(time_min, SOC_ccm * 100, '--', 'Color', color_ccm, 'LineWidth', lw_est, ...
    'DisplayName', sprintf('CCM (RMSE=%.2f%%)', metrics_ccm.RMSE));
plot(time_min, SOC_ocv * 100, ':', 'Color', color_ocv, 'LineWidth', lw_est, ...
    'DisplayName', sprintf('OCV (RMSE=%.2f%%)', metrics_ocv.RMSE));
plot(time_min, SOC_ecc * 100, '-.', 'Color', color_ecc, 'LineWidth', lw_est, ...
    'DisplayName', sprintf('ECC (RMSE=%.2f%%)', metrics_ecc.RMSE));
hold off;

xlabel(time_label, 'FontSize', 11);
ylabel('SOC (%)', 'FontSize', 11);
title('Perbandingan Estimasi SOC: CCM vs OCV vs ECC', 'FontSize', 13);
legend('Location', 'best', 'FontSize', 9);
grid on;
ylim([0 105]);
set(gca, 'FontSize', 10);

%% ========================================================================
%  PLOT 2: Error SOC vs Waktu
%  ========================================================================
figure('Name', 'Plot 2: Error SOC vs Waktu', 'NumberTitle', 'off', ...
    'Position', [100 350 900 500]);

plot(time_min, metrics_ccm.error, '-', 'Color', color_ccm, 'LineWidth', lw_est, ...
    'DisplayName', sprintf('Error CCM (MAE=%.2f%%)', metrics_ccm.MAE));
hold on;
plot(time_min, metrics_ocv.error, '-', 'Color', color_ocv, 'LineWidth', lw_est, ...
    'DisplayName', sprintf('Error OCV (MAE=%.2f%%)', metrics_ocv.MAE));
plot(time_min, metrics_ecc.error, '-', 'Color', color_ecc, 'LineWidth', lw_est, ...
    'DisplayName', sprintf('Error ECC (MAE=%.2f%%)', metrics_ecc.MAE));

% Garis referensi nol
plot([time_min(1) time_min(end)], [0 0], 'k--', 'LineWidth', 0.8, ...
    'HandleVisibility', 'off');
hold off;

xlabel(time_label, 'FontSize', 11);
ylabel('SOC Error (%)', 'FontSize', 11);
title('Error Estimasi SOC (Estimasi - Ground Truth)', 'FontSize', 13);
legend('Location', 'best', 'FontSize', 9);
grid on;
set(gca, 'FontSize', 10);

%% ========================================================================
%  PLOT 3: Data Mentah dari CSV (Verifikasi Input)
%  ========================================================================
figure('Name', 'Plot 3: Data Mentah CSV', 'NumberTitle', 'off', ...
    'Position', [150 300 900 600]);

% Gunakan data dari time-series asli
ts_time_min = (data.timeseries.Time - data.timeseries.Time(1)) / 60;

% Subplot 1: Voltage
subplot(3, 1, 1);
plot(ts_time_min, data.timeseries.Voltage, 'b-', 'LineWidth', 0.8);
ylabel('Voltage (V)', 'FontSize', 10);
title('Data Mentah dari CSV (Verifikasi Input)', 'FontSize', 13);
grid on;
set(gca, 'FontSize', 9);

% Subplot 2: Current
subplot(3, 1, 2);
plot(ts_time_min, data.timeseries.Current, 'r-', 'LineWidth', 0.8);
ylabel('Current (A)', 'FontSize', 10);
grid on;
set(gca, 'FontSize', 9);

% Subplot 3: Temperature
subplot(3, 1, 3);
plot(ts_time_min, data.timeseries.Temperature, 'Color', [0.5 0.2 0.7], 'LineWidth', 0.8);
ylabel('Temperature (°C)', 'FontSize', 10);
xlabel(time_label, 'FontSize', 10);
grid on;
set(gca, 'FontSize', 9);

%% ========================================================================
%  PLOT 4: Bar Chart Perbandingan Metrik
%  ========================================================================
figure('Name', 'Plot 4: Bar Chart Perbandingan Metrik', 'NumberTitle', 'off', ...
    'Position', [200 250 900 500]);

% Data untuk bar chart
metric_names = {'MAE (%)', 'RMSE (%)', 'Max Error (%)', 'Std Dev (%)'};
metric_values = [
    metrics_ccm.MAE, metrics_ocv.MAE, metrics_ecc.MAE;
    metrics_ccm.RMSE, metrics_ocv.RMSE, metrics_ecc.RMSE;
    metrics_ccm.MaxError, metrics_ocv.MaxError, metrics_ecc.MaxError;
    metrics_ccm.StdDevError, metrics_ocv.StdDevError, metrics_ecc.StdDevError
];

b = bar(metric_values);

% Set warna bar sesuai metode
b(1).FaceColor = color_ccm;
b(2).FaceColor = color_ocv;
b(3).FaceColor = color_ecc;

set(gca, 'XTickLabel', metric_names, 'FontSize', 10);
ylabel('Nilai (%)', 'FontSize', 11);
title('Perbandingan Metrik Akurasi Estimasi SOC', 'FontSize', 13);
legend({'CCM', 'OCV', 'ECC'}, 'Location', 'northeast', 'FontSize', 10);
grid on;

% Tambahkan nilai di atas bar
for i = 1:3
    xtips = b(i).XEndPoints;
    ytips = b(i).YEndPoints;
    labels = string(round(b(i).YData, 2));
    text(xtips, ytips, labels, 'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'bottom', 'FontSize', 8);
end

end
