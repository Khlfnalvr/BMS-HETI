function [results] = SoC_Coulomb_Counting(data_file)
    % SoC_Coulomb_Counting - Estimasi SoC menggunakan HANYA Coulomb Counting
    %
    % Fungsi ini mengimplementasikan estimasi SoC murni dengan metode Coulomb
    % Counting (integrasi arus) TANPA menggunakan koreksi OCV atau Kalman Filter.
    %
    % Input:
    %   data_file - Path ke file CSV dengan data test baterai
    %               (default: 'battery_test_data.csv')
    %
    % Output:
    %   results   - Struct berisi hasil estimasi dengan field:
    %               .time        - Waktu (detik)
    %               .current     - Arus (A)
    %               .voltage     - Tegangan (V)
    %               .temperature - Temperatur (C)
    %               .true_soc    - SoC sebenarnya (%)
    %               .soc_cc      - SoC estimasi Coulomb Counting (%)
    %               .error_cc    - Error estimasi (%)
    %               .stats       - Statistik error (mean, std, max)
    %
    % Contoh penggunaan:
    %   results = SoC_Coulomb_Counting('battery_test_data.csv');
    %
    %   % Plot hasil
    %   figure;
    %   plot(results.time/60, results.true_soc, 'k-', 'LineWidth', 2);
    %   hold on;
    %   plot(results.time/60, results.soc_cc, 'b--', 'LineWidth', 1.5);
    %   xlabel('Waktu (menit)');
    %   ylabel('SoC (%)');
    %   legend('SoC Sebenarnya', 'Coulomb Counting');
    %   grid on;

    %% Setup
    if nargin < 1
        data_file = 'battery_test_data.csv';
    end

    fprintf('========================================\n');
    fprintf('  Estimasi SoC - Coulomb Counting Saja\n');
    fprintf('========================================\n\n');

    %% Load Data
    fprintf('Membaca data test dari: %s\n', data_file);
    data = readtable(data_file);

    time = data.Time_sec;
    current = data.Current_A;  % Positif = discharge
    voltage = data.Voltage_V;
    temperature = data.Temperature_C;
    true_soc = data.True_SoC;

    n_points = length(time);
    fprintf('Jumlah data points: %d\n', n_points);
    fprintf('Durasi test: %.1f menit\n\n', time(end)/60);

    %% Inisialisasi Coulomb Counter
    % Parameter baterai
    Q_nominal = BatteryParameters.Q_NOMINAL;  % Ah
    eta = 1.0;  % Efisiensi Coulomb (100%)

    % State
    soc_cc = zeros(n_points, 1);
    soc_cc(1) = true_soc(1);  % Inisialisasi dengan SoC sebenarnya
    total_Ah = 0;

    fprintf('Inisialisasi:\n');
    fprintf('  SoC awal      : %.2f%%\n', soc_cc(1));
    fprintf('  Kapasitas     : %.2f Ah\n', Q_nominal);
    fprintf('  Efisiensi     : %.2f\n\n', eta);

    %% Proses Coulomb Counting
    fprintf('Memproses Coulomb Counting...\n');
    fprintf('Waktu(s)  Arus(A)  SoC_True  SoC_CC  Error(%%)\n');
    fprintf('------------------------------------------------\n');

    for k = 2:n_points
        % Hitung delta waktu
        dt = time(k) - time(k-1);

        % Update SoC dengan Coulomb Counting
        % SoC(k) = SoC(k-1) - (I * dt * 100) / (3600 * Q_nominal)
        delta_soc = (100.0 * current(k) * dt * eta) / (3600.0 * Q_nominal);
        soc_cc(k) = soc_cc(k-1) - delta_soc;

        % Batasi SoC ke range 0-100%
        soc_cc(k) = max(0, min(100, soc_cc(k)));

        % Update total Ah
        total_Ah = total_Ah + (current(k) * dt / 3600.0);

        % Cetak progress setiap 10 menit
        if mod(time(k), 600) == 0 || time(k) < 60
            error_cc = soc_cc(k) - true_soc(k);
            fprintf('%7.0f  %7.3f  %8.2f  %6.2f  %+6.2f\n', ...
                    time(k), current(k), true_soc(k), soc_cc(k), error_cc);
        end
    end

    fprintf('\n');

    %% Hitung Error
    error_cc = soc_cc - true_soc;

    %% Statistik
    stats.mean_error = mean(error_cc);
    stats.std_error = std(error_cc);
    stats.max_error = max(abs(error_cc));
    stats.rmse = sqrt(mean(error_cc.^2));
    stats.total_Ah = total_Ah;

    fprintf('========================================\n');
    fprintf('  Hasil Estimasi\n');
    fprintf('========================================\n\n');
    fprintf('Statistik Error Coulomb Counting:\n');
    fprintf('  Mean Error     : %+.3f%%\n', stats.mean_error);
    fprintf('  Std Deviation  : %.3f%%\n', stats.std_error);
    fprintf('  Max Error      : %.3f%%\n', stats.max_error);
    fprintf('  RMSE           : %.3f%%\n', stats.rmse);
    fprintf('  Total Ah       : %.3f Ah\n\n', stats.total_Ah);

    %% Simpan hasil
    results.time = time;
    results.current = current;
    results.voltage = voltage;
    results.temperature = temperature;
    results.true_soc = true_soc;
    results.soc_cc = soc_cc;
    results.error_cc = error_cc;
    results.stats = stats;

    %% Plot Hasil
    figure('Position', [100, 100, 1200, 800]);

    % Subplot 1: SoC Comparison
    subplot(3, 1, 1);
    plot(time/60, true_soc, 'k-', 'LineWidth', 2, 'DisplayName', 'SoC Sebenarnya');
    hold on;
    plot(time/60, soc_cc, 'b--', 'LineWidth', 1.5, 'DisplayName', 'Coulomb Counting');
    xlabel('Waktu (menit)');
    ylabel('SoC (%)');
    title('Perbandingan SoC: Coulomb Counting vs True SoC');
    legend('Location', 'best');
    grid on;

    % Subplot 2: Error
    subplot(3, 1, 2);
    plot(time/60, error_cc, 'b-', 'LineWidth', 1.5);
    hold on;
    yline(0, 'k--', 'LineWidth', 1);
    yline(stats.mean_error, 'r--', 'LineWidth', 1, 'DisplayName', sprintf('Mean: %.3f%%', stats.mean_error));
    xlabel('Waktu (menit)');
    ylabel('Error (%)');
    title('Error Estimasi Coulomb Counting');
    legend('Location', 'best');
    grid on;

    % Subplot 3: Arus dan Tegangan
    subplot(3, 1, 3);
    yyaxis left;
    plot(time/60, current, 'b-', 'LineWidth', 1.5);
    ylabel('Arus (A)');
    yyaxis right;
    plot(time/60, voltage, 'r-', 'LineWidth', 1.5);
    ylabel('Tegangan (V)');
    xlabel('Waktu (menit)');
    title('Profil Arus dan Tegangan');
    grid on;

    fprintf('Plot telah dibuat.\n');
    fprintf('========================================\n\n');

    %% Simpan hasil ke file CSV
    output_file = 'results_coulomb_counting.csv';
    output_table = table(time, current, voltage, temperature, true_soc, soc_cc, error_cc, ...
                        'VariableNames', {'Time_sec', 'Current_A', 'Voltage_V', 'Temperature_C', ...
                                         'True_SoC', 'SoC_CC', 'Error_CC'});
    writetable(output_table, output_file);
    fprintf('Hasil disimpan ke: %s\n\n', output_file);
end
