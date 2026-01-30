function Compare_Methods(data_file)
    % Compare_Methods - Membandingkan Coulomb Counting dengan Kalman Filter
    %
    % Script ini menjalankan kedua metode estimasi SoC dan membandingkan
    % performanya secara langsung.
    %
    % Input:
    %   data_file - Path ke file CSV dengan data test baterai
    %               (default: 'battery_test_data.csv')
    %
    % Contoh penggunaan:
    %   Compare_Methods('battery_test_data.csv');

    %% Setup
    if nargin < 1
        data_file = 'battery_test_data.csv';
    end

    fprintf('\n');
    fprintf('========================================================\n');
    fprintf('  PERBANDINGAN METODE ESTIMASI SoC\n');
    fprintf('  Coulomb Counting vs Kalman Filter (AUKF) dengan OCV\n');
    fprintf('========================================================\n\n');

    %% Jalankan Coulomb Counting
    fprintf('LANGKAH 1: Menjalankan Coulomb Counting...\n');
    fprintf('--------------------------------------------------------\n');
    results_cc = SoC_Coulomb_Counting(data_file);

    fprintf('\n\nTekan Enter untuk melanjutkan ke Kalman Filter...');
    pause;
    fprintf('\n\n');

    %% Jalankan Kalman Filter
    fprintf('LANGKAH 2: Menjalankan Kalman Filter dengan OCV...\n');
    fprintf('--------------------------------------------------------\n');
    results_kf = SoC_Kalman_Filter(data_file);

    fprintf('\n\nTekan Enter untuk melihat perbandingan...');
    pause;
    fprintf('\n\n');

    %% Perbandingan Statistik
    fprintf('========================================================\n');
    fprintf('  PERBANDINGAN PERFORMA\n');
    fprintf('========================================================\n\n');

    fprintf('%-25s  %12s  %12s  %12s\n', 'Metrik', 'CC Only', 'AUKF', 'Improvement');
    fprintf('------------------------------------------------------------------------\n');
    fprintf('%-25s  %+11.3f%%  %+11.3f%%  %11.3f%%\n', 'Mean Error', ...
            results_cc.stats.mean_error, results_kf.stats_aukf.mean_error, ...
            abs(results_cc.stats.mean_error) - abs(results_kf.stats_aukf.mean_error));
    fprintf('%-25s  %12.3f%%  %12.3f%%  %11.3f%%\n', 'Std Deviation', ...
            results_cc.stats.std_error, results_kf.stats_aukf.std_error, ...
            results_cc.stats.std_error - results_kf.stats_aukf.std_error);
    fprintf('%-25s  %12.3f%%  %12.3f%%  %11.3f%%\n', 'Max Error', ...
            results_cc.stats.max_error, results_kf.stats_aukf.max_error, ...
            results_cc.stats.max_error - results_kf.stats_aukf.max_error);
    fprintf('%-25s  %12.3f%%  %12.3f%%  %11.3f%%\n', 'RMSE', ...
            results_cc.stats.rmse, results_kf.stats_aukf.rmse, ...
            results_cc.stats.rmse - results_kf.stats_aukf.rmse);
    fprintf('\n');

    %% Plot Perbandingan Komprehensif
    figure('Position', [50, 50, 1400, 900]);

    % Subplot 1: Perbandingan SoC
    subplot(3, 2, 1);
    plot(results_cc.time/60, results_cc.true_soc, 'k-', 'LineWidth', 2.5);
    hold on;
    plot(results_cc.time/60, results_cc.soc_cc, 'b--', 'LineWidth', 1.8);
    plot(results_kf.time/60, results_kf.soc_aukf, 'r-', 'LineWidth', 1.8);
    xlabel('Waktu (menit)', 'FontSize', 10);
    ylabel('SoC (%)', 'FontSize', 10);
    title('Perbandingan Estimasi SoC', 'FontSize', 11, 'FontWeight', 'bold');
    legend('True SoC', 'Coulomb Counting', 'AUKF', 'Location', 'best');
    grid on;

    % Subplot 2: Perbandingan Error
    subplot(3, 2, 2);
    plot(results_cc.time/60, results_cc.error_cc, 'b-', 'LineWidth', 1.8);
    hold on;
    plot(results_kf.time/60, results_kf.error_aukf, 'r-', 'LineWidth', 1.8);
    yline(0, 'k--', 'LineWidth', 1);
    xlabel('Waktu (menit)', 'FontSize', 10);
    ylabel('Error (%)', 'FontSize', 10);
    title('Perbandingan Error Estimasi', 'FontSize', 11, 'FontWeight', 'bold');
    legend('CC Error', 'AUKF Error', 'Location', 'best');
    grid on;

    % Subplot 3: Box Plot Error
    subplot(3, 2, 3);
    boxplot([results_cc.error_cc, results_kf.error_aukf], ...
            'Labels', {'Coulomb Counting', 'AUKF'});
    ylabel('Error (%)', 'FontSize', 10);
    title('Distribusi Error', 'FontSize', 11, 'FontWeight', 'bold');
    grid on;

    % Subplot 4: Histogram Error
    subplot(3, 2, 4);
    histogram(results_cc.error_cc, 30, 'FaceColor', 'b', 'FaceAlpha', 0.5, 'DisplayName', 'CC');
    hold on;
    histogram(results_kf.error_aukf, 30, 'FaceColor', 'r', 'FaceAlpha', 0.5, 'DisplayName', 'AUKF');
    xlabel('Error (%)', 'FontSize', 10);
    ylabel('Frekuensi', 'FontSize', 10);
    title('Distribusi Error', 'FontSize', 11, 'FontWeight', 'bold');
    legend('Location', 'best');
    grid on;

    % Subplot 5: Profil Arus
    subplot(3, 2, 5);
    plot(results_cc.time/60, results_cc.current, 'b-', 'LineWidth', 1.5);
    xlabel('Waktu (menit)', 'FontSize', 10);
    ylabel('Arus (A)', 'FontSize', 10);
    title('Profil Arus Discharge', 'FontSize', 11, 'FontWeight', 'bold');
    grid on;

    % Subplot 6: Statistik Bar Chart
    subplot(3, 2, 6);
    metrics = {'Mean Error', 'Std Dev', 'Max Error', 'RMSE'};
    cc_values = [abs(results_cc.stats.mean_error), results_cc.stats.std_error, ...
                 results_cc.stats.max_error, results_cc.stats.rmse];
    aukf_values = [abs(results_kf.stats_aukf.mean_error), results_kf.stats_aukf.std_error, ...
                   results_kf.stats_aukf.max_error, results_kf.stats_aukf.rmse];

    x_pos = 1:length(metrics);
    bar_width = 0.35;
    bar(x_pos - bar_width/2, cc_values, bar_width, 'FaceColor', 'b', 'DisplayName', 'CC');
    hold on;
    bar(x_pos + bar_width/2, aukf_values, bar_width, 'FaceColor', 'r', 'DisplayName', 'AUKF');
    set(gca, 'XTick', x_pos, 'XTickLabel', metrics);
    ylabel('Error (%)', 'FontSize', 10);
    title('Perbandingan Metrik Error', 'FontSize', 11, 'FontWeight', 'bold');
    legend('Location', 'best');
    grid on;
    xtickangle(45);

    sgtitle('Perbandingan Komprehensif: Coulomb Counting vs AUKF', ...
            'FontSize', 13, 'FontWeight', 'bold');

    %% Kesimpulan
    fprintf('========================================================\n');
    fprintf('  KESIMPULAN\n');
    fprintf('========================================================\n\n');

    improvement_mean = abs(results_cc.stats.mean_error) - abs(results_kf.stats_aukf.mean_error);
    improvement_std = results_cc.stats.std_error - results_kf.stats_aukf.std_error;
    improvement_max = results_cc.stats.max_error - results_kf.stats_aukf.max_error;
    improvement_rmse = results_cc.stats.rmse - results_kf.stats_aukf.rmse;

    fprintf('Metode AUKF dengan koreksi OCV menunjukkan peningkatan:\n\n');

    if improvement_mean > 0
        fprintf('  ✓ Mean Error berkurang %.3f%% (%.1f%% improvement)\n', ...
                improvement_mean, 100*improvement_mean/abs(results_cc.stats.mean_error));
    end

    if improvement_std > 0
        fprintf('  ✓ Std Deviation berkurang %.3f%% (%.1f%% improvement)\n', ...
                improvement_std, 100*improvement_std/results_cc.stats.std_error);
    end

    if improvement_max > 0
        fprintf('  ✓ Max Error berkurang %.3f%% (%.1f%% improvement)\n', ...
                improvement_max, 100*improvement_max/results_cc.stats.max_error);
    end

    if improvement_rmse > 0
        fprintf('  ✓ RMSE berkurang %.3f%% (%.1f%% improvement)\n', ...
                improvement_rmse, 100*improvement_rmse/results_cc.stats.rmse);
    end

    fprintf('\n');
    fprintf('Kesimpulan:\n');
    if improvement_rmse > 0.5
        fprintf('  Kalman Filter dengan OCV menunjukkan performa SIGNIFIKAN\n');
        fprintf('  lebih baik dibandingkan Coulomb Counting murni.\n\n');
        fprintf('  Koreksi berbasis OCV berhasil mengurangi drift yang terjadi\n');
        fprintf('  pada metode Coulomb Counting.\n');
    elseif improvement_rmse > 0
        fprintf('  Kalman Filter dengan OCV menunjukkan sedikit peningkatan\n');
        fprintf('  dibandingkan Coulomb Counting.\n');
    else
        fprintf('  Dalam kasus ini, kedua metode memiliki performa yang sebanding.\n');
    end

    fprintf('\n========================================================\n\n');
end
