function [results] = SoC_Kalman_Filter(data_file)
    % SoC_Kalman_Filter - Estimasi SoC menggunakan Adaptive Unscented Kalman Filter
    %
    % ╔════════════════════════════════════════════════════════════════════════╗
    % ║  PENTING: Fungsi ini menggunakan AUKF dengan OCV untuk KOREKSI!      ║
    % ╚════════════════════════════════════════════════════════════════════════╝
    %
    % Algoritma:
    % 1. PREDICTION: Coulomb Counting (tanpa OCV) -> SoC drift
    % 2. UPDATE: OCV-based correction -> Mengurangi drift
    %
    % Measurement Model menggunakan OCV:
    %    V_terminal = OCV(SoC) - I*Ro - V_tr
    %    └─ OCV(SoC) diambil dari tabel lookup BatteryParameters.getOCV()
    %
    % Proses:
    % - Prediksi: SoC berkurang sesuai integrasi arus (Coulomb Counting)
    % - Koreksi: Kalman gain menyesuaikan SoC berdasarkan selisih antara
    %            tegangan terukur dan tegangan prediksi dari model OCV
    %
    % Input:
    %   data_file - Path ke file CSV dengan data test baterai
    %               (default: 'battery_test_data.csv')
    %
    % Output:
    %   results   - Struct berisi hasil estimasi dengan field:
    %               .time          - Waktu (detik)
    %               .current       - Arus (A)
    %               .voltage       - Tegangan (V)
    %               .temperature   - Temperatur (C)
    %               .true_soc      - SoC sebenarnya (%)
    %               .soc_cc        - SoC prediksi Coulomb Counting (%)
    %               .soc_aukf      - SoC estimasi AUKF (%)
    %               .v_predicted   - Tegangan prediksi (V)
    %               .innovation    - Innovation (residual) (V)
    %               .v_tr          - State tegangan transien (V)
    %               .error_cc      - Error CC (%)
    %               .error_aukf    - Error AUKF (%)
    %               .stats_cc      - Statistik error CC
    %               .stats_aukf    - Statistik error AUKF
    %
    % Contoh penggunaan:
    %   results = SoC_Kalman_Filter('battery_test_data.csv');

    %% Setup
    if nargin < 1
        data_file = 'battery_test_data.csv';
    end

    fprintf('========================================\n');
    fprintf('  Estimasi SoC - Kalman Filter + OCV\n');
    fprintf('========================================\n\n');

    %% Load Data
    fprintf('Membaca data test dari: %s\n', data_file);
    data = readtable(data_file);

    time = data.Time_sec;
    current = data.Current_A;
    voltage = data.Voltage_V;
    temperature = data.Temperature_C;
    true_soc = data.True_SoC;

    n_points = length(time);
    fprintf('Jumlah data points: %d\n', n_points);
    fprintf('Durasi test: %.1f menit\n\n', time(end)/60);

    %% Inisialisasi Parameter
    Q_nominal = BatteryParameters.Q_NOMINAL;
    eta = 1.0;

    % State: [SoC, V_tr]'
    x = [true_soc(1); 0.0];  % State awal

    % Covariance matrix
    P = [1.0, 0.0;
         0.0, 0.01];  % Inisialisasi P

    % Process noise covariance
    Q_noise = [0.01, 0.0;
               0.0, 0.001];

    % Measurement noise covariance
    R = 0.001;  % Variance noise tegangan (1mV std)

    % UKF parameters
    alpha = 0.1;
    beta = 2.0;
    kappa = 0.0;
    L = 2;  % State dimension
    lambda = alpha^2 * (L + kappa) - L;

    % Weights untuk sigma points
    Wm = zeros(2*L+1, 1);
    Wc = zeros(2*L+1, 1);
    Wm(1) = lambda / (L + lambda);
    Wc(1) = lambda / (L + lambda) + (1 - alpha^2 + beta);
    for i = 2:(2*L+1)
        Wm(i) = 1 / (2 * (L + lambda));
        Wc(i) = 1 / (2 * (L + lambda));
    end

    fprintf('Inisialisasi AUKF:\n');
    fprintf('  SoC awal      : %.2f%%\n', x(1));
    fprintf('  V_tr awal     : %.3f V\n', x(2));
    fprintf('  Temperatur    : %.1f°C\n\n', temperature(1));

    %% Alokasi Array Hasil
    soc_cc = zeros(n_points, 1);
    soc_aukf = zeros(n_points, 1);
    v_predicted = zeros(n_points, 1);
    innovation = zeros(n_points, 1);
    v_tr_history = zeros(n_points, 1);

    % Inisialisasi
    soc_cc(1) = true_soc(1);
    soc_aukf(1) = x(1);
    v_tr_history(1) = x(2);

    %% Proses AUKF
    fprintf('Memproses AUKF...\n');
    fprintf('Waktu(s)  Arus(A)  True_SoC  SoC_CC  SoC_AUKF  Error_AUKF  Innovation\n');
    fprintf('--------------------------------------------------------------------------\n');

    for k = 2:n_points
        dt = time(k) - time(k-1);
        temp = temperature(k);
        I = current(k);
        V_meas = voltage(k);

        %% 1. PREDIKSI dengan Coulomb Counting (tanpa OCV)
        delta_soc = (100.0 * I * dt * eta) / (3600.0 * Q_nominal);
        soc_predicted = x(1) - delta_soc;
        soc_predicted = max(0, min(100, soc_predicted));
        soc_cc(k) = soc_predicted;

        %% 2. AUKF Prediction Step
        % Generate sigma points
        sqrt_P = chol((L + lambda) * P, 'lower');
        sigma_points = zeros(L, 2*L+1);
        sigma_points(:, 1) = x;
        for i = 1:L
            sigma_points(:, i+1) = x + sqrt_P(:, i);
            sigma_points(:, i+L+1) = x - sqrt_P(:, i);
        end

        % Propagate sigma points melalui state transition
        sigma_points_pred = zeros(L, 2*L+1);
        for i = 1:(2*L+1)
            sp = sigma_points(:, i);

            % State transition model
            % x(1) = SoC - delta_soc (Coulomb Counting)
            soc_sp = sp(1) - (100.0 * I * dt * eta) / (3600.0 * Q_nominal);
            soc_sp = max(0, min(100, soc_sp));

            % x(2) = V_tr dynamics: dV_tr/dt = (I*Rtr - V_tr)/tau
            Rtr = BatteryParameters.getRtr(sp(1), temp);
            tau = BatteryParameters.getTau(sp(1), temp);
            dv_tr = (I * Rtr - sp(2)) / tau;
            v_tr_sp = sp(2) + dv_tr * dt;

            sigma_points_pred(:, i) = [soc_sp; v_tr_sp];
        end

        % Predicted mean and covariance
        x_pred = zeros(L, 1);
        for i = 1:(2*L+1)
            x_pred = x_pred + Wm(i) * sigma_points_pred(:, i);
        end

        P_pred = Q_noise;
        for i = 1:(2*L+1)
            diff = sigma_points_pred(:, i) - x_pred;
            P_pred = P_pred + Wc(i) * (diff * diff');
        end

        %% 3. AUKF Update Step (menggunakan OCV untuk koreksi)
        % Generate sigma points dari prediksi
        sqrt_P_pred = chol((L + lambda) * P_pred, 'lower');
        sigma_points_update = zeros(L, 2*L+1);
        sigma_points_update(:, 1) = x_pred;
        for i = 1:L
            sigma_points_update(:, i+1) = x_pred + sqrt_P_pred(:, i);
            sigma_points_update(:, i+L+1) = x_pred - sqrt_P_pred(:, i);
        end

        % Transform sigma points melalui measurement model
        % ╔═══════════════════════════════════════════════════════════╗
        % ║  INI ADALAH BAGIAN KUNCI: PENGGUNAAN OCV UNTUK KOREKSI!  ║
        % ╚═══════════════════════════════════════════════════════════╝
        Z_sigma = zeros(1, 2*L+1);
        for i = 1:(2*L+1)
            sp = sigma_points_update(:, i);

            % *** MEASUREMENT MODEL MENGGUNAKAN OCV ***
            % Model: V_terminal = OCV(SoC) - I*Ro - V_tr
            %
            % Di sini OCV(SoC) digunakan untuk memprediksi tegangan terminal
            % berdasarkan SoC yang di-estimate. Selisih antara tegangan
            % terukur dan prediksi (innovation) akan digunakan oleh Kalman
            % gain untuk mengoreksi estimasi SoC.
            %
            OCV = BatteryParameters.getOCV(sp(1), temp);  % ← OCV dari lookup table!
            Ro = BatteryParameters.getRo(sp(1), temp);
            V_term = OCV - I * Ro - sp(2);

            Z_sigma(i) = V_term;
        end

        % Predicted measurement
        z_pred = 0;
        for i = 1:(2*L+1)
            z_pred = z_pred + Wm(i) * Z_sigma(i);
        end
        v_predicted(k) = z_pred;

        % Innovation covariance
        Pzz = R;
        for i = 1:(2*L+1)
            diff_z = Z_sigma(i) - z_pred;
            Pzz = Pzz + Wc(i) * diff_z * diff_z;
        end

        % Cross-covariance
        Pxz = zeros(L, 1);
        for i = 1:(2*L+1)
            diff_x = sigma_points_update(:, i) - x_pred;
            diff_z = Z_sigma(i) - z_pred;
            Pxz = Pxz + Wc(i) * diff_x * diff_z;
        end

        % Kalman gain
        K = Pxz / Pzz;

        % Innovation (measurement residual)
        innov = V_meas - z_pred;
        innovation(k) = innov;

        % Update state
        x = x_pred + K * innov;
        x(1) = max(0, min(100, x(1)));  % Batasi SoC

        % Update covariance
        P = P_pred - K * Pzz * K';

        % Simpan hasil
        soc_aukf(k) = x(1);
        v_tr_history(k) = x(2);

        % Cetak progress
        if mod(time(k), 600) == 0 || time(k) < 60
            error_aukf = soc_aukf(k) - true_soc(k);
            fprintf('%7.0f  %7.3f  %8.2f  %6.2f  %8.2f  %+9.3f  %+10.4f\n', ...
                    time(k), I, true_soc(k), soc_cc(k), soc_aukf(k), error_aukf, innov);
        end
    end

    fprintf('\n');

    %% Hitung Error
    error_cc = soc_cc - true_soc;
    error_aukf = soc_aukf - true_soc;

    %% Statistik
    stats_cc.mean_error = mean(error_cc);
    stats_cc.std_error = std(error_cc);
    stats_cc.max_error = max(abs(error_cc));
    stats_cc.rmse = sqrt(mean(error_cc.^2));

    stats_aukf.mean_error = mean(error_aukf);
    stats_aukf.std_error = std(error_aukf);
    stats_aukf.max_error = max(abs(error_aukf));
    stats_aukf.rmse = sqrt(mean(error_aukf.^2));

    fprintf('========================================\n');
    fprintf('  Hasil Estimasi\n');
    fprintf('========================================\n\n');
    fprintf('Statistik Error Coulomb Counting:\n');
    fprintf('  Mean Error     : %+.3f%%\n', stats_cc.mean_error);
    fprintf('  Std Deviation  : %.3f%%\n', stats_cc.std_error);
    fprintf('  Max Error      : %.3f%%\n', stats_cc.max_error);
    fprintf('  RMSE           : %.3f%%\n\n', stats_cc.rmse);

    fprintf('Statistik Error AUKF:\n');
    fprintf('  Mean Error     : %+.3f%%\n', stats_aukf.mean_error);
    fprintf('  Std Deviation  : %.3f%%\n', stats_aukf.std_error);
    fprintf('  Max Error      : %.3f%%\n', stats_aukf.max_error);
    fprintf('  RMSE           : %.3f%%\n\n', stats_aukf.rmse);

    fprintf('Peningkatan (CC vs AUKF):\n');
    fprintf('  Reduksi Mean Error : %.3f%%\n', abs(stats_cc.mean_error) - abs(stats_aukf.mean_error));
    fprintf('  Reduksi Std Dev    : %.3f%%\n', stats_cc.std_error - stats_aukf.std_error);
    fprintf('  Reduksi Max Error  : %.3f%%\n\n', stats_cc.max_error - stats_aukf.max_error);

    %% Simpan hasil
    results.time = time;
    results.current = current;
    results.voltage = voltage;
    results.temperature = temperature;
    results.true_soc = true_soc;
    results.soc_cc = soc_cc;
    results.soc_aukf = soc_aukf;
    results.v_predicted = v_predicted;
    results.innovation = innovation;
    results.v_tr = v_tr_history;
    results.error_cc = error_cc;
    results.error_aukf = error_aukf;
    results.stats_cc = stats_cc;
    results.stats_aukf = stats_aukf;

    %% Plot Hasil
    figure('Position', [100, 100, 1200, 900]);

    % Subplot 1: SoC Comparison
    subplot(4, 1, 1);
    plot(time/60, true_soc, 'k-', 'LineWidth', 2.5, 'DisplayName', 'SoC Sebenarnya');
    hold on;
    plot(time/60, soc_aukf, 'r-', 'LineWidth', 1.8, 'DisplayName', 'AUKF (dengan OCV)');
    plot(time/60, soc_cc, 'b--', 'LineWidth', 1.3, 'DisplayName', 'Prediksi CC (tanpa OCV)');
    xlabel('Waktu (menit)', 'FontSize', 10);
    ylabel('SoC (%)', 'FontSize', 10);
    title('KALMAN FILTER (AUKF + OCV) - Perbandingan SoC', 'FontSize', 12, 'FontWeight', 'bold');
    legend('Location', 'best', 'FontSize', 9);
    grid on;
    ylim([0 100]);

    % Subplot 2: Error
    subplot(4, 1, 2);
    plot(time/60, error_aukf, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Error AUKF');
    hold on;
    yline(0, 'k--', 'LineWidth', 1);
    yline(stats_aukf.mean_error, 'r--', 'LineWidth', 1, 'DisplayName', sprintf('Mean: %+.3f%%', stats_aukf.mean_error));
    xlabel('Waktu (menit)', 'FontSize', 10);
    ylabel('Error (%)', 'FontSize', 10);
    title('Error Estimasi AUKF', 'FontSize', 11, 'FontWeight', 'bold');
    legend('Location', 'best', 'FontSize', 9);
    grid on;

    % Subplot 3: Arus
    subplot(4, 1, 3);
    plot(time/60, current, 'b-', 'LineWidth', 1.5);
    xlabel('Waktu (menit)', 'FontSize', 10);
    ylabel('Arus (A)', 'FontSize', 10);
    title('Profil Arus Discharge', 'FontSize', 11, 'FontWeight', 'bold');
    grid on;

    % Subplot 4: Tegangan dan Innovation
    subplot(4, 1, 4);
    yyaxis left;
    plot(time/60, voltage, 'k-', 'LineWidth', 1.5, 'DisplayName', 'Measured');
    hold on;
    plot(time/60, v_predicted, 'r--', 'LineWidth', 1.3, 'DisplayName', 'Predicted (OCV-based)');
    ylabel('Tegangan (V)', 'FontSize', 10);
    yyaxis right;
    plot(time/60, innovation*1000, 'm-', 'LineWidth', 1.0, 'DisplayName', 'Innovation');
    yline(0, 'k:', 'LineWidth', 0.5);
    ylabel('Innovation (mV)', 'FontSize', 10);
    xlabel('Waktu (menit)', 'FontSize', 10);
    title('Tegangan Terminal dan Innovation', 'FontSize', 11, 'FontWeight', 'bold');
    legend('Location', 'best', 'FontSize', 9);
    grid on;

    fprintf('Plot telah dibuat.\n');
    fprintf('========================================\n\n');

    %% Simpan hasil ke file CSV
    output_file = 'results_kalman_filter.csv';
    output_table = table(time, current, voltage, temperature, true_soc, ...
                        soc_cc, soc_aukf, v_predicted, innovation, v_tr_history, ...
                        error_cc, error_aukf, ...
                        'VariableNames', {'Time_sec', 'Current_A', 'Voltage_V', 'Temperature_C', ...
                                         'True_SoC', 'SoC_CC', 'SoC_AUKF', 'V_predicted', ...
                                         'Innovation', 'V_tr', 'Error_CC', 'Error_AUKF'});
    writetable(output_table, output_file);
    fprintf('Hasil disimpan ke: %s\n\n', output_file);
end
