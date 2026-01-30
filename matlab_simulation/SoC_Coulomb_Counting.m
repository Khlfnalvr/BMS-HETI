function [results] = SoC_Coulomb_Counting(data_file, initial_soc, ocv_csv_file)
    % SoC_Coulomb_Counting - Estimasi SoC menggunakan HANYA Coulomb Counting
    %
    % Fungsi ini mengimplementasikan estimasi SoC murni dengan metode Coulomb
    % Counting (integrasi arus) TANPA menggunakan koreksi OCV atau Kalman Filter.
    %
    % Input:
    %   data_file    - Path ke file CSV dengan data test baterai
    %                  Format 1: battery_test_data.csv (kolom: Time_sec, Current_A, Voltage_V, Temperature_C, True_SoC)
    %                  Format 2: Experimental_data_fresh_cell.csv (kolom: Time, Current, Voltage, Temperature)
    %   initial_soc  - (Optional) SoC awal dalam % (default: 80)
    %                  Hanya digunakan jika data tidak memiliki kolom True_SoC
    %   ocv_csv_file - (Optional) Path ke OCV CSV file (kolom: SOC, V0)
    %                  Jika tidak disediakan, gunakan parameter default
    %
    % Output:
    %   results   - Struct berisi hasil estimasi dengan field:
    %               .time        - Waktu (detik)
    %               .current     - Arus (A)
    %               .voltage     - Tegangan (V)
    %               .temperature - Temperatur (C)
    %               .true_soc    - SoC sebenarnya (%) - hanya jika ada di data
    %               .soc_cc      - SoC estimasi Coulomb Counting (%)
    %               .error_cc    - Error estimasi (%) - hanya jika true_soc ada
    %               .stats       - Statistik error - hanya jika true_soc ada
    %
    % Contoh penggunaan:
    %   % Dengan data lama (ada True_SoC)
    %   results = SoC_Coulomb_Counting('battery_test_data.csv');
    %
    %   % Dengan data baru (tanpa True_SoC)
    %   results = SoC_Coulomb_Counting('Experimental_data_fresh_cell.csv', 80);
    %
    %   % Dengan data dan OCV custom
    %   results = SoC_Coulomb_Counting('Experimental_data_fresh_cell.csv', 80, 'OCV_vs_SOC_curve.csv');

    %% Setup
    if nargin < 1
        data_file = 'battery_test_data.csv';
    end
    if nargin < 2
        initial_soc = 80;  % Default SoC awal
    end
    if nargin < 3
        ocv_csv_file = '';  % Gunakan default
    end

    fprintf('========================================\n');
    fprintf('  Estimasi SoC - Coulomb Counting Saja\n');
    fprintf('========================================\n\n');

    %% Load OCV data jika ada
    soc_points_ocv = [];
    ocv_values = [];
    if ~isempty(ocv_csv_file)
        [soc_points_ocv, ocv_values] = BatteryParameters.loadOCVFromCSV(ocv_csv_file);
    end

    %% Load Data
    fprintf('Membaca data test dari: %s\n', data_file);
    data = readtable(data_file);

    % Deteksi format CSV
    col_names = data.Properties.VariableNames;

    % Format 1: battery_test_data.csv (Time_sec, Current_A, Voltage_V, Temperature_C, True_SoC)
    % Format 2: Experimental_data_fresh_cell.csv (Time, Current, Voltage, Temperature)

    if ismember('Time_sec', col_names)
        % Format 1 - dengan True_SoC
        time = data.Time_sec;
        current = data.Current_A;
        voltage = data.Voltage_V;
        temperature = data.Temperature_C;
        true_soc = data.True_SoC;
        has_true_soc = true;
        fprintf('Format: Data dengan True_SoC (untuk validasi)\n');
    elseif ismember('Time', col_names)
        % Format 2 - tanpa True_SoC
        time = data.Time;
        current = data.Current;
        voltage = data.Voltage;
        temperature = data.Temperature;
        true_soc = [];
        has_true_soc = false;
        fprintf('Format: Data eksperimental (tanpa True_SoC)\n');
        fprintf('Initial SoC: %.2f%%\n', initial_soc);
    else
        error('Format CSV tidak dikenali. Pastikan ada kolom Time atau Time_sec');
    end

    n_points = length(time);
    fprintf('Jumlah data points: %d\n', n_points);
    fprintf('Durasi test: %.1f detik (%.1f menit)\n\n', time(end), time(end)/60);

    %% Inisialisasi Coulomb Counter
    % Parameter baterai
    Q_nominal = BatteryParameters.Q_NOMINAL;  % Ah
    eta = 1.0;  % Efisiensi Coulomb (100%)

    % State
    soc_cc = zeros(n_points, 1);
    if has_true_soc
        soc_cc(1) = true_soc(1);  % Inisialisasi dengan SoC sebenarnya
    else
        soc_cc(1) = initial_soc;  % Gunakan initial_soc yang diberikan user
    end
    total_Ah = 0;

    fprintf('Inisialisasi:\n');
    fprintf('  SoC awal      : %.2f%%\n', soc_cc(1));
    fprintf('  Kapasitas     : %.2f Ah\n', Q_nominal);
    fprintf('  Efisiensi     : %.2f\n\n', eta);

    %% Proses Coulomb Counting
    fprintf('Memproses Coulomb Counting...\n');
    if has_true_soc
        fprintf('Waktu(s)  Arus(A)  Tegangan(V)  Temp(C)  SoC_True  SoC_CC  Error(%%)\n');
        fprintf('-------------------------------------------------------------------------\n');
    else
        fprintf('Waktu(s)  Arus(A)  Tegangan(V)  Temp(C)  SoC_CC\n');
        fprintf('--------------------------------------------------\n');
    end

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

        % Cetak progress setiap 10 menit (atau 600 detik)
        if mod(floor(time(k)), 600) == 0 || time(k) < 60
            if has_true_soc
                error_cc_k = soc_cc(k) - true_soc(k);
                fprintf('%7.0f  %7.3f  %11.3f  %7.1f  %8.2f  %6.2f  %+6.2f\n', ...
                        time(k), current(k), voltage(k), temperature(k), ...
                        true_soc(k), soc_cc(k), error_cc_k);
            else
                fprintf('%7.0f  %7.3f  %11.3f  %7.1f  %6.2f\n', ...
                        time(k), current(k), voltage(k), temperature(k), soc_cc(k));
            end
        end
    end

    fprintf('\n');

    %% Hitung Error (hanya jika ada True_SoC)
    if has_true_soc
        error_cc = soc_cc - true_soc;
    else
        error_cc = [];
    end

    %% Statistik
    fprintf('========================================\n');
    fprintf('  Hasil Estimasi\n');
    fprintf('========================================\n\n');

    if has_true_soc
        stats.mean_error = mean(error_cc);
        stats.std_error = std(error_cc);
        stats.max_error = max(abs(error_cc));
        stats.rmse = sqrt(mean(error_cc.^2));
        stats.total_Ah = total_Ah;

        fprintf('Statistik Error Coulomb Counting:\n');
        fprintf('  Mean Error     : %+.3f%%\n', stats.mean_error);
        fprintf('  Std Deviation  : %.3f%%\n', stats.std_error);
        fprintf('  Max Error      : %.3f%%\n', stats.max_error);
        fprintf('  RMSE           : %.3f%%\n', stats.rmse);
        fprintf('  Total Ah       : %.3f Ah\n\n', stats.total_Ah);
    else
        stats.total_Ah = total_Ah;
        stats.final_soc = soc_cc(end);

        fprintf('Hasil Coulomb Counting:\n');
        fprintf('  SoC Awal       : %.2f%%\n', soc_cc(1));
        fprintf('  SoC Akhir      : %.2f%%\n', stats.final_soc);
        fprintf('  Delta SoC      : %.2f%%\n', soc_cc(1) - stats.final_soc);
        fprintf('  Total Ah       : %.3f Ah\n', stats.total_Ah);
        fprintf('  Total Waktu    : %.1f detik (%.2f jam)\n\n', time(end), time(end)/3600);
    end

    %% Simpan hasil
    results.time = time;
    results.current = current;
    results.voltage = voltage;
    results.temperature = temperature;
    results.true_soc = true_soc;
    results.soc_cc = soc_cc;
    results.error_cc = error_cc;
    results.stats = stats;

    %% Plot Hasil (4 Figure Terpisah)

    % Figure 1: SoC Comparison
    figure('Position', [50, 600, 1000, 400], 'Name', 'CC - SoC Estimation');
    if has_true_soc
        plot(time/60, true_soc, 'k-', 'LineWidth', 2.5, 'DisplayName', 'SoC Sebenarnya');
        hold on;
        plot(time/60, soc_cc, 'b-', 'LineWidth', 1.8, 'DisplayName', 'Coulomb Counting');
        title('COULOMB COUNTING - Perbandingan SoC', 'FontSize', 13, 'FontWeight', 'bold');
    else
        plot(time/60, soc_cc, 'b-', 'LineWidth', 2.0, 'DisplayName', 'Coulomb Counting');
        hold on;
        title('COULOMB COUNTING - Estimasi SoC', 'FontSize', 13, 'FontWeight', 'bold');
    end
    xlabel('Waktu (menit)', 'FontSize', 11);
    ylabel('SoC (%)', 'FontSize', 11);
    legend('Location', 'best', 'FontSize', 10);
    grid on;
    ylim([0 100]);

    % Figure 2: Error (hanya jika ada True_SoC)
    if has_true_soc
        figure('Position', [50, 150, 1000, 400], 'Name', 'CC - Error');
        plot(time/60, error_cc, 'b-', 'LineWidth', 1.8);
        hold on;
        yline(0, 'k--', 'LineWidth', 1);
        yline(stats.mean_error, 'r--', 'LineWidth', 1.5, 'DisplayName', sprintf('Mean: %+.3f%%', stats.mean_error));
        xlabel('Waktu (menit)', 'FontSize', 11);
        ylabel('Error (%)', 'FontSize', 11);
        title('COULOMB COUNTING - Error Estimasi', 'FontSize', 13, 'FontWeight', 'bold');
        legend('Location', 'best', 'FontSize', 10);
        grid on;
    end

    % Figure 3: Arus
    figure('Position', [1100, 600, 1000, 400], 'Name', 'CC - Current Profile');
    plot(time/60, current, 'b-', 'LineWidth', 1.8);
    xlabel('Waktu (menit)', 'FontSize', 11);
    ylabel('Arus (A)', 'FontSize', 11);
    title('COULOMB COUNTING - Profil Arus Discharge', 'FontSize', 13, 'FontWeight', 'bold');
    grid on;

    % Figure 4: Tegangan
    figure('Position', [1100, 150, 1000, 400], 'Name', 'CC - Voltage');
    plot(time/60, voltage, 'r-', 'LineWidth', 1.8);
    xlabel('Waktu (menit)', 'FontSize', 11);
    ylabel('Tegangan (V)', 'FontSize', 11);
    title('COULOMB COUNTING - Tegangan Terminal', 'FontSize', 13, 'FontWeight', 'bold');
    grid on;

    fprintf('Plot telah dibuat.\n');
    fprintf('========================================\n\n');

    %% Simpan hasil ke file CSV
    output_file = 'results_coulomb_counting.csv';
    if has_true_soc
        output_table = table(time, current, voltage, temperature, true_soc, soc_cc, error_cc, ...
                            'VariableNames', {'Time_sec', 'Current_A', 'Voltage_V', 'Temperature_C', ...
                                             'True_SoC', 'SoC_CC', 'Error_CC'});
    else
        output_table = table(time, current, voltage, temperature, soc_cc, ...
                            'VariableNames', {'Time_sec', 'Current_A', 'Voltage_V', 'Temperature_C', ...
                                             'SoC_CC'});
    end
    writetable(output_table, output_file);
    fprintf('Hasil disimpan ke: %s\n\n', output_file);
end
