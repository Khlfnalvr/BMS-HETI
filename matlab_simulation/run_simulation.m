% run_simulation.m
% Script cepat untuk menjalankan simulasi estimasi SoC
%
% Cara menggunakan:
%   1. Buka MATLAB
%   2. Navigate ke folder matlab_simulation
%   3. Ketik: run_simulation
%   4. Pilih opsi yang diinginkan

function run_simulation()
    clc;

    fprintf('\n');
    fprintf('╔════════════════════════════════════════════════════════════╗\n');
    fprintf('║                                                            ║\n');
    fprintf('║        SIMULASI ESTIMASI STATE OF CHARGE (SoC)           ║\n');
    fprintf('║              Battery Management System                    ║\n');
    fprintf('║                                                            ║\n');
    fprintf('╚════════════════════════════════════════════════════════════╝\n');
    fprintf('\n');

    fprintf('Pilih metode estimasi yang ingin dijalankan:\n\n');
    fprintf('  [1] Coulomb Counting Saja (tanpa OCV)\n');
    fprintf('  [2] Kalman Filter dengan OCV\n');
    fprintf('  [3] Bandingkan Kedua Metode (RECOMMENDED)\n');
    fprintf('  [4] Plot Kurva OCV Baterai\n');
    fprintf('  [5] Keluar\n\n');

    choice = input('Masukkan pilihan Anda (1-5): ');

    fprintf('\n');

    switch choice
        case 1
            fprintf('Menjalankan Coulomb Counting...\n');
            fprintf('════════════════════════════════════════════════════════════\n\n');
            try
                results = SoC_Coulomb_Counting('battery_test_data.csv');
                fprintf('\n✓ Simulasi selesai!\n');
                fprintf('  Hasil disimpan di: results_coulomb_counting.csv\n\n');

                prompt_again();
            catch ME
                fprintf('\n✗ Error: %s\n', ME.message);
                fprintf('  Pastikan file battery_test_data.csv ada.\n\n');
            end

        case 2
            fprintf('Menjalankan Kalman Filter dengan OCV...\n');
            fprintf('════════════════════════════════════════════════════════════\n\n');
            try
                results = SoC_Kalman_Filter('battery_test_data.csv');
                fprintf('\n✓ Simulasi selesai!\n');
                fprintf('  Hasil disimpan di: results_kalman_filter.csv\n\n');

                prompt_again();
            catch ME
                fprintf('\n✗ Error: %s\n', ME.message);
                fprintf('  Pastikan file battery_test_data.csv ada.\n\n');
            end

        case 3
            fprintf('Menjalankan Perbandingan Metode...\n');
            fprintf('════════════════════════════════════════════════════════════\n\n');
            try
                Compare_Methods('battery_test_data.csv');
                fprintf('\n✓ Perbandingan selesai!\n\n');

                prompt_again();
            catch ME
                fprintf('\n✗ Error: %s\n', ME.message);
                fprintf('  Pastikan file battery_test_data.csv ada.\n\n');
            end

        case 4
            fprintf('Menampilkan Kurva OCV...\n');
            fprintf('════════════════════════════════════════════════════════════\n\n');
            try
                BatteryParameters.plotOCVCurve();
                fprintf('\n✓ Plot selesai!\n\n');

                prompt_again();
            catch ME
                fprintf('\n✗ Error: %s\n', ME.message);
            end

        case 5
            fprintf('Terima kasih telah menggunakan simulator!\n\n');
            return;

        otherwise
            fprintf('Pilihan tidak valid. Silakan coba lagi.\n\n');
            run_simulation();
    end
end

function prompt_again()
    fprintf('\nIngin menjalankan simulasi lain?\n');
    response = input('(y/n): ', 's');

    if strcmpi(response, 'y') || strcmpi(response, 'yes')
        fprintf('\n');
        run_simulation();
    else
        fprintf('\nTerima kasih telah menggunakan simulator!\n\n');
    end
end
