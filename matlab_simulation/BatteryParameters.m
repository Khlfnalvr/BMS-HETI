classdef BatteryParameters
    % BatteryParameters - Class untuk parameter baterai dan fungsi OCV lookup
    %
    % Class ini menyimpan semua parameter baterai termasuk:
    % - Kapasitas nominal dan tegangan nominal
    % - Tabel OCV (Open Circuit Voltage) vs SoC
    % - Resistansi seri (Ro) dan resistansi transien (Rtr)
    % - Konstanta waktu (tau)
    %
    % Dapat menggunakan data default atau load dari CSV file

    properties (Constant)
        % Spesifikasi Baterai
        Q_NOMINAL = 2.6;    % Kapasitas nominal (Ah)
        V_NOMINAL = 3.7;    % Tegangan nominal (V)

        % Titik breakpoint SoC default (%)
        SOC_POINTS = [0, 10, 25, 50, 75, 90, 100];

        % Tabel OCV default pada 25C (V) - dari optimasi AVOA
        OCV_25C = [3.00, 3.06, 3.13, 3.22, 3.42, 3.58, 3.70];

        % Tabel OCV default pada 45C (V)
        OCV_45C = [3.2109, 3.2812, 3.3409, 3.4269, 3.6409, 3.7954, 3.9009];

        % Resistansi seri Ro pada 25C (Ohm)
        RO_25C = [0.0515, 0.0508, 0.0512, 0.0519, 0.0519, 0.0525, 0.0528];

        % Resistansi seri Ro pada 45C (Ohm)
        RO_45C = [0.0801, 0.0801, 0.0803, 0.0798, 0.0799, 0.0800, 0.0800];

        % Resistansi transien Rtr pada 25C (Ohm)
        RTR_25C = [0.0137, 0.0097, 0.0075, 0.0062, 0.0061, 0.0061, 0.0056];

        % Resistansi transien Rtr pada 45C (Ohm)
        RTR_45C = [0.0083, 0.0078, 0.0080, 0.0079, 0.0086, 0.0081, 0.0080];

        % Konstanta waktu tau pada 25C (detik)
        TAU_25C = [104.65, 115.36, 193.45, 120.54, 143.71, 124.36, 109.71];

        % Konstanta waktu tau pada 45C (detik)
        TAU_45C = [157.0, 166.0, 226.0, 150.0, 198.0, 154.0, 160.0];
    end

    methods (Static)
        function [soc_points, ocv_values] = loadOCVFromCSV(csv_file)
            % loadOCVFromCSV - Load OCV data dari CSV file
            %
            % Input:
            %   csv_file - Path ke file CSV dengan kolom: SOC, V0
            %
            % Output:
            %   soc_points  - Array SoC points (%)
            %   ocv_values  - Array OCV values (V)
            %
            % Format CSV:
            %   SOC,V0
            %   0,3.00
            %   10,3.06
            %   ...

            fprintf('Loading OCV data dari: %s\n', csv_file);

            % Baca CSV
            data = readtable(csv_file);

            % Ambil kolom SOC dan V0
            if ismember('SOC', data.Properties.VariableNames) && ...
               ismember('V0', data.Properties.VariableNames)
                soc_points = data.SOC;
                ocv_values = data.V0;

                % Sort berdasarkan SoC (ascending)
                [soc_points, idx] = sort(soc_points);
                ocv_values = ocv_values(idx);

                fprintf('  Loaded %d OCV data points\n', length(soc_points));
                fprintf('  SoC range: %.1f%% - %.1f%%\n', min(soc_points), max(soc_points));
                fprintf('  OCV range: %.3fV - %.3fV\n\n', min(ocv_values), max(ocv_values));
            else
                error('CSV file harus memiliki kolom: SOC dan V0');
            end
        end

        function ocv = getOCV(soc, temperature, soc_points, ocv_values)
            % getOCV - Mendapatkan OCV berdasarkan SoC dan temperatur
            %
            % Input:
            %   soc         - State of Charge (0-100%)
            %   temperature - Temperatur baterai (Celsius)
            %   soc_points  - (Optional) Custom SoC points dari CSV
            %   ocv_values  - (Optional) Custom OCV values dari CSV
            %
            % Output:
            %   ocv         - Open Circuit Voltage (V)

            % Batasi SoC ke range valid
            soc = max(0, min(100, soc));

            % Gunakan custom data jika disediakan
            if nargin >= 4 && ~isempty(soc_points) && ~isempty(ocv_values)
                % Interpolasi dari custom data
                ocv = interp1(soc_points, ocv_values, soc, 'linear', 'extrap');
            else
                % Gunakan data default
                % Pilih tabel berdasarkan temperatur
                if temperature <= 35.0
                    ocv_table = BatteryParameters.OCV_25C;
                else
                    ocv_table = BatteryParameters.OCV_45C;
                end

                % Interpolasi linear
                ocv = interp1(BatteryParameters.SOC_POINTS, ocv_table, soc, 'linear', 'extrap');
            end
        end

        function ro = getRo(soc, temperature)
            % getRo - Mendapatkan resistansi seri berdasarkan SoC dan temperatur
            %
            % Input:
            %   soc         - State of Charge (0-100%)
            %   temperature - Temperatur baterai (Celsius)
            %
            % Output:
            %   ro          - Resistansi seri (Ohm)

            soc = max(0, min(100, soc));

            if temperature <= 35.0
                ro_table = BatteryParameters.RO_25C;
            else
                ro_table = BatteryParameters.RO_45C;
            end

            ro = interp1(BatteryParameters.SOC_POINTS, ro_table, soc, 'linear', 'extrap');
        end

        function rtr = getRtr(soc, temperature)
            % getRtr - Mendapatkan resistansi transien berdasarkan SoC dan temperatur
            %
            % Input:
            %   soc         - State of Charge (0-100%)
            %   temperature - Temperatur baterai (Celsius)
            %
            % Output:
            %   rtr         - Resistansi transien (Ohm)

            soc = max(0, min(100, soc));

            if temperature <= 35.0
                rtr_table = BatteryParameters.RTR_25C;
            else
                rtr_table = BatteryParameters.RTR_45C;
            end

            rtr = interp1(BatteryParameters.SOC_POINTS, rtr_table, soc, 'linear', 'extrap');
        end

        function tau = getTau(soc, temperature)
            % getTau - Mendapatkan konstanta waktu berdasarkan SoC dan temperatur
            %
            % Input:
            %   soc         - State of Charge (0-100%)
            %   temperature - Temperatur baterai (Celsius)
            %
            % Output:
            %   tau         - Konstanta waktu (detik)

            soc = max(0, min(100, soc));

            if temperature <= 35.0
                tau_table = BatteryParameters.TAU_25C;
            else
                tau_table = BatteryParameters.TAU_45C;
            end

            tau = interp1(BatteryParameters.SOC_POINTS, tau_table, soc, 'linear', 'extrap');
        end

        function plotOCVCurve()
            % plotOCVCurve - Plot kurva OCV vs SoC untuk kedua temperatur

            soc_range = 0:0.1:100;
            ocv_25 = zeros(size(soc_range));
            ocv_45 = zeros(size(soc_range));

            for i = 1:length(soc_range)
                ocv_25(i) = BatteryParameters.getOCV(soc_range(i), 25);
                ocv_45(i) = BatteryParameters.getOCV(soc_range(i), 45);
            end

            figure;
            plot(soc_range, ocv_25, 'b-', 'LineWidth', 2);
            hold on;
            plot(soc_range, ocv_45, 'r-', 'LineWidth', 2);
            plot(BatteryParameters.SOC_POINTS, BatteryParameters.OCV_25C, 'bo', 'MarkerSize', 8);
            plot(BatteryParameters.SOC_POINTS, BatteryParameters.OCV_45C, 'ro', 'MarkerSize', 8);
            grid on;
            xlabel('State of Charge (%)');
            ylabel('Open Circuit Voltage (V)');
            title('Kurva OCV vs SoC');
            legend('25°C', '45°C', 'Data 25°C', 'Data 45°C');
        end
    end
end
