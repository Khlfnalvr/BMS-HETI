function data = load_data(ocv_file, ts_file)
% ==========================================================================
%  LOAD_DATA - Memuat dan memvalidasi data CSV untuk simulasi SOC
% ==========================================================================
%  Fungsi ini membaca dua file CSV:
%    1. OCV-SOC lookup table (kolom: SOC, V0/OCV)
%    2. Time-series data eksperimen (kolom: Time, Current, Voltage, Temperature)
%
%  Deteksi otomatis separator (koma/semicolon) dan header.
%  Validasi kelengkapan kolom dan tampilkan error yang jelas jika ada masalah.
%
%  Input:
%    ocv_file - path ke file CSV OCV-SOC lookup table
%    ts_file  - path ke file CSV time-series data eksperimen
%
%  Output:
%    data - struct dengan field:
%      .ocv_table.SOC  - vektor SOC (dalam persen, 0-100)
%      .ocv_table.OCV  - vektor OCV (Volt)
%      .timeseries.Time        - vektor waktu (detik)
%      .timeseries.Current     - vektor arus (Ampere)
%      .timeseries.Voltage     - vektor tegangan terminal (Volt)
%      .timeseries.Temperature - vektor temperatur (°C)
%
%  Referensi: Data eksperimen baterai Li-ion NMC 21700
% ==========================================================================

fprintf('=== LOADING DATA ===\n\n');

%% ========================================================================
%  1. Load OCV-SOC Lookup Table
%  ========================================================================

fprintf('Loading OCV-SOC lookup table dari: %s\n', ocv_file);

if ~isfile(ocv_file)
    error('load_data:FileNotFound', ...
        'File OCV-SOC tidak ditemukan: %s\nPastikan file CSV ada di lokasi yang benar.', ocv_file);
end

% Deteksi separator dengan membaca baris pertama
fid = fopen(ocv_file, 'r');
first_line = fgetl(fid);
fclose(fid);

if contains(first_line, ';')
    ocv_opts = detectImportOptions(ocv_file, 'Delimiter', ';');
else
    ocv_opts = detectImportOptions(ocv_file, 'Delimiter', ',');
end

ocv_raw = readtable(ocv_file, ocv_opts);

% Validasi kolom - cari kolom SOC dan OCV/V0
col_names = ocv_raw.Properties.VariableNames;
fprintf('  -> Kolom terdeteksi: %s\n', strjoin(col_names, ', '));

% Cari kolom SOC
soc_col = find_column(col_names, {'SOC', 'soc', 'Soc'});
if isempty(soc_col)
    error('load_data:MissingColumn', ...
        'Kolom SOC tidak ditemukan di file OCV-SOC.\nKolom yang tersedia: %s', strjoin(col_names, ', '));
end

% Cari kolom OCV (bisa bernama V0, OCV, Vocv)
ocv_col = find_column(col_names, {'V0', 'OCV', 'ocv', 'Vocv', 'vocv', 'Voltage'});
if isempty(ocv_col)
    error('load_data:MissingColumn', ...
        'Kolom OCV/V0 tidak ditemukan di file OCV-SOC.\nKolom yang tersedia: %s', strjoin(col_names, ', '));
end

% Ekstrak data
data.ocv_table.SOC = ocv_raw.(soc_col);
data.ocv_table.OCV = ocv_raw.(ocv_col);

% Cek apakah ada kolom SCV (Static Cell Voltage) - opsional
scv_col = find_column(col_names, {'SCV', 'scv', 'Scv'});
if ~isempty(scv_col)
    data.ocv_table.SCV = ocv_raw.(scv_col);
    fprintf('  -> Kolom SCV ditemukan - dapat digunakan untuk estimasi Rb\n');
else
    data.ocv_table.SCV = [];
    fprintf('  -> Kolom SCV tidak tersedia\n');
end

fprintf('  -> OCV-SOC table loaded: %d data points\n', length(data.ocv_table.SOC));
fprintf('  -> SOC range: %.1f%% to %.1f%%\n', min(data.ocv_table.SOC), max(data.ocv_table.SOC));
fprintf('  -> OCV range: %.4f V to %.4f V\n', min(data.ocv_table.OCV), max(data.ocv_table.OCV));

%% ========================================================================
%  2. Load Time-Series Data
%  ========================================================================

fprintf('\nLoading time-series data dari: %s\n', ts_file);

if ~isfile(ts_file)
    error('load_data:FileNotFound', ...
        'File time-series tidak ditemukan: %s\nPastikan file CSV ada di lokasi yang benar.', ts_file);
end

% Deteksi separator
fid = fopen(ts_file, 'r');
first_line = fgetl(fid);
fclose(fid);

if contains(first_line, ';')
    ts_opts = detectImportOptions(ts_file, 'Delimiter', ';');
else
    ts_opts = detectImportOptions(ts_file, 'Delimiter', ',');
end

ts_raw = readtable(ts_file, ts_opts);

% Validasi kolom time-series
col_names_ts = ts_raw.Properties.VariableNames;
fprintf('  -> Kolom terdeteksi: %s\n', strjoin(col_names_ts, ', '));

% Cari kolom yang dibutuhkan
time_col = find_column(col_names_ts, {'Time', 'time', 'TIME', 't', 'T'});
curr_col = find_column(col_names_ts, {'Current', 'current', 'CURRENT', 'I', 'i'});
volt_col = find_column(col_names_ts, {'Voltage', 'voltage', 'VOLTAGE', 'V', 'v', 'Volt'});
temp_col = find_column(col_names_ts, {'Temperature', 'temperature', 'TEMPERATURE', 'Temp', 'temp'});

% Validasi keberadaan kolom wajib
missing_cols = {};
if isempty(time_col), missing_cols{end+1} = 'Time'; end
if isempty(curr_col), missing_cols{end+1} = 'Current'; end
if isempty(volt_col), missing_cols{end+1} = 'Voltage'; end
if isempty(temp_col), missing_cols{end+1} = 'Temperature'; end

if ~isempty(missing_cols)
    error('load_data:MissingColumns', ...
        'Kolom berikut tidak ditemukan di file time-series: %s\nKolom yang tersedia: %s', ...
        strjoin(missing_cols, ', '), strjoin(col_names_ts, ', '));
end

% Ekstrak data
data.timeseries.Time = ts_raw.(time_col);
data.timeseries.Current = ts_raw.(curr_col);
data.timeseries.Voltage = ts_raw.(volt_col);
data.timeseries.Temperature = ts_raw.(temp_col);

% Pastikan data numerik (handle jika terbaca sebagai cell/string)
if iscell(data.timeseries.Time)
    data.timeseries.Time = cellfun(@str2double, data.timeseries.Time);
end
if iscell(data.timeseries.Current)
    data.timeseries.Current = cellfun(@str2double, data.timeseries.Current);
end
if iscell(data.timeseries.Voltage)
    data.timeseries.Voltage = cellfun(@str2double, data.timeseries.Voltage);
end
if iscell(data.timeseries.Temperature)
    data.timeseries.Temperature = cellfun(@str2double, data.timeseries.Temperature);
end

% Hapus baris dengan NaN (data corrupt)
valid_idx = ~isnan(data.timeseries.Time) & ~isnan(data.timeseries.Current) & ...
            ~isnan(data.timeseries.Voltage) & ~isnan(data.timeseries.Temperature);
n_removed = sum(~valid_idx);
if n_removed > 0
    fprintf('  -> WARNING: %d baris dengan NaN dihapus\n', n_removed);
    data.timeseries.Time = data.timeseries.Time(valid_idx);
    data.timeseries.Current = data.timeseries.Current(valid_idx);
    data.timeseries.Voltage = data.timeseries.Voltage(valid_idx);
    data.timeseries.Temperature = data.timeseries.Temperature(valid_idx);
end

N = length(data.timeseries.Time);
duration_s = data.timeseries.Time(end) - data.timeseries.Time(1);

fprintf('  -> Time-series loaded: %d samples\n', N);
fprintf('  -> Duration: %.2f s (%.2f jam)\n', duration_s, duration_s/3600);
fprintf('  -> Current range: %.3f A to %.3f A\n', min(data.timeseries.Current), max(data.timeseries.Current));
fprintf('  -> Voltage range: %.4f V to %.4f V\n', min(data.timeseries.Voltage), max(data.timeseries.Voltage));
fprintf('  -> Temperature range: %.1f °C to %.1f °C\n', min(data.timeseries.Temperature), max(data.timeseries.Temperature));

fprintf('\n=== DATA LOADING SELESAI ===\n\n');

end

%% ========================================================================
%  HELPER: Cari nama kolom yang cocok dari daftar kandidat
%  ========================================================================
function col_name = find_column(available_cols, candidates)
    col_name = '';
    for i = 1:length(candidates)
        idx = find(strcmpi(available_cols, candidates{i}), 1);
        if ~isempty(idx)
            col_name = available_cols{idx};
            return;
        end
    end
end
