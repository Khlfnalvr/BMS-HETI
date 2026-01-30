# Panduan Penggunaan - Data CSV Custom

## 📋 Format File CSV Yang Diperlukan

### 1. File OCV Curve: `OCV_vs_SOC_curve.csv`

Format CSV untuk parameter OCV baterai:

```csv
SOC,V0
0,3.0
10,3.06
20,3.12
30,3.18
40,3.24
50,3.30
60,3.36
70,3.42
80,3.50
90,3.60
100,3.70
```

**Kolom:**
- `SOC`: State of Charge dalam persen (0-100%)
- `V0`: Open Circuit Voltage dalam Volt

**Catatan:**
- Data harus terurut berdasarkan SoC (ascending)
- Minimal 2 data points, maksimal unlimited
- Akan diinterpolasi secara linear

---

### 2. File Data Eksperimen: `Experimental_data_fresh_cell.csv`

Format CSV untuk data eksperimental:

```csv
Time,Current,Voltage,Temperature
0,0.5,3.7,25
1,0.5,3.69,25
2,0.5,3.68,25
3,0.5,3.67,25
4,0.5,3.66,25
...
```

**Kolom:**
- `Time`: Waktu dalam detik
- `Current`: Arus dalam Ampere (positif = discharge, negatif = charge)
- `Voltage`: Tegangan terminal dalam Volt
- `Temperature`: Temperatur dalam Celsius

**Catatan:**
- TIDAK ada kolom `True_SoC` (karena data eksperimental tidak punya ground truth)
- Data harus terurut berdasarkan waktu
- Sampling rate bebas (dt akan dihitung otomatis)

---

## 🚀 Cara Menggunakan

### Opsi 1: Gunakan Data Default

```matlab
% Menggunakan parameter default dan data test bawaan
cd matlab_simulation
results_cc = SoC_Coulomb_Counting('battery_test_data.csv');
results_kf = SoC_Kalman_Filter('battery_test_data.csv');
```

---

### Opsi 2: Gunakan Data Custom (RECOMMENDED untuk Anda!)

```matlab
% Dengan data eksperimental Anda sendiri
cd matlab_simulation

% Parameter
initial_soc = 80;  % SoC awal dalam % (sesuaikan dengan kondisi baterai Anda)
ocv_file = 'OCV_vs_SOC_curve.csv';
data_file = 'Experimental_data_fresh_cell.csv';

% Run Coulomb Counting
results_cc = SoC_Coulomb_Counting(data_file, initial_soc, ocv_file);

% Run Kalman Filter
results_kf = SoC_Kalman_Filter(data_file, initial_soc, ocv_file);
```

---

### Opsi 3: Gunakan Data Custom Tanpa File OCV

```matlab
% Jika tidak punya file OCV, gunakan parameter default
cd matlab_simulation

initial_soc = 80;
data_file = 'Experimental_data_fresh_cell.csv';

% Kosongkan parameter ocv_file atau jangan diisi
results_cc = SoC_Coulomb_Counting(data_file, initial_soc);
results_kf = SoC_Kalman_Filter(data_file, initial_soc);
```

---

## 📊 Output yang Dihasilkan

### Dengan Data Default (Ada True_SoC)

**Plot:**
1. Figure 1: Perbandingan SoC (True vs Estimated) ← Ada garis hitam untuk True SoC
2. Figure 2: Error Estimasi ← Ada error plot
3. Figure 3: Profil Arus
4. Figure 4: Tegangan Terminal

**CSV Output:**
- `results_coulomb_counting.csv`: Time_sec, Current_A, Voltage_V, Temperature_C, True_SoC, SoC_CC, **Error_CC**
- `results_kalman_filter.csv`: ..., True_SoC, SoC_CC, SoC_AUKF, V_predicted, Innovation, V_tr, **Error_CC**, **Error_AUKF**

**Statistik:**
- Mean Error, Std Deviation, Max Error, RMSE

---

### Dengan Data Custom (Tanpa True_SoC)

**Plot:**
1. Figure 1: Estimasi SoC ← Hanya garis estimasi (tidak ada true SoC)
2. Figure 2: TIDAK ADA (karena tidak ada true SoC untuk hitung error)
3. Figure 3: Profil Arus
4. Figure 4: Tegangan Terminal

**CSV Output:**
- `results_coulomb_counting.csv`: Time_sec, Current_A, Voltage_V, Temperature_C, SoC_CC ← **TIDAK ADA** Error_CC
- `results_kalman_filter.csv`: Time_sec, Current_A, Voltage_V, Temperature_C, SoC_CC, SoC_AUKF, V_predicted, Innovation, V_tr

**Statistik:**
- SoC Awal, SoC Akhir, Delta SoC, Total Ah, Total Waktu
- TIDAK ADA mean error/std/max karena tidak ada true SoC

---

## 💡 Tips & Best Practices

### 1. Menentukan Initial SoC

Jika tidak tahu SoC awal, ada beberapa cara:

**Metode 1: Dari OCV**
```matlab
% Ukur OCV baterai saat rest (setelah istirahat minimal 1 jam)
ocv_measured = 3.65;  % contoh

% Load OCV curve
[soc_points, ocv_values] = BatteryParameters.loadOCVFromCSV('OCV_vs_SOC_curve.csv');

% Interpolasi balik untuk dapat SoC
initial_soc = interp1(ocv_values, soc_points, ocv_measured, 'linear');
fprintf('Estimated Initial SoC: %.2f%%\n', initial_soc);
```

**Metode 2: Dari Tegangan Terminal**
```matlab
% Gunakan tegangan awal (perkiraan kasar)
v_initial = 3.65;

% Rule of thumb untuk Li-ion:
% 3.0V ≈ 0%, 3.3V ≈ 10%, 3.6V ≈ 50%, 3.9V ≈ 90%, 4.2V ≈ 100%
```

**Metode 3: Konservatif**
```matlab
% Jika ragu, gunakan nilai tengah
initial_soc = 50;  % 50% adalah nilai aman
```

---

### 2. Membuat File OCV_vs_SOC_curve.csv

**Sumber Data OCV:**

1. **Dari Datasheet Baterai**
   - Cari discharge curve pada datasheet
   - Ambil titik-titik pada arus yang sangat kecil (mendekati OCV)

2. **Dari Eksperimen**
   ```matlab
   % Prosedur:
   % 1. Charge baterai penuh (100%)
   % 2. Discharge dengan arus kecil (C/20) sambil catat voltage
   % 3. Istirahatkan tiap 10% SoC selama 1 jam
   % 4. Ukur OCV setelah rest
   % 5. Ulangi sampai 0%
   ```

3. **Dari Literatur**
   - Cari paper untuk jenis baterai yang sama
   - Gunakan data OCV dari penelitian yang sudah publish

**Contoh Membuat CSV:**
```matlab
% Data OCV dari eksperimen
soc = [0 10 20 30 40 50 60 70 80 90 100]';
ocv = [3.0 3.06 3.12 3.18 3.24 3.30 3.36 3.42 3.50 3.60 3.70]';

% Buat table
T = table(soc, ocv, 'VariableNames', {'SOC', 'V0'});

% Save ke CSV
writetable(T, 'OCV_vs_SOC_curve.csv');
```

---

### 3. Format Data Eksperimental

**Konversi dari Format Lain:**

Jika data Anda punya format berbeda, convert dulu:

```matlab
% Contoh: data dari logger punya kolom t, i, v, T
raw_data = readtable('raw_battery_data.csv');

% Rename kolom
data = table(raw_data.t, raw_data.i, raw_data.v, raw_data.T, ...
             'VariableNames', {'Time', 'Current', 'Voltage', 'Temperature'});

% Save dengan format yang benar
writetable(data, 'Experimental_data_fresh_cell.csv');
```

**Konvensi Arus:**
- Positif = Discharge (baterai mengeluarkan arus)
- Negatif = Charge (baterai menerima arus)

Jika data Anda terbalik, flip:
```matlab
data.Current = -data.Current;
```

---

### 4. Troubleshooting

**Problem: Error "CSV file harus memiliki kolom: SOC dan V0"**

Solusi:
```matlab
% Check nama kolom di file Anda
data = readtable('OCV_vs_SOC_curve.csv');
data.Properties.VariableNames

% Pastikan nama persis: 'SOC' dan 'V0' (case-sensitive!)
```

**Problem: OCV interpolation warning**

Solusi:
```matlab
% Pastikan data OCV cover range SoC yang cukup
% Minimal dari 0% sampai 100%
% Jika data hanya 20%-80%, extend:

soc_ext = [0; soc; 100];
ocv_ext = [3.0; ocv; 3.7];  % extrapolate secara linear
```

**Problem: Initial SoC tidak akurat**

Solusi:
```matlab
% Coba beberapa nilai dan lihat mana yang paling masuk akal
for init_soc = 70:5:90
    results = SoC_Kalman_Filter('Experimental_data_fresh_cell.csv', init_soc, 'OCV_vs_SOC_curve.csv');
    fprintf('Initial SoC: %.0f%%, Final SoC: %.2f%%\n', init_soc, results.soc_aukf(end));
end
```

---

## 📈 Analisis Hasil

### Membandingkan CC vs AUKF

```matlab
% Run kedua metode
results_cc = SoC_Coulomb_Counting('Experimental_data_fresh_cell.csv', 80, 'OCV_vs_SOC_curve.csv');
results_kf = SoC_Kalman_Filter('Experimental_data_fresh_cell.csv', 80, 'OCV_vs_SOC_curve.csv');

% Plot perbandingan
figure;
plot(results_cc.time/60, results_cc.soc_cc, 'b-', 'LineWidth', 2, 'DisplayName', 'Coulomb Counting');
hold on;
plot(results_kf.time/60, results_kf.soc_aukf, 'r-', 'LineWidth', 2, 'DisplayName', 'Kalman Filter (AUKF)');
xlabel('Time (minutes)');
ylabel('SoC (%)');
title('Comparison: CC vs AUKF');
legend('Location', 'best');
grid on;

% Lihat perbedaan
diff_soc = results_cc.soc_cc - results_kf.soc_aukf;
figure;
plot(results_cc.time/60, diff_soc, 'k-', 'LineWidth', 1.5);
xlabel('Time (minutes)');
ylabel('SoC Difference (%)');
title('SoC Difference: CC - AUKF');
grid on;

fprintf('Max difference: %.2f%%\n', max(abs(diff_soc)));
fprintf('Mean difference: %.2f%%\n', mean(abs(diff_soc)));
```

---

## ✅ Checklist Sebelum Running

- [ ] File `OCV_vs_SOC_curve.csv` ada dan formatnya benar (kolom: SOC, V0)
- [ ] File `Experimental_data_fresh_cell.csv` ada dan formatnya benar (kolom: Time, Current, Voltage, Temperature)
- [ ] Initial SoC sudah ditentukan (estimasi dari OCV atau konservatif)
- [ ] Konvensi arus benar (positif = discharge)
- [ ] Data OCV cover range SoC yang cukup (minimal 0-100%)
- [ ] Sudah di directory `matlab_simulation/`

---

## 📞 Bantuan Lebih Lanjut

Jika ada masalah:

1. Check format CSV dengan:
   ```matlab
   data = readtable('file.csv');
   data.Properties.VariableNames  % lihat nama kolom
   head(data, 10)  % lihat 10 baris pertama
   ```

2. Plot data mentah untuk verifikasi:
   ```matlab
   data = readtable('Experimental_data_fresh_cell.csv');
   figure;
   subplot(3,1,1); plot(data.Time/60, data.Current); ylabel('Current (A)'); title('Raw Data Check');
   subplot(3,1,2); plot(data.Time/60, data.Voltage); ylabel('Voltage (V)');
   subplot(3,1,3); plot(data.Time/60, data.Temperature); ylabel('Temp (C)'); xlabel('Time (min)');
   ```

3. Test dengan data kecil dulu:
   ```matlab
   % Ambil hanya 100 data points pertama
   data = readtable('Experimental_data_fresh_cell.csv');
   data_small = data(1:100, :);
   writetable(data_small, 'test_small.csv');

   % Test
   results = SoC_Coulomb_Counting('test_small.csv', 80, 'OCV_vs_SOC_curve.csv');
   ```

---

**Selamat menggunakan! 🚀🔋**
