# Simulasi MATLAB - Estimasi State of Charge (SoC) Baterai

## 📋 Deskripsi

Repositori ini berisi implementasi MATLAB untuk estimasi State of Charge (SoC) baterai menggunakan dua metode berbeda:

1. **Coulomb Counting (CC)** - Metode murni integrasi arus **❌ TANPA koreksi OCV**
2. **Adaptive Unscented Kalman Filter (AUKF)** - Metode Kalman Filter **✅ DENGAN koreksi berbasis OCV** (Open Circuit Voltage)

Implementasi ini adalah konversi dari kode C yang sebelumnya dibuat, untuk memudahkan simulasi dan analisis di MATLAB.

### ✅ Konfirmasi Penggunaan OCV

**PENTING:** Kalman Filter dalam `SoC_Kalman_Filter.m` **BENAR-BENAR MENGGUNAKAN OCV** untuk koreksi!

- **Bukti:** Line 204 di `SoC_Kalman_Filter.m`
- **Kode:** `OCV = BatteryParameters.getOCV(sp(1), temp);`
- **Model:** `V_terminal = OCV(SoC) - I*Ro - V_tr`

Lihat file `OCV_USAGE_CONFIRMATION.md` untuk dokumentasi lengkap tentang penggunaan OCV.

## 📁 Struktur File

```
matlab_simulation/
├── README.md                      # File ini
├── OCV_USAGE_CONFIRMATION.md      # Dokumentasi penggunaan OCV (PENTING!)
├── battery_test_data.csv          # Data test baterai (110 menit discharge)
├── BatteryParameters.m            # Class parameter baterai dan fungsi OCV lookup
├── SoC_Coulomb_Counting.m        # Estimasi SoC dengan CC TANPA OCV
├── SoC_Kalman_Filter.m           # Estimasi SoC dengan AUKF + OCV
├── Compare_Methods.m              # Script perbandingan kedua metode
├── run_simulation.m               # Menu interaktif untuk running
├── results_coulomb_counting.csv  # Hasil CC (dibuat saat running)
└── results_kalman_filter.csv     # Hasil AUKF (dibuat saat running)
```

## 🚀 Cara Menggunakan

### Opsi 1: Jalankan Coulomb Counting Saja

```matlab
% Di MATLAB Command Window
cd matlab_simulation
results_cc = SoC_Coulomb_Counting('battery_test_data.csv');
```

Script ini akan:
- ✅ Menjalankan estimasi SoC menggunakan HANYA Coulomb Counting
- ✅ Menampilkan statistik error (mean, std, max)
- ✅ Membuat plot perbandingan SoC
- ✅ Menyimpan hasil ke `results_coulomb_counting.csv`

### Opsi 2: Jalankan Kalman Filter dengan OCV

```matlab
% Di MATLAB Command Window
cd matlab_simulation
results_kf = SoC_Kalman_Filter('battery_test_data.csv');
```

Script ini akan:
- ✅ Menjalankan estimasi SoC dengan AUKF + koreksi OCV
- ✅ Menampilkan statistik error untuk CC dan AUKF
- ✅ Membuat plot perbandingan lengkap (SoC, error, voltage, innovation)
- ✅ Menyimpan hasil ke `results_kalman_filter.csv`

### Opsi 3: Bandingkan Kedua Metode (RECOMMENDED!)

```matlab
% Di MATLAB Command Window
cd matlab_simulation
Compare_Methods('battery_test_data.csv');
```

Script ini akan:
- ✅ Menjalankan KEDUA metode secara berurutan
- ✅ Menampilkan perbandingan statistik side-by-side
- ✅ Membuat plot perbandingan komprehensif
- ✅ Memberikan kesimpulan otomatis

## 📊 Output yang Dihasilkan

### ✅ Format Output Telah Distandarisasi!

Kedua metode sekarang menghasilkan output dengan format yang **KONSISTEN** untuk memudahkan perbandingan:

#### Plot (4 Figure Terpisah untuk Kedua Metode)

**Setiap metode menghasilkan 4 figure window terpisah agar lebih besar dan jelas:**

1. **Figure 1:** Perbandingan SoC (True vs Estimated)
2. **Figure 2:** Error Estimasi
3. **Figure 3:** Profil Arus Discharge
4. **Figure 4:** Tegangan Terminal (+ Innovation untuk AUKF)

**Keuntungan 4 figure terpisah:**
- ✅ Ukuran plot lebih besar dan jelas
- ✅ Mudah difokuskan pada 1 aspek
- ✅ Dapat di-zoom dan di-save secara individual
- ✅ Tidak kecil-kecil seperti subplot

#### File CSV

**Coulomb Counting** (`results_coulomb_counting.csv`):
```
Time_sec, Current_A, Voltage_V, Temperature_C, True_SoC, SoC_CC, Error_CC
```

**Kalman Filter** (`results_kalman_filter.csv`):
```
Time_sec, Current_A, Voltage_V, Temperature_C, True_SoC,
SoC_CC, SoC_AUKF, V_predicted, Innovation, V_tr, Error_CC, Error_AUKF
```

### Perbandingan Metode

| Metode | Menggunakan OCV? | Input | Koreksi | Drift |
|--------|------------------|-------|---------|-------|
| **Coulomb Counting** | ❌ TIDAK | Arus saja | Tidak ada | Ya |
| **Kalman Filter** | ✅ YA | Arus + Voltage | OCV-based | Minimal |

**Ekspektasi Performa:**
- AUKF Mean Error: < ±1%
- AUKF Std Deviation: < 0.5%
- AUKF Max Error: < 2%
- AUKF harus lebih baik dari CC karena ada koreksi OCV

## 🔬 Detail Algoritma

### 1. Coulomb Counting (Tanpa OCV)

```matlab
% Update SoC dengan integrasi arus
delta_soc = (100 * I * dt) / (3600 * Q_nominal)
SoC(k) = SoC(k-1) - delta_soc
```

**Tidak menggunakan:**
- ❌ Open Circuit Voltage (OCV)
- ❌ Koreksi berbasis pengukuran tegangan
- ❌ Feedback loop

### 2. Kalman Filter (Dengan OCV)

**State Vector:**
```
x = [SoC, V_tr]'
```
Dimana:
- `SoC` = State of Charge (%)
- `V_tr` = Tegangan transien internal (V)

**Prediction Step (Coulomb Counting):**
```matlab
SoC_pred = SoC - (100 * I * dt) / (3600 * Q_nominal)
V_tr_pred = V_tr + (I*Rtr - V_tr)/tau * dt
```

**Update Step (Menggunakan OCV):**
```matlab
V_predicted = OCV(SoC) - I*Ro - V_tr  % MENGGUNAKAN OCV!
innovation = V_measured - V_predicted
SoC_corrected = SoC_pred + K * innovation
```

**Perbedaan Utama:**
- ✅ Menggunakan OCV untuk memprediksi tegangan
- ✅ Koreksi SoC berdasarkan selisih tegangan prediksi vs measurement
- ✅ Adaptive noise estimation

## 📈 Interpretasi Hasil

### Plot yang Dihasilkan

1. **SoC Comparison**
   - Hitam (solid): SoC sebenarnya (ground truth)
   - Biru (dashed): Coulomb Counting
   - Merah (solid): AUKF

2. **Error Plot**
   - Menunjukkan error estimasi terhadap true SoC
   - Error positif = overestimate
   - Error negatif = underestimate

3. **Voltage Comparison** (hanya AUKF)
   - Hitam: Tegangan terukur
   - Merah: Tegangan prediksi dari model

4. **Innovation** (hanya AUKF)
   - Residual antara measurement dan prediksi
   - Idealnya berdistribusi Gaussian dengan mean ~0

### Statistik Error

- **Mean Error**: Bias sistematis (should be near 0)
- **Std Deviation**: Precision/consistency
- **Max Error**: Worst-case error
- **RMSE**: Overall accuracy metric

## 🔧 Modifikasi Parameter

### Mengubah Parameter Kalman Filter

Edit file `SoC_Kalman_Filter.m`:

```matlab
% Process noise covariance
Q_noise = [0.01, 0.0;      % Tuning: lebih besar = trust model lebih sedikit
           0.0, 0.001];

% Measurement noise covariance
R = 0.001;                  % Tuning: lebih besar = trust sensor lebih sedikit

% UKF parameters
alpha = 0.1;                % Spread sigma points (0.001 - 1)
beta = 2.0;                 % Prior knowledge (2 for Gaussian)
```

### Mengubah Parameter Baterai

Edit file `BatteryParameters.m`:

```matlab
% Kapasitas dan tegangan nominal
Q_NOMINAL = 2.6;            % Ah
V_NOMINAL = 3.7;            % V

% Tabel OCV
SOC_POINTS = [0, 10, 25, 50, 75, 90, 100];
OCV_25C = [3.00, 3.06, 3.13, 3.22, 3.42, 3.58, 3.70];
```

## 📝 Membuat Data Test Sendiri

Format CSV yang diperlukan:

```csv
Time_sec,Current_A,Voltage_V,Temperature_C,True_SoC
0,1.300,3.425,25.0,79.99
1,1.300,3.426,25.0,79.97
2,1.300,3.424,25.0,79.96
...
```

**Kolom:**
- `Time_sec`: Waktu dalam detik (monotonically increasing)
- `Current_A`: Arus dalam Ampere (positif = discharge, negatif = charge)
- `Voltage_V`: Tegangan terminal dalam Volt
- `Temperature_C`: Temperatur dalam Celsius
- `True_SoC`: SoC sebenarnya untuk validasi (%)

## 🐛 Troubleshooting

### Error: "Unable to read file"
**Solusi:**
```matlab
% Pastikan Anda berada di directory yang benar
pwd
cd matlab_simulation
```

### Error: "Undefined function or variable"
**Solusi:**
```matlab
% Tambahkan directory ke MATLAB path
addpath('matlab_simulation')
```

### Plot tidak muncul
**Solusi:**
```matlab
% Buka figure windows secara manual
figure(1)
figure(2)
```

### Hasil tidak akurat
**Solusi:**
1. Periksa parameter baterai (Q_nominal, OCV table)
2. Tune parameter Kalman Filter (Q, R)
3. Verifikasi data input (format CSV benar)

## 📚 Referensi

1. **Coulomb Counting**: Simple current integration method
2. **Unscented Kalman Filter**: Nonlinear state estimation with sigma points
3. **OCV-SoC Relationship**: Battery characteristic curve for SoC inference
4. **Adaptive Filtering**: Online noise estimation for better convergence

## 💡 Tips Penggunaan

1. **Untuk Analisis Cepat**: Gunakan `Compare_Methods.m`
2. **Untuk Detail CC**: Gunakan `SoC_Coulomb_Counting.m`
3. **Untuk Detail AUKF**: Gunakan `SoC_Kalman_Filter.m`
4. **Untuk Ekspor Data**: Hasil otomatis tersimpan dalam CSV
5. **Untuk Publikasi**: Gunakan `print` atau `saveas` untuk save plot

## 🎯 Contoh Analisis Lanjut

### Membandingkan Beberapa Skenario

```matlab
% Scenario 1: Default parameters
results1 = SoC_Kalman_Filter('battery_test_data.csv');

% Scenario 2: Tuning parameters
% (Edit parameter di dalam file terlebih dahulu)
results2 = SoC_Kalman_Filter('battery_test_data.csv');

% Compare
figure;
plot(results1.time/60, results1.error_aukf, 'b-', 'LineWidth', 2);
hold on;
plot(results2.time/60, results2.error_aukf, 'r-', 'LineWidth', 2);
legend('Default', 'Tuned');
xlabel('Time (min)');
ylabel('Error (%)');
title('Parameter Tuning Comparison');
grid on;
```

### Plot Kurva OCV

```matlab
% Plot OCV curve
BatteryParameters.plotOCVCurve();
```

## 📧 Kontak

Jika ada pertanyaan atau masalah, silakan buat issue di repository ini.

---

**Selamat Bereksperimen! 🚀🔋**
