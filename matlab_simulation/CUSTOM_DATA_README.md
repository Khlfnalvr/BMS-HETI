# ✅ UPDATE: Dukungan untuk Data CSV Custom

## 🎉 Apa yang Sudah Diupdate?

Simulasi MATLAB sekarang mendukung penggunaan data CSV Anda sendiri!

### File Yang Sudah Diupdate:

1. ✅ **BatteryParameters.m**
   - Tambahan fungsi `loadOCVFromCSV()` untuk load OCV dari file CSV
   - Fungsi `getOCV()` sekarang bisa terima custom OCV data

2. ✅ **SoC_Coulomb_Counting.m** - **SIAP DIGUNAKAN!**
   - Support 2 format CSV (dengan/tanpa True_SoC)
   - Bisa load OCV custom dari CSV
   - Parameter `initial_soc` untuk set SoC awal
   - Plot dan output otomatis menyesuaikan

3. ⚠️ **SoC_Kalman_Filter.m** - **MASIH DALAM PROSES**
   - Header function sudah diupdate
   - Load data sudah diupdate
   - **BELUM SELESAI:** Masih perlu update semua pemanggilan `getOCV()` (ada ~10+ tempat)
   - **REKOMENDASI:** Gunakan Coulomb Counting dulu untuk data custom

---

## 🚀 Cara Menggunakan (READY TO USE!)

### Langkah 1: Siapkan File CSV

Anda perlu 2 file CSV:

**1. OCV_vs_SOC_curve.csv** - Parameter OCV baterai Anda
```csv
SOC,V0
0,3.0
10,3.06
20,3.12
...
100,3.70
```

**2. Experimental_data_fresh_cell.csv** - Data eksperimen Anda
```csv
Time,Current,Voltage,Temperature
0,0.5,3.7,25
1,0.5,3.69,25
2,0.5,3.68,25
...
```

**Template sudah disediakan:**
- `OCV_vs_SOC_curve_template.csv`
- `Experimental_data_fresh_cell_template.csv`

Copy dan edit sesuai data Anda!

---

### Langkah 2: Jalankan Simulasi

```matlab
cd matlab_simulation

% Set parameter
initial_soc = 80;  % SoC awal dalam % (sesuaikan dengan kondisi baterai Anda)

% Run Coulomb Counting dengan data Anda
results = SoC_Coulomb_Counting('Experimental_data_fresh_cell.csv', ...
                                initial_soc, ...
                                'OCV_vs_SOC_curve.csv');
```

**Output:**
- Figure 1: Estimasi SoC
- Figure 2: TIDAK ADA (karena tidak ada True_SoC)
- Figure 3: Profil Arus
- Figure 4: Tegangan Terminal
- File: `results_coulomb_counting.csv`

---

### Langkah 3: Analisis Hasil

```matlab
% Lihat hasil
fprintf('SoC Awal  : %.2f%%\n', results.soc_cc(1));
fprintf('SoC Akhir : %.2f%%\n', results.soc_cc(end));
fprintf('Delta SoC : %.2f%%\n', results.soc_cc(1) - results.soc_cc(end));

% Plot tambahan
figure;
plot(results.time/3600, results.soc_cc, 'b-', 'LineWidth', 2);
xlabel('Waktu (jam)');
ylabel('SoC (%)');
title('Estimasi SoC dengan Coulomb Counting');
grid on;
```

---

## 📖 Dokumentasi Lengkap

Baca file `USAGE_GUIDE.md` untuk:
- Format detail CSV yang diperlukan
- Cara menentukan initial SoC
- Cara membuat file OCV dari datasheet/eksperimen
- Troubleshooting
- Tips & best practices

---

## ⚠️ Status Kalman Filter

### Apa yang Sudah Dikerjakan:
✅ Function signature dengan parameter tambahan
✅ Load OCV dari CSV
✅ Load data dengan format baru

### Apa yang Masih Perlu Dikerjakan:
❌ Update semua pemanggilan `BatteryParameters.getOCV(sp(1), temp)`
   Menjadi: `BatteryParameters.getOCV(sp(1), temp, soc_points_ocv, ocv_values)`

❌ Ada ~10+ tempat yang perlu diupdate dalam loop AUKF

❌ Update plot dan output untuk handle data tanpa True_SoC

**Estimasi:** Perlu ~30-45 menit untuk selesaikan semua update

**Workaround Sementara:**
- Gunakan `SoC_Coulomb_Counting()` dengan data custom Anda
- Untuk Kalman Filter, gunakan data default dulu (`battery_test_data.csv`)

---

## 🔄 Backward Compatibility

**Kabar baik:** Semua fungsi masih support format lama!

```matlab
% Cara lama (dengan data default) - MASIH WORK!
results = SoC_Coulomb_Counting('battery_test_data.csv');
results = SoC_Kalman_Filter('battery_test_data.csv');

% Cara baru (dengan data custom) - WORK untuk CC!
results = SoC_Coulomb_Counting('Experimental_data_fresh_cell.csv', 80, 'OCV_vs_SOC_curve.csv');
```

---

## 📊 Contoh Workflow

```matlab
%% 1. Load dan verifikasi OCV data
[soc_pts, ocv_vals] = BatteryParameters.loadOCVFromCSV('OCV_vs_SOC_curve.csv');

%% 2. Plot OCV curve untuk verifikasi
figure;
plot(soc_pts, ocv_vals, 'bo-', 'LineWidth', 1.5, 'MarkerSize', 8);
xlabel('SoC (%)');
ylabel('OCV (V)');
title('OCV Curve dari CSV');
grid on;

%% 3. Load dan verifikasi data eksperimen
data = readtable('Experimental_data_fresh_cell.csv');
figure;
subplot(3,1,1); plot(data.Time/60, data.Current); ylabel('Arus (A)');
subplot(3,1,2); plot(data.Time/60, data.Voltage); ylabel('Tegangan (V)');
subplot(3,1,3); plot(data.Time/60, data.Temperature); ylabel('Temp (°C)');
xlabel('Waktu (menit)');

%% 4. Estimasi initial SoC dari OCV (opsional)
v_initial = data.Voltage(1);
initial_soc_estimated = interp1(ocv_vals, soc_pts, v_initial);
fprintf('Estimated Initial SoC from OCV: %.2f%%\n', initial_soc_estimated);

%% 5. Run simulasi
initial_soc = 80;  % atau gunakan initial_soc_estimated
results = SoC_Coulomb_Counting('Experimental_data_fresh_cell.csv', ...
                                initial_soc, ...
                                'OCV_vs_SOC_curve.csv');

%% 6. Analisis hasil
fprintf('\n=== HASIL ESTIMASI ===\n');
fprintf('Durasi test: %.2f jam\n', results.time(end)/3600);
fprintf('SoC awal   : %.2f%%\n', results.soc_cc(1));
fprintf('SoC akhir  : %.2f%%\n', results.soc_cc(end));
fprintf('Energi out : %.3f Ah\n', results.stats.total_Ah);
```

---

## ❓ FAQ

**Q: Apakah saya harus punya file OCV?**
A: Tidak wajib. Jika tidak punya, sistem akan gunakan OCV default. Tapi untuk akurasi lebih baik, sebaiknya gunakan OCV sesuai baterai Anda.

**Q: Bagaimana cara dapat initial_soc?**
A: Ada 3 cara:
1. Ukur OCV setelah baterai rest → interpolasi dari OCV curve
2. Gunakan tegangan terminal awal (perkiraan kasar)
3. Gunakan nilai konservatif (50%)

**Q: Format data saya berbeda, bagaimana?**
A: Baca section "Format Data Eksperimental" di `USAGE_GUIDE.md`. Ada contoh convert dari format lain.

**Q: Kapan Kalman Filter siap?**
A: Membutuhkan update tambahan (~30-45 menit coding). Untuk saat ini, gunakan Coulomb Counting yang sudah fully functional.

**Q: Apakah ada impact ke performa?**
A: Tidak ada impact. Load CSV hanya dilakukan sekali di awal.

---

## 🎯 Kesimpulan

**SIAP DIGUNAKAN SEKARANG:**
- ✅ Coulomb Counting dengan data custom
- ✅ Load OCV dari CSV
- ✅ Support data eksperimental (tanpa True_SoC)

**MASIH DALAM PENGEMBANGAN:**
- ⚠️ Kalman Filter dengan data custom (perlu 30-45 menit lagi)

**REKOMENDASI:**
Gunakan `SoC_Coulomb_Counting()` dengan data Anda untuk testing dan validasi awal. Hasil sudah cukup baik untuk kebanyakan aplikasi!

---

**Happy Coding! 🚀🔋**
