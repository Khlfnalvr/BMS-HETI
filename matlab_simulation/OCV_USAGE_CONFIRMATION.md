# KONFIRMASI: Penggunaan OCV dalam Metode Estimasi SoC

## 📋 Ringkasan

File ini mengkonfirmasi penggunaan OCV (Open Circuit Voltage) dalam implementasi estimasi SoC.

---

## ✅ COULOMB COUNTING - `SoC_Coulomb_Counting.m`

### Apakah menggunakan OCV?
**❌ TIDAK**

### Algoritma
```matlab
% Update SoC dengan integrasi arus SAJA
delta_soc = (100 * I * dt * eta) / (3600 * Q_nominal)
SoC(k) = SoC(k-1) - delta_soc
```

### Karakteristik
- ✅ Sederhana dan cepat
- ✅ Tidak memerlukan model baterai
- ❌ **TIDAK menggunakan OCV sama sekali**
- ❌ **TIDAK ada koreksi berbasis tegangan**
- ❌ **TIDAK ada feedback loop**
- ⚠️  Akumulasi error (drift) seiring waktu

### Alur Kerja
```
┌─────────────┐
│   Arus (I)  │
└──────┬──────┘
       │
       ▼
┌─────────────────────┐
│  Integrasi Arus     │
│  (Coulomb Counting) │
└──────┬──────────────┘
       │
       ▼
┌─────────────┐
│  SoC Output │  ← Hanya dari CC, TIDAK ada koreksi OCV
└─────────────┘
```

---

## ✅ KALMAN FILTER (AUKF) - `SoC_Kalman_Filter.m`

### Apakah menggunakan OCV?
**✅ YA, BENAR-BENAR MENGGUNAKAN OCV UNTUK KOREKSI!**

### Bukti Penggunaan OCV

#### 1. Dokumentasi Header (Line 4-19)
```matlab
% ╔════════════════════════════════════════════════════════════════════════╗
% ║  PENTING: Fungsi ini menggunakan AUKF dengan OCV untuk KOREKSI!      ║
% ╚════════════════════════════════════════════════════════════════════════╝
%
% Measurement Model menggunakan OCV:
%    V_terminal = OCV(SoC) - I*Ro - V_tr
%    └─ OCV(SoC) diambil dari tabel lookup BatteryParameters.getOCV()
```

#### 2. Kode Implementasi (Line 204)
```matlab
% *** MEASUREMENT MODEL MENGGUNAKAN OCV ***
OCV = BatteryParameters.getOCV(sp(1), temp);  % ← OCV dari lookup table!
Ro = BatteryParameters.getRo(sp(1), temp);
V_term = OCV - I * Ro - sp(2);
```

**Lokasi:** `matlab_simulation/SoC_Kalman_Filter.m:204`

### Algoritma Detail

#### Prediction Step (tanpa OCV)
```matlab
% 1. Prediksi SoC dengan Coulomb Counting
SoC_pred = SoC - (100 * I * dt * eta) / (3600 * Q_nominal)

% 2. Prediksi V_tr (tegangan transien)
V_tr_pred = V_tr + (I*Rtr - V_tr)/tau * dt
```

#### Update Step (MENGGUNAKAN OCV!)
```matlab
% 3. Prediksi tegangan terminal MENGGUNAKAN OCV
OCV = BatteryParameters.getOCV(SoC_pred, temp)  % ← OCV lookup!
V_predicted = OCV - I*Ro - V_tr_pred

% 4. Hitung innovation (selisih measurement vs prediksi)
innovation = V_measured - V_predicted

% 5. Koreksi SoC menggunakan Kalman gain
SoC_corrected = SoC_pred + K * innovation
```

### Alur Kerja
```
┌─────────────┐         ┌──────────────┐
│   Arus (I)  │         │ Voltage (V)  │
└──────┬──────┘         └──────┬───────┘
       │                       │
       ▼                       │
┌─────────────────────┐        │
│  Prediksi dengan CC │        │
│  (tanpa OCV)        │        │
└──────┬──────────────┘        │
       │                       │
       ▼                       ▼
┌─────────────────────┐  ┌─────────────────────┐
│  SoC_predicted      │→ │  OCV Lookup Table   │ ← MENGGUNAKAN OCV!
└─────────────────────┘  └──────┬──────────────┘
                                │
                                ▼
                         ┌─────────────────────┐
                         │  V_predicted        │
                         │  = OCV - I*Ro - Vtr │
                         └──────┬──────────────┘
                                │
                                ▼
                         ┌─────────────────────┐
                         │  Innovation         │
                         │  = V_meas - V_pred  │
                         └──────┬──────────────┘
                                │
                                ▼
                         ┌─────────────────────┐
                         │  Kalman Gain (K)    │
                         └──────┬──────────────┘
                                │
                                ▼
                         ┌─────────────────────┐
                         │  SoC_corrected      │ ← Hasil AUKF dengan koreksi OCV
                         │  = SoC_pred + K*inov│
                         └─────────────────────┘
```

### Karakteristik
- ✅ **MENGGUNAKAN OCV untuk prediksi tegangan**
- ✅ **Ada koreksi berbasis voltage measurement**
- ✅ **Ada feedback loop untuk mengurangi drift**
- ✅ Lebih akurat dari CC murni
- ⚠️  Lebih kompleks, memerlukan tuning parameter

---

## 🔍 Perbedaan Utama

| Aspek | Coulomb Counting | Kalman Filter + OCV |
|-------|------------------|---------------------|
| **Menggunakan OCV?** | ❌ TIDAK | ✅ YA |
| **Input** | Arus saja | Arus + Tegangan |
| **Koreksi** | Tidak ada | OCV-based |
| **Feedback Loop** | Tidak ada | Ada |
| **Akurasi** | Drift seiring waktu | Lebih akurat |
| **Kompleksitas** | Sederhana | Kompleks |
| **Kode OCV** | Tidak ada | Line 204 di `SoC_Kalman_Filter.m` |

---

## 📊 Output yang Dihasilkan

### Coulomb Counting
**File:** `results_coulomb_counting.csv`

**Kolom:**
- Time_sec
- Current_A
- Voltage_V
- Temperature_C
- True_SoC
- SoC_CC ← Estimasi dari CC saja (TIDAK menggunakan OCV)
- Error_CC

**Plot (4 subplot):**
1. SoC Comparison (True vs CC)
2. Error
3. Arus
4. Tegangan

### Kalman Filter
**File:** `results_kalman_filter.csv`

**Kolom:**
- Time_sec
- Current_A
- Voltage_V
- Temperature_C
- True_SoC
- SoC_CC ← Prediksi tanpa OCV
- SoC_AUKF ← **Estimasi final DENGAN koreksi OCV**
- V_predicted ← **Tegangan prediksi dari model OCV**
- Innovation ← Selisih V_measured - V_predicted
- V_tr
- Error_CC
- Error_AUKF

**Plot (4 subplot):**
1. SoC Comparison (True vs AUKF vs CC)
2. Error AUKF
3. Arus
4. Tegangan (Measured vs **Predicted from OCV**) + Innovation

---

## 🎯 Cara Memverifikasi Penggunaan OCV

### Metode 1: Baca Kode
```matlab
% Buka file
edit matlab_simulation/SoC_Kalman_Filter.m

% Cari line 204, Anda akan menemukan:
OCV = BatteryParameters.getOCV(sp(1), temp);  % ← OCV lookup!
```

### Metode 2: Jalankan dan Lihat Output
```matlab
% Run Kalman Filter
results = SoC_Kalman_Filter('battery_test_data.csv');

% Cek apakah v_predicted menggunakan OCV
% Jika v_predicted berbeda dari voltage measurement,
% dan innovation tidak nol, maka OCV sedang digunakan!
figure;
plot(results.time/60, results.voltage, 'k-', 'LineWidth', 2);
hold on;
plot(results.time/60, results.v_predicted, 'r--', 'LineWidth', 1.5);
legend('Measured', 'Predicted (OCV-based)');
xlabel('Time (min)');
ylabel('Voltage (V)');
title('Bukti Penggunaan OCV: Predicted Voltage dari Model OCV');
```

### Metode 3: Bandingkan SoC_CC vs SoC_AUKF
```matlab
% Jika SoC_AUKF berbeda dari SoC_CC, berarti ada koreksi dari OCV!
results = SoC_Kalman_Filter('battery_test_data.csv');

figure;
plot(results.time/60, results.soc_cc, 'b--', 'LineWidth', 2);
hold on;
plot(results.time/60, results.soc_aukf, 'r-', 'LineWidth', 2);
legend('CC Prediction (no OCV)', 'AUKF (with OCV correction)');
xlabel('Time (min)');
ylabel('SoC (%)');
title('Perbedaan menunjukkan adanya koreksi OCV');

% Jika garis berbeda, berarti OCV correction bekerja!
```

---

## 📖 Referensi Kode

### Tabel OCV di BatteryParameters.m
```matlab
% Line 14-19
SOC_POINTS = [0, 10, 25, 50, 75, 90, 100];
OCV_25C = [3.00, 3.06, 3.13, 3.22, 3.42, 3.58, 3.70];
OCV_45C = [3.2109, 3.2812, 3.3409, 3.4269, 3.6409, 3.7954, 3.9009];

% Fungsi lookup (Line 36-54)
function ocv = getOCV(soc, temperature)
    if temperature <= 35.0
        ocv_table = BatteryParameters.OCV_25C;
    else
        ocv_table = BatteryParameters.OCV_45C;
    end
    ocv = interp1(BatteryParameters.SOC_POINTS, ocv_table, soc, 'linear');
end
```

---

## ✅ KESIMPULAN

1. **Coulomb Counting (`SoC_Coulomb_Counting.m`):**
   - ❌ TIDAK menggunakan OCV
   - Hanya integrasi arus
   - Drift seiring waktu

2. **Kalman Filter (`SoC_Kalman_Filter.m`):**
   - ✅ MENGGUNAKAN OCV untuk koreksi (Line 204)
   - Model: `V_terminal = OCV(SoC) - I*Ro - V_tr`
   - Koreksi SoC berdasarkan selisih tegangan measurement vs prediksi
   - Mengurangi drift dengan feedback loop

3. **Output kedua metode sudah distandarisasi:**
   - Plot: 4 subplot dengan format konsisten
   - CSV: Struktur kolom yang jelas
   - Mudah dibandingkan side-by-side

**Penggunaan OCV di Kalman Filter sudah DIKONFIRMASI dan DIDOKUMENTASIKAN dengan jelas!** ✅
