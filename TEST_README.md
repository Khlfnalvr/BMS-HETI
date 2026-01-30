# BMS-HETI Testing Guide / Panduan Testing BMS-HETI

## English Version

### Overview

This testing framework allows you to verify the functionality of:
1. **Coulomb Counting (CC)** algorithm
2. **Open Circuit Voltage (OCV)** lookup and interpolation
3. **Adaptive Unscented Kalman Filter (AUKF)** for SoC estimation

The test uses dummy battery data without modifying any existing source files.

### File Structure

```
BMS-HETI/
├── test_data/
│   ├── battery_test_data.csv    # Input: Dummy test data
│   └── test_results.csv         # Output: Test results (generated)
├── TestSoCEstimator.c           # Test program source code
├── Makefile                     # Build configuration
├── TEST_README.md               # This file
└── [existing source files]      # Not modified
```

### Test Data Description

**File:** `test_data/battery_test_data.csv`

**Contents:** Realistic battery discharge scenario (~110 minutes)
- **Phase 1 (0-30 min):** 0.5C discharge (1.3A) - SoC: 80% → 65%
- **Phase 2 (30-40 min):** Rest period (0A) - SoC: ~62% (recovery)
- **Phase 3 (40-77 min):** 1C discharge (2.6A) - SoC: 62% → 60%
- **Phase 4 (77-90 min):** Rest period (0A) - SoC: ~60% (recovery)
- **Phase 5 (90-110 min):** 0.5C discharge (1.3A) - SoC: 60% → 59%

**Data columns:**
- Time (seconds)
- Current (Amperes, negative = discharge)
- Terminal Voltage (Volts)
- Temperature (Celsius)
- True SoC (% for validation)

### How to Build and Run

#### 1. Build the test program

```bash
make test
```

This will:
- Compile all necessary source files
- Build the test executable (`bms_test`)
- Run the test automatically

#### 2. Run test manually (after building)

```bash
./bms_test
```

#### 3. View results

Results are saved in `test_data/test_results.csv` with columns:
- Time, Current, Voltage, Temperature, True_SoC
- SoC_CC (Coulomb Counting estimate)
- SoC_AUKF (Kalman Filter estimate)
- Error_CC, Error_AUKF
- V_predicted (Predicted voltage)
- Innovation (Kalman innovation)
- Vtr (Transient voltage state)

### Expected Output

The test program displays:

1. **Progress Table:** Real-time SoC estimates during testing
2. **Performance Statistics:**
   - Mean error (bias)
   - Standard deviation (precision)
   - Maximum error (worst case)
3. **OCV Function Test:** Verification of OCV lookup at different SoC levels
4. **Coulomb Counting Direct Test:** Standalone CC algorithm verification

### Performance Criteria

**Good performance indicators:**
- AUKF Mean Error: < ±1%
- AUKF Std Deviation: < 0.5%
- AUKF Max Error: < 2%
- AUKF should perform better than CC alone

### Analyzing Results

You can analyze `test_data/test_results.csv` using:
- **Excel/LibreOffice:** Open CSV and create charts
- **Python:** Use pandas and matplotlib
- **MATLAB/Octave:** Load and plot data

**Example Python analysis:**
```python
import pandas as pd
import matplotlib.pyplot as plt

# Read results
df = pd.read_csv('test_data/test_results.csv')

# Plot SoC comparison
plt.figure(figsize=(12, 6))
plt.plot(df['Time_sec']/60, df['True_SoC'], 'k-', label='True SoC', linewidth=2)
plt.plot(df['Time_sec']/60, df['SoC_CC'], 'b--', label='CC Only')
plt.plot(df['Time_sec']/60, df['SoC_AUKF'], 'r-', label='AUKF')
plt.xlabel('Time (minutes)')
plt.ylabel('SoC (%)')
plt.legend()
plt.grid(True)
plt.title('SoC Estimation Comparison')
plt.show()

# Plot errors
plt.figure(figsize=(12, 6))
plt.plot(df['Time_sec']/60, df['Error_CC'], 'b-', label='CC Error')
plt.plot(df['Time_sec']/60, df['Error_AUKF'], 'r-', label='AUKF Error')
plt.xlabel('Time (minutes)')
plt.ylabel('Error (%)')
plt.legend()
plt.grid(True)
plt.title('SoC Estimation Errors')
plt.show()
```

### Modifying Test Data

To create your own test scenarios, edit `test_data/battery_test_data.csv`:
1. Keep the header row intact
2. Use comma-separated values
3. Ensure Time is monotonically increasing
4. Current: negative = discharge, positive = charge
5. Voltage: realistic range (2.5V - 4.2V for Li-ion)
6. Temperature: any reasonable value (0-50°C)
7. True_SoC: for validation purposes

### Troubleshooting

**Problem:** `Cannot open input file`
- **Solution:** Ensure `test_data/battery_test_data.csv` exists

**Problem:** Compilation errors
- **Solution:** Check that all source files (.c and .h) are present

**Problem:** Results seem incorrect
- **Solution:** Verify test data format and battery parameters

### Additional Commands

```bash
# Build both main and test programs
make all

# Run the original main program
make run

# Clean build files
make clean

# Clean everything including test results
make clean-all

# Show all available commands
make help
```

---

## Versi Indonesia

### Ringkasan

Framework testing ini memungkinkan Anda memverifikasi fungsionalitas:
1. Algoritma **Coulomb Counting (CC)**
2. Lookup dan interpolasi **Open Circuit Voltage (OCV)**
3. **Adaptive Unscented Kalman Filter (AUKF)** untuk estimasi SoC

Test menggunakan data dummy tanpa memodifikasi file source yang sudah ada.

### Struktur File

```
BMS-HETI/
├── test_data/
│   ├── battery_test_data.csv    # Input: Data test dummy
│   └── test_results.csv         # Output: Hasil test (di-generate)
├── TestSoCEstimator.c           # Source code program test
├── Makefile                     # Konfigurasi build
├── TEST_README.md               # File ini
└── [file source yang ada]       # Tidak dimodifikasi
```

### Deskripsi Data Test

**File:** `test_data/battery_test_data.csv`

**Isi:** Skenario discharge baterai realistis (~110 menit)
- **Fase 1 (0-30 mnt):** Discharge 0.5C (1.3A) - SoC: 80% → 65%
- **Fase 2 (30-40 mnt):** Istirahat (0A) - SoC: ~62% (recovery)
- **Fase 3 (40-77 mnt):** Discharge 1C (2.6A) - SoC: 62% → 60%
- **Fase 4 (77-90 mnt):** Istirahat (0A) - SoC: ~60% (recovery)
- **Fase 5 (90-110 mnt):** Discharge 0.5C (1.3A) - SoC: 60% → 59%

**Kolom data:**
- Time (detik)
- Current (Ampere, negatif = discharge)
- Terminal Voltage (Volt)
- Temperature (Celsius)
- True SoC (% untuk validasi)

### Cara Build dan Menjalankan

#### 1. Build program test

```bash
make test
```

Ini akan:
- Compile semua file source yang diperlukan
- Build executable test (`bms_test`)
- Menjalankan test secara otomatis

#### 2. Jalankan test manual (setelah build)

```bash
./bms_test
```

#### 3. Lihat hasil

Hasil disimpan di `test_data/test_results.csv` dengan kolom:
- Time, Current, Voltage, Temperature, True_SoC
- SoC_CC (estimasi Coulomb Counting)
- SoC_AUKF (estimasi Kalman Filter)
- Error_CC, Error_AUKF
- V_predicted (Tegangan prediksi)
- Innovation (Inovasi Kalman)
- Vtr (State tegangan transien)

### Output yang Diharapkan

Program test menampilkan:

1. **Tabel Progress:** Estimasi SoC real-time selama testing
2. **Statistik Performa:**
   - Mean error (bias)
   - Standard deviation (presisi)
   - Maximum error (kasus terburuk)
3. **Test Fungsi OCV:** Verifikasi OCV lookup di berbagai level SoC
4. **Test Langsung Coulomb Counting:** Verifikasi algoritma CC standalone

### Kriteria Performa

**Indikator performa bagus:**
- AUKF Mean Error: < ±1%
- AUKF Std Deviation: < 0.5%
- AUKF Max Error: < 2%
- AUKF harus lebih baik dari CC saja

### Menganalisis Hasil

Anda bisa menganalisis `test_data/test_results.csv` menggunakan:
- **Excel/LibreOffice:** Buka CSV dan buat grafik
- **Python:** Gunakan pandas dan matplotlib
- **MATLAB/Octave:** Load dan plot data

**Contoh analisis Python:**
```python
import pandas as pd
import matplotlib.pyplot as plt

# Baca hasil
df = pd.read_csv('test_data/test_results.csv')

# Plot perbandingan SoC
plt.figure(figsize=(12, 6))
plt.plot(df['Time_sec']/60, df['True_SoC'], 'k-', label='SoC Sebenarnya', linewidth=2)
plt.plot(df['Time_sec']/60, df['SoC_CC'], 'b--', label='CC Saja')
plt.plot(df['Time_sec']/60, df['SoC_AUKF'], 'r-', label='AUKF')
plt.xlabel('Waktu (menit)')
plt.ylabel('SoC (%)')
plt.legend()
plt.grid(True)
plt.title('Perbandingan Estimasi SoC')
plt.show()

# Plot error
plt.figure(figsize=(12, 6))
plt.plot(df['Time_sec']/60, df['Error_CC'], 'b-', label='Error CC')
plt.plot(df['Time_sec']/60, df['Error_AUKF'], 'r-', label='Error AUKF')
plt.xlabel('Waktu (menit)')
plt.ylabel('Error (%)')
plt.legend()
plt.grid(True)
plt.title('Error Estimasi SoC')
plt.show()
```

### Memodifikasi Data Test

Untuk membuat skenario test sendiri, edit `test_data/battery_test_data.csv`:
1. Pertahankan baris header
2. Gunakan nilai yang dipisahkan koma
3. Pastikan Time meningkat monoton
4. Current: negatif = discharge, positif = charge
5. Voltage: range realistis (2.5V - 4.2V untuk Li-ion)
6. Temperature: nilai wajar (0-50°C)
7. True_SoC: untuk tujuan validasi

### Troubleshooting

**Masalah:** `Cannot open input file`
- **Solusi:** Pastikan `test_data/battery_test_data.csv` ada

**Masalah:** Error kompilasi
- **Solusi:** Cek bahwa semua file source (.c dan .h) ada

**Masalah:** Hasil tampak salah
- **Solusi:** Verifikasi format data test dan parameter baterai

### Perintah Tambahan

```bash
# Build program main dan test
make all

# Jalankan program main original
make run

# Bersihkan file build
make clean

# Bersihkan semua termasuk hasil test
make clean-all

# Tampilkan semua perintah yang tersedia
make help
```

---

## What Gets Tested / Yang Ditest

### 1. Coulomb Counting Algorithm
- ✓ Current integration over time
- ✓ SoC calculation from Ah counting
- ✓ Accumulation accuracy

### 2. OCV Lookup Function
- ✓ Interpolation at different SoC levels (0%, 10%, 25%, 50%, 75%, 90%, 100%)
- ✓ Temperature interpolation (25°C and 45°C)
- ✓ Combined SoC-Temperature interpolation

### 3. Kalman Filter (AUKF)
- ✓ State prediction with CCM
- ✓ Voltage-based correction using OCV
- ✓ Innovation (measurement residual) calculation
- ✓ Adaptive noise estimation
- ✓ State covariance update
- ✓ Transient voltage state estimation

### 4. Hybrid Estimator
- ✓ Integration of CC and AUKF
- ✓ Two-step estimation process
- ✓ Comparison between CC-only and AUKF-corrected estimates

---

## File Safety / Keamanan File

**Important:** This testing framework is designed to be non-destructive:
- ✓ No existing source files (.c, .h) are modified
- ✓ Test data is stored in separate `test_data/` directory
- ✓ Results are written to new files only
- ✓ Original implementations remain intact

**Penting:** Framework testing ini dirancang non-destruktif:
- ✓ Tidak ada file source (.c, .h) yang dimodifikasi
- ✓ Data test disimpan di direktori `test_data/` terpisah
- ✓ Hasil hanya ditulis ke file baru
- ✓ Implementasi original tetap utuh
