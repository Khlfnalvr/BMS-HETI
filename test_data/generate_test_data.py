#!/usr/bin/env python3
"""
Generate realistic battery test data for BMS testing
Based on proper battery model with OCV, resistance, and transient dynamics
"""

import csv
import math
import random

# Battery parameters (from Bateryparameter.h)
Q_NOMINAL = 2.6  # Ah
V_NOMINAL = 3.7  # V

# OCV lookup table at 25°C
SOC_POINTS = [0, 10, 25, 50, 75, 90, 100]
OCV_25C = [3.000, 3.060, 3.130, 3.220, 3.420, 3.580, 3.700]
OCV_45C = [3.211, 3.281, 3.341, 3.427, 3.641, 3.795, 3.901]

# Resistance parameters at 25°C (estimated realistic values)
RO_25C = [0.050, 0.045, 0.040, 0.035, 0.035, 0.040, 0.050]  # Ohm
RTR_25C = [0.030, 0.025, 0.020, 0.018, 0.020, 0.025, 0.030]  # Ohm
TAU_25C = [30, 35, 40, 45, 40, 35, 30]  # seconds

def interpolate(x_points, y_points, x_query):
    """Linear interpolation"""
    # Handle boundary cases
    if x_query <= x_points[0]:
        return y_points[0]
    if x_query >= x_points[-1]:
        return y_points[-1]

    # Find the interval
    for i in range(len(x_points) - 1):
        if x_points[i] <= x_query <= x_points[i+1]:
            # Linear interpolation
            alpha = (x_query - x_points[i]) / (x_points[i+1] - x_points[i])
            return y_points[i] + alpha * (y_points[i+1] - y_points[i])

    return y_points[-1]

def get_ocv(soc, temp=25.0):
    """Get OCV for given SoC and temperature"""
    if temp <= 25.0:
        return interpolate(SOC_POINTS, OCV_25C, soc)
    elif temp >= 45.0:
        return interpolate(SOC_POINTS, OCV_45C, soc)
    else:
        # Linear interpolation between 25°C and 45°C
        alpha = (temp - 25.0) / 20.0
        ocv_25 = interpolate(SOC_POINTS, OCV_25C, soc)
        ocv_45 = interpolate(SOC_POINTS, OCV_45C, soc)
        return (1 - alpha) * ocv_25 + alpha * ocv_45

def get_resistance(soc, temp=25.0):
    """Get battery resistances for given SoC and temperature"""
    ro = interpolate(SOC_POINTS, RO_25C, soc)
    rtr = interpolate(SOC_POINTS, RTR_25C, soc)
    tau = interpolate(SOC_POINTS, TAU_25C, soc)
    return ro, rtr, tau

def simulate_battery_discharge():
    """Generate realistic battery discharge test data"""

    # Test scenario
    initial_soc = 80.0  # %
    initial_temp = 25.0  # °C
    dt = 1.0  # seconds

    # Discharge profile
    # Phase 1: 0-1800s (30 min) - 0.5C discharge (1.3A)
    # Phase 2: 1800-2400s (10 min) - Rest
    # Phase 3: 2400-4620s (37 min) - 1C discharge (2.6A)
    # Phase 4: 4620-5400s (13 min) - Rest
    # Phase 5: 5400-6600s (20 min) - 0.5C discharge (1.3A)

    times = []
    currents = []
    voltages = []
    temperatures = []
    true_socs = []

    # State variables
    soc = initial_soc
    temp = initial_temp
    v_tr = 0.0  # Transient voltage state

    # Simulation
    t = 0
    while t <= 6600:
        # Determine current based on phase
        if t < 1800:
            current = 1.3  # 0.5C discharge
            temp_rise_rate = 0.001  # °C/s
        elif t < 2400:
            current = 0.0  # Rest
            temp_rise_rate = -0.005  # Cooling
        elif t < 4620:
            current = 2.6  # 1C discharge
            temp_rise_rate = 0.002  # °C/s
        elif t < 5400:
            current = 0.0  # Rest
            temp_rise_rate = -0.002  # Cooling
        else:
            current = 1.3  # 0.5C discharge
            temp_rise_rate = 0.001  # °C/s

        # Update SoC (Coulomb counting)
        delta_soc = (100.0 * current * dt) / (3600.0 * Q_NOMINAL)
        soc = max(0.0, min(100.0, soc - delta_soc))

        # Update temperature
        temp += temp_rise_rate * dt
        temp = max(25.0, min(50.0, temp))

        # Get battery parameters
        ocv = get_ocv(soc, temp)
        ro, rtr, tau = get_resistance(soc, temp)

        # Update transient voltage (first-order RC dynamics)
        # dV_tr/dt = (I * Rtr - V_tr) / tau
        dv_tr = (current * rtr - v_tr) / tau
        v_tr += dv_tr * dt

        # Calculate terminal voltage
        # V_terminal = V_ocv - I * Ro - V_tr
        v_terminal = ocv - current * ro - v_tr

        # Add measurement noise
        v_terminal += random.gauss(0, 0.001)  # 1mV noise

        # Record data (sample every second for first minute, then every 60s)
        if t <= 60 or t % 60 == 0 or abs(t - 1800) < 30 or abs(t - 2400) < 30 or \
           abs(t - 4620) < 30 or abs(t - 5400) < 30:
            times.append(t)
            currents.append(current)
            voltages.append(v_terminal)
            temperatures.append(temp)
            true_socs.append(soc)

        t += dt

    return times, currents, voltages, temperatures, true_socs

def write_csv(filename, times, currents, voltages, temperatures, true_socs):
    """Write test data to CSV file"""
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Time_sec', 'Current_A', 'Voltage_V', 'Temperature_C', 'True_SoC'])
        for i in range(len(times)):
            writer.writerow([
                f'{times[i]:.0f}',
                f'{currents[i]:.3f}',
                f'{voltages[i]:.3f}',
                f'{temperatures[i]:.1f}',
                f'{true_socs[i]:.2f}'
            ])
    print(f"Generated {len(times)} data points in {filename}")

if __name__ == '__main__':
    # Generate test data
    print("Generating realistic battery test data...")
    times, currents, voltages, temperatures, true_socs = simulate_battery_discharge()

    # Write to CSV
    write_csv('battery_test_data.csv', times, currents, voltages, temperatures, true_socs)

    print(f"\nTest profile:")
    print(f"  Initial SoC: {true_socs[0]:.2f}%")
    print(f"  Final SoC: {true_socs[-1]:.2f}%")
    print(f"  Duration: {times[-1]/60:.1f} minutes")
    print(f"  Capacity used: {true_socs[0] - true_socs[-1]:.2f}%")
    print(f"  Expected Ah: {Q_NOMINAL * (true_socs[0] - true_socs[-1]) / 100:.3f} Ah")
