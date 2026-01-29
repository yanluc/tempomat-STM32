import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import least_squares
from scipy.interpolate import interp1d
import glob
import re

# --- KONFIGURACJA STAŁYCH ---
R = 7.5                # Rezystancja [Ohm]
V_SOURCE = 12.0        # Napięcie zasilacza [V]
PWM_MAX = 255          # Maksymalna wartość PWM

# --- ZNANA BEZWŁADNOŚĆ (J) ---
WHEEL_MASS = 0.0695    # kg
WHEEL_RADIUS = 0.05    # m
J_WHEEL = 0.5 * WHEEL_MASS * WHEEL_RADIUS**2
# ----------------------------

# --- NOWA KONFIGURACJA PRÓBKOWANIA ---
TARGET_SAMPLE_RATE = 50.0
DELTA_T = 1.0 / TARGET_SAMPLE_RATE
# ------------------------------------

def resample_and_interpolate_data(t_raw, omega_raw, sample_dt):
    """
    Interpoluje surowe dane pomiarowe do stałego interwału czasowego (sample_dt)
    przy użyciu interpolacji liniowej.
    """
    if len(t_raw) < 2:
        return np.array([]), np.array([])
        
    # 1. Określenie nowego, ujednoliconego wektora czasu
    t_start = t_raw[0]
    t_end = t_raw[-1]
    
    # Tworzymy nowy wektor czasu z równymi krokami
    t_new = np.arange(t_start, t_end, sample_dt)
    
    # Upewniamy się, że ostatni punkt jest uwzględniony
    if t_end not in t_new:
        t_new = np.append(t_new, t_end)
    
    # 2. Tworzenie funkcji interpolującej
    # 'linear' - interpolacja liniowa
    # bounds_error=False - nie rzuca błędu, jeśli t_new wychodzi poza zakres t_raw
    # fill_value="extrapolate" - ekstrapoluje wartości na krańcach
    interpolation_function = interp1d(t_raw, omega_raw, kind='linear', fill_value="extrapolate", bounds_error=False)
    
    # 3. Obliczanie nowych wartości prędkości w ujednoliconych punktach czasowych
    omega_new = interpolation_function(t_new)
    
    return t_new, omega_new

def load_datasets():
    """Wczytuje, normalizuje czas i INTERPOLUJE dane."""
    files = glob.glob("serial_data_*.csv")
    datasets = []
    
    print(f"Znaleziono plików: {len(files)}")
    
    for file in files:
        match = re.search(r'(\d+)', file)
        if not match: continue
        pwm_val = int(match.group(1))
        voltage = (pwm_val / PWM_MAX) * V_SOURCE
        
        try:
            df = pd.read_csv(file)
            t_raw = df['Timestamp'].values
            t_raw = t_raw - t_raw[0]
            omega_raw = df['Value'].values # rad/s
            
            # --- ZASTOSOWANIE INTERPOLACJI ---
            t_new, omega_new = resample_and_interpolate_data(t_raw, omega_raw, DELTA_T)
            
            datasets.append({
                'pwm': pwm_val,
                'voltage': voltage,
                't': t_new,          # Ujednolicony czas
                'omega': omega_new,  # Interpolowana prędkość
                'filename': file
            })
            print(f"  -> Wczytano i interpolowano {file} (Uz={voltage:.2f}V, próbek: {len(t_new)})")
        except Exception as e:
            print(f"  Błąd przy wczytywaniu/interpolacji {file}: {e}")
            
    datasets.sort(key=lambda x: x['pwm'])
    return datasets

def dc_motor_step_response(t, V, k_phi, J_val, b, R_val):
    if (R_val * b + k_phi**2) == 0:
        return np.zeros_like(t)
    denominator = R_val * b + k_phi**2
    tau = (J_val * R_val) / denominator
    omega_steady = (k_phi * V) / denominator
    return omega_steady * (1 - np.exp(-t / tau))

def residuals_function_j_known(params, datasets, R_val, J_val):
    k_phi, b = params
    all_residuals = []
    for data in datasets:
        t = data['t']
        omega_meas = data['omega']
        V = data['voltage']
        omega_model = dc_motor_step_response(t, V, k_phi, J_val, b, R_val)
        diff = omega_meas - omega_model
        all_residuals.extend(diff)
    return np.array(all_residuals)

def calculate_acceleration(U_target, omega_current, k_phi, J_val, b, R_val):
    dynamic_resistance = (k_phi**2 / R_val) + b
    drive_torque = (k_phi * U_target) / R_val
    dot_omega = (1 / J_val) * (drive_torque - omega_current * dynamic_resistance)
    return dot_omega

def calculate_voltage_target(omega_current, dot_omega_target, k_phi, J_val, b, R_val):
    dynamic_resistance_term = omega_current * ((k_phi**2 / R_val) + b)
    inertia_term = J_val * dot_omega_target
    total_torque_req_term = inertia_term + dynamic_resistance_term
    voltage_target = (R_val / k_phi) * total_torque_req_term
    return voltage_target

def main():
    datasets = load_datasets()
    if not datasets:
        return

    # --- OPTYMALIZACJA (Estymacja k_phi i b) ---
    x0 = [0.02, 0.00001]
    bounds = ([0, 0], [np.inf, np.inf])
    res = least_squares(
        residuals_function_j_known, x0, bounds=bounds, 
        args=(datasets, R, J_WHEEL), verbose=0
    )
    k_phi_opt, b_opt = res.x
    
    print("\n" + "="*50)
    print("ZIDENTYFIKOWANE PARAMETRY MODELU")
    print("="*50)
    print(f"Stała silnika (k_fi):       {k_phi_opt:.6f} [V/(rad/s)]")
    print(f"Współczynnik tarcia (b):    {b_opt:.8f} [Nm/(rad/s)]")
    print(f"stała czasowa według wzoru{J_WHEEL/(b_opt+k_phi_opt**2/R)}")
    denom = R * b_opt + k_phi_opt**2
    tau_avg = (J_WHEEL * R) / denom
    print("-" * 50)
    print(f"Średnia stała czasowa (tau): {tau_avg:.4f} s")
    print("="*50)

    # --- DEMONSTRACJA OBLICZEŃ---
    omega_ss = datasets[-1]['omega'][-1] 
    U_test = datasets[-1]['voltage']
    
    print("\n" + "="*50)
    print("ANALIZA NAPIĘCIA ZADANEGO (U_zadane)")
    print("="*50)
    
    dot_omega_req_1 = 10.0
    U_req_1 = calculate_voltage_target(omega_ss, dot_omega_req_1, k_phi_opt, J_WHEEL, b_opt, R)
    print(f"1. Aktualny stan: {omega_ss:.2f} rad/s")
    print(f"   Wymagane przyspieszenie: +{dot_omega_req_1:.1f} rad/s^2")
    print(f"   => Napięcie zadane U_req = {U_req_1:.2f} V")
    
    dot_omega_req_2 = 0.0 
    U_req_2 = calculate_voltage_target(omega_ss, dot_omega_req_2, k_phi_opt, J_WHEEL, b_opt, R)
    print(f"\n2. Aktualny stan: {omega_ss:.2f} rad/s")
    print(f"   Wymagane przyspieszenie: {dot_omega_req_2:.1f} rad/s^2 (Utrzymanie prędkości)")
    print(f"   => Napięcie zadane U_req = {U_req_2:.2f} V (Powinno być ~{U_test:.2f}V)")

    dot_omega_req_3 = -5.0
    U_req_3 = calculate_voltage_target(omega_ss, dot_omega_req_3, k_phi_opt, J_WHEEL, b_opt, R)
    print(f"\n3. Aktualny stan: {omega_ss:.2f} rad/s")
    print(f"   Wymagane przyspieszenie: {dot_omega_req_3:.1f} rad/s^2 (Hamowanie)")
    print(f"   => Napięcie zadane U_req = {U_req_3:.2f} V (Mniejsze niż U_req_2)")
    print("="*50)
    
    # --- WIZUALIZACJA ---
    plt.figure(figsize=(12, 8))
    
    # Wykres 1: Dopasowanie modelu
    plt.subplot(2, 1, 1)
    colors = plt.cm.viridis(np.linspace(0, 1, len(datasets)))
    
    for i, data in enumerate(datasets):
        t = data['t']
        meas = data['omega'] 
        V = data['voltage']
        pwm = data['pwm']
        
        model = dc_motor_step_response(t, V, k_phi_opt, J_WHEEL, b_opt, R)
        
        plt.plot(t, meas, color=colors[i], alpha=0.5, label=f'Pomiar interpolowany {pwm} PWM') 
        plt.plot(t, model, '-', color=colors[i], linewidth=2, label=f'Model {V:.1f}V')

    plt.title('Dopasowanie modelu dynamicznego (J wymuszone) - Dane po interpolacji')
    plt.ylabel('Prędkość [rad/s]')
    plt.xlabel('Czas [s]')
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    plt.grid(True)

    # Wykres 2: Błędy
    plt.subplot(2, 1, 2)
    for i, data in enumerate(datasets):
        t = data['t']
        meas = data['omega']
        V = data['voltage']
        model = dc_motor_step_response(t, V, k_phi_opt, J_WHEEL, b_opt, R)
        plt.plot(t, meas - model, label=f'Błąd PWM {data["pwm"]}')
    plt.title('Błąd dopasowania')
    plt.ylabel('Różnica [rad/s]')
    plt.xlabel('Czas [s]')
    plt.grid(True)
    plt.axhline(0, color='black', linestyle='--')
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()