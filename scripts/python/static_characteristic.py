import serial
import time
import re
import matplotlib.pyplot as plt
import numpy as np
import csv
import os

# Configuration
SERIAL_PORT = '/dev/ttyACM0' # Sprawdz czy to właściwy port
BAUD_RATE = 115200
VOLTAGE_RANGE = np.linspace(-12.0, 12.0, 25) # -12V, -11V, ..., 12V
STABILIZE_TIME = 5.0  # seconds to wait for steady state
MEASURE_TIME = 2.0    # seconds to average measurements
OUTPUT_DIR = os.path.dirname(os.path.abspath(__file__))

def measure_static_characteristic():
    try:
        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUD_RATE,
            timeout=0.1
        )
        print(f"Connected to {SERIAL_PORT}")
    except Exception as e:
        print(f"Error: {e}")
        return

    # Data storage
    measured_voltages = []
    measured_velocities = []
    
    print("Starting static characteristic measurement (U = f(omega))...")
    
    try:
        for u in VOLTAGE_RANGE:
            print(f"Applying {u:.2f}V...", end=" ", flush=True)
            ser.write(f"U{u:.2f}\n".encode('ascii'))
            
            # 1. Wait for stabilization
            time.sleep(STABILIZE_TIME)
            
            # 2. Measure and average
            buffer_velocities = []
            start_measure = time.time()
            
            while time.time() - start_measure < MEASURE_TIME:
                line = ser.readline().decode('ascii', errors='replace').strip()
                if not line:
                    continue
                
                # Match typical output format: V: 1.23 rad/s ... Out: 1.23V
                match = re.search(r'V:\s*([-+]?\d*\.\d+|\d+)', line)
                if match:
                    v = float(match.group(1))
                    buffer_velocities.append(v)
            
            if buffer_velocities:
                avg_v = np.mean(buffer_velocities)
                measured_voltages.append(u)
                measured_velocities.append(avg_v)
                print(f"Done. Avg Velocity: {avg_v:.2f} rad/s")
            else:
                print("Failed (no data).")
                
    except KeyboardInterrupt:
        print("\nMeasurement interrupted by user.")
    finally:
        ser.write(b"U0.0\n") # Stop motor
        ser.close()

    if not measured_voltages:
        print("No data collected.")
        return

    # --- DATA ANALYSIS & CALCULATION ---
    u_data = np.array(measured_voltages)
    w_data = np.array(measured_velocities)

    # Filtracja: bierzemy pod uwagę tylko punkty, gdzie silnik faktycznie się kręci
    # (ignorujemy strefę martwą, żeby nie zaniżać nachylenia)
    velocity_threshold = 1.0 # rad/s
    active_mask = np.abs(w_data) > velocity_threshold
    
    if np.sum(active_mask) > 1:
        # Dopasowanie liniowe y = ax + b dla aktywnych punktów
        # a = K_stat (slope), b = intercept
        slope, intercept = np.polyfit(u_data[active_mask], w_data[active_mask], 1)
        
        K_stat = slope
        
        print("\n" + "="*40)
        print(" CALCULATED PARAMETERS")
        print("="*40)
        print(f"Static Gain (K_stat): {K_stat:.4f} [rad/s / V]")
        print(f"Linear Fit Equation:  w = {slope:.2f}*U + {intercept:.2f}")
        print("="*40 + "\n")
    else:
        print("\nNot enough data points in motion to calculate gain accurately.")
        slope, intercept = 0, 0
        K_stat = 0

    # Save to CSV
    csv_path = os.path.join(OUTPUT_DIR, "static_characteristic.csv")
    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Voltage (V)', 'Velocity (rad/s)', 'Calculated K_stat', str(K_stat)])
        for row in zip(u_data, w_data):
            writer.writerow(row)
    print(f"Data saved to {csv_path}")

    # --- PLOTTING ---
    plt.figure(figsize=(10, 6))
    
    # Characteristic omega = f(U)
    plt.plot(u_data, w_data, 'ro-', linewidth=2, label='Measured Data')
    
    # Linear fit visualization
    if K_stat != 0:
        # Rysujemy linię trendu dla całego zakresu napięć
        fit_line = slope * u_data + intercept
        plt.plot(u_data, fit_line, 'b--', alpha=0.6, 
                 label=f'Linear Fit (K={K_stat:.3f})')
        
        # Wyświetlenie wzoru na wykresie
        plt.text(0, np.max(w_data)*0.8, 
                 f'$K_{{stat}} = {K_stat:.2f}$ rad/s/V', 
                 fontsize=12, bbox=dict(facecolor='white', alpha=0.8))
    
    plt.title(f'Static Characteristic (Calc. Gain K={K_stat:.2f})')
    plt.xlabel('Voltage U [V]')
    plt.ylabel('Angular Velocity $\omega$ [rad/s]')
    plt.grid(True, which='both', linestyle='--', alpha=0.7)
    plt.legend()
    
    # Save Plot
    png_path = os.path.join(OUTPUT_DIR, "static_characteristic.png")
    plt.savefig(png_path)
    print(f"Plot saved to {png_path}")
    
    plt.show()

if __name__ == "__main__":
    measure_static_characteristic()