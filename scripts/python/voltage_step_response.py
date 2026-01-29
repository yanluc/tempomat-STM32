import serial
import time
import re
import matplotlib.pyplot as plt
import numpy as np
import csv
import os

# Configuration
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200
VOLTAGE_STEP = 10.0  # Volts
DURATION = 20.0      # seconds
STABILIZE_TIME = 10.0 
OUTPUT_DIR = os.path.dirname(os.path.abspath(__file__))

def measure_voltage_step():
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
    times = []
    velocities = []
    voltages = []
    
    print(f"Stabilizing at 0 rad/s for {STABILIZE_TIME}s...")
    ser.write(b"0.0\n")
    start_wait = time.time()
    while time.time() - start_wait < STABILIZE_TIME:
        ser.readline() # Flush buffer
    
    print(f"Applying voltage step: {VOLTAGE_STEP}V...")
    ser.write(f"U{VOLTAGE_STEP:.2f}\n".encode('ascii'))
    
    start_time = time.time()
    try:
        while True:
            elapsed = time.time() - start_time
            if elapsed > DURATION:
                break
            
            line = ser.readline().decode('ascii', errors='replace').strip()
            if not line:
                continue
                
            match = re.search(r'V:\s*([-+]?\d*\.\d+|\d+).*Out:\s*([-+]?\d*\.\d+|\d+)', line)
            if match:
                v = float(match.group(1))
                out = float(match.group(2))
                times.append(elapsed)
                velocities.append(v)
                voltages.append(out)
                
    except KeyboardInterrupt:
        print("Measurement interrupted.")
    finally:
        ser.write(b"U0.0\n") # Stop motor
        ser.close()

    if not times:
        print("No data collected.")
        return

    # Process Results
    times = np.array(times)
    velocities = np.array(velocities)
    
    # Calculate Steady State (average of last 10%)
    ss_index = int(len(velocities) * 0.9)
    v_ss = np.mean(velocities[ss_index:])
    
    # Calculate Gain K = v_ss / VOLTAGE_STEP
    gain_k = v_ss / VOLTAGE_STEP
    
    # Calculate Time Constant (Time to reach 63.2% of steady state)
    target_v = 0.632 * v_ss
    idx_tau = np.where(velocities >= target_v)[0]
    if len(idx_tau) > 0:
        tau = times[idx_tau[0]]
    else:
        tau = 0

    print("\n--- Motor Characteristics (Open Loop) ---")
    print(f"Steady State Velocity: {v_ss:.2f} rad/s")
    print(f"Motor Gain (K):        {gain_k:.2f} rad/(s*V)")
    print(f"Time Constant (tau):   {tau:.3f} s")
    print("-----------------------------------------")

    # Save to CSV
    csv_path = os.path.join(OUTPUT_DIR, "voltage_step.csv")
    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Time (s)', 'Velocity (rad/s)', 'Voltage (V)'])
        for row in zip(times, velocities, voltages):
            writer.writerow(row)

    # Plot
    fig, ax1 = plt.subplots(figsize=(10, 6))
    ax1.plot(times, velocities, 'b-', label='Velocity')
    ax1.axhline(y=v_ss, color='g', linestyle=':', label='Steady State')
    ax1.axhline(y=target_v, color='r', linestyle='--', label='63.2% SS')
    ax1.axvline(x=tau, color='r', linestyle='--', alpha=0.5)
    
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Velocity (rad/s)', color='b')
    ax1.set_title(f'Open Loop Voltage Step response ({VOLTAGE_STEP}V)')
    ax1.grid(True)
    ax1.legend(loc='lower right')

    # Annotate metrics
    textstr = '\n'.join((
        f'K: {gain_k:.2f} rad/(s*V)',
        f'tau: {tau:.3f} s',
        f'V_ss: {v_ss:.2f} rad/s'
    ))
    props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
    ax1.text(0.05, 0.95, textstr, transform=ax1.transAxes, fontsize=12,
            verticalalignment='top', bbox=props)

    plt.tight_layout()
    png_path = os.path.join(OUTPUT_DIR, "voltage_step.png")
    plt.savefig(png_path)
    print(f"Plot saved to {png_path}")
    plt.show()

if __name__ == "__main__":
    measure_voltage_step()
