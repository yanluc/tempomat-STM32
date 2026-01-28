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
STEP_VALUE = 100.0  # rad/s
DURATION = 5.0      # seconds
STABILIZE_TIME = 10.0 # seconds before step
OUTPUT_DIR = os.path.dirname(os.path.abspath(__file__))

def measure_step_response():
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
    setpoints = []
    outputs = []
    
    print(f"Stabilizing at 0 rad/s for {STABILIZE_TIME}s...")
    ser.write(b"0.0\n")
    start_wait = time.time()
    while time.time() - start_wait < STABILIZE_TIME:
        ser.readline() # Flush buffer
    
    print(f"Applying step to {STEP_VALUE} rad/s...")
    ser.write(f"{STEP_VALUE:.2f}\n".encode('ascii'))
    
    start_time = time.time()
    try:
        while True:
            elapsed = time.time() - start_time
            if elapsed > DURATION:
                break
            
            line = ser.readline().decode('ascii', errors='replace').strip()
            if not line:
                continue
                
            # Pattern: V: 5.00 rad/s, Err: 0.00 rad/s, Out: 5.00V
            match = re.search(r'V:\s*([-+]?\d*\.\d+|\d+).*Out:\s*([-+]?\d*\.\d+|\d+)', line)
            if match:
                v = float(match.group(1))
                out = float(match.group(2))
                times.append(elapsed)
                velocities.append(v)
                setpoints.append(STEP_VALUE)
                outputs.append(out)
                
    except KeyboardInterrupt:
        print("Measurement interrupted.")
    finally:
        ser.write(b"0.0\n") # Stop motor
        ser.close()

    if not times:
        print("No data collected.")
        return

    # Process Results
    times = np.array(times)
    velocities = np.array(velocities)
    
    # Calculate Steady State (average of last 10% of samples)
    ss_index = int(len(velocities) * 0.9)
    steady_state = np.mean(velocities[ss_index:])
    
    # Peak value for overshoot
    peak_value = np.max(velocities)
    overshoot = max(0, (peak_value - steady_state) / abs(steady_state) * 100) if abs(steady_state) > 1e-3 else 0
    
    # Settling Time (2% criterion)
    tolerance = 0.02 * abs(steady_state) if abs(steady_state) > 1e-3 else 0.02 * STEP_VALUE
    
    # Find last time signal was outside the tolerance band
    outside_indices = np.where(np.abs(velocities - steady_state) > tolerance)[0]
    if len(outside_indices) > 0:
        settling_time = times[outside_indices[-1]]
    else:
        settling_time = 0

    print("\n--- Step Response Analysis ---")
    print(f"Steady State:    {steady_state:.2f} rad/s")
    print(f"Peak Value:      {peak_value:.2f} rad/s")
    print(f"Overshoot:       {overshoot:.2f} %")
    print(f"Settling Time:   {settling_time:.2f} s")
    print("------------------------------")

    # Save to CSV
    csv_path = os.path.join(OUTPUT_DIR, "step_response.csv")
    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Time (s)', 'Velocity (rad/s)', 'Setpoint (rad/s)', 'Control Signal (V)'])
        for row in zip(times, velocities, setpoints, outputs):
            writer.writerow(row)
    print(f"Data saved to {csv_path}")

    # Plot
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    
    ax1.plot(times, velocities, 'b-', label='Actual Velocity')
    ax1.axhline(y=STEP_VALUE, color='r', linestyle='--', label='Setpoint')
    ax1.axhline(y=steady_state, color='g', linestyle=':', label='Steady State')
    ax1.set_ylabel('Velocity (rad/s)')
    ax1.set_title(f'Step Response Analysis (Step: {STEP_VALUE} rad/s)')
    ax1.grid(True)
    ax1.legend()
    
    # Annotate metrics
    textstr = '\n'.join((
        f'Overshoot: {overshoot:.2f}%',
        f'Settling Time: {settling_time:.2f}s',
        f'Steady State: {steady_state:.2f} rad/s'
    ))
    props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
    ax1.text(0.05, 0.95, textstr, transform=ax1.transAxes, fontsize=10,
            verticalalignment='top', bbox=props)

    ax2.plot(times, outputs, 'k-', label='Control Signal')
    ax2.set_ylabel('Output Voltage (V)')
    ax2.set_xlabel('Time (s)')
    ax2.grid(True)
    ax2.legend()

    plt.tight_layout()
    png_path = os.path.join(OUTPUT_DIR, "step_response.png")
    plt.savefig(png_path)
    print(f"Plot saved to {png_path}")
    plt.show()

if __name__ == "__main__":
    measure_step_response()
