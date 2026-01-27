import tkinter as tk
from tkinter import ttk
import serial
import threading
import collections
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import time
import re

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200
HISTORY_SIZE = 100

class MotorControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("STM32 Velocity Control")
        
        self.times = collections.deque(maxlen=HISTORY_SIZE)
        self.velocities = collections.deque(maxlen=HISTORY_SIZE)
        self.setpoints = collections.deque(maxlen=HISTORY_SIZE)
        self.start_time = time.time()
        
        self.current_setpoint = 5.0
        self.actual_velocity = 0.0
        self.is_running = True
        
        try:
            self.ser = serial.Serial(
                port=SERIAL_PORT,
                baudrate=BAUD_RATE,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
        except Exception as e:
            print(f"Serial Error: {e}")
            self.ser = None

        self.setup_ui()
        
        self.thread = threading.Thread(target=self.serial_reader, daemon=True)
        self.thread.start()
        
        self.update_ui()

    def setup_ui(self):
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        ctrl_frame = ttk.LabelFrame(main_frame, text="Controls", padding="10")
        ctrl_frame.grid(row=0, column=0, sticky=(tk.N, tk.S), padx=(0, 10))

        ttk.Label(ctrl_frame, text="Set Velocity (rad/s):").grid(row=0, column=0, pady=(0, 5))
        
        self.setpoint_label = ttk.Label(ctrl_frame, text="0.0")
        self.setpoint_label.grid(row=1, column=0)

        self.scale = ttk.Scale(ctrl_frame, from_=-300, to=300, orient=tk.VERTICAL, length=250, command=self.on_scale_change)
        self.scale.set(0) # Now label exists
        self.scale.grid(row=2, column=0, pady=10)

        ttk.Button(ctrl_frame, text="STOP", command=lambda: self.scale.set(0)).grid(row=3, column=0, pady=10)

        # Stats Frame
        stats_frame = ttk.LabelFrame(ctrl_frame, text="Statistics", padding="5")
        stats_frame.grid(row=4, column=0, sticky=(tk.W, tk.E), pady=10)

        ttk.Label(stats_frame, text="Abs Error:").grid(row=0, column=0, sticky=tk.W)
        self.abs_err_label = ttk.Label(stats_frame, text="0.0")
        self.abs_err_label.grid(row=0, column=1, sticky=tk.E)

        ttk.Label(stats_frame, text="Rel Error:").grid(row=1, column=0, sticky=tk.W)
        self.rel_err_label = ttk.Label(stats_frame, text="0.0 %")
        self.rel_err_label.grid(row=1, column=1, sticky=tk.E)

        plot_frame = ttk.Frame(main_frame)
        plot_frame.grid(row=0, column=1, sticky=(tk.W, tk.E, tk.N, tk.S))

        self.fig, self.ax = plt.subplots(figsize=(8, 4), dpi=100)
        self.ax.set_title("Motor Velocity")
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Velocity (rad/s)")
        self.ax.grid(True)
        
        self.line_vel, = self.ax.plot([], [], 'b-', label="Actual")
        self.line_set, = self.ax.plot([], [], 'r--', label="Setpoint")
        self.ax.legend(loc='upper right')

        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def on_scale_change(self, value):
        val = float(value)
        self.current_setpoint = val
        if hasattr(self, 'setpoint_label'):
            self.setpoint_label.config(text=f"{val:.2f}")
        if self.ser:
            try:
                self.ser.write(f"{val:.2f}\n".encode('ascii'))
            except:
                pass

    def serial_reader(self):
        while self.is_running:
            if self.ser and self.ser.in_waiting:
                try:
                    line = self.ser.readline().decode('ascii', errors='replace').strip()
                    # Pattern: V: 5.00 rad/s, Err: 0.00 rad/s...
                    match = re.search(r'V:\s*([-+]?\d*\.\d+|\d+)', line)
                    if match:
                        self.actual_velocity = float(match.group(1))
                        curr_time = time.time() - self.start_time
                        
                        self.times.append(curr_time)
                        self.velocities.append(self.actual_velocity)
                        self.setpoints.append(self.current_setpoint)
                except Exception as e:
                    print(f"Read Error: {e}")
            time.sleep(0.005)

    def update_ui(self):
        if len(self.times) > 0:
            self.line_vel.set_data(list(self.times), list(self.velocities))
            self.line_set.set_data(list(self.times), list(self.setpoints))
            
            self.ax.set_xlim(min(self.times), max(self.times))
            
            all_vals = list(self.velocities) + list(self.setpoints)
            if all_vals:
                vmin, vmax = min(all_vals), max(all_vals)
                margin = max(5.0, (vmax - vmin) * 0.1)
                self.ax.set_ylim(vmin - margin, vmax + margin)
            
            self.canvas.draw()
            
            abs_err = abs(self.current_setpoint - self.actual_velocity)
            self.abs_err_label.config(text=f"{abs_err:.2f}")
            
            if abs(self.current_setpoint) > 0.01:
                rel_err = (abs_err / abs(self.current_setpoint)) * 100.0
                self.rel_err_label.config(text=f"{rel_err:.1f} %")
            else:
                self.rel_err_label.config(text="N/A")
            
        self.root.after(50, self.update_ui)

    def on_closing(self):
        self.is_running = False
        if self.ser:
            self.ser.close()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = MotorControlGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()
