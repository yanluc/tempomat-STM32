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
HISTORY_SIZE = 1500
WINDOW_LENGTH = 20.0

class MotorControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("STM32 Velocity Control")
        
        self.times = collections.deque(maxlen=HISTORY_SIZE)
        self.velocities = collections.deque(maxlen=HISTORY_SIZE)
        self.setpoints = collections.deque(maxlen=HISTORY_SIZE)
        self.outputs = collections.deque(maxlen=HISTORY_SIZE)
        self.start_time = time.time()
        
        self.current_setpoint = 0.0
        self.actual_velocity = 0.0
        self.control_signal = 0.0
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

        self.scale = ttk.Scale(ctrl_frame, from_=300, to=-300, orient=tk.VERTICAL, length=250, command=self.on_scale_change)
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

        ttk.Label(stats_frame, text="Control (V):").grid(row=2, column=0, sticky=tk.W)
        self.control_label = ttk.Label(stats_frame, text="0.0")
        self.control_label.grid(row=2, column=1, sticky=tk.E)

        plot_frame = ttk.Frame(main_frame)
        plot_frame.grid(row=0, column=1, sticky=(tk.W, tk.E, tk.N, tk.S))

        # Create two subplots: one for Velocity, one for Control Signal
        self.fig, (self.ax_vel, self.ax_ctrl) = plt.subplots(2, 1, figsize=(8, 6), dpi=100, sharex=True)
        self.fig.tight_layout(pad=3.0)
        
        # Velocity Axis
        self.ax_vel.set_title("Motor Velocity")
        self.ax_vel.set_ylabel("Velocity (rad/s)")
        self.ax_vel.grid(True)
        self.line_vel, = self.ax_vel.plot([], [], 'b-', label="Actual")
        self.line_set, = self.ax_vel.plot([], [], 'r--', label="Setpoint")
        self.ax_vel.legend(loc='upper right')
        
        # Control Signal Axis
        self.ax_ctrl.set_title("Control Signal")
        self.ax_ctrl.set_xlabel("Time (s)")
        self.ax_ctrl.set_ylabel("Control (V)")
        self.ax_ctrl.grid(True)
        self.line_ctrl, = self.ax_ctrl.plot([], [], 'g-', label="Voltage[V]")
        self.ax_ctrl.legend(loc='upper right')

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
                    # Pattern: V: 5.00 rad/s, Err: 0.00 rad/s, Out: 5.00V
                    match = re.search(r'V:\s*([-+]?\d*\.\d+|\d+).*Out:\s*([-+]?\d*\.\d+|\d+)', line)
                    if match:
                        self.actual_velocity = float(match.group(1))
                        self.control_signal = float(match.group(2))
                        curr_time = time.time() - self.start_time
                        
                        self.times.append(curr_time)
                        self.velocities.append(self.actual_velocity)
                        self.setpoints.append(self.current_setpoint)
                        self.outputs.append(self.control_signal)
                except Exception as e:
                    print(f"Read Error: {e}")
            time.sleep(0.005)

    def update_ui(self):
        if len(self.times) > 0:
            t_list = list(self.times)
            
            # Update Velocity Plot
            self.line_vel.set_data(t_list, list(self.velocities))
            self.line_set.set_data(t_list, list(self.setpoints))
            
            # Update Control Plot
            self.line_ctrl.set_data(t_list, list(self.outputs))
            
            # Set constant 20s window
            last_t = t_list[-1]
            if last_t < WINDOW_LENGTH:
                self.ax_vel.set_xlim(0, WINDOW_LENGTH)
            else:
                self.ax_vel.set_xlim(last_t - WINDOW_LENGTH, last_t)
            
            # Velocity Y scaling
            all_vel_vals = list(self.velocities) + list(self.setpoints)
            if all_vel_vals:
                vmin, vmax = min(all_vel_vals), max(all_vel_vals)
                margin = max(5.0, (vmax - vmin) * 0.1)
                self.ax_vel.set_ylim(vmin - margin, vmax + margin)
            
            # Control Y scaling
            ctrl_vals = list(self.outputs)
            if ctrl_vals:
                cvmin, cvmax = min(ctrl_vals), max(ctrl_vals)
                margin = max(1.0, (cvmax - cvmin) * 0.2)
                self.ax_ctrl.set_ylim(cvmin - margin, cvmax + margin)
            
            self.canvas.draw()
            
            abs_err = abs(self.current_setpoint - self.actual_velocity)
            self.abs_err_label.config(text=f"{abs_err:.2f}")
            self.control_label.config(text=f"{self.control_signal:.2f}")
            
            if abs(self.current_setpoint) > 0.01:
                rel_err = (abs_err / abs(self.current_setpoint)) * 100.0
                self.rel_err_label.config(text=f"{rel_err:.1f} %")
            else:
                rel_err = 0.0
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
