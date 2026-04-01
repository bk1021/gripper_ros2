#!/usr/bin/env python3

import argparse
import sys
import serial
import time
import struct
import threading
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import csv
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from collections import deque

# --- Enums matching your C++ code ---
GRIPPER_STATES = {
    0: "IDLE",
    1: "OPENING",
    2: "APPROACHING",
    3: "GRASPING",
    4: "FAULT",
    5: "CALIBRATING_FORCE",
    6: "MANUAL_TUNE",
    7: "ZEROING_MOTOR",
    8: "SAVING_FLASH",
    9: "ABORTED"
}

FAULT_REASONS = {
    0: "NONE",
    1: "MECH_LIMIT_OPEN",
    2: "MECH_LIMIT_CLOSE",
    3: "MOTOR_MODE_ERROR",
    4: "CAN_TX_JAMMED",
    5: "MOTOR_RX_TIMEOUT",
    6: "REALTIME_DEADLINE_MISSED",
    7: "MOTOR_HARDWARE_ERROR"
}

# --- Parameter ID Mappings ---
CONFIG_PARAMS = {
    "Kp": 0, "Ki": 1, "Kd": 2, "Max Integral": 3, "Max Torque": 4,
    "Open Pos": 5, "Close Pos": 6, "Approach Limit": 7,
    "Open Kp": 8, "Open Kd": 9, "Approach Kd": 10, "Approach Vel": 11,
    "Deadband": 12, "LPF Alpha": 13, "Contact Thresh": 14,
    "Telemetry(ms)": 18, "Virtual Mass": 19, "Virtual Damping": 20, 
    "Admit Max Vel": 21, "Admit Kd": 22, "Control Strategy": 23,
    "Contact Cond": 24, "Calib Limit": 25  # <--- NEW PARAMS ADDED HERE
}
READ_ONLY_PARAMS = {
    "PC3 Zero ADC": 15, "PC2 Zero ADC": 16
}

# --- SERIAL CONTROLLER (BACKEND THREAD) ---
class GripperControllerSerial:
    def __init__(self, port='COM5', baudrate=921600):
        self.ser = serial.Serial(port, baudrate, timeout=0)
        self.ser.reset_input_buffer()
        
        self.telemetry = {
            "state": "UNKNOWN",
            "fault": "UNKNOWN",
            "pc3_grams": 0.0,
            "pc2_grams": 0.0,
            "engine_cmd": 0.0,
            "position": 0.0,
            "velocity": 0.0,
            "torque": 0.0
        }
        
        self.live_config = {}
        
        # --- NEW: Thread-Safe Graph Buffers (500 pts = 5 seconds @ 10ms) ---
        self.max_pts = 500
        self.data_pc3 = deque([0]*self.max_pts, maxlen=self.max_pts)
        self.data_pc2 = deque([0]*self.max_pts, maxlen=self.max_pts)
        self.data_avg = deque([0]*self.max_pts, maxlen=self.max_pts)
        self.data_cmd = deque([0]*self.max_pts, maxlen=self.max_pts)
        self.data_target = deque([100.0]*self.max_pts, maxlen=self.max_pts)

        self.is_recording = False
        self.record_start_time = 0.0
        self.recorded_data = []
        self.target_force_log = 100.0

        self.running = True
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()

    def _rx_loop(self):
        while self.running:
            if self.ser.in_waiting >= 17:
                byte = self.ser.read(1)
                if byte == b'\xAA':
                    packet_rest = self.ser.read(16)
                    if len(packet_rest) == 16:
                        packet = byte + packet_rest
                        if packet[4:8] == b'\x00\x00\x01\x00':
                            self._parse_telemetry(packet)
                        elif packet[4:8] == b'\x00\x00\x01\x01':
                            self._parse_motor_data(packet)
                        elif packet[4:8] == b'\x00\x00\x01\x02':
                            self._parse_config_data(packet)
            else:
                time.sleep(0.001)

    def _parse_telemetry(self, packet):
        data = packet[8:16]
        state_code = data[0]
        self.telemetry["state"] = GRIPPER_STATES.get(state_code, f"UNKNOWN({state_code})")
        fault_code = data[1]
        self.telemetry["fault"] = FAULT_REASONS.get(fault_code, f"UNKNOWN({fault_code})")
        
        pc3_raw, pc2_raw, cmd_raw = struct.unpack_from('>hhh', data, offset=2)
        self.telemetry["pc3_grams"] = pc3_raw / 10.0
        self.telemetry["pc2_grams"] = pc2_raw / 10.0
        self.telemetry["engine_cmd"] = cmd_raw / 1000.0

        # --- NEW: Direct Deque Injection (Zero Latency Graph Update) ---
        self.data_pc3.append(self.telemetry["pc3_grams"])
        self.data_pc2.append(self.telemetry["pc2_grams"])
        avg_force = (self.telemetry["pc3_grams"] + self.telemetry["pc2_grams"]) / 2.0
        self.data_avg.append(avg_force)
        self.data_cmd.append(self.telemetry["engine_cmd"])
        self.data_target.append(self.target_force_log)

        # CSV Logging
        if self.is_recording:
            rel_time = time.time() - self.record_start_time
            self.recorded_data.append([
                round(rel_time, 4), self.telemetry["state"], self.target_force_log, 
                self.telemetry["pc3_grams"], self.telemetry["pc2_grams"], avg_force, 
                self.telemetry["engine_cmd"], self.telemetry["position"], 
                self.telemetry["velocity"], self.telemetry["torque"]
            ])

    def _parse_motor_data(self, packet):
        data = packet[8:16]
        pos_raw, vel_raw, trq_raw = struct.unpack_from('>ihh', data, offset=0)
        self.telemetry["position"] = pos_raw / 10000.0
        self.telemetry["velocity"] = vel_raw / 1000.0
        self.telemetry["torque"] = trq_raw / 1000.0

    def _parse_config_data(self, packet):
        data = packet[8:16]
        if data[0] == 0x06: 
            param_id = data[1]
            val = struct.unpack_from('>f', data, offset=2)[0]
            self.live_config[param_id] = val

    def _send_can_message(self, can_id: int, data_bytes: list):
        dlc = len(data_bytes)
        padded_data = data_bytes + [0x00] * (8 - dlc)
        id3, id2, id1, id0 = (can_id >> 24) & 0xFF, (can_id >> 16) & 0xFF, (can_id >> 8) & 0xFF, can_id & 0xFF
        frame = [0xAA, 0x00, 0x00, dlc, id3, id2, id1, id0] + padded_data + [0x7A]
        self.ser.write(bytes(frame))

    def clear_fault(self):
        self._send_can_message(0x200, [0x01])

    def set_mode(self, mode: int):
        self._send_can_message(0x200, [0x02, mode & 0xFF])

    def set_target_force(self, force_grams: float):
        force_int = int(max(0, min(force_grams, 65535)))
        high_byte = (force_int >> 8) & 0xFF
        low_byte  = force_int & 0xFF
        self._send_can_message(0x200, [0x03, high_byte, low_byte])

    def set_tuning_param(self, param_id: int, value: float):
        packed_float = struct.pack('>f', float(value)) 
        payload = [0x04, param_id, packed_float[0], packed_float[1], packed_float[2], packed_float[3]]
        self._send_can_message(0x200, payload)
        
    def request_parameter(self, param_id: int):
        self._send_can_message(0x200, [0x06, param_id])
        
    def set_manual_position(self, target_pos: float):
        packed_float = struct.pack('>f', float(target_pos))
        payload = [0x05, packed_float[0], packed_float[1], packed_float[2], packed_float[3]]
        self._send_can_message(0x200, payload)

    def close(self):
        self.running = False
        self.rx_thread.join()
        self.ser.close()


# --- GUI APPLICATION (FRONTEND THREAD) ---
class GripperGUI:
    def __init__(self, root, controller):
        self.root = root
        self.root.title("Gripper Control & Telemetry")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.controller = controller

        self.confirmed_target = 100.0
        self.param_entries = {} 

        self.setup_ui()
        self.setup_plots()
        
        # Start the Slow UI Renderer Loop (50ms)
        self.render_gui_loop()

        # Full Startup Fetch 
        self.root.after(500, self.cmd_fetch_config)

    def setup_ui(self):
        control_frame = ttk.Frame(self.root, padding="10")
        control_frame.pack(side=tk.LEFT, fill=tk.Y)

        # Live Status
        status_lf = ttk.LabelFrame(control_frame, text="Live Status", padding="10")
        status_lf.pack(fill=tk.X, pady=5)
        self.lbl_state = ttk.Label(status_lf, text="State: --", font=("Helvetica", 12, "bold"))
        self.lbl_state.pack(anchor=tk.W, pady=2)
        self.lbl_fault = ttk.Label(status_lf, text="Fault: --", foreground="red", font=("Helvetica", 10))
        self.lbl_fault.pack(anchor=tk.W, pady=2)

        ttk.Separator(status_lf, orient='horizontal').pack(fill=tk.X, pady=5)
        self.lbl_pos = ttk.Label(status_lf, text="Position: 0.000 rad")
        self.lbl_pos.pack(anchor=tk.W)
        self.lbl_vel = ttk.Label(status_lf, text="Velocity: 0.000 rad/s")
        self.lbl_vel.pack(anchor=tk.W)
        self.lbl_trq = ttk.Label(status_lf, text="Torque: 0.000 N.m")
        self.lbl_trq.pack(anchor=tk.W)

        # Commands
        cmd_lf = ttk.LabelFrame(control_frame, text="Commands", padding="10")
        cmd_lf.pack(fill=tk.X, pady=5)
        
        ttk.Button(cmd_lf, text="Clear Faults", command=self.controller.clear_fault).grid(row=0, column=0, padx=2, pady=2, sticky=tk.EW)
        
        ttk.Button(cmd_lf, text="Zero Motor Pos", command=lambda: self.controller.set_mode(7)).grid(row=0, column=1, padx=2, pady=2, sticky=tk.EW)
        ttk.Button(cmd_lf, text="Auto-Calibrate Force", command=lambda: self.controller.set_mode(5)).grid(row=1, column=0, columnspan=2, padx=2, pady=2, sticky=tk.EW)
        
        ttk.Separator(cmd_lf, orient='horizontal').grid(row=2, column=0, columnspan=2, pady=5, sticky=tk.EW)
        
        ttk.Button(cmd_lf, text="Idle / Relax", command=lambda: self.controller.set_mode(0)).grid(row=3, column=0, padx=2, pady=2, sticky=tk.EW)
        ttk.Button(cmd_lf, text="Open Gripper", command=lambda: self.controller.set_mode(1)).grid(row=3, column=1, padx=2, pady=2, sticky=tk.EW)
        
        ttk.Button(cmd_lf, text="Grasp (PID)", command=self.cmd_grasp_pid).grid(row=4, column=0, padx=2, pady=2, sticky=tk.EW)
        ttk.Button(cmd_lf, text="Grasp (Admit)", command=self.cmd_grasp_admit).grid(row=4, column=1, padx=2, pady=2, sticky=tk.EW)

        # Target Force
        force_lf = ttk.LabelFrame(control_frame, text="Target Force (Grams)", padding="10")
        force_lf.pack(fill=tk.X, pady=5)
        self.lbl_actual_force = ttk.Label(force_lf, text="Actual Target: -- g", font=("Helvetica", 10, "bold"), foreground="#006300")
        self.lbl_actual_force.pack(side=tk.TOP, anchor=tk.W, pady=(0, 5))
        self.force_var = tk.DoubleVar(value=150.0)
        self.force_spin = ttk.Spinbox(force_lf, from_=0, to=3000, increment=10, textvariable=self.force_var, width=10)
        self.force_spin.pack(side=tk.LEFT, padx=5)
        ttk.Button(force_lf, text="Set Force", command=self.send_force).pack(side=tk.LEFT)
        
        # Manual Position Tuning
        manual_lf = ttk.LabelFrame(control_frame, text="Manual Position Tune", padding="10")
        manual_lf.pack(fill=tk.X, pady=5)
        ttk.Button(manual_lf, text="Enter Tune Mode", command=self.cmd_enter_manual_tune).pack(fill=tk.X, pady=2)
  
        self.pos_var = tk.DoubleVar(value=0.0)
        pos_row = ttk.Frame(manual_lf)
        pos_row.pack(fill=tk.X, pady=2)
        ttk.Scale(pos_row, from_=1.0, to=-4.0, orient=tk.HORIZONTAL, variable=self.pos_var).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=5)  
        ttk.Entry(pos_row, textvariable=self.pos_var, width=8).pack(side=tk.LEFT, padx=5) 
        ttk.Button(pos_row, text="Step", command=self.send_manual_step, width=6).pack(side=tk.RIGHT)

        # Data Logger
        log_lf = ttk.LabelFrame(control_frame, text="Data Logger", padding="10")
        log_lf.pack(fill=tk.X, pady=5)
        self.btn_record = ttk.Button(log_lf, text="Start Recording", command=self.toggle_recording)
        self.btn_record.pack(fill=tk.X, pady=2)
        ttk.Button(log_lf, text="Save to CSV", command=self.save_csv).pack(fill=tk.X, pady=2)
        self.lbl_log_status = ttk.Label(log_lf, text="Ready", foreground="gray")
        self.lbl_log_status.pack(pady=2)

        # Config Dashboard
        config_frame = ttk.Frame(self.root, padding="10")
        config_frame.pack(side=tk.LEFT, fill=tk.Y)
        
        tune_lf = ttk.LabelFrame(config_frame, text="Live Tuning & Configuration", padding="10")
        tune_lf.pack(fill=tk.BOTH, expand=True)
        
        ttk.Button(tune_lf, text="Fetch Config from STM32", command=self.cmd_fetch_config).grid(row=0, column=0, columnspan=2, pady=5, padx=2, sticky=tk.EW)
        
        ttk.Button(tune_lf, text="Save to STM32 Flash", command=lambda: self.controller.set_mode(8)).grid(row=0, column=2, pady=5, padx=2, sticky=tk.EW)

        row_idx = 1
        for name, param_id in list(CONFIG_PARAMS.items()) + list(READ_ONLY_PARAMS.items()):
            ttk.Label(tune_lf, text=f"{name}:").grid(row=row_idx, column=0, sticky=tk.E, padx=2, pady=2)
            ent = ttk.Entry(tune_lf, width=10)
            ent.grid(row=row_idx, column=1, padx=2, pady=2)
            self.param_entries[param_id] = ent
            
            if name in READ_ONLY_PARAMS:
                ent.config(state='readonly')
            else:
                btn = ttk.Button(tune_lf, text="Set", width=4, command=lambda pid=param_id, e=ent: self.cmd_set_param(pid, e))
                btn.grid(row=row_idx, column=2, padx=2, pady=2, sticky=tk.W)
            row_idx += 1

    def setup_plots(self):
        plot_frame = ttk.Frame(self.root)
        plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        plt.style.use('dark_background')
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(8, 6), dpi=100)
        self.fig.tight_layout(pad=3.0)

        # Ax1: Forces (Pointed directly to Controller's Deques)
        self.line_pc3, = self.ax1.plot(self.controller.data_pc3, label='Sensor 1 (PC3)', color='#00FF00', lw=1.0, alpha=0.6)
        self.line_pc2, = self.ax1.plot(self.controller.data_pc2, label='Sensor 2 (PC2)', color='#00BCFF', lw=1.0, alpha=0.6)
        self.line_avg, = self.ax1.plot(self.controller.data_avg, label='Average Force', color='#FFFF00', lw=2.0, linestyle='-')
        self.line_target, = self.ax1.plot(self.controller.data_target, label='Target Force', color='#FF4444', lw=2.0, linestyle=':')

        self.ax1.set_title("Real-Time Grip Force")
        self.ax1.set_ylabel("Grams")
        self.ax1.set_ylim(-10, 500)
        self.ax1.legend(loc="upper left")
        self.ax1.grid(True, alpha=0.3)

        # Ax2: Engine Command Output (Pointed directly to Controller's Deque)
        self.line_cmd, = self.ax2.plot(self.controller.data_cmd, label='Active Engine Command', color='#FF00FF', lw=1.5)
        self.ax2.set_title("Physics Engine Command Output")
        self.ax2.set_ylabel("Command (Nm or rad/s)")
        
        self.ax2.set_ylim(-3.0, 3.0)
        self.ax2.legend(loc="upper left")
        self.ax2.grid(True, alpha=0.3)

        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    # --- Smart Grasp Strategy Builders ---
    def cmd_grasp_pid(self):
        self.controller.set_tuning_param(23, 0.0)
        self.root.after(20, lambda: self.controller.set_mode(2))
        self.root.after(100, lambda: self.controller.request_parameter(23))

    def cmd_grasp_admit(self):
        self.controller.set_tuning_param(23, 1.0)
        self.root.after(20, lambda: self.controller.set_mode(2))
        self.root.after(100, lambda: self.controller.request_parameter(23))

    def send_force(self):
        try:
            val = float(self.force_var.get())
            self.controller.set_target_force(val)
            self.root.after(50, lambda: self.controller.request_parameter(17))
        except ValueError:
            messagebox.showerror("Error", "Invalid force value.")

    def cmd_set_param(self, param_id, entry_widget):
        try:
            val = float(entry_widget.get())
            self.controller.set_tuning_param(param_id, val)
            self.root.after(50, lambda: self.controller.request_parameter(param_id))
        except ValueError:
            messagebox.showerror("Error", "Invalid numeric value.")
            
    def cmd_fetch_config(self):
        for i, param_id in enumerate(list(CONFIG_PARAMS.values()) + list(READ_ONLY_PARAMS.values())):
            self.root.after(i * 10, lambda pid=param_id: self.controller.request_parameter(pid))
        self.root.after(len(CONFIG_PARAMS) * 10 + 50, lambda: self.controller.request_parameter(17))

    def cmd_enter_manual_tune(self):
        current_pos = self.controller.telemetry['position']
        self.pos_var.set(round(current_pos, 3))
        self.controller.set_mode(6)
            
    def send_manual_step(self):
        self.controller.set_manual_position(float(self.pos_var.get()))

    def toggle_recording(self):
        if not self.controller.is_recording:
            self.controller.recorded_data = [] 
            self.controller.target_force_log = self.confirmed_target 
            self.controller.record_start_time = time.time()
            self.controller.is_recording = True
            self.btn_record.config(text="Stop Recording")
            self.lbl_log_status.config(text="Recording...", foreground="red")
        else:
            self.controller.is_recording = False
            self.btn_record.config(text="Start Recording")
            pts = len(self.controller.recorded_data)
            self.lbl_log_status.config(text=f"Stopped ({pts} pts)", foreground="green")

    def save_csv(self):
        if self.controller.is_recording:
            messagebox.showwarning("Warning", "Stop recording before saving!")
            return
        if not self.controller.recorded_data:
            messagebox.showinfo("Info", "No data to save.")
            return

        timestamp = time.strftime("%Y%m%d_%H%M%S")
        unique_filename = f"engine_analysis_{timestamp}.csv"

        filepath = filedialog.asksaveasfilename(
            defaultextension=".csv", 
            filetypes=[("CSV Files", "*.csv")],
            initialfile=unique_filename
        )
        if filepath:
            with open(filepath, mode='w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(["Time_Sec", "State", "Target_Grams", "PC3_Grams", "PC2_Grams", 
                                 "Avg_Grams", "Engine_Cmd", "Position_Rad", "Velocity_RadS", "Torque_Nm"])
                writer.writerows(self.controller.recorded_data)
            messagebox.showinfo("Saved", f"Successfully saved {len(self.controller.recorded_data)} lines to CSV.")

    # --- Decoupled GUI Renderer ---
    def render_gui_loop(self):
        # SLOW LOOP: 20FPS (50ms) UI Updates and Plot Rendering
        t = self.controller.telemetry
        self.lbl_state.config(text=f"State: {t['state']}")
        self.lbl_fault.config(text=f"Fault: {t['fault']}")
        
        if t['fault'] != "NONE":
            self.lbl_fault.config(foreground="red")
        else:
            self.lbl_fault.config(foreground="green")

        self.lbl_pos.config(text=f"Position: {t['position']:.3f} rad")
        self.lbl_vel.config(text=f"Velocity: {t['velocity']:.3f} rad/s")
        self.lbl_trq.config(text=f"Torque: {t['torque']:.3f} N.m")

        # Sync GUI Target Force Variable
        if 17 in self.controller.live_config:
            self.confirmed_target = self.controller.live_config.pop(17)
            self.controller.target_force_log = self.confirmed_target
            self.lbl_actual_force.config(text=f"Actual Target: {self.confirmed_target:.1f} g")

        # Sync Text Entries
        for param_id, val in self.controller.live_config.items():
            if param_id in self.param_entries:
                ent = self.param_entries[param_id]
                is_readonly = str(ent['state']) == 'readonly'
                if is_readonly: ent.config(state='normal')
                ent.delete(0, tk.END)
                ent.insert(0, f"{val:.5f}")
                if is_readonly: ent.config(state='readonly')
        self.controller.live_config.clear()

        # Recording Status
        if self.controller.is_recording:
            pts = len(self.controller.recorded_data)
            if pts % 50 == 0:
                self.lbl_log_status.config(text=f"Recording ({pts} pts)")

        # Push the synchronized hardware deques to the Matplotlib lines
        self.line_pc3.set_ydata(self.controller.data_pc3)
        self.line_pc2.set_ydata(self.controller.data_pc2)
        self.line_avg.set_ydata(self.controller.data_avg)
        self.line_target.set_ydata(self.controller.data_target)
        self.line_cmd.set_ydata(self.controller.data_cmd)

        # Dynamic Y-Axis limits
        max_force = max(max(self.controller.data_pc3), max(self.controller.data_pc2), self.confirmed_target)
        if max_force > self.ax1.get_ylim()[1] - 50:
            self.ax1.set_ylim(-10, max_force + 100)
        elif max_force < 400 and self.ax1.get_ylim()[1] > 500:
            self.ax1.set_ylim(-10, 500)

        # Draw the frame and schedule the next one
        self.canvas.draw_idle()
        self.root.after(50, self.render_gui_loop)

    def on_closing(self):
        self.controller.set_mode(0)
        time.sleep(0.1)
        self.controller.close()
        self.root.destroy()
        self.root.quit()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Gripper Serial Control & Telemetry GUI")
    parser.add_argument("port", type=str, nargs='?', default="/dev/ttyUSB0", help="Serial port (Default: /dev/ttyUSB0)")
    parser.add_argument("baud", type=int, nargs='?', default=921600, help="Baud rate (Default: 921600)")
    args = parser.parse_args()

    if len(sys.argv) < 2:
        print(f"[WARNING] No port specified. Defaulting to: {args.port}")
    if len(sys.argv) < 3:
        print(f"[WARNING] No baud rate specified. Defaulting to: {args.baud}")
    print("-" * 40)

    try:
        gripper = GripperControllerSerial(port=args.port, baudrate=args.baud)
        root = tk.Tk()
        app = GripperGUI(root, gripper)
        root.mainloop()
    except serial.SerialException as e:
        print(f"Failed to connect on {args.port}: {e}")
        print("\nLinux Pro-Tip: If you get a 'Permission denied' error, run:")
        print(f"  sudo chmod a+rw {args.port}")
        print("  Or permanently fix it by adding your user to the 'dialout' group:\n  sudo usermod -a -G dialout $USER\n")
        