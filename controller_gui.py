#!/usr/bin/env python3
"""
PS5 Controller GUI for Kart Medulla System
Visualizes controller inputs, steering angles, and PID output
"""

import tkinter as tk
from tkinter import ttk
import serial
import threading
import time
import sys

class ControllerGUI:
    def __init__(self, root, serial_port='/dev/cu.usbserial-0001'):
        self.root = root
        self.root.title("Kart Medulla - PS5 Controller Monitor")
        self.root.geometry("900x600")
        self.root.configure(bg='#1e1e1e')

        # Serial connection
        self.serial_port = serial_port
        self.ser = None
        self.running = False

        # Controller data
        self.data = {
            'lx': 0, 'ly': 0, 'rx': 0, 'ry': 0,
            'l2': 0, 'r2': 0, 'buttons': '0x0000',
            'battery': 0, 'target': 0.0, 'actual': 0.0,
            'error': 0.0, 'pid': 0.0, 'sensor_status': 'UNKNOWN'
        }

        # Update throttling
        self.last_update = 0
        self.update_interval = 0.05  # 50ms = 20 FPS (smooth enough)
        self.pending_update = False

        self.setup_gui()
        self.connect_serial()

    def setup_gui(self):
        # Title
        title = tk.Label(self.root, text="PS5 Controller Monitor",
                        font=('Arial', 24, 'bold'), bg='#1e1e1e', fg='#00ff00')
        title.pack(pady=10)

        # Main container
        main_frame = tk.Frame(self.root, bg='#1e1e1e')
        main_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=10)

        # Left side - Joysticks
        left_frame = tk.Frame(main_frame, bg='#2e2e2e', relief=tk.RAISED, bd=2)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)

        tk.Label(left_frame, text="Joysticks", font=('Arial', 16, 'bold'),
                bg='#2e2e2e', fg='#ffffff').pack(pady=5)

        # Left joystick canvas
        tk.Label(left_frame, text="Left Stick (LX/LY)", bg='#2e2e2e', fg='#aaaaaa').pack()
        self.left_canvas = tk.Canvas(left_frame, width=150, height=150, bg='#1a1a1a', highlightthickness=0)
        self.left_canvas.pack(pady=5)
        self.draw_joystick_base(self.left_canvas)

        # Right joystick canvas
        tk.Label(left_frame, text="Right Stick (RX/RY)", bg='#2e2e2e', fg='#aaaaaa').pack(pady=(10,0))
        self.right_canvas = tk.Canvas(left_frame, width=150, height=150, bg='#1a1a1a', highlightthickness=0)
        self.right_canvas.pack(pady=5)
        self.draw_joystick_base(self.right_canvas)

        # Middle - Triggers and Buttons
        middle_frame = tk.Frame(main_frame, bg='#2e2e2e', relief=tk.RAISED, bd=2)
        middle_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)

        tk.Label(middle_frame, text="Triggers & Buttons", font=('Arial', 16, 'bold'),
                bg='#2e2e2e', fg='#ffffff').pack(pady=5)

        # L2 Trigger
        tk.Label(middle_frame, text="L2 Trigger", bg='#2e2e2e', fg='#aaaaaa').pack(pady=(10,0))
        self.l2_var = tk.StringVar(value="0")
        self.l2_bar = ttk.Progressbar(middle_frame, length=200, mode='determinate', maximum=1023)
        self.l2_bar.pack()
        self.l2_label = tk.Label(middle_frame, textvariable=self.l2_var, bg='#2e2e2e', fg='#00ff00', font=('Arial', 12))
        self.l2_label.pack()

        # R2 Trigger
        tk.Label(middle_frame, text="R2 Trigger", bg='#2e2e2e', fg='#aaaaaa').pack(pady=(10,0))
        self.r2_var = tk.StringVar(value="0")
        self.r2_bar = ttk.Progressbar(middle_frame, length=200, mode='determinate', maximum=1023)
        self.r2_bar.pack()
        self.r2_label = tk.Label(middle_frame, textvariable=self.r2_var, bg='#2e2e2e', fg='#00ff00', font=('Arial', 12))
        self.r2_label.pack()

        # Buttons display
        tk.Label(middle_frame, text="Buttons", bg='#2e2e2e', fg='#aaaaaa').pack(pady=(10,0))
        self.buttons_var = tk.StringVar(value="0x0000")
        tk.Label(middle_frame, textvariable=self.buttons_var, bg='#2e2e2e', fg='#ffff00',
                font=('Courier', 14, 'bold')).pack()

        # Battery
        tk.Label(middle_frame, text="Battery", bg='#2e2e2e', fg='#aaaaaa').pack(pady=(10,0))
        self.battery_var = tk.StringVar(value="0")
        tk.Label(middle_frame, textvariable=self.battery_var, bg='#2e2e2e', fg='#00ffff',
                font=('Arial', 18, 'bold')).pack()

        # Right side - Steering
        right_frame = tk.Frame(main_frame, bg='#2e2e2e', relief=tk.RAISED, bd=2)
        right_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)

        tk.Label(right_frame, text="Steering Control", font=('Arial', 16, 'bold'),
                bg='#2e2e2e', fg='#ffffff').pack(pady=5)

        # Steering gauge
        self.steering_canvas = tk.Canvas(right_frame, width=250, height=200, bg='#1a1a1a', highlightthickness=0)
        self.steering_canvas.pack(pady=10)

        # Steering values
        info_frame = tk.Frame(right_frame, bg='#2e2e2e')
        info_frame.pack(pady=5)

        tk.Label(info_frame, text="Target:", bg='#2e2e2e', fg='#aaaaaa', font=('Arial', 10)).grid(row=0, column=0, sticky='e', padx=5)
        self.target_var = tk.StringVar(value="0.0°")
        tk.Label(info_frame, textvariable=self.target_var, bg='#2e2e2e', fg='#00ff00', font=('Arial', 12, 'bold')).grid(row=0, column=1, sticky='w')

        tk.Label(info_frame, text="Actual:", bg='#2e2e2e', fg='#aaaaaa', font=('Arial', 10)).grid(row=1, column=0, sticky='e', padx=5)
        self.actual_var = tk.StringVar(value="0.0°")
        tk.Label(info_frame, textvariable=self.actual_var, bg='#2e2e2e', fg='#00ffff', font=('Arial', 12, 'bold')).grid(row=1, column=1, sticky='w')

        tk.Label(info_frame, text="Error:", bg='#2e2e2e', fg='#aaaaaa', font=('Arial', 10)).grid(row=2, column=0, sticky='e', padx=5)
        self.error_var = tk.StringVar(value="0.0°")
        tk.Label(info_frame, textvariable=self.error_var, bg='#2e2e2e', fg='#ff6600', font=('Arial', 12, 'bold')).grid(row=2, column=1, sticky='w')

        tk.Label(info_frame, text="PID:", bg='#2e2e2e', fg='#aaaaaa', font=('Arial', 10)).grid(row=3, column=0, sticky='e', padx=5)
        self.pid_var = tk.StringVar(value="0.000")
        tk.Label(info_frame, textvariable=self.pid_var, bg='#2e2e2e', fg='#ff00ff', font=('Arial', 12, 'bold')).grid(row=3, column=1, sticky='w')

        # Sensor status
        self.sensor_var = tk.StringVar(value="NO_SENSOR")
        sensor_label = tk.Label(right_frame, textvariable=self.sensor_var, bg='#2e2e2e',
                               fg='#ff6600', font=('Arial', 10))
        sensor_label.pack(pady=5)

        # Status bar
        self.status_var = tk.StringVar(value="Connecting...")
        status_bar = tk.Label(self.root, textvariable=self.status_var, bg='#333333', fg='#00ff00',
                             font=('Arial', 10), anchor='w', relief=tk.SUNKEN)
        status_bar.pack(side=tk.BOTTOM, fill=tk.X)

    def draw_joystick_base(self, canvas):
        """Draw joystick background"""
        canvas.create_oval(10, 10, 140, 140, fill='#333333', outline='#555555', width=2)
        canvas.create_line(75, 20, 75, 130, fill='#444444', dash=(2, 2))
        canvas.create_line(20, 75, 130, 75, fill='#444444', dash=(2, 2))

    def update_joystick(self, canvas, x, y):
        """Update joystick indicator position"""
        # Map controller values to canvas coordinates
        # x: -511 to 512, y: -511 to 512
        canvas_x = 75 + (x / 511.0) * 50
        canvas_y = 75 + (y / 511.0) * 50

        # Clear previous indicator
        canvas.delete('indicator')

        # Draw new indicator
        r = 15
        canvas.create_oval(canvas_x - r, canvas_y - r, canvas_x + r, canvas_y + r,
                          fill='#00ff00', outline='#00aa00', width=2, tags='indicator')

    def draw_steering_gauge(self, target, actual):
        """Draw steering angle gauge"""
        canvas = self.steering_canvas

        # Only delete dynamic elements, not the background
        canvas.delete('needle')

        # Draw background (only once, tagged 'static')
        if not hasattr(self, '_gauge_drawn'):
            canvas.create_arc(25, 50, 225, 250, start=0, extent=180,
                             fill='#333333', outline='#555555', width=2, style=tk.PIESLICE, tags='static')

            # Draw angle markers (static)
            for angle in [-45, -30, -15, 0, 15, 30, 45]:
                import math
                rad = math.radians(90 + angle)
                x1 = 125 + 90 * math.cos(rad)
                y1 = 150 - 90 * math.sin(rad)
                x2 = 125 + 100 * math.cos(rad)
                y2 = 150 - 100 * math.sin(rad)
                canvas.create_line(x1, y1, x2, y2, fill='#666666', width=1, tags='static')
                canvas.create_text(125 + 110 * math.cos(rad),
                                 150 - 110 * math.sin(rad),
                                 text=f"{angle}°", fill='#888888', font=('Arial', 8), tags='static')

            canvas.create_oval(120, 145, 130, 155, fill='#ffffff', tags='static')
            self._gauge_drawn = True

        # Draw target angle (green) - dynamic
        import math
        target_rad = math.radians(90 + target)
        target_x = 125 + 80 * math.cos(target_rad)
        target_y = 150 - 80 * math.sin(target_rad)
        canvas.create_line(125, 150, target_x, target_y, fill='#00ff00', width=3, arrow=tk.LAST, tags='needle')

        # Draw actual angle (cyan) - dynamic
        actual_rad = math.radians(90 + actual)
        actual_x = 125 + 70 * math.cos(actual_rad)
        actual_y = 150 - 70 * math.sin(actual_rad)
        canvas.create_line(125, 150, actual_x, actual_y, fill='#00ffff', width=2, arrow=tk.LAST, tags='needle')

    def connect_serial(self):
        """Connect to ESP32 serial port"""
        try:
            self.ser = serial.Serial(self.serial_port, 115200, timeout=0.1)
            self.ser.reset_input_buffer()
            self.running = True
            self.status_var.set(f"Connected to {self.serial_port}")

            # Start reading thread
            self.read_thread = threading.Thread(target=self.read_serial, daemon=True)
            self.read_thread.start()

        except Exception as e:
            self.status_var.set(f"ERROR: {e}")
            self.root.after(2000, self.connect_serial)  # Retry

    def read_serial(self):
        """Read serial data in background thread"""
        while self.running:
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line.startswith('DATA,'):
                        self.parse_data(line)
            except Exception as e:
                print(f"Read error: {e}")
            time.sleep(0.005)  # Faster read loop

    def parse_data(self, line):
        """Parse CSV data line"""
        try:
            parts = line.split(',')
            if len(parts) >= 14:
                self.data['lx'] = int(parts[1])
                self.data['ly'] = int(parts[2])
                self.data['rx'] = int(parts[3])
                self.data['ry'] = int(parts[4])
                self.data['l2'] = int(parts[5])
                self.data['r2'] = int(parts[6])
                self.data['buttons'] = parts[7]
                self.data['battery'] = int(parts[8])
                self.data['target'] = float(parts[10])
                self.data['actual'] = float(parts[11])
                self.data['error'] = float(parts[12])
                self.data['pid'] = float(parts[13])
                self.data['sensor_status'] = parts[14] if len(parts) > 14 else 'UNKNOWN'

                # Throttle GUI updates to prevent lag
                current_time = time.time()
                if not self.pending_update and (current_time - self.last_update) >= self.update_interval:
                    self.pending_update = True
                    self.last_update = current_time
                    self.root.after(0, self.update_display)
        except Exception as e:
            print(f"Parse error: {e}")

    def update_display(self):
        """Update all GUI elements"""
        try:
            # Joysticks
            self.update_joystick(self.left_canvas, self.data['lx'], self.data['ly'])
            self.update_joystick(self.right_canvas, self.data['rx'], self.data['ry'])

            # Triggers
            self.l2_bar['value'] = self.data['l2']
            self.l2_var.set(f"{self.data['l2']}")
            self.r2_bar['value'] = self.data['r2']
            self.r2_var.set(f"{self.data['r2']}")

            # Buttons and battery
            self.buttons_var.set(self.data['buttons'])
            self.battery_var.set(f"{self.data['battery']}")

            # Steering
            self.target_var.set(f"{self.data['target']:.1f}°")
            self.actual_var.set(f"{self.data['actual']:.1f}°")
            self.error_var.set(f"{self.data['error']:.2f}°")
            self.pid_var.set(f"{self.data['pid']:.3f}")
            self.sensor_var.set(self.data['sensor_status'])

            # Steering gauge
            self.draw_steering_gauge(self.data['target'], self.data['actual'])
        finally:
            self.pending_update = False

    def close(self):
        """Clean up on exit"""
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()

def main():
    # Check if port specified
    port = '/dev/cu.usbserial-0001'
    if len(sys.argv) > 1:
        port = sys.argv[1]

    root = tk.Tk()
    app = ControllerGUI(root, port)

    def on_closing():
        app.close()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()

if __name__ == "__main__":
    main()
