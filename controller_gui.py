#!/usr/bin/env python3
"""
PS5 Controller GUI Monitor
Displays controller inputs in a graphical window
"""

import serial
import tkinter as tk
from tkinter import ttk
import re
import threading
import time

class PS5ControllerGUI:
    def __init__(self, root, port='/dev/cu.usbserial-0001', baud=115200):
        self.root = root
        self.root.title("PS5 Controller Monitor")
        self.root.geometry("800x600")
        self.root.configure(bg='#1a1a1a')

        # Serial connection
        self.serial_port = port
        self.baud_rate = baud
        self.ser = None
        self.running = True

        # Controller state
        self.state = {
            'lstick_x': 0,
            'lstick_y': 0,
            'rstick_x': 0,
            'rstick_y': 0,
            'l2': 0,
            'r2': 0,
            'buttons': set(),
            'connected': False,
            'battery': 0,
            'charging': False
        }

        # Debug info
        self.debug_lines = []
        self.last_serial_time = time.time()

        self.setup_gui()
        self.start_serial_thread()

    def setup_gui(self):
        # Main container
        main_frame = tk.Frame(self.root, bg='#1a1a1a')
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Title
        title = tk.Label(main_frame, text="PS5 CONTROLLER MONITOR",
                        font=('Courier', 20, 'bold'),
                        bg='#1a1a1a', fg='#00ff00')
        title.pack(pady=10)

        # Connection status
        self.status_label = tk.Label(main_frame, text="⏳ CONNECTING...",
                                     font=('Courier', 12),
                                     bg='#1a1a1a', fg='#ffaa00')
        self.status_label.pack(pady=5)

        # Create canvas for drawing
        self.canvas = tk.Canvas(main_frame, width=760, height=500,
                               bg='#2a2a2a', highlightthickness=2,
                               highlightbackground='#00ff00')
        self.canvas.pack(pady=10)

        # Start update loop
        self.update_display()

    def draw_stick(self, x, y, stick_x, stick_y, label):
        """Draw an analog stick visualization"""
        radius = 60
        center_x = x + radius
        center_y = y + radius

        # Outer circle (bounds)
        self.canvas.create_oval(x, y, x + radius*2, y + radius*2,
                               outline='#444444', width=2)

        # Center crosshair
        self.canvas.create_line(center_x - 10, center_y, center_x + 10, center_y,
                               fill='#666666', width=1)
        self.canvas.create_line(center_x, center_y - 10, center_x, center_y + 10,
                               fill='#666666', width=1)

        # Stick position (map -128 to 127 to radius)
        stick_pos_x = center_x + (stick_x / 128.0) * (radius - 10)
        stick_pos_y = center_y - (stick_y / 128.0) * (radius - 10)  # Inverted Y for correct visualization

        # Draw stick circle
        stick_r = 15
        self.canvas.create_oval(stick_pos_x - stick_r, stick_pos_y - stick_r,
                               stick_pos_x + stick_r, stick_pos_y + stick_r,
                               fill='#00ff00', outline='#00aa00', width=2)

        # Label
        self.canvas.create_text(center_x, y + radius*2 + 20,
                               text=label, fill='#aaaaaa',
                               font=('Courier', 12, 'bold'))

        # Values
        self.canvas.create_text(center_x, y + radius*2 + 40,
                               text=f"X:{stick_x:4d} Y:{stick_y:4d}",
                               fill='#666666', font=('Courier', 10))

    def draw_trigger(self, x, y, value, label):
        """Draw trigger bar"""
        width = 150
        height = 30

        # Background
        self.canvas.create_rectangle(x, y, x + width, y + height,
                                     outline='#444444', width=2)

        # Fill bar
        fill_width = int((value / 255.0) * width)
        if fill_width > 0:
            self.canvas.create_rectangle(x, y, x + fill_width, y + height,
                                         fill='#00ff00', outline='')

        # Label
        self.canvas.create_text(x - 30, y + height/2,
                               text=label, fill='#aaaaaa',
                               font=('Courier', 12, 'bold'))

        # Value
        self.canvas.create_text(x + width + 40, y + height/2,
                               text=f"{value:3d}",
                               fill='#00ff00', font=('Courier', 12))

    def draw_button(self, x, y, label, pressed):
        """Draw a button"""
        size = 50
        color = '#00ff00' if pressed else '#333333'
        outline = '#00aa00' if pressed else '#555555'

        self.canvas.create_rectangle(x, y, x + size, y + size,
                                     fill=color, outline=outline, width=2)

        self.canvas.create_text(x + size/2, y + size/2,
                               text=label, fill='#000000' if pressed else '#666666',
                               font=('Courier', 10, 'bold'))

    def draw_dpad_button(self, x, y, w, h, label, pressed):
        """Draw D-pad button"""
        color = '#00ff00' if pressed else '#333333'
        outline = '#00aa00' if pressed else '#555555'

        self.canvas.create_rectangle(x, y, x + w, y + h,
                                     fill=color, outline=outline, width=2)

        if label:
            self.canvas.create_text(x + w/2, y + h/2,
                                   text=label, fill='#000000' if pressed else '#666666',
                                   font=('Courier', 8, 'bold'))

    def update_display(self):
        """Update the canvas display"""
        self.canvas.delete('all')

        if not self.state['connected']:
            self.canvas.create_text(380, 150,
                                   text="⏳ WAITING FOR CONTROLLER...",
                                   fill='#666666', font=('Courier', 16, 'bold'))

            # Show debug info
            self.canvas.create_text(380, 200,
                                   text="SERIAL DEBUG (last 10 lines):",
                                   fill='#00ff00', font=('Courier', 10, 'bold'))

            # Time since last data
            time_since = int(time.time() - self.last_serial_time)
            self.canvas.create_text(380, 220,
                                   text=f"Last data: {time_since}s ago",
                                   fill='#ffaa00', font=('Courier', 9))

            # Show recent serial lines
            y_pos = 250
            for i, line in enumerate(self.debug_lines[-10:]):
                color = '#aaaaaa' if i < len(self.debug_lines)-1 else '#00ff00'
                # Truncate long lines
                display_line = line[:80] + "..." if len(line) > 80 else line
                self.canvas.create_text(380, y_pos,
                                       text=display_line,
                                       fill=color, font=('Courier', 8))
                y_pos += 20

            # Instructions
            self.canvas.create_text(380, 450,
                                   text="Press PS button to wake controller",
                                   fill='#666666', font=('Courier', 10))

            self.root.after(100, self.update_display)
            return

        # Left stick
        self.draw_stick(50, 50, self.state['lstick_x'], self.state['lstick_y'],
                       "LEFT STICK")

        # Right stick
        self.draw_stick(250, 50, self.state['rstick_x'], self.state['rstick_y'],
                       "RIGHT STICK")

        # Triggers
        self.draw_trigger(480, 60, self.state['l2'], "L2")
        self.draw_trigger(480, 110, self.state['r2'], "R2")

        # Face buttons (Diamond layout)
        btn_x = 500
        btn_y = 200
        self.draw_button(btn_x + 50, btn_y, "△", "TRIANGLE" in self.state['buttons'])
        self.draw_button(btn_x + 100, btn_y + 50, "○", "CIRCLE" in self.state['buttons'])
        self.draw_button(btn_x + 50, btn_y + 100, "✕", "CROSS" in self.state['buttons'])
        self.draw_button(btn_x, btn_y + 50, "□", "SQUARE" in self.state['buttons'])

        # D-Pad (Cross layout)
        dpad_x = 50
        dpad_y = 230
        dpad_size = 30
        self.draw_dpad_button(dpad_x + dpad_size, dpad_y, dpad_size, dpad_size,
                             "▲", "UP" in self.state['buttons'])
        self.draw_dpad_button(dpad_x + dpad_size*2, dpad_y + dpad_size, dpad_size, dpad_size,
                             "►", "RIGHT" in self.state['buttons'])
        self.draw_dpad_button(dpad_x + dpad_size, dpad_y + dpad_size*2, dpad_size, dpad_size,
                             "▼", "DOWN" in self.state['buttons'])
        self.draw_dpad_button(dpad_x, dpad_y + dpad_size, dpad_size, dpad_size,
                             "◄", "LEFT" in self.state['buttons'])

        # Shoulder buttons
        shoulder_y = 380
        self.canvas.create_text(100, shoulder_y - 20, text="SHOULDER BUTTONS",
                               fill='#aaaaaa', font=('Courier', 10, 'bold'))

        small_btn_size = 40
        self.draw_button(40, shoulder_y, "L1", "L1" in self.state['buttons'])
        self.draw_button(100, shoulder_y, "L2", "L2" in self.state['buttons'])
        self.draw_button(160, shoulder_y, "R1", "R1" in self.state['buttons'])
        self.draw_button(220, shoulder_y, "R2", "R2" in self.state['buttons'])

        # Special buttons
        special_y = 450
        self.canvas.create_text(100, special_y - 20, text="SPECIAL",
                               fill='#aaaaaa', font=('Courier', 10, 'bold'))

        spec_size = 35
        self.draw_button(20, special_y, "PS", "PS" in self.state['buttons'])
        self.draw_button(70, special_y, "SHR", "SHARE" in self.state['buttons'])
        self.draw_button(120, special_y, "OPT", "OPTIONS" in self.state['buttons'])

        # Numeric display of key values
        info_x = 450
        info_y = 380
        self.canvas.create_text(info_x, info_y, text="INPUT VALUES",
                               fill='#00ff00', font=('Courier', 14, 'bold'),
                               anchor='w')

        info_y += 30
        self.canvas.create_text(info_x, info_y,
                               text=f"L-Stick: X={self.state['lstick_x']:4d}  Y={self.state['lstick_y']:4d}",
                               fill='#aaaaaa', font=('Courier', 11),
                               anchor='w')

        info_y += 25
        self.canvas.create_text(info_x, info_y,
                               text=f"R-Stick: X={self.state['rstick_x']:4d}  Y={self.state['rstick_y']:4d}",
                               fill='#aaaaaa', font=('Courier', 11),
                               anchor='w')

        info_y += 25
        self.canvas.create_text(info_x, info_y,
                               text=f"L2={self.state['l2']:3d}  R2={self.state['r2']:3d}",
                               fill='#aaaaaa', font=('Courier', 11),
                               anchor='w')

        info_y += 25
        charging = "⚡" if self.state.get('charging', False) else ""
        battery_color = '#00ff00' if self.state.get('battery', 0) > 50 else '#ffaa00' if self.state.get('battery', 0) > 20 else '#ff0000'
        self.canvas.create_text(info_x, info_y,
                               text=f"Battery: {self.state.get('battery', 0)}% {charging}",
                               fill=battery_color, font=('Courier', 11, 'bold'),
                               anchor='w')

        self.root.after(20, self.update_display)  # 50 Hz update rate for smooth visuals

    def parse_serial_line(self, line):
        """Parse CSV serial output and update state

        Format:
        - DATA,LX,LY,RX,RY,L2,R2,BUTTONS,BATTERY,CHARGING,AUDIO,MIC
        - HB,uptime_sec,connected (0/1)
        """
        line = line.strip()

        # Update debug info
        if line:
            self.debug_lines.append(line)
            if len(self.debug_lines) > 50:  # Keep last 50 lines
                self.debug_lines.pop(0)
            self.last_serial_time = time.time()

        # Parse CSV data
        if line.startswith("DATA,"):
            try:
                parts = line.split(",")
                if len(parts) >= 12:
                    # Analog sticks
                    self.state['lstick_x'] = int(parts[1])
                    self.state['lstick_y'] = int(parts[2])
                    self.state['rstick_x'] = int(parts[3])
                    self.state['rstick_y'] = int(parts[4])

                    # Triggers
                    self.state['l2'] = int(parts[5])
                    self.state['r2'] = int(parts[6])

                    # Buttons - clear and repopulate
                    self.state['buttons'].clear()
                    if parts[7]:  # If button string is not empty
                        button_list = parts[7].split("|")
                        for btn in button_list:
                            self.state['buttons'].add(btn)

                    # Battery and charging status
                    # Battery value appears to be inverted (0-100 becomes 100-0)
                    raw_battery = int(parts[8])
                    self.state['battery'] = 100 - raw_battery  # Correct inversion
                    self.state['charging'] = parts[9] == "1"

                    # Mark as connected when we receive data
                    if not self.state['connected']:
                        self.state['connected'] = True
                        self.status_label.config(text="✓ CONNECTED", fg='#00ff00')
            except (ValueError, IndexError) as e:
                pass  # Ignore malformed lines

        # Parse heartbeat
        elif line.startswith("HB,"):
            try:
                parts = line.split(",")
                if len(parts) >= 3:
                    connected = parts[2] == "1"
                    if not connected and self.state['connected']:
                        # Controller disconnected
                        self.state['connected'] = False
                        self.status_label.config(text="✗ DISCONNECTED", fg='#ff0000')
            except (ValueError, IndexError):
                pass

        # Legacy messages for connection status
        elif "CONNECTED!" in line or "CONTROLLER CONNECTED" in line:
            self.state['connected'] = True
            self.status_label.config(text="✓ CONNECTED", fg='#00ff00')

        elif "DISCONNECTED" in line or "CONTROLLER DISCONNECTED" in line:
            self.state['connected'] = False
            self.status_label.config(text="✗ DISCONNECTED", fg='#ff0000')

    def serial_reader(self):
        """Background thread to read serial data"""
        try:
            print(f"[DEBUG] Opening serial port: {self.serial_port}")
            # Open serial port WITHOUT toggling DTR/RTS to avoid resetting ESP32
            self.ser = serial.Serial(
                self.serial_port,
                self.baud_rate,
                timeout=1,
                dsrdtr=False,  # Don't toggle DTR (prevents ESP32 reset)
                rtscts=False   # Don't use hardware flow control
            )
            print("[DEBUG] Serial port opened successfully")
            time.sleep(0.5)
            self.ser.reset_input_buffer()

            packet_count = 0
            while self.running:
                if self.ser.in_waiting > 0:
                    try:
                        line = self.ser.readline().decode('utf-8', errors='ignore')
                        if 'DATA,' in line:
                            packet_count += 1
                            if packet_count == 1 or packet_count % 100 == 0:
                                print(f"[DEBUG] Received {packet_count} data packets")
                        self.parse_serial_line(line)
                    except Exception as e:
                        print(f"[DEBUG] Parse error: {e}")
                time.sleep(0.001)  # Minimal delay for faster serial reads

        except Exception as e:
            print(f"Serial error: {e}")
        finally:
            if self.ser:
                self.ser.close()

    def start_serial_thread(self):
        """Start background thread for serial reading"""
        thread = threading.Thread(target=self.serial_reader, daemon=True)
        thread.start()

    def on_closing(self):
        """Clean up on window close"""
        self.running = False
        time.sleep(0.2)
        if self.ser:
            self.ser.close()
        self.root.destroy()

if __name__ == '__main__':
    root = tk.Tk()
    app = PS5ControllerGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()
