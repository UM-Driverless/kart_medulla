#!/usr/bin/env python3
import serial
import struct
import sys
import time

def crc8(length, msg_type, payload):
    crc = 0x00
    crc ^= length
    for _ in range(8): crc = (crc << 1) ^ 0x07 if (crc & 0x80) else (crc << 1); crc &= 0xFF
    crc ^= msg_type
    for _ in range(8): crc = (crc << 1) ^ 0x07 if (crc & 0x80) else (crc << 1); crc &= 0xFF
    for b in payload:
        crc ^= b
        for _ in range(8): crc = (crc << 1) ^ 0x07 if (crc & 0x80) else (crc << 1); crc &= 0xFF
    return crc

def main():
    port = '/dev/cu.usbmodem5C372070281'
    baud = 115200
    try:
        ser = serial.Serial()
        ser.port = port
        ser.baudrate = baud
        ser.timeout = 0.5
        # Prevent ESP32 reset
        ser.setDTR(False)
        ser.setRTS(False)
        ser.open()
        # Explicitly clear DTR/RTS again just in case
        ser.dtr = False
        ser.rts = False
    except Exception as e:
        print(f"Failed to open {port}: {e}")
        return

    print(f"Listening on {port} at {baud} baud (DTR/RTS False)...")
    
    SOF = 0xAA
    
    while True:
        try:
            b = ser.read(1)
            if not b: continue
            if b[0] != SOF: continue
            
            b_len = ser.read(1)
            if not b_len: continue
            length = b_len[0]
            
            b_type = ser.read(1)
            if not b_type: continue
            msg_type = b_type[0]
            
            payload = b''
            if length > 0:
                payload = ser.read(length)
                if len(payload) != length: continue
                
            b_crc = ser.read(1)
            if not b_crc: continue
            msg_crc = b_crc[0]
            
            calc_crc = crc8(length, msg_type, payload)
            if calc_crc != msg_crc:
                continue
                
            if msg_type == 0x04: # ESP_ACT_STEERING
                if length == 12 or length == 16:
                    if length == 16:
                        val1, val2, val3, pressure = struct.unpack('>iiii', payload)
                    else:
                        val1, val2, val3 = struct.unpack('>iii', payload)
                        pressure = -1
                    angle_rad = val1 / 1000.0
                    raw = val2
                    pid = val3 / 1000.0
                    print(f"[STEERING/PRESSURE] angle: {angle_rad:.3f} rad, raw: {raw}, pid: {pid:.3f}, pres_adc: {pressure}")
            elif msg_type == 0x0B: # ESP_HEALTH_STATUS
                if length == 16:
                    flags, agc, heap_kb, errors = struct.unpack('>iiii', payload)
                    magnet_ok = bool(flags & 1)
                    i2c_ok = bool(flags & 2)
                    print(f"[HEALTH] flags: {flags:02x} (I2C:{i2c_ok} Magnet:{magnet_ok}), AGC: {agc}, errors: {errors}")
                
        except KeyboardInterrupt:
            break

if __name__ == "__main__":
    main()
