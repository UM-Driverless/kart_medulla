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
                if length in (12, 16, 20):
                    # Historic frames carried pressure/comp_duty in fields 4-5;
                    # those now live in ESP_PNEUMATIC (0x0C). Read only the 3 core
                    # fields so both old and new firmware decode.
                    val1, val2, val3 = struct.unpack('>iii', payload[:12])
                    raw = val2
                    pid = val3 / 1000.0
                    # Field 4 is the validity flag, added when the steering sensor
                    # moved to the MT6701's PWM output. Firmware that has no angle
                    # sends INT32_MIN in field 1 rather than a plausible-looking
                    # number, so the reading is unusable whichever field you trust.
                    valid = True
                    if length >= 16:
                        valid = bool(struct.unpack('>i', payload[12:16])[0])
                    if not valid or val1 == -2**31:
                        print(f"[STEERING] angle: INVALID (no sensor reading), pid: {pid:.3f}")
                    else:
                        angle_rad = val1 / 1000.0
                        print(f"[STEERING] angle: {angle_rad:.3f} rad, raw: {raw}, pid: {pid:.3f}")
            elif msg_type == 0x0C: # ESP_PNEUMATIC
                # The frame has grown twice by appending fields; decode the two
                # core ones and whatever else is present. It was 8 bytes when this
                # branch was written and is 28 now, which silently stopped it
                # decoding at all.
                if length >= 8:
                    pressure, comp_duty = struct.unpack('>ii', payload[:8])
                    n = length // 4
                    rest = struct.unpack('>%di' % (n - 2), payload[8:n*4]) if n > 2 else ()
                    tail = ""
                    if n >= 4:
                        tail += f", pres2: {rest[0]}, comp_state: {rest[1]}"
                    if n >= 5:
                        # control_task's own iteration count. Differences between
                        # consecutive frames divided by the wall-clock gap give the
                        # real control-loop rate; the frame arrival rate does not.
                        tail += f", control_iters: {rest[2]}"
                    if n >= 7:
                        tail += f", ledc_duty: {rest[3]}, gpio_init: {rest[4]}"
                    # verified calibration (kart-brain tasks.md): SDE5 1 V/bar, ÷3 divider,
                    # bar = 3.0 * V_adc; raw ADC → V via linear 12-bit / 3.3 V model.
                    pressure_bar = 3.0 * (pressure / 4095.0 * 3.3)
                    state = "ON" if comp_duty > 0 else "off"
                    print(f"[PNEUMATIC] pres_adc: {pressure} ({pressure_bar:.2f} bar), "
                          f"compressor: {state} {comp_duty}/255 ({comp_duty * 100 / 255:.0f}%){tail}")
            elif msg_type == 0x0B: # ESP_HEALTH_STATUS
                if length in (16, 24):
                    # Fields 5-6 (steering PWM frame counters) were appended when the
                    # steering sensor moved to the MT6701's PWM output; older firmware
                    # sends only the first four.
                    flags, agc, heap_kb, errors = struct.unpack('>iiii', payload[:16])
                    magnet_ok = bool(flags & 1)
                    i2c_ok = bool(flags & 2)
                    steer_ok = bool(flags & 8)
                    # bit 4: steering fault latched — EBS fired and throttle refused,
                    # and it stays that way until the ESP32 is rebooted.
                    steer_trip = bool(flags & 16)
                    extra = ""
                    if steer_trip:
                        extra += "  *** STEERING FAULT LATCHED: EBS FIRED ***"
                    if length == 24:
                        frames, rejects = struct.unpack('>ii', payload[16:24])
                        # frames flat at 0 = no edges arriving at all (dead sensor or
                        # unplugged lead); rejects climbing with frames flat = edges at
                        # the wrong rate (sensor out of 994 Hz PWM mode, or noise).
                        extra = f", steer frames: {frames}, rejects: {rejects}"
                    print(f"[HEALTH] flags: {flags:02x} (I2C:{i2c_ok} Magnet:{magnet_ok} "
                          f"Steer:{steer_ok}), AGC: {agc}, errors: {errors}{extra}")
                
        except KeyboardInterrupt:
            break

if __name__ == "__main__":
    main()
