# ESP32 Serial Communication Protocol

This document describes the CSV-based serial protocol used for communication between the ESP32 and monitoring applications (e.g., the Python GUI).

## Overview

The protocol uses compact CSV (Comma-Separated Values) format to minimize bandwidth and prevent ESP32 watchdog timeouts. All messages are newline-terminated.

**Baud Rate:** 115200

## Message Types

### 1. Controller Data (DATA)

Sent at 10Hz when controller is connected.

**Format:**
```
DATA,LX,LY,RX,RY,L2,R2,BUTTONS,BATTERY,CHARGING,AUDIO,MIC
```

**Fields:**
| Field | Type | Range | Description |
|-------|------|-------|-------------|
| LX | int | -128 to 127 | Left stick X axis |
| LY | int | -128 to 127 | Left stick Y axis |
| RX | int | -128 to 127 | Right stick X axis |
| RY | int | -128 to 127 | Right stick Y axis |
| L2 | int | 0 to 255 | Left trigger pressure |
| R2 | int | 0 to 255 | Right trigger pressure |
| BUTTONS | string | | Pipe-separated button list (see below) |
| BATTERY | int | 0 to 100 | Battery percentage |
| CHARGING | int | 0 or 1 | 1 if charging, 0 otherwise |
| AUDIO | int | 0 or 1 | 1 if audio enabled |
| MIC | int | 0 or 1 | 1 if microphone enabled |

**Button Names:**
- D-Pad: `UP`, `DOWN`, `LEFT`, `RIGHT`
- Face buttons: `TRIANGLE`, `CIRCLE`, `CROSS`, `SQUARE`
- Shoulder buttons: `L1`, `L2`, `R1`, `R2`
- Special buttons: `PS`, `SHARE`, `OPTIONS`
- Stick buttons: `L3`, `R3`

Buttons are pipe-separated when multiple are pressed: `TRIANGLE|CROSS|L1`
Empty string when no buttons are pressed.

**Examples:**
```
DATA,-5,10,0,2,0,128,TRIANGLE|CROSS,85,0,0,0
DATA,0,0,0,0,255,255,L2|R2,100,1,0,1
DATA,127,-128,50,-60,0,0,,45,0,0,0
```

### 2. Heartbeat (HB)

Sent every 1 second to indicate ESP32 is alive.

**Format:**
```
HB,UPTIME,CONNECTED
```

**Fields:**
| Field | Type | Range | Description |
|-------|------|-------|-------------|
| UPTIME | int | 0+ | Seconds since ESP32 boot |
| CONNECTED | int | 0 or 1 | 1 if controller connected, 0 otherwise |

**Examples:**
```
HB,52,0
HB,153,1
```

### 3. Connection Status Messages

These are legacy text messages for human readability during setup:

```
PS5 CONTROLLER CONNECTED!
PS5 CONTROLLER DISCONNECTED
Starting PS5 Bluetooth...
Waiting for controller...
```

## Data Rate

- **Controller Data:** ~60-80 bytes @ 10Hz = ~600-800 bytes/second
- **Heartbeat:** ~10 bytes @ 1Hz = ~10 bytes/second
- **Total:** ~810 bytes/second (well within 115200 baud capacity)

## Parsing Guidelines

### Python Example

```python
line = serial_port.readline().decode('utf-8').strip()

if line.startswith("DATA,"):
    parts = line.split(",")
    lstick_x = int(parts[1])
    lstick_y = int(parts[2])
    # ... parse remaining fields
    buttons = parts[7].split("|") if parts[7] else []

elif line.startswith("HB,"):
    parts = line.split(",")
    uptime = int(parts[1])
    connected = parts[2] == "1"
```

### JavaScript Example

```javascript
if (line.startsWith("DATA,")) {
    const parts = line.split(",");
    const data = {
        lstick_x: parseInt(parts[1]),
        lstick_y: parseInt(parts[2]),
        // ...
        buttons: parts[7] ? parts[7].split("|") : []
    };
}
```

## Error Handling

- Ignore malformed lines (catch parse exceptions)
- If heartbeat shows `CONNECTED=0`, controller has disconnected
- Missing DATA messages for >2 seconds indicates connection loss

## Implementation Notes

- **No Unicode characters** - All ASCII for maximum compatibility
- **Compact format** - Minimizes serial bandwidth to prevent ESP32 crashes
- **Simple parsing** - Easy to implement in any language
- **Self-describing** - Message type prefix (DATA, HB) makes protocol obvious

## See Also

- [controller_gui.py](controller_gui.py) - Python GUI implementation
- [src/main.cpp](src/main.cpp) - ESP32 firmware implementation
- [README.md](README.md) - Project documentation
