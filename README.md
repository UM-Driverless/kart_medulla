# Kart Medulla - ESP32 Controller System

ESP32-based control system for the UM Driverless autonomous go-kart. Receives commands from the Orin via UART binary protocol, runs PID-controlled steering, and outputs analog throttle/brake signals.

## Hardware Overview

* **Microcontroller:** ESP32-WROOM-32E (DevKitC V4)
* **Steering Sensor:** AS5600 magnetic angle encoder (I2C)
* **Steering Motor:** Cytron H-bridge (PWM + DIR)
* **Throttle/Brake:** DAC analog outputs
* **Comms to Orin:** UART0 binary protocol (int32 encoding, 115200 baud)

## Pin Configuration

All pin assignments are defined in [`components/km_gpio/km_gpio.h`](components/km_gpio/km_gpio.h).

### Actuator Outputs

| Function | GPIO | Type | Description |
|---|---|---|---|
| Throttle (CMD_ACC) | 25 | DAC1 | Analog throttle output (0-255) |
| Brake (CMD_BRAKE) | 26 | DAC2 | Analog brake output (0-255) |
| Steering PWM | 18 | LEDC PWM | Steering motor speed (0-255) |
| Steering DIR | 19 | Digital | Steering motor direction |

### Sensor Inputs (ADC)

| Function | GPIO | ADC Channel | Notes |
|---|---|---|---|
| Pedal Brake | 32 | ADC1_CH4 | |
| Pedal Acc | 35 | ADC1_CH7 | Input only |
| Pressure 1 | 36 (VP) | ADC1_CH0 | Input only |
| Pressure 2 | 39 (VN) | ADC1_CH3 | Input only |
| Pressure 3 | 34 | ADC1_CH6 | Input only |
| Hydraulic 1 | 27 | ADC2_CH7 | |
| Hydraulic 2 | 14 | ADC2_CH6 | |

### I2C (AS5600 Steering Encoder)

| Function | GPIO |
|---|---|
| SDA | 21 |
| SCL | 22 |

### Other Pins

| Function | GPIO | Notes |
|---|---|---|
| USB UART TX (UART0) | 1 | Binary comms to Orin |
| USB UART RX (UART0) | 3 | Binary comms from Orin |
| UART2 TX | 17 | Conflicts with HALL1 on PCB - unused |
| UART2 RX | 16 | Conflicts with HALL3 on PCB - unused |
| Motor Hall 2 | 33 | Hall 1/3 disabled (UART2 conflict) |
| SDC Emergency | 13 | Shutdown circuit status |
| Status LED | 2 | Keep LOW at boot (strap pin) |

### GPIO Restrictions

- **GPIO 6-11:** Reserved for SPI flash - do not use
- **GPIO 34-39:** Input only

## Project Structure

```
kart_medulla/
├── main/
│   └── main.c                     # Entry point, FreeRTOS tasks
├── components/
│   ├── km_gpio/                   # GPIO hardware abstraction (pin definitions, ADC, DAC, PWM, I2C)
│   ├── km_act/                    # Actuator controllers (steering, throttle, brake)
│   ├── km_pid/                    # PID controller
│   ├── km_sdir/                   # AS5600 steering encoder driver
│   ├── km_coms/                   # UART binary protocol (int32 encoding)
│   ├── km_objects/                # Shared object store (targets, actuals)
│   ├── km_rtos/                   # FreeRTOS task manager
│   ├── km_sta/                    # State machine
│   ├── km_gamc/                   # Gamepad controller (Bluepad32)
│   ├── bluepad32/                 # Bluetooth controller library
│   └── btstack/                   # Bluetooth stack
├── test/
│   ├── test_km_pid/               # PID unit tests
│   ├── test_km_act/               # Actuator unit tests
│   ├── test_km_coms/              # Comms unit tests
│   ├── test_km_objects/           # Object store unit tests
│   └── fakes/                     # Hardware stubs for native tests
├── platformio.ini                 # Build config (ESP-IDF + native test env)
└── CMakeLists.txt                 # ESP-IDF project config
```

**Framework:** ESP-IDF 5.x (via PlatformIO, espressif32@6.4.0)
**Build System:** PlatformIO
**Test Environment:** Native (runs on host with hardware fakes)

## FreeRTOS Tasks

| Task | Rate | Stack | Description |
|---|---|---|---|
| comms | 20 Hz | 4096 B | Receives/processes UART messages from Orin |
| control | 10 Hz | 4096 B | Reads AS5600, runs steering PID, sets actuators |
| heartbeat | 1 Hz | 2048 B | Sends uptime to Orin |
| health | 1 Hz | 4096 B | Monitors magnet, I2C, heap; reports to Orin |

## Steering Control

### Coordinate Convention

- **Body frame:** X forward, Y left, Z up
- **Positive steering** = left turn (matches ROS REP 103)
- AS5600 natural frame matches this convention - no negation needed

### PID Configuration (current)

| Parameter | Value |
|---|---|
| Kp | 0.15 |
| Ki | 0.0 |
| Kd | 0.01 |
| PWM limit | 0.15 (15%) |

PWM limit is kept low to protect steering gears during testing. Increase gradually as PID is tuned.

## Building and Flashing

```bash
# Build
pio run -e esp32dev

# Flash (from Orin, where ESP32 is connected via USB)
pio run -t upload -e esp32dev --upload-port /dev/ttyUSB0

# Monitor serial
pio device monitor

# Run unit tests (native, no hardware needed)
pio test -e native
```

**Upload speed:** 115200 baud (CP2102 USB-UART bridge limitation).

**Flash tip:** If ESP32 is in a crash loop, hold the **BOOT** button during flash.

## Development Notes

See [`AGENTS.md`](AGENTS.md) for detailed project documentation, error log, and conventions.
