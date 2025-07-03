# STM32 Blue Pill + AS5600 Angle Sensor

## Hardware Used

* **Microcontroller:** STM32F103C6 (Blue Pill)
* **Sensor:** AS5600 Magnetic Angle Sensor
* **Debugger & Serial Interface:** DAPLink (CMSIS-DAP)

## Wiring Connections

### STM32 Blue Pill to DAPLink

| DAPLink Pin | STM32 Blue Pill Pin |
| ----------- | ------------------- |
| SWDIO       | PA13 (SWDIO)        |
| SWCLK       | PA14 (SWCLK)        |
| UART\_TX    | PA10 (USART1\_RX)   |
| UART\_RX    | PA9 (USART1\_TX)    |
| 3.3V        | 3.3V                |
| GND         | GND                 |

### STM32 Blue Pill to AS5600

| AS5600 Pin | STM32 Blue Pill Pin |
| ---------- | ------------------- |
| SCL        | PB6 (I2C1\_SCL)     |
| SDA        | PB7 (I2C1\_SDA)     |
| VCC        | 3.3V                |
| GND        | GND                 |

## Software Setup

### PlatformIO Configuration (`platformio.ini`)

```ini
[env:bluepill_f103c6]
platform = ststm32
board = bluepill_f103c6
framework = arduino

upload_protocol = cmsis-dap
debug_tool = cmsis-dap
monitor_speed = 115200
```

### Install AS5600 Arduino Library

```bash
pio pkg install AS5600
```

## Flashing and Running

To flash and run the code, navigate to the `bluepill-angle-arduino` directory:

```bash
cd bluepill-angle-arduino
```

Then, you can use the following commands:

### Upload and run serial monitor

```bash
platformio run --target upload && platformio device monitor
```

Serial output will be available at **115200 baud** via the virtual COM port provided by DAPLink.

Example serial monitor output:

```
Angle: 90.1°
Angle: 91.7°
...
```

Ensure your wiring is correct and your DAPLink's serial port is recognized (e.g., `/dev/ttyACM0`).

## Chip Information

* **STM32F103C6**

  * ARM Cortex-M3
  * 32KB Flash, 10KB RAM
  * Operating Voltage: 3.3V

* **AS5600**

  * 12-bit Magnetic Angle Sensor
  * I²C Interface (address: `0x36`)

This guide provides all necessary information to set up and verify the functionality of the AS5600 angle sensor with an STM32 Blue Pill.
