### 
pio pkg install AS5600

### to use daplink add in platformio.ini
```ini
upload_protocol = cmsis-dap   ; <— ADD THIS LINE
debug_tool = cmsis-dap        ; <— optional, enables debugging
monitor_speed = 115200
```

### Wiring
SCL → PB6 (default STM32 I2C1 SCL)

SDA → PB7 (default STM32 I2C1 SDA)

VCC → 3.3V (Blue Pill operates at 3.3V)

GND → GND

### Program
platformio run --target upload

## stuff
--- Terminal on /dev/ttyACM0 | 115200 8-N-1

