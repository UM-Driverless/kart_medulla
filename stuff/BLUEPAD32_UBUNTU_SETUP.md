# Bluepad32 PS5 Controller Setup on Ubuntu

## Why Ubuntu?
- ✅ No macOS linker flag issues
- ✅ ESP-IDF builds work out of the box
- ✅ Better toolchain compatibility
- ✅ Faster compilation times

## Prerequisites

### Option 1: Native Ubuntu (Recommended)
- Ubuntu 20.04 or later
- Physical machine or dual-boot

### Option 2: Ubuntu on Virtual Machine
- VMWare Fusion, VirtualBox, or Parallels
- **Important**: Must pass through USB for ESP32 flashing
- Allocate at least 4GB RAM, 20GB disk

### Option 3: Ubuntu on WSL2 (Windows only)
- Windows 10/11 with WSL2
- USB passthrough via usbipd

---

## Step-by-Step Setup

### 1. Install Required Packages

```bash
# Update package list
sudo apt update

# Install build essentials
sudo apt install -y git wget flex bison gperf python3 python3-pip \
    python3-venv cmake ninja-build ccache libffi-dev libssl-dev \
    dfu-util libusb-1.0-0

# Install PlatformIO
pip3 install platformio

# Add to PATH if needed
echo 'export PATH=$PATH:~/.local/bin' >> ~/.bashrc
source ~/.bashrc
```

### 2. Clone Your Project

```bash
# Clone your kart project
cd ~
git clone <your-repo-url> kart_medulla
cd kart_medulla
git checkout ps5-controller
```

### 3. Get Bluepad32 Template

```bash
# Clone Bluepad32 template
cd ~
git clone --recursive https://github.com/ricardoquesada/esp-idf-arduino-bluepad32-template.git bluepad32_ps5

cd bluepad32_ps5
```

### 4. Create Simple PS5 Test

Replace `main/sketch.cpp` with this code:

```cpp
// Simple PS5 DualSense Controller Test
#include <Arduino.h>
#include <Bluepad32.h>

ControllerPtr myController = nullptr;

void onConnectedController(ControllerPtr ctl) {
    if (myController == nullptr) {
        Console.println("\n═══════════════════════════════");
        Console.println("  PS5 CONTROLLER CONNECTED! ✓");
        Console.println("═══════════════════════════════");

        myController = ctl;

        ControllerProperties properties = ctl->getProperties();
        Console.printf("Model: %s\\n", ctl->getModelName().c_str());
        Console.printf("VID: 0x%04x, PID: 0x%04x\\n\\n",
            properties.vendor_id, properties.product_id);
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    if (myController == ctl) {
        Console.println("\\nPS5 Controller Disconnected");
        myController = nullptr;
    }
}

void setup() {
    Console.println("\\n╔════════════════════════════════╗");
    Console.println("║  Bluepad32 PS5 Test           ║");
    Console.println("╚════════════════════════════════╝\\n");

    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys();

    Console.println("Ready! Put PS5 controller in pairing mode:");
    Console.println("  → Press SHARE + PS buttons together");
    Console.println("  → Light bar flashes rapidly\\n");
}

void loop() {
    BP32.update();

    if (myController && myController->isConnected()) {
        // Print stick values every 500ms
        static unsigned long lastPrint = 0;
        if (millis() - lastPrint >= 500) {
            lastPrint = millis();

            Console.printf("Left Stick X: %4d  Y: %4d  |  ",
                myController->axisX(), myController->axisY());
            Console.printf("Right Stick X: %4d  Y: %4d  |  ",
                myController->axisRX(), myController->axisRY());
            Console.printf("L2: %4d  R2: %4d",
                myController->brake(), myController->throttle());

            if (myController->buttons()) {
                Console.print("  [");
                if (myController->a()) Console.print("X ");
                if (myController->b()) Console.print("O ");
                if (myController->x()) Console.print("□ ");
                if (myController->y()) Console.print("△ ");
                Console.print("]");
            }

            Console.println();
        }
    }

    delay(10);
}
```

### 5. Setup USB Permissions

```bash
# Add udev rules for ESP32
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="10c4", ATTR{idProduct}=="ea60", MODE="0666"' | \\
    sudo tee /etc/udev/rules.d/99-esp32.rules

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Add your user to dialout group (for serial access)
sudo usermod -a -G dialout $USER

# Log out and back in for group changes to take effect
```

### 6. Build and Flash

```bash
cd ~/bluepad32_ps5

# Build for ESP32
pio run -e esp32dev

# Find ESP32 port
ls /dev/ttyUSB* /dev/ttyACM*
# Usually /dev/ttyUSB0 or /dev/ttyACM0

# Flash to ESP32
pio run -e esp32dev --target upload --upload-port /dev/ttyUSB0

# Monitor serial output
pio device monitor --port /dev/ttyUSB0 --baud 115200
```

### 7. Pair PS5 Controller

1. **Put controller in pairing mode:**
   - Press and hold **SHARE + PS** buttons together
   - Hold for 3-5 seconds
   - Light bar will flash blue rapidly

2. **Watch serial monitor:**
   - Should see "PS5 CONTROLLER CONNECTED! ✓"
   - Controller light bar will turn solid color

3. **Test controller:**
   - Move sticks - see values in serial output
   - Press buttons - see button names appear

---

## Troubleshooting

### ESP32 Not Found
```bash
# List USB devices
lsusb

# Check if ESP32 is detected
ls -l /dev/ttyUSB* /dev/ttyACM*

# If using VM, ensure USB passthrough is enabled
```

### Permission Denied on /dev/ttyUSB0
```bash
# Check if you're in dialout group
groups

# If not, add yourself
sudo usermod -a -G dialout $USER

# Log out and back in
```

### Build Fails
```bash
# Clean build
pio run -e esp32dev --target clean

# Try again
pio run -e esp32dev
```

### Controller Won't Pair
```bash
# Reset controller
# Press reset button on back (near L2) with paperclip
# Hold 5 seconds

# Try pairing again
# SHARE + PS buttons, hold until rapid flash
```

---

## After Testing Works

Once PS5 controller connects successfully on Ubuntu, you can:

1. **Copy your kart control code** into the Bluepad32 project
2. **Integrate with your PID/motor control**
3. **Map controller inputs to steering/throttle**

### Example Integration

```cpp
void loop() {
    BP32.update();

    if (myController && myController->isConnected()) {
        // Map left stick X to steering (-30° to +30°)
        int stickX = myController->axisX(); // -511 to 512
        float targetSteering = (stickX / 512.0) * 30.0; // degrees

        // Map R2 to throttle (0 to 100%)
        int r2Value = myController->throttle(); // 0 to 1023
        float targetThrottle = (r2Value / 1023.0) * 100.0;

        // Your existing PID control code here
        // ...
    }
}
```

---

## Files to Transfer to Ubuntu

From your macOS project:
- `src/main.cpp` (your PID/motor logic)
- `src/steering_test.h/cpp`
- `src/steering_test_sequence.h/cpp`
- `src/sine_wave_test.h/cpp`
- `CLAUDE.md` (PID tuning notes)

These can be integrated into the Bluepad32 project structure.

---

## Next Steps

1. ✅ Set up Ubuntu environment
2. ✅ Install dependencies
3. ✅ Build and flash Bluepad32
4. ✅ Test PS5 controller connection
5. ⏭️ Integrate your kart control code
6. ⏭️ Test with actual hardware
7. ⏭️ Fine-tune PID with controller input

---

## Additional Resources

- **Bluepad32 Docs**: https://bluepad32.readthedocs.io/
- **ESP-IDF PlatformIO**: https://docs.platformio.org/en/latest/platforms/espressif32.html
- **PS5 Controller Pairing**: Hold SHARE + PS for 3-5 seconds

---

**Note**: All the macOS build issues disappear on Ubuntu. The entire process should be straightforward and fast.
