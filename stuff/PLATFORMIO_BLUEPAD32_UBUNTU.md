# PlatformIO + Bluepad32 on Ubuntu - Complete Setup

**This is the best solution for PS5 DualSense controller with ESP32!**

✅ No macOS build issues
✅ Bluepad32 works perfectly
✅ PlatformIO IDE (VSCode extension)
✅ Modern PS5 controller support
✅ Easy to maintain

---

## Prerequisites

- Ubuntu 20.04 or later (or Ubuntu VM/WSL2)
- Internet connection
- ESP32 connected via USB

---

## Part 1: Install PlatformIO on Ubuntu

### Step 1: Install Dependencies

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install required packages
sudo apt install -y git python3 python3-pip python3-venv
```

### Step 2: Install VSCode

```bash
# Download and install VSCode
cd ~/Downloads
wget -O code.deb 'https://code.visualstudio.com/sha/download?build=stable&os=linux-deb-x64'
sudo dpkg -i code.deb
sudo apt-get install -f  # Fix any dependency issues
```

### Step 3: Install PlatformIO Extension

1. Open VSCode
2. Click Extensions icon (left sidebar) or press `Ctrl+Shift+X`
3. Search for "PlatformIO IDE"
4. Click **Install**
5. Wait for installation to complete (may take a few minutes)
6. Restart VSCode when prompted

### Step 4: Setup USB Permissions

```bash
# Add udev rules for ESP32
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="10c4", ATTR{idProduct}=="ea60", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="1a86", ATTR{idProduct}=="7523", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="0403", ATTR{idProduct}=="6001", MODE="0666"' | \\
    sudo tee /etc/udev/rules.d/99-platformio-udev.rules

# Reload rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Add user to dialout group
sudo usermod -a -G dialout $USER

# IMPORTANT: Log out and log back in for group changes to take effect!
```

**After running these commands, LOG OUT and LOG BACK IN** (or reboot).

---

## Part 2: Clone and Setup Bluepad32 Project

### Step 1: Clone Template

```bash
# Create workspace directory
mkdir -p ~/esp32_projects
cd ~/esp32_projects

# Clone Bluepad32 template
git clone --recursive https://github.com/ricardoquesada/esp-idf-arduino-bluepad32-template.git ps5_test

cd ps5_test
```

### Step 2: Create Simple PS5 Test

Replace `main/sketch.cpp` with this:

```bash
cat > main/sketch.cpp << 'EOF'
// PS5 DualSense Controller Test with Bluepad32
#include <Arduino.h>
#include <Bluepad32.h>

ControllerPtr myController = nullptr;

void onConnectedController(ControllerPtr ctl) {
    if (myController == nullptr) {
        Console.println("\\n╔═══════════════════════════════════╗");
        Console.println("║   PS5 CONTROLLER CONNECTED! ✓    ║");
        Console.println("╚═══════════════════════════════════╝");

        myController = ctl;

        ControllerProperties properties = ctl->getProperties();
        Console.printf("Model: %s\\n", ctl->getModelName().c_str());
        Console.printf("VID: 0x%04x, PID: 0x%04x\\n\\n",
            properties.vendor_id, properties.product_id);
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    if (myController == ctl) {
        Console.println("\\n╔═══════════════════════════════════╗");
        Console.println("║  PS5 CONTROLLER DISCONNECTED ✗   ║");
        Console.println("╚═══════════════════════════════════╝\\n");
        myController = nullptr;
    }
}

void setup() {
    Console.println("\\n╔════════════════════════════════════════╗");
    Console.println("║  Bluepad32 PS5 Controller Test       ║");
    Console.println("╚════════════════════════════════════════╝\\n");

    // Initialize Bluepad32
    BP32.setup(&onConnectedController, &onDisconnectedController);

    // Forget old pairings for easier testing
    BP32.forgetBluetoothKeys();

    Console.println("Ready! Waiting for PS5 controller...");
    Console.println("Put controller in pairing mode:");
    Console.println("  → Press and hold SHARE + PS buttons");
    Console.println("  → Light bar will flash rapidly\\n");
}

void loop() {
    // Update Bluepad32
    bool dataUpdated = BP32.update();

    if (myController && myController->isConnected() && dataUpdated) {
        // Print controller data
        Console.printf("L-Stick: X=%4d Y=%4d  |  R-Stick: X=%4d Y=%4d  |  L2=%4d R2=%4d",
            myController->axisX(),
            myController->axisY(),
            myController->axisRX(),
            myController->axisRY(),
            myController->brake(),
            myController->throttle());

        // Show buttons if pressed
        if (myController->buttons()) {
            Console.print("  [");
            if (myController->a()) Console.print("X ");
            if (myController->b()) Console.print("O ");
            if (myController->x()) Console.print("□ ");
            if (myController->y()) Console.print("△ ");
            if (myController->l1()) Console.print("L1 ");
            if (myController->r1()) Console.print("R1 ");
            if (myController->miscSystem()) Console.print("PS ");
            Console.print("]");
        }

        Console.println();
    }

    delay(10);
}
EOF
```

---

## Part 3: Build and Upload with PlatformIO

### Method A: Using VSCode (Recommended)

1. **Open project in VSCode:**
   ```bash
   code ~/esp32_projects/ps5_test
   ```

2. **Wait for PlatformIO to initialize** (bottom toolbar will appear)

3. **Select environment:**
   - Click on environment selector at bottom
   - Choose `env:esp32dev`

4. **Build the project:**
   - Click **Build** button (checkmark icon) at bottom toolbar
   - Or press `Ctrl+Alt+B`
   - Wait for build to complete (~5-10 minutes first time)

5. **Connect ESP32 via USB**

6. **Upload to ESP32:**
   - Click **Upload** button (arrow icon) at bottom toolbar
   - Or press `Ctrl+Alt+U`

7. **Open Serial Monitor:**
   - Click **Serial Monitor** button (plug icon) at bottom toolbar
   - Or press `Ctrl+Alt+S`
   - Baud rate: 115200

### Method B: Using Command Line

```bash
cd ~/esp32_projects/ps5_test

# Build
pio run -e esp32dev

# Upload (find your port first)
ls /dev/ttyUSB* /dev/ttyACM*
pio run -e esp32dev --target upload --upload-port /dev/ttyUSB0

# Monitor
pio device monitor --port /dev/ttyUSB0 --baud 115200
```

---

## Part 4: Test PS5 Controller Connection

### Step 1: Put Controller in Pairing Mode

1. **Hold SHARE + PS buttons** together
2. Hold for **3-5 seconds**
3. Light bar will **flash rapidly** (fast blue pulses)

### Step 2: Watch Serial Monitor

You should see:

```
╔════════════════════════════════════════╗
║  Bluepad32 PS5 Controller Test       ║
╚════════════════════════════════════════╝

Ready! Waiting for PS5 controller...
Put controller in pairing mode:
  → Press and hold SHARE + PS buttons
  → Light bar will flash rapidly

╔═══════════════════════════════════════╗
║   PS5 CONTROLLER CONNECTED! ✓        ║
╚═══════════════════════════════════════╝
Model: DualSense
VID: 0x054c, PID: 0x0ce6

L-Stick: X=   0 Y=   0  |  R-Stick: X=   0 Y=   0  |  L2=   0 R2=   0
```

### Step 3: Test Controller

- **Move sticks** → See values change in serial monitor
- **Press buttons** → See button names appear
- **Controller light** should turn solid color when connected

---

## Part 5: Integrate with Your Kart Code

Once PS5 controller connects successfully, integrate your existing code:

### Copy Your Files to Project

```bash
# Copy your kart control files
cp ~/kart_medulla/src/steering_test.* ~/esp32_projects/ps5_test/main/
cp ~/kart_medulla/src/steering_test_sequence.* ~/esp32_projects/ps5_test/main/
cp ~/kart_medulla/src/sine_wave_test.* ~/esp32_projects/ps5_test/main/
```

### Example Integration in sketch.cpp

```cpp
void loop() {
    BP32.update();

    if (myController && myController->isConnected()) {
        // Map left stick X to steering angle (-30° to +30°)
        int32_t stickX = myController->axisX();  // -511 to 512
        float targetSteering = (stickX / 512.0) * 30.0; // degrees

        // Map R2 trigger to throttle (0-100%)
        int32_t r2 = myController->throttle(); // 0 to 1023
        float targetThrottle = (r2 / 1023.0) * 100.0;

        // Map L2 trigger to brake (0-100%)
        int32_t l2 = myController->brake(); // 0 to 1023
        float targetBrake = (l2 / 1023.0) * 100.0;

        // Convert steering to radians for your PID
        float targetAngleRad = targetSteering * PI / 180.0;

        // Your existing PID control code here
        // ...
    }

    delay(10);
}
```

---

## Troubleshooting

### Build Fails

**Error: "Platform espressif32 not found"**
```bash
# Install platform manually
pio platform install https://github.com/pioarduino/platform-espressif32/releases/download/54.03.21/platform-espressif32.zip
```

**Error: Git submodules not initialized**
```bash
cd ~/esp32_projects/ps5_test
git submodule update --init --recursive
```

### ESP32 Not Detected

```bash
# Check if ESP32 is connected
lsusb | grep -i "CP210\\|CH340\\|FTDI"

# Check serial ports
ls -l /dev/ttyUSB* /dev/ttyACM*

# If not found, check USB cable (must be data cable, not charge-only)
```

### Permission Denied

```bash
# Verify you're in dialout group
groups | grep dialout

# If not, add yourself and REBOOT
sudo usermod -a -G dialout $USER
sudo reboot
```

### Controller Won't Pair

1. **Reset controller:**
   - Find small hole on back (near L2)
   - Press with paperclip for 5 seconds
   - Try pairing again

2. **Forget from other devices:**
   - Disconnect from Mac/PC/Phone Bluetooth
   - Then try pairing with ESP32

3. **Check ESP32 logs:**
   - Look for Bluetooth initialization messages
   - Should see "Bluepad32 initialized" or similar

---

## Quick Reference Commands

```bash
# Navigate to project
cd ~/esp32_projects/ps5_test

# Build
pio run -e esp32dev

# Upload
pio run -e esp32dev --target upload

# Monitor serial
pio device monitor --baud 115200

# Clean build
pio run -e esp32dev --target clean

# Update libraries
pio pkg update
```

---

## File Structure

```
ps5_test/
├── main/
│   ├── sketch.cpp          ← Your Arduino code here
│   ├── steering_test.cpp   ← Your kart control code
│   ├── steering_test.h
│   └── CMakeLists.txt
├── components/
│   ├── bluepad32/          ← Bluepad32 library
│   ├── bluepad32_arduino/  ← Arduino wrapper
│   └── btstack/            ← Bluetooth stack
├── platformio.ini          ← Project configuration
├── CMakeLists.txt
└── sdkconfig.defaults
```

---

## Next Steps

1. ✅ Set up Ubuntu with PlatformIO
2. ✅ Clone and build Bluepad32 project
3. ✅ Test PS5 controller connection
4. ⏭️ Integrate your kart steering/PID code
5. ⏭️ Map controller inputs to motor control
6. ⏭️ Test with actual hardware
7. ⏭️ Fine-tune PID parameters

---

## Why This Works (and macOS Didn't)

- ✅ **No miniforge conflicts** - Ubuntu doesn't have macOS conda/miniforge
- ✅ **Clean toolchain** - Linux ESP32 toolchain works perfectly
- ✅ **No linker flag issues** - No `-dead_strip_dylibs` errors
- ✅ **Native build support** - ESP-IDF designed for Linux
- ✅ **Faster builds** - Better compiler performance on Linux

This should work **first try** on Ubuntu with no build system issues!

---

## Support

If you encounter any issues:
1. Check `~/esp32_projects/ps5_test/.pio/build/esp32dev/` for build logs
2. Run `pio system info` to check PlatformIO installation
3. Verify ESP32 with `lsusb` and `ls /dev/ttyUSB*`

**All the macOS problems disappear on Ubuntu!** 🎉
