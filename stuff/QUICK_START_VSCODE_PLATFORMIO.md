# Quick Start: PS5 Controller with VSCode + PlatformIO on Ubuntu

**You already have:** VSCode with PlatformIO extension ✓
**You need:** PS5 DualSense working with ESP32

---

## Prerequisites Check

Before starting, make sure on Ubuntu you have:

- [ ] VSCode installed
- [ ] PlatformIO extension installed in VSCode
- [ ] Git installed (`sudo apt install git`)
- [ ] USB permissions configured (see below)

---

## Step 1: USB Permissions (One-Time Setup)

```bash
# Create udev rules for ESP32
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="10c4", ATTR{idProduct}=="ea60", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="1a86", ATTR{idProduct}=="7523", MODE="0666"' | \\
    sudo tee /etc/udev/rules.d/99-platformio-udev.rules

# Apply rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Add yourself to dialout group
sudo usermod -a -G dialout $USER

# CRITICAL: Log out and log back in (or reboot)
```

**⚠️ IMPORTANT: You MUST log out and log back in after this step!**

---

## Step 2: Clone Bluepad32 Project

```bash
# Create workspace
mkdir -p ~/esp32_projects
cd ~/esp32_projects

# Clone template with all submodules
git clone --recursive https://github.com/ricardoquesada/esp-idf-arduino-bluepad32-template.git ps5_kart

cd ps5_kart
```

---

## Step 3: Create Simple Test Code

Replace `main/sketch.cpp`:

```bash
cat > main/sketch.cpp << 'EOF'
#include <Arduino.h>
#include <Bluepad32.h>

ControllerPtr myController = nullptr;

void onConnectedController(ControllerPtr ctl) {
    if (myController == nullptr) {
        Console.println("\n*** PS5 CONTROLLER CONNECTED! ***\n");
        myController = ctl;
        Console.printf("Model: %s\n", ctl->getModelName().c_str());
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    if (myController == ctl) {
        Console.println("\n*** PS5 DISCONNECTED ***\n");
        myController = nullptr;
    }
}

void setup() {
    Console.println("Bluepad32 PS5 Test Starting...");

    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys();

    Console.println("\nReady! Press SHARE + PS buttons on controller\n");
}

void loop() {
    BP32.update();

    if (myController && myController->isConnected()) {
        // Print every 500ms
        static unsigned long lastPrint = 0;
        if (millis() - lastPrint >= 500) {
            lastPrint = millis();

            Console.printf("Left Stick: X=%4d Y=%4d  |  Right Stick: X=%4d Y=%4d\n",
                myController->axisX(),
                myController->axisY(),
                myController->axisRX(),
                myController->axisRY());
        }
    }

    delay(10);
}
EOF
```

---

## Step 4: Open in VSCode

```bash
# Open project in VSCode
code ~/esp32_projects/ps5_kart
```

**OR**: In VSCode, go to `File → Open Folder` → Select `ps5_kart`

---

## Step 5: Build & Upload (VSCode GUI)

### First Time Setup:

1. **Wait for PlatformIO to load**
   - Bottom toolbar should appear
   - May take 1-2 minutes on first open

2. **Select Environment**
   - Look for environment selector at bottom
   - Should show `esp32dev`
   - If not, click and select `esp32dev`

### Build:

1. **Click the ✓ (checkmark) icon** at bottom toolbar
   - Or press `Ctrl+Alt+B`
   - First build takes ~5-10 minutes (downloads toolchain)
   - Subsequent builds: ~30 seconds

### Upload:

1. **Connect ESP32 via USB**

2. **Click the → (arrow) icon** at bottom toolbar
   - Or press `Ctrl+Alt+U`
   - Takes ~30 seconds

### Monitor:

1. **Click the 🔌 (plug) icon** at bottom toolbar
   - Or press `Ctrl+Alt+S`
   - Should show serial output at 115200 baud

---

## Step 6: Pair PS5 Controller

1. **Press and hold SHARE + PS buttons** together
2. **Hold for 3-5 seconds**
3. **Light bar flashes rapidly** (fast blue pulses)
4. **Watch serial monitor** for connection message

### Expected Output:

```
Bluepad32 PS5 Test Starting...

Ready! Press SHARE + PS buttons on controller

*** PS5 CONTROLLER CONNECTED! ***

Model: DualSense
Left Stick: X=   0 Y=   0  |  Right Stick: X=   0 Y=   0
```

---

## PlatformIO VSCode Shortcuts

| Action | Icon | Keyboard |
|--------|------|----------|
| Build | ✓ | `Ctrl+Alt+B` |
| Upload | → | `Ctrl+Alt+U` |
| Monitor | 🔌 | `Ctrl+Alt+S` |
| Clean | 🗑️ | `Ctrl+Alt+C` |
| Upload & Monitor | Combined | `Ctrl+Alt+T` |

---

## Project Structure in VSCode

```
ps5_kart/
├── main/
│   └── sketch.cpp          ← Edit this file (your code)
├── components/             ← Bluepad32 library (don't modify)
├── platformio.ini          ← Project config
└── .pio/                   ← Build output (auto-generated)
```

**You only need to edit:** `main/sketch.cpp`

---

## Common Issues & Fixes

### "Permission denied" on /dev/ttyUSB0

```bash
# Verify you're in dialout group
groups

# Should show: ... dialout ...
# If not, you didn't log out after Step 1!
```

**Fix:** Log out and log back in (or reboot)

### Build fails with platform not found

VSCode terminal:
```bash
pio platform install https://github.com/pioarduino/platform-espressif32/releases/download/54.03.21/platform-espressif32.zip
```

### Can't see serial output

1. Close any other serial monitor programs
2. Check baud rate is 115200
3. Try unplugging and replugging ESP32

### Controller won't pair

1. Reset controller: Small button on back (near L2), hold 5 seconds
2. Make sure it's not paired to Mac/PC/phone
3. Try SHARE + PS buttons again

---

## Next Steps

Once PS5 controller connects successfully:

### 1. Copy Your Kart Code

```bash
# Copy your existing files
cp ~/kart_medulla/src/steering_test.* ~/esp32_projects/ps5_kart/main/
cp ~/kart_medulla/src/steering_test_sequence.* ~/esp32_projects/ps5_kart/main/
cp ~/kart_medulla/src/sine_wave_test.* ~/esp32_projects/ps5_kart/main/
```

### 2. Update CMakeLists.txt

Edit `main/CMakeLists.txt`:

```cmake
idf_component_register(
    SRCS
        "main.c"
        "sketch.cpp"
        "steering_test.cpp"              # Add these
        "steering_test_sequence.cpp"     # Add these
        "sine_wave_test.cpp"             # Add these
    INCLUDE_DIRS "."
)
```

### 3. Integrate in sketch.cpp

```cpp
void loop() {
    BP32.update();

    if (myController && myController->isConnected()) {
        // Map controller to steering
        float targetSteering = (myController->axisX() / 512.0) * 30.0; // degrees

        // Your PID control code here
        // ...
    }
}
```

---

## VSCode Tips

### Side-by-Side Editing
- Drag `sketch.cpp` tab to split editor
- View serial monitor and code simultaneously

### Search Across Files
- `Ctrl+Shift+F` - Search in all files
- Useful for finding function definitions

### Integrated Terminal
- `Ctrl+` (backtick) - Open terminal
- Can run `pio` commands directly

### Problems Panel
- `Ctrl+Shift+M` - View build errors
- Click error to jump to code

---

## Full Documentation

See `PLATFORMIO_BLUEPAD32_UBUNTU.md` for:
- Detailed troubleshooting
- Integration examples
- Advanced configuration
- All command line options

---

## Summary

**Your workflow on Ubuntu:**

1. Open VSCode → Open `ps5_kart` folder
2. Edit `main/sketch.cpp`
3. Click ✓ to build
4. Click → to upload
5. Click 🔌 to monitor
6. Press SHARE + PS on controller
7. Done! ✓

**It really is that simple on Ubuntu!**
