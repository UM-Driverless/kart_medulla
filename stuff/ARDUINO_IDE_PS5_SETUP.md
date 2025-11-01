# PS5 Controller with Arduino IDE on Ubuntu

## ⚠️ Important Limitations

**Known Issue**: The `ps5Controller` library (both original and BLACKROBOTICS fork) has **L2CAP connection failures** with your specific DualSense controller firmware. We extensively tested this on macOS and it fails to complete the Bluetooth connection.

**The issue is NOT the platform (macOS vs Ubuntu)** - it's a compatibility problem between the library and modern PS5 controller firmware.

### Your Options with Arduino IDE:

1. **Try ps5Controller library anyway** (may not work with your controller)
2. **Use PS4 controller** (libraries are more mature)
3. **Wait for library updates** (no timeline)

---

## Option 1: Try ps5Controller Library (Ubuntu + Arduino IDE)

### Prerequisites
- Ubuntu 20.04 or later
- Internet connection

### Step 1: Install Arduino IDE

```bash
# Download latest Arduino IDE 2.x
cd ~/Downloads
wget https://downloads.arduino.cc/arduino-ide/arduino-ide_2.3.4_Linux_64bit.AppImage

# Make executable
chmod +x arduino-ide_2.3.4_Linux_64bit.AppImage

# Run Arduino IDE
./arduino-ide_2.3.4_Linux_64bit.AppImage
```

### Step 2: Install ESP32 Board Support

1. Open Arduino IDE
2. Go to **File → Preferences**
3. Add to "Additional Board Manager URLs":
   ```
   https://dl.espressif.com/dl/package_esp32_index.json
   ```
4. Click **OK**
5. Go to **Tools → Board → Boards Manager**
6. Search for "esp32"
7. Install **"esp32 by Espressif Systems"** (version 3.2.0 or later)

### Step 3: Install ps5Controller Library

**Method A: ZIP File (Recommended)**

1. Download library:
   ```bash
   cd ~/Downloads
   git clone https://github.com/BLACKROBOTICS/ps5Controller.git
   cd ps5Controller
   zip -r ps5Controller.zip *
   ```

2. In Arduino IDE:
   - **Sketch → Include Library → Add .ZIP Library**
   - Select `ps5Controller.zip`

**Method B: Manual Installation**

```bash
cd ~/Arduino/libraries
git clone https://github.com/BLACKROBOTICS/ps5Controller.git
```
Restart Arduino IDE.

### Step 4: Get PS5 Controller MAC Address

**While controller is connected to Ubuntu via USB:**

```bash
# Install bluez tools
sudo apt install bluez-tools

# Find controller MAC
bluetoothctl devices

# Or check system settings
# Settings → Bluetooth → Look for "Wireless Controller"
```

**Or from phone:**
1. Pair controller to phone via Bluetooth
2. Settings → Bluetooth → Wireless Controller (i) → Look for MAC address

### Step 5: Create Test Sketch

Create new sketch in Arduino IDE:

```cpp
#include <ps5Controller.h>

void setup() {
    Serial.begin(115200);

    // Replace with YOUR controller's MAC address
    ps5.begin("AC:36:1B:09:F0:28");

    Serial.println("PS5 Controller Test");
    Serial.println("Waiting for connection...");
    Serial.println("Press SHARE + PS buttons on controller");
}

void loop() {
    if (ps5.isConnected()) {
        Serial.print("Left Stick X: ");
        Serial.print(ps5.LStickX());
        Serial.print(" Y: ");
        Serial.println(ps5.LStickY());

        if (ps5.Cross()) {
            Serial.println("X button pressed");
        }

        delay(100);
    }
}
```

### Step 6: Upload to ESP32

1. **Connect ESP32 via USB**
2. **Tools → Board → ESP32 Dev Module**
3. **Tools → Port** → Select `/dev/ttyUSB0` (or `/dev/ttyACM0`)
4. Click **Upload** button
5. Open **Serial Monitor** (115200 baud)

### Step 7: Test Connection

1. Press **SHARE + PS** buttons on controller (hold 3-5 seconds)
2. Light bar should flash rapidly
3. Watch Serial Monitor for connection status

### Expected Results

**If it works:**
```
PS5 Controller Test
Waiting for connection...
Press SHARE + PS buttons on controller
[Connection successful message]
Left Stick X: 0 Y: 0
```

**If it fails (likely):**
```
E (5004) ps5_L2CAP: L2CA_CONNECT_REQ ret=64
E (10006) ps5_L2CAP: L2CA_CONNECT_REQ ret=65
[Connection never completes]
```

---

## Option 2: Use PS4 Controller Instead

The **ps4Controller** library is more mature and stable.

### Install PS4 Library

```bash
cd ~/Arduino/libraries
git clone https://github.com/aed3/PS4-esp32.git
```

### PS4 Test Sketch

```cpp
#include <PS4Controller.h>

void setup() {
    Serial.begin(115200);
    PS4.begin("AC:36:1B:09:F0:28"); // Your MAC
    Serial.println("Ready. Press PS button on controller.");
}

void loop() {
    if (PS4.isConnected()) {
        Serial.printf("Stick: %d, %d\\n",
            PS4.LStickX(), PS4.LStickY());
        delay(100);
    }
}
```

**Note**: Requires PS4 DualShock 4 controller (not PS5 DualSense)

---

## Option 3: Bluepad32 Alternative Approach

**Important**: Bluepad32 cannot be installed via Arduino IDE Library Manager.

However, you can use the **Bluepad32 Arduino template** which includes Arduino support:

### Using Bluepad32 with Arduino IDE on Ubuntu

This requires using the command line to set up the project, but then you can edit in Arduino IDE.

```bash
# Install ESP-IDF
cd ~
git clone -b v5.3 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh esp32
source export.sh

# Clone Bluepad32 template
cd ~
git clone --recursive https://github.com/ricardoquesada/esp-idf-arduino-bluepad32-template.git my_ps5_project
cd my_ps5_project

# Build and flash
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

**Edit code**: Modify `main/sketch.cpp` in any text editor (or Arduino IDE as text editor)

---

## Recommended Path Forward

### If you MUST use Arduino IDE:

1. ✅ **Try ps5Controller on Ubuntu** (quick test, likely won't work)
2. ✅ **Use PS4 controller** (most reliable with Arduino IDE)
3. ❌ **Bluepad32** (incompatible with Arduino IDE workflow)

### If you want PS5 support that actually works:

1. ✅ **Use ESP-IDF command line on Ubuntu** (Bluepad32)
2. ✅ **Edit sketch.cpp in Arduino IDE** (as text editor only)
3. ✅ **Build/flash from terminal** (`idf.py` commands)

---

## Summary

| Solution | Arduino IDE | PS5 Support | Works? |
|----------|-------------|-------------|--------|
| ps5Controller library | ✅ Yes | ✅ Yes | ❌ No (L2CAP issue) |
| PS4Controller library | ✅ Yes | ❌ No | ✅ Yes (need PS4 controller) |
| Bluepad32 | ⚠️ Edit only | ✅ Yes | ✅ Yes (CLI build required) |

**My honest recommendation**:
- If you have a PS4 controller → Use PS4Controller with Arduino IDE
- If you need PS5 support → Use Bluepad32 on Ubuntu (command line build, Arduino IDE for editing)
- ps5Controller library likely won't work with your controller firmware

---

## Files Created

All guides are in `stuff/` directory:
- `ARDUINO_IDE_PS5_SETUP.md` (this file)
- `BLUEPAD32_UBUNTU_SETUP.md` (full Ubuntu ESP-IDF setup)
- `ps5_bluetooth_attempts.md` (all testing we did)
