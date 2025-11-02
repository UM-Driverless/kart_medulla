# PS5 Controller Bluetooth Connection Attempts

## Hardware
- **Board**: ESP32-WROOM (ESP32-D0WD-V3)
- **Controller**: DualSense Wireless Controller
- **Controller MAC**: AC:36:1B:09:F0:28
- **ESP32 MAC**: 40:22:d8:79:12:8c

## Attempt 1: rodneybakiskan/ps5-esp32
**Date**: Nov 1, 2025
**Library**: https://github.com/rodneybakiskan/ps5-esp32.git
**Version**: 2.1.0+sha.42b9ca9

### Setup
```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
lib_deps = https://github.com/rodneybakiskan/ps5-esp32.git
```

### Code
```cpp
#include <ps5Controller.h>
ps5.begin("AC:36:1B:09:F0:28");
```

### Result: FAILED ❌
**Error**: L2CAP connection errors
```
E (5005) ps5_L2CAP: L2CA_CONNECT_REQ ret=64
E (10007) ps5_L2CAP: L2CA_CONNECT_REQ ret=65
```

**Symptoms**:
- ESP32 detects pairing attempts
- Controller enters pairing mode (blue flashing)
- L2CAP layer fails to establish connection
- Connection never completes

## Attempt 2: BLACKROBOTICS/ps5Controller
**Date**: Nov 1, 2025
**Library**: https://github.com/BLACKROBOTICS/ps5Controller.git (fork for ESP32 v3.2.X+)

### Setup
```ini
lib_deps = https://github.com/BLACKROBOTICS/ps5Controller.git
```

### Result: FAILED ❌
**Error**: Same L2CAP errors as Attempt 1
```
E (5003) ps5_L2CAP: L2CA_CONNECT_REQ ret=64
E (10005) ps5_L2CAP: L2CA_CONNECT_REQ ret=65
E (15006) ps5_L2CAP: L2CA_CONNECT_REQ ret=65
```

**Notes**:
- Tried after full controller reset (5s reset button hold)
- Removed controller from Mac Bluetooth (Forget Device)
- Same underlying library as Attempt 1, just updated for newer cores
- Issue persists regardless of library version

## Analysis

### Error Code Meaning
- **L2CA_CONNECT_REQ ret=64/65**: L2CAP channel connection request failures
- ret=64 (0x40) and ret=65 (0x41) appear to be channel ID related
- Suggests Bluetooth Classic pairing protocol issue at L2CAP layer

### Possible Causes
1. **Library incompatibility** with ESP32 Bluetooth stack version
2. **DualSense PS5 controller** might require different pairing than library expects
3. **Bluetooth Classic vs BLE** - ps5-esp32 uses Classic Bluetooth
4. ESP32 Bluetooth stack configuration issues

### GitHub Issues Found
- Issue #24: "Reconnecting does not work" (open)
- Issue #30: "Sending data to controller doesn't work" (open)
- No direct mention of L2CAP error codes in issues

## Attempt 3: Bluepad32 with Arduino+ESP-IDF Hybrid
**Date**: Nov 1, 2025
**Approach**: Copy Bluepad32 components to project, use hybrid Arduino+ESP-IDF framework

### Setup
```ini
platform = https://github.com/pioarduino/platform-espressif32/releases/download/53.03.10/platform-espressif32.zip
framework = arduino, espidf
```

### Components Copied
- `components/bluepad32/` - Core Bluepad32 library
- `components/bluepad32_arduino/` - Arduino wrapper
- `components/btstack/` - Bluetooth stack

### Result: FAILED ❌
**Error**: Build system incompatibility on macOS
```
ld: Error: unable to disambiguate: -dead_strip_dylibs
```

**Root Cause**:
1. Bluepad32 requires ESP-IDF component system (not pure Arduino)
2. Hybrid Arduino+ESP-IDF only available in custom `pioarduino` platform
3. Custom platform has macOS linker flag issues
4. ESP32 cross-compiler doesn't understand macOS-specific linker flags

**Dependency Issues Found**:
- Platform 54.03.21 bundles IDF 5.4.2 but requires IDF 5.3.x (version mismatch)
- Platform 53.03.10 has toolchain incompatibility with macOS
- CMake picks up macOS system flags incompatible with ESP32 toolchain

## Conclusion

**Why changing platform won't help**:
- Standard `espressif32` platform = pure Arduino only (no ESP-IDF components)
- Bluepad32 components REQUIRE ESP-IDF framework
- Hybrid framework only in custom platform (which has bugs)

## Deep Dive: L2CAP Connection Analysis

### What's Happening
```
E (5004) ps5_L2CAP: L2CA_CONNECT_REQ ret=64
E (10006) ps5_L2CAP: L2CA_CONNECT_REQ ret=65
```

**Understanding the "errors"**:
- These aren't actually errors - 64 and 65 are **channel IDs** assigned by L2CAP
- The `ESP_LOGE` is misleading (library logs successful channel allocation as error)
- Return value of 0 = failure, non-zero = success (channel ID)

**L2CAP Connection Flow** (what should happen):
1. ✅ `L2CA_CONNECT_REQ` → Returns channel ID (64, 65) - **WE GET HERE**
2. ❌ `ps5_l2cap_connect_cfm_cback` → Connection confirmed - **NEVER CALLED**
3. ❌ `ps5_l2cap_config_ind_cback` → Config negotiation - **NEVER CALLED**
4. ❌ `ps5_l2cap_config_cfm_cback` → Sets `is_connected = true` - **NEVER CALLED**

### Root Cause
**The PS5 DualSense controller is rejecting the connection** after ESP32 initiates it.

Possible reasons:
1. **Controller firmware incompatibility** - Library developed for older firmware
2. **HID protocol mismatch** - PS5 expects specific HID descriptors
3. **Security/authentication** - DualSense may require authentication not implemented
4. **Pairing state** - Controller may be in wrong pairing mode

### Attempts Made
- ✅ Reset controller (5s reset button)
- ✅ Forget from Mac Bluetooth
- ✅ Both library versions (original + BLACKROBOTICS fork)
- ✅ Pairing while connected via USB
- ✅ Event callbacks attached
- ❌ All attempts show same behavior: connection request succeeds, but no confirmation

## Final Assessment

### What Works
- ✅ ESP32 Bluetooth stack is functioning
- ✅ ESP32 detects controller pairing attempts
- ✅ L2CAP layer initializes correctly
- ✅ Connection requests return valid channel IDs
- ✅ Code compiles and runs (BLACKROBOTICS fork)

### What Doesn't Work
- ❌ PS5 controller won't complete L2CAP connection
- ❌ No callback confirmations received
- ❌ Connection never reaches "connected" state
- ❌ ps5-esp32 library incompatible with this DualSense controller

### Conclusion
The `ps5-esp32` library has a **fundamental incompatibility** with this DualSense controller, likely due to:
- Firmware version differences
- HID protocol changes in newer controllers
- Missing authentication/security handshake

## Recommended Next Steps

### Option 1: Use Bluepad32 (Most Reliable)
**Pros:**
- Official PS5 DualSense support
- Actively maintained
- Works with modern controllers

**Cons:**
- Requires ESP-IDF framework (restructure project)
- More complex setup
- Higher learning curve

**How:**
1. Clone template: `git clone --recursive https://github.com/ricardoquesada/esp-idf-arduino-bluepad32-template.git`
2. Copy your kart control code into `main/sketch.cpp`
3. Build with PlatformIO using their platformio.ini

### Option 2: Try PS4 Controller
**Pros:**
- Simpler Arduino libraries available
- More mature ecosystem
- Can verify Bluetooth works

**Cons:**
- Need to buy/borrow PS4 controller
- Not what you wanted originally

### Option 3: Wait for Library Updates
**Pros:**
- Keep current code structure
- Simple if/when fixed

**Cons:**
- No timeline for fixes
- May never be fixed
- Library appears unmaintained

## Recommendation
**Go with Option 1 (Bluepad32)** if you want PS5 support. It's the only proven solution for modern DualSense controllers with ESP32.
