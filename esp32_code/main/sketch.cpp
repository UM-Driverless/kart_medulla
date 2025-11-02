#include "sdkconfig.h"
#include <Arduino.h>
#include <Bluepad32.h>

// Pines de salida para los gatillos y dirección
const int pinR2 = 26;        // R2 (throttle)
const int pinL2 = 25;        // L2 (brake)
const int pinDir = 14;       // Dirección (izquierda/derecha)
const int pinPWM = 27;       // PWM de fuerza de giro

// Zona muerta del joystick
const int deadzone = 50;

// Array para controlar hasta 4 mandos
ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// --------------------- Callbacks ---------------------
void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Console.printf("CALLBACK: Controller is connected, index=%d\n", i);
            ControllerProperties properties = ctl->getProperties();
            Console.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n",
                           ctl->getModelName(), properties.vendor_id, properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        Console.println("CALLBACK: Controller connected, but could not find empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            myControllers[i] = nullptr;
            Console.printf("Controller disconnected! Index=%d\n", i);
            return;
        }
    }
}

// --------------------- Funciones ---------------------
void processGamepad(ControllerPtr ctl) {
    if (!ctl->isConnected() || !ctl->hasData()) return;

    // ---------------- Gatillos ----------------
    int r2Value = ctl->throttle(); // R2
    int l2Value = ctl->brake();    // L2

    uint8_t pwmR2 = (r2Value * 255) / 1023;
    uint8_t pwmL2 = (l2Value * 255) / 1023;

    analogWrite(pinR2, pwmR2);
    analogWrite(pinL2, pwmL2);

    // ---------------- Joystick Izquierdo ----------------
    int axisX = ctl->axisX(); // -511 a 512

    uint8_t direction = 0;
    uint8_t pwmJoyStick = 0;

    // Aplicar zona muerta
    if (abs(axisX) > deadzone) {
        // Determinar dirección de giro
        if (axisX < 0) {
            direction = 0;  // izquierda
        } else {
            direction = 255; // derecha
        }

        // Magnitud de giro
        int adjustedAxis = axisX;
        if (axisX < 0) adjustedAxis = -axisX; // valor positivo
        if (adjustedAxis > 511) adjustedAxis = 511; // prevenir overflow

        pwmJoyStick = (adjustedAxis * 255) / 511;
    } else {
        // Dentro de zona muerta
        pwmJoyStick = 0;
    }

    analogWrite(pinPWM, pwmJoyStick);
    digitalWrite(pinDir, direction);

    // ---------------- Monitor Serial ----------------
    Console.printf("R2 PWM: %3d | L2 PWM: %3d | X: %4d | Dir: %3d | PWM: %3d\n",
                   pwmR2, pwmL2, axisX, direction, pwmJoyStick);
}

void processControllers() {
    for (auto ctl : myControllers) {
        if (ctl && ctl->isConnected() && ctl->hasData() && ctl->isGamepad()) {
            processGamepad(ctl);
        }
    }
}

// --------------------- Setup ---------------------
void setup() {
    Console.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Console.printf("BD Addr: %02X:%02X:%02X:%02X:%02X:%02X\n",
                   addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    // Configurar Bluepad32
    bool startScanning = true;
    BP32.setup(&onConnectedController, &onDisconnectedController, startScanning);

    // Solo mandos, sin virtual devices ni BLE
    BP32.enableVirtualDevice(false);
    BP32.enableBLEService(false);

    // Configurar pines
    pinMode(pinR2, OUTPUT);
    pinMode(pinL2, OUTPUT);
    pinMode(pinDir, OUTPUT);
    pinMode(pinPWM, OUTPUT);
}

// --------------------- Loop ---------------------
void loop() {
    BP32.update();
    processControllers();
    delay(50);
}
