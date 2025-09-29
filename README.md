# Camera Tracking with ESP32 Servo Control

This project uses **ESPRESSIF** in change of parsing (`dx, dy`) from PC via UART to control two servo motors (pan/tilt).  

# 🔌 Wiring

| ESP32 Pin | Servo      | Notes                              |
| --------- | ---------- | ---------------------------------- |
| GPIO18    | Pan servo  | PWM output                         |
| GPIO19    | Tilt servo | PWM output                         |
| GND       | Servo GND  | Common ground                      |
| 3.3V      | Servo VCC  | Power (better use external supply) |

# ⚠️ Important:

Servos can draw large current → use an external 5V supply.

Connect servo ground to ESP32 ground.

# 🛠️ Build & Flash
idf.py set-target esp32
idf.py build
idf.py -p COM5 flash monitor

