# 🤖 Arduino RC Car - DIY Ultrasonic Obstacle Avoidance

A compact **DIY Arduino RC car** that you can drive with an **IR remote**, while an **ultrasonic sensor** watches for obstacles and prevents collisions. The code includes smooth acceleration, speed limiting, and basic “blocked” logic that allows reversing out of danger.

---

## 🎮 Features

* **IR Remote control**: Forward, reverse, left, right, stop, and a “fast forward” mode
* **Ultrasonic obstacle avoidance**: Auto-stop at close range; resume when clear
* **Smooth acceleration**: Gradual ramping to target speed
* **Distance averaging**: Noise-reduced readings for stable behavior
* **Tunable constants**: Easy adjustments for speed, distances, and sample timing

---

## 🔧 Hardware

* Arduino Uno
* Dual H-Bridge motor driver (e.g., L298N)
* 4x DC gear motors + chassis + wheels
* HC-SR04 ultrasonic sensor
* IR receiver module
* IR remote
* Power source (battery pack for motors + 5V for logic)
* Jumper wires, breadboard

---

## 🧩 Pinout (matches the code)

**Motor driver (direction):**

* `LEFT_IN_1` → D2
* `LEFT_IN_2` → D4
* `RIGHT_IN_3` → D5
* `RIGHT_IN_4` → D7

**Motor driver (PWM speed):**

* `LEFT_EN_A` → D9 (PWM)
* `RIGHT_EN_B` → D10 (PWM)

**Ultrasonic Sensor:**

* `TRIG_PIN` → D12
* `ECHO_PIN` → D13

**IR Receiver:**

* `IR_RECEIVER_PIN` → D8

> Don’t forget common grounds between Arduino, motor driver, sensors, and power.

---

## 🎛️ Controls (default mapping)

* ▶️ **Play/Pause** → Forward
* ⏩ **Vol+** → Fast forward
* 🔙 **Vol−** → Backward
* ⏹️ **Power** → Stop
* ⏪ **Rewind** → Turn left
* ⏩ **Fast-forward** → Turn right

> When an obstacle is detected, the car **blocks forward/left/right** and will only **allow Reverse or Stop** until clear.

---

## ⚙️ Key Settings (edit in code)

```cpp
// Motor speeds (0–100%)
const int DEFAULT_MOTOR_SPEED = 60;
const int HIGH_MOTOR_SPEED    = 90;

// Obstacle distances (cm)
const float STOP_DISTANCE_CENTIMETRES  = 30.0;
const float CLEAR_DISTANCE_CENTIMETRES = 40.0;

// Smoothing & timing
const int   SONAR_SAMPLE_SIZE            = 5;
const unsigned long SONAR_SAMPLE_INTERVAL_MS = 75;
const unsigned long SPEED_SAMPLE_INTERVAL_MS = 10;
const int   ACCELERATION_VALUE           = 5; // ramp step (PWM units)
```

---

## 🛠️ Setup & Upload

1. **Install Arduino IDE**.
2. Install the **IRremote Library**.
3. **Wire the hardware** as per the pinout above.
4. **Open the sketch** and select your board and port.
5. **Upload** and open **Serial Monitor @ 9600 baud** to see distance logs.

---

## 🧠 How It Works

* **IR input** (`handleIR`) decodes remote commands and routes them to `direction()`.
* **Speed ramping** (`updateMotorSpeed`) smoothly approaches `targetSpeed` in small steps every `SPEED_SAMPLE_INTERVAL_MS`.
* **Ultrasonic averaging** (`averageDistanceCentimetres`) samples HC-SR04 readings and maintains a rolling average for stability.
* **Blocked logic**: If average distance ≤ `STOP_DISTANCE_CENTIMETRES`, the car stops and enters a **blocked** state. It only clears when distance ≥ `CLEAR_DISTANCE_CENTIMETRES`.

---

## ▶️ Run

* Power the motors and Arduino (share ground).
* Point your IR remote at the receiver and use the controls above.
* Watch the Serial Monitor for `DISTANCE:` logs while testing.

---

## 🧩 Troubleshooting

* **Only turns one way / wheels fight each other**: Swap a motor’s polarity or check `IN` pin wiring.
* **No IR response**: Confirm `IR_RECEIVER_PIN` (D8), remote battery, and that the `IRremote` library is installed.
* **Constant “NAN” distance**: Check HC-SR04 wiring (5V, GND, TRIG=12, ECHO=13) and ensure nothing blocks the sensor.
* **PWM not changing speed**: Make sure you’re on PWM pins **D9** and **D10** (labeled `~` on many boards).
  
---

## 🙌 Credits
- **IRremote library** by Ken Shirriff, Rafi Khan, Armin Joachimsmeyer et al.  
  Licensed under the MIT License.  

