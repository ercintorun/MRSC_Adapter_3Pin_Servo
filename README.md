# Advanced MRSC (Micro RC Stability Control) - V0.4.1

![Version](https://img.shields.io/badge/Version-0.4.1-blue)
![Platform](https://img.shields.io/badge/Platform-Arduino_Pro_Mini-orange)
![License](https://img.shields.io/badge/License-MIT-green)

## 📝 Overview
This project is an advanced, highly optimized fork of the original [MRSC by TheDIYGuy999](https://github.com/TheDIYGuy999/MRSC_Adapter_3Pin_Servo). While the original V0.3 code was designed primarily for 8MHz / 3.3V microcontrollers, this **V0.4.1 upgrade** has been fully re-engineered for **16MHz / 5V Arduino Pro Mini (ATmega328P)** boards.

This version introduces professional-grade safety algorithms, dynamic driving physics, and ultra-low latency communication for high-performance RC applications (Drift, Touring, Off-road).

---

## 🚀 Key Features & Upgrades

### 1. 16MHz Clock Optimization
Re-engineered timing intervals and PWM signal processing to run flawlessly on 5V/16MHz hardware, ensuring precise pulse width measurement and stable servo output.

### 2. Remote Gain Control (On-the-Fly Tuning)
Adjust gyro sensitivity dynamically (0% to 100%) using a spare transmitter channel (CH3/CH4). This allows the driver to adapt to different track surfaces (asphalt, tile, carpet) without stopping the vehicle.

### 3. Fast I2C Mode (400kHz)
Boosted the I2C communication clock to **400kHz**. This minimizes the data bottleneck between the MPU-6050 and the MCU, providing ultra-low latency gyro feedback for lightning-fast counter-steering.

### 4. Heading Hold Mode
Automatically locks the vehicle's yaw angle during straight-line runs. Under high gain, this feature prevents spin-outs and "fishtailing" during heavy acceleration on straightaways.

### 5. Dynamic Cornering Physics
Intelligently reduces gyro intervention proportional to the driver's steering input. This prevents the "gyro fighting" sensation during sharp turns, offering a much more natural and intuitive steering feel.

### 6. Anti-Jitter Protection (Atomic Reads)
Utilizes **Atomic Read** operations by briefly suspending interrupts during 32-bit variable copying. This completely eliminates random servo "glitches" or twitching caused by overlapping interrupt signals.

### 7. Smart Failsafe System
Continuously monitors signal integrity. If the receiver signal is lost, corrupted, or disconnected for more than **50ms**, the system instantly centers the steering servo to prevent high-speed crashes.

### 8. Auto-Calibration (End-Point Protection)
Learns your steering limits dynamically upon boot. By moving your steering to full lock once, the code saves the mechanical limits, protecting your servo from stripping gears or burning out.

---

## 🛠 Hardware Requirements
* **MCU:** Arduino Pro Mini (5V, 16MHz - ATmega328P)
* **IMU:** MPU-6050 (GY-521 Module)
* **USB-TTL:** CP2102 or FTDI for programming
* **Receiver:** Any standard PWM RC Receiver

---

## 📋 Pin Mapping

| Component | Arduino Pin | Function |
| :--- | :--- | :--- |
| **Receiver CH2** | `D4` | Steering PWM Input |
| **Receiver CH3** | `D5` | Remote Gain PWM Input |
| **Steering Servo** | `A0` | Stabilized Servo PWM Output |
| **Inversion Jumper**| `A2` | Bridge to GND to reverse gyro direction |
| **MPU-6050 SDA** | `A4` | I2C Data Line |
| **MPU-6050 SCL** | `A5` | I2C Clock Line |

---

## ⚙️ Installation & Usage
1. Mount the MPU-6050 **completely parallel** to the car's chassis.
2. Connect the pins as shown in the table above.
3. Upload the code using the **"ATmega328P (Old Bootloader)"** setting in Arduino IDE.
4. **Calibration:** Upon power-up, move your transmitter's steering stick to full left and full right once. The system will learn these limits.
5. **Direction Check:** If the car counter-steers in the wrong direction, connect pin **A2 to GND** to digitally invert the logic.

---

## 🤝 Credits & Acknowledgments
* Original logic by **TheDIYGuy999** ([Original Repository](https://github.com/TheDIYGuy999/MRSC_Adapter_3Pin_Servo)).
* V0.4.1 upgrades and 16MHz optimizations developed with the assistance of an **AI Tutor**.

---
*Developed for the RC community. Feel free to fork and improve!*
