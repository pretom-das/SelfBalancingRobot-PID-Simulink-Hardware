# PID Controlled Self-Balancing Robot

This repository contains all resources for the project:

**"PID Controlled Self-Balancing Robot using MATLAB Simulink and Hardware Implementation: Evaluating Performance under Disturbance"**

---

## 📂 Folder Structure
- `3d-print/` – STL files for chassis, wheels, and mounts.
- `hardware/` – Arduino code for MPU6050, PID controller, motor driver, and OLED HUD.
- `Simulink/` – MATLAB Simulink models and simulation results.
- `papers/` – Research paper (LaTeX + figures).

---

## 🚀 Features
- Real-time balancing using MPU6050 DMP.
- PID controller designed in MATLAB Simulink and validated in hardware.
- OLED HUD for live visualization of angles, PID output, and setpoint.
- STL files for custom 3D-printed chassis and parts.
- Disturbance performance evaluation (push tests, uneven surface, etc.).

---

## 🛠️ Requirements
- MATLAB/Simulink R2021a or later
- Arduino IDE
- MPU6050, L298N, DC motors, SH1106 OLED
- 3D printer for mechanical parts

---

## 📊 Results
- Successfully balanced robot under controlled disturbances.
- Simulink and hardware results compared for validation.

---


---

## Bill of Materials

* Arduino **UNO** (or Nano with same pins)
* **MPU6050** 6-axis IMU (I²C)
* **SH1106** 1.3″ 128×64 OLED (I²C, 0x3C default)
* **L298N** dual H-bridge motor driver
* 2× DC gear motors (left/right)
* Battery pack (recommended **2S Li-ion/LiPo 7.4 V** or **6×AA NiMH 7.2 V**)
* 100–470 µF electrolytic capacitor (across motor supply near L298N)
* Slide switch (battery power)
* Wires, breadboard/headers

> **Power rule:** share **common GND** between UNO, L298N, MPU6050, OLED.

---

## ⚡ Wiring & Power

### I²C Devices (MPU6050 + SH1106 OLED)

| Function  | MPU6050        | SH1106 OLED    | UNO Pin                                  |
| --------- | -------------- | -------------- | ---------------------------------------- |
| Power     | VCC (3.3–5 V)* | VCC (3.3–5 V)* | **5V** (or 3.3V if required)             |
| Ground    | GND            | GND            | **GND**                                  |
| I²C Clock | SCL            | SCL            | **A5 (SCL)**                             |
| I²C Data  | SDA            | SDA            | **A4 (SDA)**                             |

*Most breakout boards are 5 V-tolerant on I²C; check yours.*

**I²C addresses:**
- MPU6050: `0x68` (AD0=LOW) or `0x69` (AD0=HIGH)
- SH1106: `0x3C` (your sketch uses this)

---

### Motor Driver (L298N) ↔ Arduino

* `ENA → D5 (PWM)`
* `IN1 → D7`, `IN2 → D8`
* `ENB → D6 (PWM)`
* `IN3 → D12`, `IN4 → D13`

| L298N Pin         | Connect to                                                                        |
| ----------------- | --------------------------------------------------------------------------------- |
| **ENA**           | UNO **D5** (PWM)                                                                  |
| **IN1**           | UNO **D7**                                                                        |
| **IN2**           | UNO **D8**                                                                        |
| **IN3**           | UNO **D12**                                                                       |
| **IN4**           | UNO **D13**                                                                       |
| **ENB**           | UNO **D6** (PWM)                                                                  |
| **OUT1/OUT2**     | Left motor terminals                                                              |
| **OUT3/OUT4**     | Right motor terminals                                                             |
| **12V (V_s)**     | Battery **+** (motor supply)                                                      |
| **GND**           | Battery **−** **and** UNO **GND** (common ground)                                 |
| **5V (on L298N)** | **Do NOT** feed UNO from here. Leave unconnected or remove jumper if present.     |

---

### Powering the System

- **UNO logic:** from USB (development) or a 5 V buck converter from the battery.  
- **Motors:** battery → L298N **12V (V_s)**.  
- **Grounds:** UNO **GND** ↔ L298N **GND** ↔ battery **−** ↔ sensor GND.  

**Decoupling:** add a **100–470 µF** electrolytic across **V_s–GND** near the L298N.





## 📄 License
This project is licensed under the MIT License.


