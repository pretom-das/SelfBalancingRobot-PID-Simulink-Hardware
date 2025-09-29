Here’s a **ready-to-paste README wiring section** for your self-balancing robot (UNO + MPU6050 + SH1106 + L298N). It matches the pin names in your sketch.

# Wiring & Power

## Bill of Materials

* Arduino **UNO** (or Nano with same pins)
* **MPU6050** 6-axis IMU (I²C)
* **SH1106** 1.3″ 128×64 OLED (I²C, 0x3C default)
* **L298N** dual H-bridge motor driver
* 2× DC gear motors (left/right)
* Battery pack (recommended **2S Li-ion/LiPo 7.4 V** or **6×AA NiMH 7.2 V**)
* 100–470 µF electrolytic cap across motor supply (near L298N)
* Slide switch (battery power)
* Wires, breadboard/headers

> **Power rule:** share **common GND** between UNO, L298N, MPU6050, OLED.

---

## Pin Map (Arduino UNO)

### I²C Devices (MPU6050 + SH1106 OLED)

| Function  | MPU6050        | SH1106 OLED    | UNO Pin                                  |
| --------- | -------------- | -------------- | ---------------------------------------- |
| Power     | VCC (3.3–5 V)* | VCC (3.3–5 V)* | **5V** (or 3.3V if your modules require) |
| Ground    | GND            | GND            | **GND**                                  |
| I²C Clock | SCL            | SCL            | **A5 (SCL)**                             |
| I²C Data  | SDA            | SDA            | **A4 (SDA)**                             |

* Most breakout boards are 5 V-tolerant on I²C; check yours. Both modules can run from 3.3 V as well, but keep **both** on the same logic level as the UNO I²C.

**I²C addresses**

* MPU6050: `0x68` (AD0=LOW) or `0x69` (AD0=HIGH)
* SH1106: `0x3C` (your sketch uses this)

---

### Motor Driver (L298N) ↔ Arduino

Your sketch uses:

* `ENA → D5 (PWM)`
* `IN1 → D7`, `IN2 → D8`
* `ENB → D6 (PWM)`
* `IN3 → D12`, `IN4 → D13`

**Connect:**

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
| **5V (on L298N)** | **Do NOT** feed UNO from here. Leave unconnected or remove 5 V jumper if present. |

> If your L298N has a **5V-EN** jumper: with **motor V_s ≥ 7 V**, the board’s regulator can power *its own* logic. Still **do not** back-power the UNO from the L298N 5 V pin.

---

### Powering the System

* **UNO logic:** from USB during development, or a clean 5 V regulator (e.g., buck converter from the battery) into **5V** pin.
  *Avoid VIN/barrel jack if your battery is only ~7–8 V; the onboard linear regulator may overheat / drop too much.*
* **Motors:** battery → L298N **12V (V_s)**.
* **Grounds:** connect UNO **GND** ↔ L298N **GND** ↔ battery **−** ↔ sensor GND.

**Decoupling:** place a **100–470 µF** electrolytic across **V_s–GND** at the L298N, plus a **0.1 µF** ceramic near each module (optional but helpful).

---

## Quick Text Schematic (overview)

```
Battery + ------- L298N 12V (V_s)
Battery - -----+-- L298N GND
               | 
               +-- UNO GND --- MPU6050 GND --- OLED GND
UNO 5V  --------- MPU6050 VCC --- OLED VCC
UNO A5 (SCL) ---- MPU6050 SCL --- OLED SCL
UNO A4 (SDA) ---- MPU6050 SDA --- OLED SDA

UNO D5 (PWM) ---- L298N ENA
UNO D7 ---------- L298N IN1
UNO D8 ---------- L298N IN2
UNO D6 (PWM) ---- L298N ENB
UNO D12 --------- L298N IN3
UNO D13 --------- L298N IN4

L298N OUT1/OUT2 -> Left Motor
L298N OUT3/OUT4 -> Right Motor
```

---

## Motor Polarity & Direction

* If wheels spin the **wrong way**, swap a motor’s **OUTx** wires **or** use the code switches:

  * `#define DRIVE_SIGN (-1)` → ensures **negative angle ⇒ wheels go backward** (as requested).
  * If the robot “pushes the fall,” flip sensor signs: `SIGN_PITCH` or `SIGN_GYRO` to `-1`.

---

## First-Power Checklist

1. **Upload code** with robot wheels lifted off the ground.
2. Open Serial Monitor @ **115200** (optional).
3. Verify OLED shows **“Calibrating…”**, then **“Auto-tuning…”** or **“BALANCING”**.
4. Nudge the chassis: angle on OLED should increase for **nose-up**.
5. If motion pushes the robot further over, flip `SIGN_PITCH` or `SIGN_GYRO` (not wiring).
6. If motors just buzz, increase `MOTOR_DEADBAND` (30→40–50) and/or `relay_h` (90→120).

---

## Common Gotchas

* **No common ground** → nothing works reliably.
* **Feeding UNO from L298N 5 V** → brownouts/reset under motor load. Use a separate 5 V regulator for the UNO.
* **Wrong I²C address** → blank OLED. Your code uses `0x3C`. Some SH1106 boards can be `0x3D`.
* **Loose Dupont leads** on ENA/ENB → no PWM control.

---

## Optional: Power Budget Tips

* L298N is lossy; with low-voltage batteries the headroom may be small. If you can, use **higher-torque gear motors** or a more efficient driver (TB6612FNG, BTS7960) later.
* Keep wires short and thick on motor supply; add the bulk cap at L298N.

---

## Testing Sequence

1. **I²C check:** run an I²C scanner; expect `0x68` (MPU) and `0x3C` (OLED).
2. **Sensor sign:** tilt forward/back and watch angle on OLED.
3. **Lifted wheel test:** in **RUN** mode, tilt the robot by hand; wheels should spin to resist.
4. **Floor test:** start near upright, be ready to catch. Use the built-in tilt cut-off (±35°).

---

Copy this into your repo’s `README.md` under a “Wiring & Power” section, and you’re set. If you want, I can add a neat wiring diagram (Fritzing-style PNG/SVG) for the same pinout.
