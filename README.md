# Robotics Lab Assignment 2 — Human Augmentation Device (1-DoF Linear Actuator + IMU Gesture)

Implementation of a human augmentation device to assist a **1-DoF linear motion** using an **IMU gesture** and a **stepper-driven slider**.  
**Note:** final version has **NO Y motor control** (pitch is only streamed/visualized).

**Course**: Robotics, P2 2025–2026  
**Instructor**: joao.silva.sequeira@tecnico.ulisboa.pt  
**Due**: 16-01-2026, 23:59:59

## Repository contents
- `arduino/` (or `.ino`) — **Main firmware**: IMU + complementary filter + X-gesture + stepper homing/center + button push + CSV stream  
- `processing/` (or `.pde`) — **Digital Twin UI**: serial CSV parsing + 3D board + live plots + handshake

---

## First Steps — Setup

### Prerequisites
- Arduino IDE
- Processing IDE (for the digital twin)

### Run (Arduino)
1. Open `codeLab2/codeLab2.ino` and **upload** to Arduino Uno.
2. Open **Serial Monitor @ 115200**.
3. At boot:
   - keep the IMU **still for ~2s** (gyro bias calibration)
   - the slider will **home automatically** (LEFT → RIGHT → move to center)

### Gesture execution (X / Roll)
The “Push” action triggers when the **roll angle** exceeds the threshold and is held briefly:
- `|roll| > 90°` for at least `120 ms`
- cooldown after trigger: `700 ms`

### Serial Commands
Type in Serial Monitor (case-sensitive):
- `H` : Home + center
- `P` : Manual button push
- `C` : Calibrate gyro bias (keep IMU still)
- `S` : Start CSV streaming (prints header)
- `s` : Stop CSV streaming
- `?` : Help

---

## Digital Twin Visualization (Processing)
1. Open `processing/digital_twin.pde`.
2. Set the correct serial port index:
   ```java
   final int PORT_INDEX = 2;


(Processing prints `Serial.list()` at startup)
3. Run the sketch.

**Handshake:** Processing sends `S` every 500 ms until it receives the CSV header.
**Controls:** mouse drag = rotate camera, `C` reconnect, `I/O/U` invert axes, `R` reset yaw.

### CSV stream (12 fields)

```text
t_ms,ax,ay,az,gx,gy,gz,angleX_deg,angleY_deg,angleY_smooth,gestureX,posSteps
```
