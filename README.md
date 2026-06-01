#  Vision-Controlled Robotic Arm

Control a 4-servo robotic arm using nothing but your body — no joysticks, no controllers. A Python vision pipeline reads your arm angles and hand gestures in real time via webcam and translates them into smooth servo movements on an Arduino.

---

##  Features

- **Real-time pose tracking** — maps your right arm's elbow angle and lateral position to servo motors
- **Gesture-based mode switching** — make a fist to cycle between control modes without touching anything
- **Pinch-to-grip** — pinch your thumb and index finger to close the robotic gripper
- **Secondary arm control** — mode 3 lets you drive a second joint using your left arm
- **Smooth motion** — Arduino-side `moveSmooth()` prevents jerky servo jumps; Python-side exponential filtering kills noise
- **Throttled serial comms** — commands are sent at a fixed interval and only when values actually change, so the Arduino never gets flooded

---

##  Project Structure

```
robotic-arm/
├── arm_control.ino       # Arduino sketch — receives serial commands, drives 4 servos
└── arm_vision.py         # Python script — webcam → MediaPipe → serial → Arduino
```

---

##  Hardware Requirements

| Component | Details |
|---|---|
| Arduino (Uno/Nano/Mega) | Any board with 4 PWM-capable digital pins |
| Servo motors × 4 | Standard 5V hobby servos |
| Webcam | Any USB or built-in camera |
| USB cable | Arduino ↔ PC serial connection |
| Power supply | Servos draw significant current — use a dedicated 5V rail, not the Arduino's 5V pin |

**Servo wiring:**

| Servo | Arduino Pin | Function |
|---|---|---|
| S1 | D9 | Right elbow |
| S2 | D10 | Base rotation |
| S3 | D11 | Secondary elbow (left arm) |
| S4 | D12 | Gripper (open/close) |

---

##  Software Requirements

### Python dependencies

```bash
pip install opencv-python mediapipe pyserial
```

Tested on Python .

### Arduino

- Arduino IDE 1.8+ or Arduino CLI
- `Servo.h` — included with the Arduino IDE (no extra install needed)

---

##  Setup & Usage

### 1. Flash the Arduino

1. Open `arm_control.ino` in the Arduino IDE.
2. Select your board and port under **Tools**.
3. Upload. Open the Serial Monitor at **9600 baud** — you should see `READY`.

### 2. Configure the serial port

Edit the top of `arm_vision.py`:

```python
SERIAL_PORT = 'COM3'      # Windows example
# SERIAL_PORT = '/dev/ttyUSB0'   # Linux
# SERIAL_PORT = '/dev/tty.usbmodem14101'  # macOS
```

### 3. Run the Python script

```bash
python arm_vision.py
```

A webcam window will open. Stand back so your full upper body is visible.

### 4. Press `Q` to quit.

---

##  Control Modes

Cycle modes by **making a fist** — each fist closes the loop to the next mode.

| Mode | Name | What it does |
|---|---|---|
| 0 | **ARM CONTROL** | Right arm angle → S1 (elbow). Wrist horizontal position → S2 (rotation). |
| 1 | **GRIP CONTROL** | Pinch thumb + index finger → close gripper (S4). Release → open. |
| 2 | **HOLD** | Freezes all servos at their current positions. |
| 3 | **SECONDARY ARM** | Left arm angle drives S3 (secondary elbow). Highlighted with an orange border. |

---

##  Serial Protocol

The Python script and Arduino communicate over a simple newline-terminated text protocol.

**Python → Arduino:**

```
S1 <angle>    # Set servo 1 to angle (0–180)
S2 <angle>    # Set servo 2 to angle (0–180)
S3 <angle>    # Set servo 3 to angle (0–180)
S4 OPEN       # Open gripper (90°)
S4 CLOSE      # Close gripper (25°)
```

**Arduino → Python:**

```
READY         # Sent once on boot
OK            # Sent after every successful command
ERR           # Sent on unrecognised command
```

---

##  Tuning

| Parameter | Location | Effect |
|---|---|---|
| `SEND_INTERVAL` | `arm_vision.py` | Seconds between serial bursts. Lower = more responsive, higher = more stable. Default: `0.15` |
| `STEP_DELAY` | `arm_control.ino` | Milliseconds per degree of servo movement. Lower = faster, higher = smoother. Default: `12` |
| `alpha` in `filter_signal()` | `arm_vision.py` | EMA smoothing factor (0–1). Lower = smoother but laggier. Default: `0.12` |
| `dead_zone` in `filter_signal()` | `arm_vision.py` | Minimum change (degrees) before a new value is accepted. Default: `3` |
| Detection confidence thresholds | `arm_vision.py` | Raise if you get false positives; lower if tracking drops out. |

---

##  Troubleshooting

**Serial connection fails on startup**
- Check the port name in `arm_vision.py` matches what appears in Device Manager (Windows) or `ls /dev/tty*` (Linux/macOS).
- Make sure the Arduino IDE's Serial Monitor is closed — only one program can hold the port at a time.

**Servos twitch or stutter**
- Power your servos from a dedicated supply, not the Arduino's 5V pin. Voltage drops cause erratic behaviour.
- Increase `STEP_DELAY` in the `.ino` file for smoother movement.

**Pose/hand not detected**
- Ensure adequate, even lighting. MediaPipe struggles with backlighting.
- Move further from the camera so your full upper body is in frame.
- Lower `min_detection_confidence` thresholds slightly in `arm_vision.py`.

**Gripper doesn't respond in mode 1**
- The grip mode reads hand landmarks, so your hand must also be visible to the camera while in mode 1.

---
