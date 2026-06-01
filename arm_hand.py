import cv2
import mediapipe as mp
import math
import serial
import time

# ---------------- SERIAL ----------------
# Change 'COM3' to your Arduino port (e.g. '/dev/ttyUSB0' on Linux/Mac)
SERIAL_PORT = 'COM3'
BAUD_RATE = 9600

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)  # Wait for Arduino to reset
    print(f"Connected to Arduino on {SERIAL_PORT}")
except serial.SerialException as e:
    print(f"Serial error: {e}")
    ser = None

def send_servo(label, value):
    """Send a single servo command and wait for OK."""
    if ser is None:
        return
    cmd = f"{label} {value}\n"
    ser.write(cmd.encode())
    # Read response (non-blocking, best-effort)
    if ser.in_waiting:
        ser.readline()

def send_grip(state):
    """Send S4 OPEN or S4 CLOSE."""
    if ser is None:
        return
    cmd = f"S4 {'OPEN' if state == 0 else 'CLOSE'}\n"
    ser.write(cmd.encode())
    if ser.in_waiting:
        ser.readline()

# ---------------- INIT ----------------
mp_hands = mp.solutions.hands
mp_pose = mp.solutions.pose

hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=1,
    min_detection_confidence=0.7,
    min_tracking_confidence=0.7
)

pose = mp_pose.Pose(
    static_image_mode=False,
    model_complexity=0,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5
)

cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)

# ---------------- STATE ----------------
prev_elbow  = 90
prev_rot    = 90
prev_grip   = 0
prev_elbow2 = 90

# Track last sent values to avoid spamming identical commands
last_sent = {'S1': -1, 'S2': -1, 'S3': -1, 'S4': -1}

mode = 0
prev_fist = False

# Send interval throttle (seconds) — Arduino's moveSmooth blocks, so don't flood it
SEND_INTERVAL = 0.15
last_send_time = 0

# ---------------- UTIL ----------------
def get_angle(p1, p2, p3):
    a = [p1.x - p2.x, p1.y - p2.y]
    b = [p3.x - p2.x, p3.y - p2.y]
    dot = a[0]*b[0] + a[1]*b[1]
    ma = (a[0]**2 + a[1]**2)**0.5
    mb = (b[0]**2 + b[1]**2)**0.5
    return math.degrees(math.acos(dot / (ma * mb + 1e-6)))

def is_fist(lm):
    def ang(p1, p2, p3):
        a = [p1.x - p2.x, p1.y - p2.y]
        b = [p3.x - p2.x, p3.y - p2.y]
        dot = a[0]*b[0] + a[1]*b[1]
        ma = (a[0]**2 + a[1]**2)**0.5
        mb = (b[0]**2 + b[1]**2)**0.5
        return math.degrees(math.acos(dot / (ma * mb + 1e-6)))
    return (
        ang(lm[5],  lm[6],  lm[8])  < 90 and
        ang(lm[9],  lm[10], lm[12]) < 90 and
        ang(lm[13], lm[14], lm[16]) < 90 and
        ang(lm[17], lm[18], lm[20]) < 90
    )

def is_pinch(lm):
    thumb = lm[4]
    index = lm[8]
    dist = ((thumb.x - index.x)**2 + (thumb.y - index.y)**2)**0.5
    return dist < 0.05

def filter_signal(new, prev, alpha=0.12, max_step=3, dead_zone=3):
    if abs(new - prev) < dead_zone:
        return prev
    smooth = alpha * new + (1 - alpha) * prev
    smooth = max(prev - max_step, min(prev + max_step, smooth))
    return smooth

def map_elbow(a):
    a = max(30, min(160, a))
    return (a - 30) / 130 * 180

def map_rot(x):
    x = max(-0.3, min(0.3, x))
    return (x + 0.3) / 0.6 * 180

def try_send(label, value):
    """Send only if value changed by more than 1 degree."""
    if abs(value - last_sent[label]) > 1:
        send_servo(label, value)
        last_sent[label] = value

def try_send_grip(value):
    """Send grip only on state change."""
    if value != last_sent['S4']:
        send_grip(value)
        last_sent['S4'] = value

# ---------------- LOOP ----------------
while True:
    ret, img = cap.read()
    if not ret:
        break

    img = cv2.flip(img, 1)
    rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    hand_res = hands.process(rgb)
    pose_res = pose.process(rgb)

    elbow_servo  = int(prev_elbow)
    rot_servo    = int(prev_rot)
    grip_servo   = int(prev_grip)
    elbow2_servo = int(prev_elbow2)
    raw_elbow2_angle = 0.0

    # ---------------- MODE SWITCH (FIST) ----------------
    if hand_res.multi_hand_landmarks:
        for hand in hand_res.multi_hand_landmarks:
            lm = hand.landmark
            fist_now = is_fist(lm)
            if fist_now and not prev_fist:
                mode = (mode + 1) % 4
            prev_fist = fist_now

    # ---------------- POSE CONTROL ----------------
    if pose_res.pose_landmarks:
        lm = pose_res.pose_landmarks.landmark

        s = lm[mp_pose.PoseLandmark.RIGHT_SHOULDER]
        e = lm[mp_pose.PoseLandmark.RIGHT_ELBOW]
        w = lm[mp_pose.PoseLandmark.RIGHT_WRIST]

        if s.visibility > 0.6 and e.visibility > 0.6 and w.visibility > 0.6:

            if mode == 0:  # ARM CONTROL
                raw_elbow = map_elbow(get_angle(s, e, w))
                prev_elbow = filter_signal(raw_elbow, prev_elbow)
                elbow_servo = int(prev_elbow)

                raw_rot = map_rot(w.x - s.x)
                prev_rot = filter_signal(raw_rot, prev_rot)
                rot_servo = int(prev_rot)

            elif mode == 1:  # GRIP CONTROL
                if hand_res.multi_hand_landmarks:
                    for hand in hand_res.multi_hand_landmarks:
                        hlm = hand.landmark
                        grip_servo = 1 if is_pinch(hlm) else 0
                        prev_grip = grip_servo

            elif mode == 2:  # HOLD — keep all previous values
                pass

            elif mode == 3:  # SECONDARY ARM
                ls = lm[mp_pose.PoseLandmark.LEFT_SHOULDER]
                le = lm[mp_pose.PoseLandmark.LEFT_ELBOW]
                lw = lm[mp_pose.PoseLandmark.LEFT_WRIST]

                if ls.visibility > 0.6 and le.visibility > 0.6 and lw.visibility > 0.6:
                    raw_elbow2_angle = get_angle(ls, le, lw)
                    raw_elbow2 = map_elbow(raw_elbow2_angle)
                    prev_elbow2 = filter_signal(raw_elbow2, prev_elbow2)
                    elbow2_servo = int(prev_elbow2)

    # ---------------- SERIAL SEND (throttled) ----------------
    now = time.time()
    if now - last_send_time >= SEND_INTERVAL:
        last_send_time = now
        try_send('S1', elbow_servo)
        try_send('S2', rot_servo)
        try_send('S3', elbow2_servo)
        try_send_grip(grip_servo)

    # ---------------- UI ----------------
    cv2.putText(img, f"MODE: {mode}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    modes = ["ARM CONTROL", "GRIP CONTROL", "HOLD", "SECONDARY ARM"]
    cv2.putText(img, modes[mode], (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.putText(img, f"Elbow (S1): {elbow_servo}", (10, 100),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.putText(img, f"Rot   (S2): {rot_servo}", (10, 130),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
    cv2.putText(img, f"Grip  (S4): {'CLOSE' if grip_servo else 'OPEN'}", (10, 160),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    label_color = (0, 200, 255) if mode == 3 else (120, 120, 120)
    cv2.putText(img, f"Elbow2 (S3): {elbow2_servo}", (10, 200),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, label_color, 2)
    if mode == 3:
        cv2.rectangle(img, (5, 185), (360, 245), (0, 200, 255), 2)

    ser_status = "SERIAL: OK" if ser else "SERIAL: DISCONNECTED"
    cv2.putText(img, ser_status, (10, 240),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (0, 255, 100) if ser else (0, 0, 255), 2)

    cv2.imshow("Industrial Robotic Arm System", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
if ser:
    ser.close()