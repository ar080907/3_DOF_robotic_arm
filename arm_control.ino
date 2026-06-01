/*
  Robotic Arm - Serial Control
  Receives commands from Python camera script.

  Protocol (one command per line):
    S1 <angle>   → servo1 (elbow,  0-180)
    S2 <angle>   → servo2 (rotation, 0-180)
    S3 <angle>   → servo3 (elbow2, 0-180)
    S4 OPEN      → servo4 grip open  (90°)
    S4 CLOSE     → servo4 grip close (25°)
  
  Reply: "OK\n" after every command.
*/

#include <Servo.h>

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

int current1 = 90;
int current2 = 90;
int current3 = 90;
int current4 = 90;

// Smaller step delay = faster but more load on motor
const int STEP_DELAY = 12;

void moveSmooth(Servo &s, int &currentPos, int target, int stepDelay) {
  target = constrain(target, 0, 180);
  if (currentPos == target) return;

  int step = (currentPos < target) ? 1 : -1;
  while (currentPos != target) {
    currentPos += step;
    s.write(currentPos);
    delay(stepDelay);
  }
}

void setup() {
  Serial.begin(9600);

  servo1.attach(9);
  servo2.attach(10);
  servo3.attach(11);
  servo4.attach(12);

  servo1.write(90);
  servo2.write(90);
  servo3.write(90);
  servo4.write(90);
  delay(500);

  Serial.println("READY");
}

void loop() {
  if (!Serial.available()) return;

  String input = Serial.readStringUntil('\n');
  input.trim();
  if (input.length() == 0) return;

  // --- S4 special commands ---
  if (input == "S4 OPEN") {
    moveSmooth(servo4, current4, 90, STEP_DELAY);
    Serial.println("OK");
    return;
  }
  if (input == "S4 CLOSE") {
    moveSmooth(servo4, current4, 25, STEP_DELAY);
    Serial.println("OK");
    return;
  }

  // --- Generic: "Sx <angle>" ---
  int spaceIdx = input.indexOf(' ');
  if (spaceIdx == -1) {
    Serial.println("ERR");
    return;
  }

  String servoName = input.substring(0, spaceIdx);
  int angle = constrain(input.substring(spaceIdx + 1).toInt(), 0, 180);

  if      (servoName == "S1") moveSmooth(servo1, current1, angle, STEP_DELAY);
  else if (servoName == "S2") moveSmooth(servo2, current2, angle, STEP_DELAY);
  else if (servoName == "S3") moveSmooth(servo3, current3, angle, STEP_DELAY);
  else {
    Serial.println("ERR");
    return;
  }

  Serial.println("OK");
}
