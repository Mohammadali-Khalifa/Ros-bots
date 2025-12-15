#include <ESP32Servo.h>

Servo Left_arm;
Servo Right_arm;
Servo Gripper;

int Degrees = 10;
int Delay = 15;

int Arm = 0;                 // arm angle (0 to 160)
int Gripper_pos = 100;       // gripper angle

int Gripper_open = 100;
int Gripper_close = 180;

void setup() {
  Serial.begin(115200);

  Left_arm.attach(D9);
  Right_arm.attach(D8);
  Gripper.attach(D10);

  // start positions
  Left_arm.write(Arm);
  Right_arm.write(180 - Arm);      // mirrored
  Gripper.write(Gripper_pos);

  Serial.println("u=up d=down o=open c=close E=stop");
}

void loop() {
  if (!Serial.available()) return;

  char cmd = Serial.read();

  if (cmd == '\n' || cmd == '\r') return; //reads after the end space or return

  if (cmd == 'E' || cmd == 'e') {
    Left_arm.detach();
    Right_arm.detach();
    Gripper.detach();
    Serial.println("STOP");
    return;
  }
  //lines 38 5o 44 e stops eveyrthing

  if (!Left_arm.attached()) Left_arm.attach(D9);
  if (!Right_arm.attached()) Right_arm.attach(D8);
  if (!Gripper.attached()) Gripper.attach(D10);
  // lines 47 to 49 resumes all movment after e stop

  if (cmd == 'u' || cmd == 'U') {
    Arm += Degrees;
    if (Arm > 160) Arm = 160;
  }
  // lines 52 to 55 moves arm up by degrees amount when U is read

  if (cmd == 'd' || cmd == 'D') {
    Arm -= Degrees;
    if (Arm < 0) Arm = 0;
  }
  // lines 58 to 61 moves arm down by degrees amount when D is read

  if (cmd == 'o' || cmd == 'O') {
    Gripper_pos -= Degrees;
    if (Gripper_pos < Gripper_open) Gripper_pos = Gripper_open;
  }
  // lines 64 to 67 opens the griper by degrees amount when O is read

  if (cmd == 'c' || cmd == 'C') {
    Gripper_pos += Degrees;
    if (Gripper_pos > Gripper_close) Gripper_pos = Gripper_close;
  }
  // lines 70 to 73 closes the griper by degrees amount when O is read

  Left_arm.write(Arm);
  Right_arm.write(180 - Arm);      // mirrored
  Gripper.write(Gripper_pos);

  delay(Delay);
}
