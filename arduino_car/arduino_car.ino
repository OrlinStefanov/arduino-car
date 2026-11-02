// Arduino car with a three-servo claw, driven by Mayday over Bluetooth or by the IR remote.
//
// A Bluetooth module (HC-05 / HC-06, 9600 baud) on the hardware serial pins 0 and 1 carries
// one text command per line (see car_protocol.h). The IR remote keeps working. Whoever sent the
// last drive command is in charge; MODE puts the car on one of its self-driving programs.
//
// Safety: a drive command without a time stops 600 ms after the last one, and a self-driving mode
// started over Bluetooth ends after 3 s without any Bluetooth command (Mayday pings every second),
// so a lost link or a closed page stops the car. Nothing but the answer to "?" is ever written to Serial, because
// that is the Bluetooth link.
#include <Servo.h>
#include "IR_remote.h"
#include "Keymap.h"
#include "car_protocol.h"

// ---- Pins ----
const int PIN_LEFT_DIR = 2, PIN_LEFT_PWM = 5, PIN_RIGHT_DIR = 4, PIN_RIGHT_PWM = 6;
const int PIN_CLAW = 9, PIN_ARM = 10, PIN_BASE = 11;
const int PIN_TRIG = 12, PIN_ECHO = 13;
const int PIN_LINE_LEFT = 7, PIN_LINE_CENTER = 8, PIN_LINE_RIGHT = A1;
const int PIN_IR = 3;

// ---- Claw: roles as the original remote code used them (servo 1 = claw, 2 = arm, 3 = base) ----
const int CLAW_MIN = 50, CLAW_MAX = 180;     // below 50 the claw squeezes itself
const int ARM_MIN = 0, ARM_MAX = 135;
const int BASE_MIN = 0, BASE_MAX = 180;
const int CLAW_START = CLAW_MIN, ARM_START = 0, BASE_START = 0;
const int SERVO_STEP = 5;                    // degrees per IR key press

const unsigned long DEADMAN_MS = 600;        // a drive without a time runs this long
const unsigned long STEP_MS = 50;            // how often a self-driving mode looks around
const unsigned long LINK_TIMEOUT_MS = 3000;  // a mode started over Bluetooth needs a command at least this often

const int BLACK_LINE = 1;

Servo clawServo, armServo, baseServo;
int clawAngle = CLAW_START, armAngle = ARM_START, baseAngle = BASE_START;

IRremote ir(PIN_IR);

byte mode = MODE_MANUAL;
bool modeNeedsLink = false;                  // started over Bluetooth, so it ends when the link goes quiet
unsigned long linkSeen = 0;
bool driving = false;
unsigned long driveUntil = 0;
unsigned long nextStep = 0;
byte avoidPhase = 0;                         // 0 cruising, 1 backing up, 2 turning away
unsigned long avoidUntil = 0;

CarLine btLine;

// ---- Setup / loop ----

void setup()
{
  Serial.begin(9600);                        // the Bluetooth module

  // The IR receiver's timer must be set up after the Arduino core has set up Timer 2 (the core
  // does it after global constructors run), otherwise the remote never decodes anything.
  ir.begin();

  clawServo.attach(PIN_CLAW);
  armServo.attach(PIN_ARM);
  baseServo.attach(PIN_BASE);
  setServo(PART_CLAW, CLAW_START);
  setServo(PART_ARM, ARM_START);
  setServo(PART_BASE, BASE_START);

  pinMode(PIN_LEFT_DIR, OUTPUT);
  pinMode(PIN_LEFT_PWM, OUTPUT);
  pinMode(PIN_RIGHT_DIR, OUTPUT);
  pinMode(PIN_RIGHT_PWM, OUTPUT);
  Stop();

  pinMode(PIN_LINE_LEFT, INPUT);
  pinMode(PIN_LINE_CENTER, INPUT);
  pinMode(PIN_LINE_RIGHT, INPUT);

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  carLineInit(&btLine);
}

void loop()
{
  readBluetooth();
  readRemote();

  unsigned long now = millis();
  if (modeNeedsLink && (long)(now - linkSeen) > (long)LINK_TIMEOUT_MS) setMode(MODE_MANUAL);
  if (mode == MODE_MANUAL) {
    if (driving && (long)(now - driveUntil) >= 0) Stop();      // the timed move, or the dead-man stop
  } else if ((long)(now - nextStep) >= 0) {
    nextStep = now + STEP_MS;
    switch (mode) {
      case MODE_AVOID:  ObstacleMovement(); break;
      case MODE_FOLLOW: Follow(); break;
      case MODE_LINE:   LineMove(); break;
    }
  }
}

// ---- Commands (Bluetooth) ----

void readBluetooth()
{
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (carLineFeed(&btLine, c)) {
      CarCmd cmd;
      if (carParse(btLine.buf, &cmd)) runCommand(cmd);
      carLineInit(&btLine);
    }
  }
}

void runCommand(const CarCmd &cmd)
{
  linkSeen = millis();                       // any command, "?" included, shows Mayday is still there
  switch (cmd.type) {
    case CMD_DRIVE:
      takeOver();
      drive(cmd.arg, cmd.value, cmd.ms);
      break;
    case CMD_STOP:
      setMode(MODE_MANUAL);
      break;
    case CMD_SERVO:
      setServo(cmd.arg, cmd.value);
      break;
    case CMD_MODE:
      setMode(cmd.arg);
      modeNeedsLink = cmd.arg != MODE_MANUAL;
      break;
    case CMD_STATUS:
      sendStatus();
      break;
  }
}

// A drive command puts a person back in charge. Only a real change of driver stops the wheels,
// or a held button would stutter the motors on every repeat.
void takeOver()
{
  if (mode != MODE_MANUAL) setMode(MODE_MANUAL);
}

void setMode(byte next)
{
  Stop();                                    // every change of driver starts from standing still
  mode = next;
  modeNeedsLink = false;                     // only a Bluetooth command sets it (see runCommand)
  avoidPhase = 0;
  nextStep = millis();
}

void sendStatus()
{
  Serial.print(F("MAYDAY-CAR 1 mode="));
  switch (mode) {
    case MODE_AVOID:  Serial.print(F("avoid")); break;
    case MODE_FOLLOW: Serial.print(F("follow")); break;
    case MODE_LINE:   Serial.print(F("line")); break;
    default:          Serial.print(F("manual")); break;
  }
  Serial.print(F(" claw="));
  Serial.print(clawAngle);
  Serial.print(F(" arm="));
  Serial.print(armAngle);
  Serial.print(F(" base="));
  Serial.println(baseAngle);
}

// ---- Commands (IR remote) ----

void readRemote()
{
  // One call per pass (each consumes a key). A key stays reported for about a millisecond, which is
  // many passes, so act only when it first appears: a held key comes as a new frame every ~110 ms.
  static byte previous = 0;
  byte code = ir.getCode();
  bool fresh = code != 0 && code != previous;
  previous = code;
  if (!fresh) return;
  byte key = ir.getIrKey(code, IR_TYPE_NORMAL);

  switch (key) {
    case IR_KEYCODE_UP:    takeOver(); drive(DIR_F, 100, 300); break;
    case IR_KEYCODE_DOWN:  takeOver(); drive(DIR_B, 100, 300); break;
    case IR_KEYCODE_LEFT:  takeOver(); drive(DIR_L, 70, 300); break;
    case IR_KEYCODE_RIGHT: takeOver(); drive(DIR_R, 70, 300); break;
    case IR_KEYCODE_OK:    setMode(MODE_MANUAL); break;
    case IR_KEYCODE_9:     setServo(PART_CLAW, clawAngle - SERVO_STEP); break;
    case IR_KEYCODE_7:     setServo(PART_CLAW, clawAngle + SERVO_STEP); break;
    case IR_KEYCODE_2:     setServo(PART_ARM, armAngle + SERVO_STEP); break;
    case IR_KEYCODE_8:     setServo(PART_ARM, armAngle - SERVO_STEP); break;
    case IR_KEYCODE_4:     setServo(PART_BASE, baseAngle + SERVO_STEP); break;
    case IR_KEYCODE_6:     setServo(PART_BASE, baseAngle - SERVO_STEP); break;
    case IR_KEYCODE_STAR:  setMode(MODE_AVOID); break;
    case IR_KEYCODE_POUND: setMode(MODE_FOLLOW); break;
    case IR_KEYCODE_0:     setMode(MODE_LINE); break;
    case IR_KEYCODE_5:     setMode(MODE_MANUAL); break;
  }
}

// ---- Claw ----

void setServo(byte part, int degrees)
{
  switch (part) {
    case PART_CLAW:
      clawAngle = constrain(degrees, CLAW_MIN, CLAW_MAX);
      clawServo.write(clawAngle);
      break;
    case PART_ARM:
      armAngle = constrain(degrees, ARM_MIN, ARM_MAX);
      armServo.write(armAngle);
      break;
    case PART_BASE:
      baseAngle = constrain(degrees, BASE_MIN, BASE_MAX);
      baseServo.write(baseAngle);
      break;
  }
}

// ---- Wheels ----

// Start a move. `ms` of 0 means "until the dead-man stop", so the sender must keep repeating it.
void drive(byte dir, int speed, unsigned int ms)
{
  if (speed <= 0) { Stop(); return; }
  switch (dir) {
    case DIR_F: Move_F(speed); break;
    case DIR_B: Move_B(speed); break;
    case DIR_L: RotateLeft(speed); break;
    case DIR_R: RotateRight(speed); break;
  }
  driving = true;
  driveUntil = millis() + (ms ? ms : DEADMAN_MS);
}

void Move_F(int speed)
{
  digitalWrite(PIN_LEFT_DIR, HIGH);
  analogWrite(PIN_LEFT_PWM, speed);
  digitalWrite(PIN_RIGHT_DIR, LOW);
  analogWrite(PIN_RIGHT_PWM, speed);
}

void Move_B(int speed)
{
  digitalWrite(PIN_LEFT_DIR, LOW);
  analogWrite(PIN_LEFT_PWM, speed);
  digitalWrite(PIN_RIGHT_DIR, HIGH);
  analogWrite(PIN_RIGHT_PWM, speed);
}

void RotateLeft(int speed)
{
  digitalWrite(PIN_LEFT_DIR, LOW);
  analogWrite(PIN_LEFT_PWM, speed);
  digitalWrite(PIN_RIGHT_DIR, LOW);
  analogWrite(PIN_RIGHT_PWM, speed);
}

void RotateRight(int speed)
{
  digitalWrite(PIN_LEFT_DIR, HIGH);
  analogWrite(PIN_LEFT_PWM, speed);
  digitalWrite(PIN_RIGHT_DIR, HIGH);
  analogWrite(PIN_RIGHT_PWM, speed);
}

void Stop()
{
  digitalWrite(PIN_LEFT_DIR, LOW);
  analogWrite(PIN_LEFT_PWM, 0);
  digitalWrite(PIN_RIGHT_DIR, LOW);
  analogWrite(PIN_RIGHT_PWM, 0);
  driving = false;
}

// ---- Self-driving modes ----

// Distance ahead in cm, or -1 when no echo came back within 30 ms (nothing in range).
int checkdistance()
{
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  unsigned long us = pulseIn(PIN_ECHO, HIGH, 30000UL);
  if (us == 0) return -1;
  return (int)(us / 58);
}

// Line tracking: three sensors under the front.
void LineMove()
{
  int left = digitalRead(PIN_LINE_LEFT);
  int center = digitalRead(PIN_LINE_CENTER);
  int right = digitalRead(PIN_LINE_RIGHT);

  if (left != BLACK_LINE && center == BLACK_LINE && right != BLACK_LINE) {
    Move_F(120);
  } else if (left == BLACK_LINE && center == BLACK_LINE && right != BLACK_LINE) {
    RotateLeft(80);
  } else if (left == BLACK_LINE && center != BLACK_LINE && right != BLACK_LINE) {
    RotateLeft(120);
  } else if (left != BLACK_LINE && center != BLACK_LINE && right == BLACK_LINE) {
    RotateRight(120);
  } else if (left != BLACK_LINE && center == BLACK_LINE && right == BLACK_LINE) {
    RotateRight(80);
  } else if (left == BLACK_LINE && center == BLACK_LINE && right == BLACK_LINE) {
    Stop();
  }
}

// Edge check from the original sketch: back away when a sensor sees no ground. It is not
// attached to a mode (it blocks for 3 s); call it from loop() to try it.
void FallCheck()
{
  bool onGround = digitalRead(PIN_LINE_LEFT) != BLACK_LINE && digitalRead(PIN_LINE_CENTER) != BLACK_LINE
                  && digitalRead(PIN_LINE_RIGHT) != BLACK_LINE;
  if (onGround) {
    Move_F(60);
  } else {
    Move_B(60);
    delay(3000);
  }
}

// Obstacle avoidance: cruise, back up when something is close, turn away when it is nearer than 25 cm.
void ObstacleMovement()
{
  unsigned long now = millis();
  if (avoidPhase != 0) {                     // a back-up or a turn is under way
    if ((long)(now - avoidUntil) < 0) return;
    Stop();
    avoidPhase = 0;
    return;
  }

  int distance = checkdistance();
  if (distance < 0) {
    Move_F(70);                              // nothing in range: creep forward
  } else if (distance <= 15) {
    Move_B(100);
    avoidPhase = 1;
    avoidUntil = now + 600;
  } else if (distance <= 25) {
    RotateLeft(100);
    avoidPhase = 2;
    avoidUntil = now + 600;
  } else {
    int s = 70;
    if (distance > 70) s = 100;
    if (distance > 100) s = 180;
    Move_F(s);
  }
}

// Following: keep roughly 15-20 cm behind whatever is in front.
void Follow()
{
  int distance = checkdistance();
  if (distance < 0) { Stop(); return; }

  if (distance < 15) {
    Move_B(80);
  } else if (distance <= 20) {
    Stop();
  } else if (distance <= 25) {
    Move_F(80);
  } else if (distance <= 30) {
    Move_F(100);
  } else {
    Stop();
  }
}
