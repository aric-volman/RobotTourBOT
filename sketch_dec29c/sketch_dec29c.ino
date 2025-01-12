#include "RP2040_PWM.h"
#define MOTOR1_ENB 9
#define MOTOR1_IN4 8
#define MOTOR1_IN3 10
#define MOTOR2_IN2 11
#define MOTOR2_IN1 12
#define MOTOR2_ENA 13
#define MOTOR3_IN2 14
#define MOTOR3_IN1 15
#define MOTOR3_ENA 22
// Minimum freq recommended: 300k HZ (higher frequencies allows the motor to go at lower speeds)
#define FREQUENCY 300000

enum MotorDir {
  CW,
  CCW,
  STOP,
  BRAKE
};

enum MotorNum {
  ONE,
  TWO,
  THREE
};

enum Invert {
  INVERTED,
  NOT_INVERTED
};

/*
RP2040_PWM* motor1;
RP2040_PWM* motor2;
RP2040_PWM* motor3;
*/

Invert inverts[3] = {
  INVERTED, INVERTED, INVERTED
};

RP2040_PWM* motors[3] {
  new RP2040_PWM(MOTOR1_ENB, FREQUENCY, 0), 
  new RP2040_PWM(MOTOR2_ENA, FREQUENCY, 0), 
  new RP2040_PWM(MOTOR3_ENA, FREQUENCY, 0)};

void setup() {
  setupMotors();

  // Write signal LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.begin(9600);
  Serial.println("Finished with motor setup!");

}
void loop() {


setMotorSignedPWM(TWO, 0.0);
  setMotorSignedPWM(THREE, 0.0);
setMotorSignedPWM(ONE, 0.0);
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

void setupMotors() {

  pinMode(MOTOR1_ENB, OUTPUT);
  pinMode(MOTOR1_IN4, OUTPUT);
  pinMode(MOTOR1_IN3, OUTPUT);
  pinMode(MOTOR2_ENA, OUTPUT);
  pinMode(MOTOR2_IN2, OUTPUT);
  pinMode(MOTOR2_IN1, OUTPUT);
  pinMode(MOTOR3_ENA, OUTPUT);
  pinMode(MOTOR3_IN2, OUTPUT);
  pinMode(MOTOR3_IN1, OUTPUT);

}

void switchDirection(MotorNum num, MotorDir dir, Invert invert) {
  byte in1 = 0;
  byte in2 = 0;
  bool in1Logic = false;
  bool in2Logic = false;

  if (invert == INVERTED) {
    in1Logic = !in1Logic;
    in2Logic = !in2Logic;
  }

  if (num == ONE) {
    in1 = MOTOR1_IN3;
    in2 = MOTOR1_IN4;
  } else if (num == TWO) {
    in1 = MOTOR2_IN1;
    in2 = MOTOR2_IN2;
  } else if (num == THREE) {
    in1 = MOTOR3_IN1;
    in2 = MOTOR3_IN2;
  }

  if (dir == CCW) {
    in2Logic = !in2Logic;
    in1Logic = in1Logic;
  } else if (dir == CW) {
    in2Logic = in2Logic;
    in1Logic = !in1Logic;
  } else if (dir == STOP) {
    in2Logic = false;
    in1Logic = false;
  } else if (dir == BRAKE) {
    in2Logic = true;
    in1Logic = true;
  }

  digitalWrite(in2, in2Logic);
  digitalWrite(in1, in1Logic);
}

void setMotorSignedPWM(MotorNum num, float dutyCyclePercent) {
  byte enPin = 0;
  MotorDir dir = STOP;

  if (dutyCyclePercent < -1.0f | dutyCyclePercent > 1.0f) {
    dutyCyclePercent = sgn(dutyCyclePercent)*1.0f;
  }

  if (dutyCyclePercent < 0) {
    dir = CW;
  } else if (dutyCyclePercent > 0) {
    dir = CCW;
  } else {
    dir = STOP;
  }

  if (num == ONE) {
    enPin = MOTOR1_ENB;
  } else if (num == TWO) {
    enPin = MOTOR2_ENA;
  } else if (num == THREE) {
    enPin = MOTOR3_ENA;
  }

  switchDirection(num, dir, inverts[num]);
  motors[num]->setPWM(enPin, FREQUENCY, fabs(dutyCyclePercent)*100.0f);

}
