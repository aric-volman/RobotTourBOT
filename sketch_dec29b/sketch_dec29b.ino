#define MOTOR1_ENB 9
#define MOTOR1_IN4 8
#define MOTOR1_IN3 10
#define MOTOR2_IN2 11
#define MOTOR2_IN1 12
#define MOTOR2_ENA 13
#define MOTOR3_IN2 14
#define MOTOR3_IN1 15
#define MOTOR3_ENA 22

#define TOGGLE(x) digitalWrite(x, digitalRead(x) ? LOW : HIGH)
float DUTY_CYCLE = 0.5; // 0.0 - 1.0
float FREQUENCY = 1000; // unit: hz
byte PIN = MOTOR3_ENA;

#include "RP2040_PWM.h"
RP2040_PWM* PWM_Instance;

void setup() {
PWM_Instance = new RP2040_PWM(PIN, 20000, 0);

  pinMode(PIN, OUTPUT);
  pinMode(MOTOR3_IN2, OUTPUT);
  pinMode(MOTOR3_IN1, OUTPUT);

pinMode(LED_BUILTIN, OUTPUT);

  digitalWrite(MOTOR3_IN2, HIGH);

  digitalWrite(MOTOR3_IN1, LOW);
  digitalWrite(LED_BUILTIN, HIGH);

 // TOGGLE(PIN);

}
void loop() {/*
  TOGGLE(PIN);
  //delayMicroseconds(1.0/FREQUENCY * 1000000 * DUTY_CYCLE);
  delayMicroseconds(2);
  TOGGLE(PIN);
  //delayMicroseconds(1.0/FREQUENCY * 1000000 * (1-DUTY_CYCLE));
  delayMicroseconds(2);
  */

  //analogWrite(PIN, 100);
}