#include <L298NX2.h>

#define MOTOR1_ENB 9
#define MOTOR1_IN4 8
#define MOTOR1_IN3 10
#define MOTOR2_IN2 11
#define MOTOR2_IN1 12
#define MOTOR2_ENA 13
#define MOTOR3_IN2 14
#define MOTOR3_IN1 15
#define MOTOR3_ENA 22

#define _PWM_LOGLEVEL_        3
#include "RP2040_PWM.h"

//RP2040_PWM* PWM_Instance;

//float frequency;
//float dutyCycle;

/*L298N motor_1(MOTOR1_ENB, MOTOR1_IN3, MOTOR1_IN4);
L298N motor_2(MOTOR2_ENA, MOTOR2_IN1, MOTOR2_IN2); 
L298N motor_3(MOTOR3_ENA, MOTOR3_IN1, MOTOR3_IN2);
*/
void setup() {

  //assigns pin 25 (built in LED), with frequency of 20 KHz and a duty cycle of 0%
//PWM_Instance = new RP2040_PWM(MOTOR1_ENB, 20000, 0);

 // Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  /*pinMode(MOTOR1_ENB, OUTPUT);

  pinMode(MOTOR1_IN4, OUTPUT);
  pinMode(MOTOR1_IN3, OUTPUT);*/

}

void loop() {
  // put your main code here, to run repeatedly:

  //motor_1.setSpeed(255);
 // digitalWrite(MOTOR1_IN4, HIGH);

 // digitalWrite(MOTOR1_IN3, LOW);

  //digitalWrite(MOTOR1_ENB, HIGH);
 // analogWrite(MOTOR1_ENB, 100);

 // frequency = 20000;
 // dutyCycle = 90;

 // PWM_Instance->setPWM(MOTOR1_ENB, frequency, dutyCycle);

  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
 digitalWrite(LED_BUILTIN, LOW);
}
