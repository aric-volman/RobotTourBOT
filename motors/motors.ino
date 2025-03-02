#include "RP2040_PWM.h"
#include "Wire.h"
#include <MPU6050_light.h>
#include "pio_encoder.h"

#define MOTOR1_ENB 0
#define MOTOR1_IN4 1
#define MOTOR1_IN3 8
#define MOTOR2_IN2 9
#define MOTOR2_IN1 10
#define MOTOR2_ENA 11

#define ENCODER_PIN_1 12
#define ENCODER_PIN_2 14

// Minimum freq recommended: 300k HZ (higher frequencies allows the motor to go at lower speeds)
#define FREQUENCY 300000
#define ANALOGBUTTON 26
#define LOOPTIME_DT 10 // 10 milliseconds looptime
#define MAX_ANGLE_DEV 0.1

#define SDA 6
#define SCL 7

#define BUTTON_DEBOUNCE_TIME 250

#define TURNKP 0.005

MPU6050 mpu(Wire1);

//PioEncoder encoder1(ENCODER_PIN_1);
//PioEncoder encoder2(ENCODER_PIN_2);

float yawOffset = 0;
float finalYaw = 0;

int buttonState = 0;
bool toggleButton = false;

unsigned long buttonTimer = 0;
unsigned long loopTimer = 0;
unsigned long commandTimer = 0;
unsigned long velocityTimer1 = 0;
unsigned long velocityTimer2 = 0;

double currentVelocity1 = 0.0;
double currentVelocity2 = 0.0;
double prevVelocity1 = 0.0;
double prevVelocity2 = 0.0;

int encoderCount1 = 0;
int encoderCount2 = 0;
int prevEncoderCount1 = 0;
int prevEncoderCount2 = 0;

bool prevChanA1 = false;
bool prevChanB1 = false;
bool prevChanA2 = false;
bool prevChanB2 = false;
bool chanA1 = false;
bool chanB1 = false;
bool chanA2 = false;
bool chanB2 = false;

enum Commands {
  FORWARD,
  BACKWARD,
  TURN_LEFT,
  TURN_RIGHT
};

const int numberOfCommands = 1;

int commands[numberOfCommands][2] = {
  {TURN_LEFT, 90}
};

enum ProgramState {
  RUNNING_PROGRAM,
  STOP_PROGRAM
};

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

enum EncoderFlip {
  NOT_FLIPPED,
  FLIPPED
};

Invert inverts[2] = {
  NOT_INVERTED, NOT_INVERTED
};

EncoderFlip encoderFlips[2] = {
  NOT_FLIPPED, FLIPPED
};

RP2040_PWM* motors[2] {
  new RP2040_PWM(MOTOR1_ENB, FREQUENCY, 0), 
  new RP2040_PWM(MOTOR2_ENA, FREQUENCY, 0)};

int programState = RUNNING_PROGRAM;

void setup() {

  Wire1.setSDA(SDA);
  Wire1.setSCL(SCL);
  Wire1.begin();
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");

  setupMotors();

  // Write signal LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  pinMode(ANALOGBUTTON, INPUT);

  pinMode(ENCODER_PIN_1, INPUT);
  pinMode(ENCODER_PIN_2, INPUT);
  pinMode((ENCODER_PIN_1)+1, INPUT);
  pinMode((ENCODER_PIN_2)+1, INPUT);
  

  Serial.begin(9600);
  Serial.println("Finished with motor setup!");

}

void loop() {

  updateEncoderCounts();

  Serial.print(currentVelocity1);
  Serial.print(",");
  Serial.println(currentVelocity2);

  mpu.update();

  if (analogRead(ANALOGBUTTON) == 1023 && (millis()-buttonTimer)>BUTTON_DEBOUNCE_TIME) {
    toggleButton = true;
    buttonTimer = millis();
  }

  if (toggleButton && (millis()-loopTimer)>LOOPTIME_DT) { // If the button was pressed
    Serial.println("Executing program from button!");

    //setMotorSignedPWM(ONE, 1.0f);

    if (programState == STOP_PROGRAM) {
      toggleButton = false;
    }
    loopTimer = millis();
  }

  prevEncoderCount1 = encoderCount1;
  prevEncoderCount2 = encoderCount2;
  prevVelocity1 = currentVelocity1;
  prevVelocity2 = currentVelocity2;

}

void updateEncoderCounts(){
  chanA1 = digitalRead(ENCODER_PIN_1);
  chanB1 = digitalRead((ENCODER_PIN_1)+1);
  chanA2 = digitalRead(ENCODER_PIN_2);
  chanB2 = digitalRead((ENCODER_PIN_2)+1);

  // Check for state changes and update counter based on direction
  if (prevChanA1 != chanA1) {
    if (chanA1 != chanB1) {
      int change = 1;
      if (encoderFlips[0] == FLIPPED) {
        change *= -1;
      }
      
      // A changed before B, increment counter
      encoderCount1 += change;
    } else {
      int change = 1;
      if (encoderFlips[0] == FLIPPED) {
        change *= -1;
      }

      // A changed after B, decrement counter
      encoderCount1 -= change;
    }
  }

    // Check for state changes and update counter based on direction
  if (prevChanA2 != chanA2) {
    if (chanA2 != chanB2) {
      int change = 1;
      if (encoderFlips[1] == FLIPPED) {
        change *= -1;
      }

      // A changed before B, increment counter
      encoderCount2 += change;
    } else {
      int change = 1;
      if (encoderFlips[1] == FLIPPED) {
        change *= -1;
      }

      // A changed after B, decrement counter
      encoderCount2 -= change;
    }
  }

  prevChanA1 = chanA1;
  prevChanB1 = chanB1;
  prevChanA2 = chanA2;
  prevChanB2 = chanB2;

  if (encoderCount1 != prevEncoderCount1) {
    currentVelocity1 = ((encoderCount1 - prevEncoderCount1)/((micros()-velocityTimer1)/1000000.0));
    velocityTimer1 = micros();
  }

  if (encoderCount2 != prevEncoderCount2) {
    currentVelocity2 = ((encoderCount2 - prevEncoderCount2)/((micros()-velocityTimer2)/1000000.0));
    velocityTimer2 = micros();
  }
}

void calculateYawOffset() {
  finalYaw = mpu.getAngleZ()-yawOffset;
}

void turnRight() {
  setMotorSignedPWM(ONE, 0.0);
  setMotorSignedPWM(TWO, 0.0);

  commandTimer = millis();
  double angle = 0.0;

  yawOffset = mpu.getAngleZ();
  calculateYawOffset();

  while (finalYaw < (90.0 + MAX_ANGLE_DEV)) {

    if ((millis()-commandTimer)>2000) {
      commandTimer = millis();
      break;
    }

    if (finalYaw > 90.0 + MAX_ANGLE_DEV && finalYaw < 90.0 - MAX_ANGLE_DEV) {
      break;
    }

    mpu.update();
    calculateYawOffset();
	  Serial.println(finalYaw);
    setMotorSignedPWM(ONE, -TURNKP*(90.0-finalYaw));
    setMotorSignedPWM(TWO, TURNKP*(90.0-finalYaw));

  }

  setMotorSignedPWM(ONE, 0.0);
  setMotorSignedPWM(TWO, 0.0);

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
  }

  switchDirection(num, dir, inverts[num]);
  motors[num]->setPWM(enPin, FREQUENCY, fabs(dutyCyclePercent)*100.0f);

}
