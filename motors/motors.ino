#include <Wire.h>

// Connect to the two encoder outputs!
#define ENCODER_A   12
#define ENCODER_B   11
#define MOTOR_1     9
#define MOTOR_IN1   10

// These let us convert ticks-to-RPM
#define GEARING     20
#define ENCODERMULT 12


volatile float RPM = 0;
volatile uint32_t lastA = 0;
volatile bool motordir = false;

void interruptA() {
  motordir = digitalRead(ENCODER_B);
  

  digitalWrite(LED_BUILTIN, HIGH);
  uint32_t currA = micros();
  if (lastA < currA) {
    // did not wrap around
    float rev = currA - lastA;  // us
    rev = 1.0 / rev;            // rev per us
    rev *= 1000000;             // rev per sec
    rev *= 60;                  // rev per min
    rev /= GEARING;             // account for gear ratio
    rev /= ENCODERMULT;         // account for multiple ticks per rotation
    RPM = rev;
  }
  lastA = currA;
  digitalWrite(LED_BUILTIN, LOW);
}

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(ENCODER_B, INPUT_PULLUP);
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(MOTOR_1, OUTPUT);
  pinMode(MOTOR_IN1, OUTPUT);

  attachInterrupt(ENCODER_A, interruptA, RISING);

  delay(100);

  // turn on motor M1
  digitalWrite(MOTOR_IN1, HIGH);
}

void printRPM() {

    Serial.print("Direction: ");
    if (motordir) {
      Serial.println("forward @ ");
    } else {
      Serial.println("backward @ ");
    }
    Serial.print(RPM); Serial.println(" RPM");
}

int i;
void loop() {
  delay(50);
  for (i=0; i<255; i++) {
    analogWrite(MOTOR_1, i);
    delay(20);
    printRPM();
  }

  for (i=255; i!=0; i--) {
    
    analogWrite(MOTOR_1, i);
    delay(20);
    printRPM();
  }

 
  for (i=0; i<255; i++) {
    
    analogWrite(MOTOR_1, i);
    delay(20);
    printRPM();
  }

  for (i=255; i!=0; i--) {
    
    analogWrite(MOTOR_1, i);
    delay(20);
    printRPM();
  }
}