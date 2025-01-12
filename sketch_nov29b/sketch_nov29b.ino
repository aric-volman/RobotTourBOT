#include <Wire.h>

#define ENCODER_A   2
#define ENCODER_B   3
#define GEARING     20
#define ENCODERMULT 12

volatile signed long Count = 0;
volatile signed long CountA = 0;
volatile signed long CountB = 0;
volatile float RPM = 0;
volatile uint32_t lastA = 0;

void setup() {
  // put your setup code here, to run once:
    Serial.begin(9600); 

  pinMode(ENCODER_B, INPUT_PULLUP);
  pinMode(ENCODER_A, INPUT_PULLUP);

 attachInterrupt(digitalPinToInterrupt(ENCODER_A), interruptA, RISING);
}

void loop() {
  // put your main code here, to run repeatedly:
  readEncoder(ENCODER_A, ENCODER_B);
 //Serial.println(Count);

 //Serial.println(analogRead(ENCODER_B)/1023.0);
}

void interruptA() {

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
}

void readEncoder(int pinA, int pinB) {

  //Serial.println(readA);
  //Serial.println(readB);
  Serial.println(RPM);

}
