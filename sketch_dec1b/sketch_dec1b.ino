#include "pio_encoder.h"

// List of encoder pins that work
// 12
// 10
// 8
PioEncoder encoder(10);

void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  encoder.begin();
Serial.begin();
    pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
}

// the loop function runs over and over again forever
void loop() {
  delay(20);
  
  Serial.println(encoder.getCount());

}
