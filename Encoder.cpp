#include "Encoder.h"
#include "pio_encoder.h"

Encoder(int pinNum, int PPR) {
    previousCount = 0;
    currentCount = 0;
    RPM = 0.0;
    PPR = PPR;
    pinNum = pinNum;
    encoder = PioEncoder encoder(pinNum);
    deltaT = 1000;
    previousTime = micros();
}

void Encoder::initEncoder() {
  encoder.begin();
}

float Encoder::getRPM() {
    return RPM;
};

int Encoder::getCurrentCount() {
    return currentCount;
};

void Encoder::setPPR(int) {
    PPR = 12;
};

void Encoder::updateRPM() {
    unsigned long newDeltaT = micros() - previousTime;

    if (newDeltaT > deltaT) {
        //Serial.println("Hello");
        double deltaTMinutes = newDeltaT/60000000.0;

        RPM = (float)(((currentCount-previousCount)/(PPR*20))/deltaTMinutes);

        previousCount = currentCount;
        previousTime = micros();
    }
}
