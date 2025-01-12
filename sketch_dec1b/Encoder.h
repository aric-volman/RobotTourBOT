#include "pio_encoder.h"

class Encoder {
    private:
    int previousCount;
    int currentCount;
    int deltaT;
    unsigned long previousTime;

    float RPM;
    int PPR;
    int pinNum;
    PioEncoder encoder;

    public:
    float getRPM();
    int getCurrentCount();
    int getPreviousCount();
    void setPPR(int);
    void updateRPM();
    void initEncoder();

}
