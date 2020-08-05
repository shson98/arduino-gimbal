#ifndef SERIAL_REPORTER_H
#define SERIAL_REPORTER_H
#include <Arduino.h>
#include "MPU9250_custom.h"

class SerialReporter {
    public:
    void reportAccelGyroFilteredXYZ(const Axis&, const Axis&, const Axis&);
    void reportToVisualizer(const Axis&);

    private:
    char textBuffer1[8];
    char textBuffer2[8];
    char textBuffer3[8];
    char textBuffer[38];
};

#endif