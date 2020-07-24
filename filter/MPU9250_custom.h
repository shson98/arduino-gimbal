#ifndef MPU9250_CUSTOM_H
#define MPU9250_CUSTOM_H
#include <MPU9250_asukiaaa.h>

class Axis {
    public:
    float x, y, z;

    Axis(float x = 0, float y = 0, float z = 0) { setAll(x, y, z); }
    void setAll(float, float, float);
    void addAll(Axis);
    void subtractAll(Axis);
    void multiplyAll(Axis);
    void divideAll(float);
    void integrateAll(Axis, float);
};

class MPU9250_custom : public MPU9250_asukiaaa {
    public:
    MPU9250_custom(uint8_t address = MPU9250_ADDRESS_AD0_LOW) : MPU9250_asukiaaa(address) {};
    void start();
    void calibrate(float, int, int);
    void setMagMinMaxAndSetOffset(int);
    void updateRaws(Axis&, Axis&, Axis&);
    void updateRawsCalibrated(Axis&, Axis&, Axis&);
    bool isUpdate(int);

    private: 
    uint8_t sensorId;
    Axis accelZero, gyroDrift;
    unsigned long int prevMicros;
    unsigned long int currentMicros;
};

#endif
