#include <math.h>
#include "MPU9250_custom.h"


void MPU9250_custom::start() {
    //flush
    while(!Serial); 
    Serial.println("started");
    prevMicros = micros();
    currentMicros = micros();

    //detecting
    /*
    while (this->readId(&sensorId) != 0) {
        Serial.println("Cannot find device to read sensorId, retry in 2 seconds");
        delay(2000);
    }
    */
}

void MPU9250_custom::calibrate(float accelCalibScale, int count, int interval) {
    
    Serial.println("Accel and Gyro calibration starting in 2 seconds");
    delay(2000);
    Serial.println("started");
    Axis accelRaw, gyroRaw, magRaw;
    for(int i = 0; i < count; i++) {
        updateRaws(accelRaw, gyroRaw, magRaw);

        accelZero.integrateAll(accelRaw, accelCalibScale);
        gyroDrift.addAll(gyroRaw);
        delayMicroseconds(interval);
    }

    accelZero.divideAll(count);
    gyroDrift.divideAll(count);

    Serial.println(accelZero.x);
    Serial.println(gyroDrift.x);
}

void MPU9250_custom::setMagMinMaxAndSetOffset(int seconds) {
    Serial.println("Start scanning values of magnetometer to get offset values.");
    Serial.println("Rotate your device for " + String(seconds) + " seconds.");
    unsigned long calibStartAt = millis();
    float magX, magXMin, magXMax, magY, magYMin, magYMax, magZ, magZMin, magZMax;

    this->magUpdate();
    magXMin = magXMax = this->magX();
    magYMin = magYMax = this->magY();
    magZMin = magZMax = this->magZ();

    while(millis() - calibStartAt < (unsigned long) seconds * 1000) {
        delay(100);
        this->magUpdate();
        magX = this->magX();
        magY = this->magY();
        magZ = this->magZ();
        if (magX > magXMax) magXMax = magX;
        if (magY > magYMax) magYMax = magY;
        if (magZ > magZMax) magZMax = magZ;
        if (magX < magXMin) magXMin = magX;
        if (magY < magYMin) magYMin = magY;
        if (magZ < magZMin) magZMin = magZ;
    }

    this->magXOffset = - (magXMax + magXMin) / 2;
    this->magYOffset = - (magYMax + magYMin) / 2;
    this->magZOffset = - (magZMax + magZMin) / 2;


    Serial.println("Magnetic offset calibration complete.");
    delay(5000);
}

void MPU9250_custom::updateRaws(Axis& accelRaw, Axis& gyroRaw, Axis& magRaw) {
    accelUpdate();
    gyroUpdate();
    magUpdate();
    accelRaw.setAll(accelX(), accelY(), accelZ());
    gyroRaw.setAll(gyroX(), gyroY(), gyroZ());
    magRaw.setAll(magX(), magY(), magZ());
}

void MPU9250_custom::updateRawsCalibrated(Axis& accelRaw, Axis& gyroRaw, Axis& magRaw) {
    Axis accelAngle;
    updateRaws(accelRaw, gyroRaw, magRaw);

    //Adjust Accel raw.
    accelAngle.x = 
        (atan(-accelRaw.y/sqrt(pow(accelRaw.x,2)+pow(accelRaw.z,2))) - 
        atan(-accelZero.y/sqrt(pow(accelZero.x,2)+pow(accelZero.z,2)))) * 
        180/PI;
    accelAngle.y =
        -((atan(-accelRaw.x/sqrt(pow(accelRaw.y,2)+pow(accelRaw.z,2))) - 
        atan(-accelZero.x/sqrt(pow(accelZero.y,2)+pow(accelZero.z,2)))) * 
        180/PI);

    accelRaw.setAll(accelAngle.x, accelAngle.y, accelAngle.z);
    
    //Adjust Gyro Raw.
    gyroRaw.subtractAll(gyroDrift);
}

bool MPU9250_custom::isUpdate(int interval) {
    currentMicros = micros();
    if(currentMicros-prevMicros > interval) {
        prevMicros = micros();
        return true;
    } else {
        return false;
    }
    
}

void Axis::setAll(float x, float y, float z) {
    this->x = x;
    this->y = y;
    this->z = z;
}

void Axis::addAll(const Axis& d) {
    this->x += d.x;
    this->y += d.y;
    this->z += d.z;
}

void Axis::subtractAll(const Axis& d) {
    this->x -= d.x;
    this->y -= d.y;
    this->z -= d.z;
}

void Axis::multiplyAll(const Axis& d) {
    this->x *= d.x;
    this->y *= d.y;
    this->z *= d.z;
}

void Axis::divideAll(float num) {
    this->x /= num;
    this->y /= num;
    this->z /= num;
}

void Axis::integrateAll(const Axis& df, float dt) {
    this->x += dt*df.x;
    this->y += dt*df.y;
    this->z += dt*df.z;
}
