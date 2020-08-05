#include <Servo.h>
#include <math.h>
#include "MPU9250_custom.h"
#include "SerialReporter.h"

#define CALIB_COUNT 1000
#define ACCEL_CALIB_SCALE 10.6998
#define MAG_CALIB_SEC 5

#define GYRO_INTEGRAL_SCALE -6.8

#define SENSOR_REFRESH_INTERVAL 1000
#define DT (float)SENSOR_REFRESH_INTERVAL/1000000

SerialReporter reporter;

MPU9250_custom mySensor;
Axis accelRaw, gyroRaw, magRaw;
Axis accelAngle;
Axis gyroAngle;
Axis angle;
float mDirection;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  Serial.println("test");
  mySensor.start();
  mySensor.setWire(&Wire);
  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();
  mySensor.calibrate(ACCEL_CALIB_SCALE, CALIB_COUNT, SENSOR_REFRESH_INTERVAL);
  mySensor.setMagMinMaxAndSetOffset(MAG_CALIB_SEC);
}

void loop() {
  //refresh sensor every designated interval.
  if(mySensor.isUpdate(SENSOR_REFRESH_INTERVAL)) {
    // Calibrated raw values update
    // accelRaw is relative from zero.
    // Drift is eliminated from gyroRaw.
    mySensor.updateRawsCalibrated(accelRaw, gyroRaw, magRaw);
    mDirection=mySensor.magHorizDirection();

    //Gyro Value processing
    gyroAngle.integrateAll(gyroRaw, DT * GYRO_INTEGRAL_SCALE);

    //Report
    //reporter.reportToVisualizer(accelAngle);
    Serial.print("Accel (");
    Serial.print(accelRaw.x);
    Serial.print(", ");
    Serial.print(accelRaw.y);
    Serial.print(", ");
    Serial.print(accelRaw.z);
    Serial.print(") Mag(");
    Serial.print(magRaw.x);
    Serial.print(", ");
    Serial.print(magRaw.y);
    Serial.print(", ");
    Serial.print(magRaw.z);
    Serial.println(")");
  }
}
