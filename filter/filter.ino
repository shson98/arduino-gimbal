#include <Servo.h>
#include <math.h>
#include "MPU9250_custom.h"
#include "SerialReporter.h"
#include <SoftwareSerial.h>
SoftwareSerial mySerial(2, 3);
#define CALIB_COUNT 1000
#define ACCEL_CALIB_SCALE 10.6998
#define MAG_CALIB_SEC 10

#define GYRO_INTEGRAL_SCALE -10

#define SENSOR_REFRESH_INTERVAL 1000
#define DT (float)SENSOR_REFRESH_INTERVAL/1000000

#define MOTOR_REFRESH_INTERVAL 1000
char n = '\n';
Servo myservo1;
Servo myservo2;
Servo myservo3;

SerialReporter reporter;

MPU9250_custom mySensor;
Axis accelRaw, gyroRaw, magRaw;
Axis accelAngle;
Axis gyroAngle;
Axis angle;
float mDirection;

// map function setting
float map(float val, float a, float b, float c, float d) {
  return (val-a)*(d-c)/(b-a)+c;
}

unsigned long motorPrevMicros;
unsigned long motorCurrentMicros;

void setup() {
  mySerial.begin(115200);
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

  myservo1.attach(9);
  myservo2.attach(10);
//  myservo3.attach(11);

  motorPrevMicros = micros();
  motorCurrentMicros = micros();
}

void loop() {
  //refresh sensor every designated interval.
  if(mySensor.isUpdate(SENSOR_REFRESH_INTERVAL)) {
    // Calibrated raw values update
    // accelRaw is relative from zero.
    // Drift is eliminated from gyroRaw.
    mySensor.updateRawsCalibrated(accelRaw, gyroRaw, magRaw);
    mySensor.magUpdate();
    mDirection=mySensor.magHorizDirection();

    //Angle defined by Acceleration
    if(accelRaw.x>0) {
      accelAngle.x=map(accelRaw.x,0,73,0,83);
    } else {
      accelAngle.x=map(accelRaw.x,0,96,0,84);
    }
    if(accelRaw.y>0) {
      accelAngle.y=map(accelRaw.y,0,96,0,84);
    } else {
      accelAngle.y=map(accelRaw.y,0,73,0,83);
    }

    //Gyro Value processing
    gyroAngle.integrateAll(gyroRaw, DT * GYRO_INTEGRAL_SCALE);

    //Filtering
    //new angle = 0.98*(previous angle + gyro delta) + 0.02 * accel angle.
    angle.x=0.98*(angle.x + (gyroRaw.x * DT * GYRO_INTEGRAL_SCALE))+0.02*accelAngle.x;
    angle.y=0.98*(angle.y + (gyroRaw.y * DT * GYRO_INTEGRAL_SCALE))+0.02*accelAngle.y;
    //angle.z=angle.z + (gyroRaw.z * DT * GYRO_INTEGRAL_SCALE);
    angle.z=mDirection;
    //Report
    reporter.reportAccelGyroFilteredXYZ(accelAngle, gyroAngle, angle);
  }
    mySerial.write(n);
    mySerial.write(90+angle.x);
    mySerial.write(90+angle.y);
//    mySerial.write("n");
//  motorCurrentMicros = micros();
//  if(motorCurrentMicros - motorPrevMicros > MOTOR_REFRESH_INTERVAL) {
//    motorPrevMicros = micros();
//    myservo1.write(((float)90)+angle.x);
//    myservo2.write(((float)90)-angle.y); //fliped
//    myservo3.write(((float)90)+angle.z); //fliped
//    
 //}

}
