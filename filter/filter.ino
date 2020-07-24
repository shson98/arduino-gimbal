#include <Servo.h>
#include <math.h>
#include "MPU9250_custom.h"

#define CALIB_MAG_SEC 5
#define CALIB_COUNT 1000

#define ACCEL_CALIB_SCALE 10.6998
#define GYRO_INTEGRAL_SCALE -6.6

#define LOOP_MICRO_SEC 1000
#define DT (float)LOOP_MICRO_SEC/1000000

Servo myservo1;
Servo myservo2;

MPU9250_custom mySensor;
Axis accelRaw, gyroRaw, magRaw;
Axis accelAngle;
Axis gyroAngle;
Axis angle;
float mDirection;

char textBuffer1[7];
char textBuffer2[7];
char textBuffer[30];

// map function setting
float map(float val, float a, float b, float c, float d){
  return (val-a)*(d-c)/(b-a)+c;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  mySensor.setWire(&Wire);
  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();
  mySensor.calibrate(ACCEL_CALIB_SCALE, CALIB_COUNT, LOOP_MICRO_SEC);
  mySensor.setMagMinMaxAndSetOffset(CALIB_MAG_SEC);

  myservo1.attach(9);
  myservo2.attach(10);
}

void loop() {
  //refresh sensor every 500 microseconds.
  if(mySensor.isUpdate(LOOP_MICRO_SEC)) {

    // Calibrated raw values update
    // accelRaw is relative from zero.
    // Drift is eliminated from gyroRaw.
    mySensor.updateRawsCalibrated(accelRaw, gyroRaw, magRaw);
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
    angle.x=0.98*(angle.x + (gyroRaw.x * DT * -GYRO_INTEGRAL_SCALE))+0.02*accelAngle.x;
    angle.y=0.98*(angle.y + (gyroRaw.y * DT * -GYRO_INTEGRAL_SCALE))+0.02*accelAngle.y;

    //Report
    dtostrf(accelAngle.x, 4, 2, textBuffer1);
    dtostrf(accelAngle.y, 4, 2, textBuffer2);
    sprintf(textBuffer, "Accel angle: (%7s, %7s) ", textBuffer1, textBuffer2);
    Serial.print(textBuffer);

    dtostrf(gyroAngle.x, 4, 2, textBuffer1);
    dtostrf(gyroAngle.y, 4, 2, textBuffer2);
    sprintf(textBuffer, "Gyro angle: (%7s, %7s) ", textBuffer1, textBuffer2);
    Serial.print(textBuffer);

    dtostrf(angle.x, 4, 2, textBuffer1);
    dtostrf(angle.y, 4, 2, textBuffer2);
    sprintf(textBuffer, "Filtered angle: (%7s, %7s)", textBuffer1, textBuffer2);
    Serial.println(textBuffer);
  }

  //Motor output
  /*
  myservo1.write(90-angle.x);
  myservo2.write(90+angle.x); //fliped
  */
}
