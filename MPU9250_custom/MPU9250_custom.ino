#include "MPU9250_custom.h"
#include <math.h>

#define CALIB_MAG_SEC 5
#define CALIB_COUNT 1000
#define ACCEL_CALIB_SCALE 10.6998
#define GYRO_INTEGRAL_SCALE -6.6
#define LOOP_MICRO_SEC 1000
#define DT (float)LOOP_MICRO_SEC/1000000

MPU9250_custom mySensor;

Axis accelRaw, gyroRaw, magRaw;
Axis accelZero;
Axis gyroDrift, gyroAngle;
Axis angle;
float mDirection;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mySensor.setWire(&Wire);
  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();
  mySensor.calibrate(accelZero, ACCEL_CALIB_SCALE, gyroDrift, CALIB_COUNT, LOOP_MICRO_SEC);
  mySensor.setMagMinMaxAndSetOffset(CALIB_MAG_SEC);
}

void loop() {
  mySensor.updateRaws(accelRaw, gyroRaw, magRaw);
  mDirection=mySensor.magHorizDirection();

  gyroRaw.subtractAll(gyroDrift);
  gyroAngle.integrateAll(gyroRaw, DT * GYRO_INTEGRAL_SCALE);

/*
  // Angle by Acceleration
  
  //Serial.print(String(aX*10.6998)+" ");
  //Serial.print(String(aY*10.6998)+" ");
  //Serial.print(String(aZ*10.6998)+" ");
  Serial.print(" x angle:"+ String(abs((atan(-aY/sqrt(pow(aX,2)+pow(aZ,2)))-atan(-Y0/sqrt(pow(X0,2)+pow(Z0,2))))*180/PI)));
  Serial.print(" y angle:"+ String(abs((atan(-aX/sqrt(pow(aY,2)+pow(aZ,2)))-atan(-X0/sqrt(pow(Y0,2)+pow(Z0,2))))*180/PI)));
  Serial.print(" || ");
*/
  // Angle by Gyro

//  Serial.print(" wx:"+String(gX-WX0));
//  Serial.print(" wy:"+String(gY-WY0));
//  Serial.print(" wz:"+String(gZ-WZ0));
  Serial.println(String(-gyroAngle.x)+"/"+gyroAngle.y+"/"+-gyroAngle.z);

  // Angle by Magnetic
//  Serial.print(" sensorId: " + String(sensorId));

//  Serial.println("mySensor.magXOffset = " + String(mySensor.magXOffset) + ";");
//  Serial.println("mySensor.maxYOffset = " + String(mySensor.magYOffset) + ";");
//  Serial.println("mySensor.magZOffset = " + String(mySensor.magZOffset) + ";");
//  Serial.print(" magX: " + String(mX));
//  Serial.print(" maxY: " + String(mY));
//  Serial.print(" magZ: " + String(mZ));
//  Serial.println(" horizontal direction: " + String(abs(mDirection)));

//  Serial.println("at " + String(millis()) + "ms");
//  Serial.println(""); // Add an empty line
  

  delayMicroseconds(LOOP_MICRO_SEC);
}
