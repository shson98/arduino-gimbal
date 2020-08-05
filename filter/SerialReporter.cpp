#include "SerialReporter.h"

void SerialReporter::reportAccelGyroFilteredXYZ(const Axis& accelAngle, const Axis& gyroAngle, const Axis& angle) {
    dtostrf(accelAngle.x, 4, 2, textBuffer1);
    dtostrf(accelAngle.y, 4, 2, textBuffer2);
    dtostrf(accelAngle.z, 4, 2, textBuffer3);
    sprintf(textBuffer, "Accel angle: (%7s, %7s, %7s) ", textBuffer1, textBuffer2, textBuffer3);
    Serial.print(textBuffer);

    dtostrf(gyroAngle.x, 4, 2, textBuffer1);
    dtostrf(gyroAngle.y, 4, 2, textBuffer2);
    dtostrf(gyroAngle.z, 4, 2, textBuffer3);
    sprintf(textBuffer, "Gyro angle: (%7s, %7s, %7s) ", textBuffer1, textBuffer2, textBuffer3);
    Serial.print(textBuffer);

    dtostrf(angle.x, 4, 2, textBuffer1);
    dtostrf(angle.y, 4, 2, textBuffer2);
    dtostrf(angle.z, 4, 2, textBuffer3);
    sprintf(textBuffer, "Filtered angle: (%7s, %7s, %7s)", textBuffer1, textBuffer2, textBuffer3);
    Serial.println(textBuffer);
}

void SerialReporter::reportToVisualizer(const Axis& angle) {
    Serial.print(angle.x);
    Serial.print("/");
    Serial.print(angle.y);
    Serial.print("/");
    Serial.println(angle.z);    
}