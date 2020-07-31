#include "SerialReporter.h"

void SerialReporter::reportAccelGyroFilteredXY(const Axis& accelAngle, const Axis& gyroAngle, const Axis& angle) {
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

void SerialReporter::reportToVisualizer(const Axis& angle) {
    Serial.print(angle.x);
    Serial.print("/");
    Serial.print(angle.y);
    Serial.print("/");
    Serial.println(angle.z);    
}