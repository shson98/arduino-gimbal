#include <Servo.h>
#include <SoftwareSerial.h>

#define ANGLE_OFFSET (10)

char n = '\n';
Servo myservo1;
Servo myservo2;
SoftwareSerial mySerial(2, 3); // RX, TX

byte previousBuffX = 90;
byte previousBuffY = 90;

byte currentBuffX;
byte currentBuffY;


void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  Serial.println("Goodnight moon!");
  // set the data rate for the SoftwareSerial port
  mySerial.begin(115200);
  myservo1.attach(9);
  myservo2.attach(10);
}

void loop() { // run over and over
  if (mySerial.available()) {
    if (mySerial.read() == n) {
      while ((currentBuffX = mySerial.read()) == -1); //flush
      if (abs(currentBuffX - previousBuffX) <= ANGLE_OFFSET) {
        myservo1.write(currentBuffX);
        previousBuffX = currentBuffX;
      }

      while ((currentBuffY = mySerial.read()) == -1);
      if (abs(currentBuffY - previousBuffY) <= ANGLE_OFFSET) {
        myservo2.write(currentBuffY);
        previousBuffY = currentBuffY;
      }

      Serial.print(previousBuffX);
      Serial.println(previousBuffY);
      
    }
  }

}
