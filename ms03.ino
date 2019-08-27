#include <Servo.h>

Servo servo1;
Servo servo2;

void setup() {
  servo1.attach(9);
  servo2.attach(10);
  Serial.begin(9600);
    servo1.write(90);
  servo2.write(90);

}

void loop() {
  servo1.write(75);
  servo2.write(75);
  Serial.println("angle=75");
  delay(500);
  servo1.write(105);
  servo2.write(105);
  Serial.println("angle=105");
  delay(500);

}
