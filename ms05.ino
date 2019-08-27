#include <Servo.h>

Servo servo1;
Servo servo2;

int offset1;
int offset2;

void setup() {
  servo1.attach(9);
  servo2.attach(10);
    offset1 = -5;
  offset2 = -5;
  servo1.write((90 + offset1));
  servo2.write((90 + offset2));

}

void loop() {
  servo1.write((80 + offset1));
  servo2.write((100 + offset2));
  delay(100);
  servo1.write((80 + offset1));
  servo2.write((80 + offset2));
  delay(200);
  servo1.write((100 + offset1));
  servo2.write((80 + offset2));
  delay(100);
  servo1.write((100 + offset1));
  servo2.write((100 + offset2));
  delay(200);

}
