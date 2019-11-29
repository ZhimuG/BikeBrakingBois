#include <PWMServo.h>

PWMServo myservoF;  // create servo object to control a servo
PWMServo myservoB;  // create servo object to control a servo
void setup() {
  // put your setup code here, to run once:
  myservoB.attach(3);
  myservoF.attach(2);
  myservoB.write(10);
  myservoF.write(170);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print(analogRead(31));
  Serial.print(",");
  Serial.println(analogRead(32));
  delayMicroseconds(100);
}
