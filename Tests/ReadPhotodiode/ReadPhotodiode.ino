#include <PWMServo.h>

PWMServo myservoF;  // create servo object to control a servo
PWMServo myservoB;  // create servo object to control a servo
void setup() {
  // put your setup code here, to run once:
    myservoB.attach(2);         // attaches the servo on pin 9 to the servo object
  myservoF.attach(3);         // attaches the servo on pin 9 to the servo object
  //  Serial.begin(9600);
  myservoB.write(10);
  myservoF.write(170);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(analogRead(31));
  delayMicroseconds(100);
}
