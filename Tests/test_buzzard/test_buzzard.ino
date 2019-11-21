#include <PWMServo.h>
PWMServo myservoF;  // create servo object to control a servo
PWMServo myservoB;  // create servo object to control a servo
void setup() {
    myservoF.attach(2);         // attaches the servo on pin 9 to the servo
 myservoB.attach(3);
 myservoB.write(10);
 myservoF.write(170);
  // put your setup code here, to run once:
  pinMode(14, OUTPUT);
    digitalWrite(14, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
//  digitalWrite(14, HIGH);
  Serial.println("Hello");
}
