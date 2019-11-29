/* Sweep
 by BARRAGAN <http://barraganstudio.com> 
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://arduino.cc/en/Tutorial/Sweep
*/ 

#include <PWMServo.h>

PWMServo myservoF;  // create servo object to control a servo
PWMServo myservoB;  // create servo object to control a servo
 
int pos = 0;    // variable to store the servo position 
 
void setup() 
{ 
  myservoF.attach(2);  // attaches the servo on pin 9 to the servo object
  myservoB.attach(3);
  myservoB.write(10);
  myservoF.write(170);
} 
 
void loop() 
{ 
  for(pos = 10; pos <= 170; pos += 1) // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    myservoF.write(170-pos);              // tell servo to go to position in variable 'pos' 
    myservoB.write(pos);
    Serial.println(analogRead(31));
//  Serial.print(",");
//  Serial.println(analogRead(32));
    delayMicroseconds(100);                       // waits 15ms for the servo to reach the position 
  } 
  for(pos = 170; pos>=0; pos-=1)     // goes from 180 degrees to 0 degrees 
  {                                
    myservoF.write(170-pos);              // tell servo to go to position in variable 'pos' 
    myservoB.write(pos);
      Serial.println(analogRead(31));
//  Serial.print(",");
//  Serial.println(analogRead(32));
    delayMicroseconds(100);                       // waits 15ms for the servo to reach the position 
  } 
} 
