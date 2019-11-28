/* Sweep
 by BARRAGAN <http://barraganstudio.com> 
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://arduino.cc/en/Tutorial/Sweep
*/ 

#include <Servo.h> 
 
Servo myservoF;  // create servo object to control a servo 
                // twelve servo objects can be created on most boards
Servo myservoB;
 
int pos = 0;    // variable to store the servo position 
 
void setup() 
{ 
  myservoF.attach(9);  // attaches the servo on pin 9 to the servo object
  myservoB.attach(10);
  myservoB.write(10);
  myservoF.write(170);
} 
 
void loop() 
{ 
  for(pos = 10; pos <= 170; pos += 1) // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    myservoF.write(170-pos);              // tell servo to go to position in variable 'pos' 
    myservoB.write(pos);
    delay(15);                       // waits 15ms for the servo to reach the position 
  } 
  for(pos = 180; pos>=0; pos-=1)     // goes from 180 degrees to 0 degrees 
  {                                
    myservoF.write(170-pos);              // tell servo to go to position in variable 'pos' 
    myservoB.write(pos);
    delay(15);                       // waits 15ms for the servo to reach the position 
  } 
} 
