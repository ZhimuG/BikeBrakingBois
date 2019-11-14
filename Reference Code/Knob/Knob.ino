// Controlling a servo position using a potentiometer (variable resistor)
// by Michal Rinott <http://people.interaction-ivrea.it/m.rinott>

// http://arduiniana.org/libraries/pwmservo/

//   Board                     SERVO_PIN_A   SERVO_PIN_B   SERVO_PIN_C
//   -----                     -----------   -----------   -----------
//   Arduino Uno, Duemilanove       9            10          (none)
//   Arduino Mega                  11            12            13
//   Sanguino                      13            12          (none)
//   Teensy 1.0                    17            18            15
//   Teensy 2.0                    14            15             4
//   Teensy++ 1.0 or 2.0           25            26            27
//   Teensy LC & 3.x                 (all PWM pins are usable)

#include <PWMServo.h>

PWMServo myservoF;  // create servo object to control a servo
PWMServo myservoB;

const int potpin = A3;  // analog pin used to connect the potentiometer
int val;    // variable to read the value from the analog pin

void setup() {
  myservoF.attach(2);         // attaches the servo on pin 9 to the servo object
  myservoB.attach(3);
  //myservo.attach(SERVO_PIN_A, 1000, 2000); // some motors need min/max setting
}

void loop() {
//  val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023)
//  Serial.println(val);
//  val = map(val, 0, 1023, 0, 179);     // scale it to use it with the servo (value between 0 and 180)
  int i = 0;
  while(1){
  if(i<170){
    i+=10;
    myservoF.write(i);                  // sets the servo position according to the scaled value
    myservoB.write(i);
  }else{
    i = 1;
    myservoF.write(i);                  // sets the servo position according to the scaled value
    myservoB.write(i);
  }
  delay(15);}                           // waits for the servo to get there
}

// Modular code starts here:
// Get potentiometer input
//int getPotentiometerInput(const int potpin){
//  int val = 0;
//  val = analogRead(potpin);
//  return val;
//}
//
//// Write PWM signal to servomotor
//void writeToMotor(int* forcePWM, int* servoPorts){
//  PWMServo myservoF, myservoB;
//  myservoF.attach(servoPorts[0]);
//  myservoB.attach(servoPorts[1]);
//  myservoF.write(forcePWM[0]);
//  myservoB.write(forcePWM[1]);
//}
