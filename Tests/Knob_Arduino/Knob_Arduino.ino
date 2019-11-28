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

#include <Servo.h>

Servo myservoF;  // create servo object to control a servo
Servo myservoB;

const int potpin = A0;  // changed from A3
int val;    // variable to read the value from the analog pin

int ReadPot(const int potpin){
  val=0;
  for(int i=0; i<5; i++){
    val += analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023)
    delay(5);
  }
  val = val/5;
  Serial.println(val);
  int rounder = 10;
   if(val>180){
    val=220;
  }
  val = val/rounder;
//  Serial.println(val);
//  delay(20);
  val = map(val, 0, 22, 10, 170);     // scale it to use it with the servo (value between 0 and 180)
  return val;
}

void MoveMotors(int PWM){
  //Serial.println("success");
  myservoF.write(170-PWM);
  myservoB.write(10+PWM);
}

void setup() {
  // put your setup code here, to run once:
  myservoB.attach(9);         // changed from 2
  myservoF.attach(10);         // changed from 3
  //  Serial.begin(9600);
  myservoB.write(10);
  myservoF.write(170);
}

void loop() {
  val = ReadPot(potpin);            // reads the value of the potentiometer (value between 0 and 1023)
//  Serial.println(val);
//  val = map(val, 0, 1023, 0, 179);     // scale it to use it with the servo (value between 0 and 180)
  MoveMotors(val);
  Serial.println(analogRead(31));
  delay(15);                           // waits for the servo to get there
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
