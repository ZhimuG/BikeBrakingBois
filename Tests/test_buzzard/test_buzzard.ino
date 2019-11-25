#include <PWMServo.h>
PWMServo myservoF;  // create servo object to control a servo
PWMServo myservoB;  // create servo object to control a servo
void setup() {
    myservoF.attach(2);         // attaches the servo on pin 9 to the servo
 myservoB.attach(3);
 myservoB.write(10);
 myservoF.write(170);
  // put your setup code here, to run once:
  buzz_setup(14);
}

void buzz_setup(int pwmpin){
  pinMode(pwmpin, OUTPUT);  
  digitalWrite(pwmpin, LOW);
}

void make_buzz(int pwmpin, int Frequency, int elapsedTime){
  double dTime = 1000 / (Frequency*2);
  int count = elapsedTime/dTime;
  for(int i=0; i<count; i++){
    digitalWrite(pwmpin, HIGH);
    delay(dTime);
    digitalWrite(pwmpin, LOW);
    delay(dTime);
  }
}

void buzz_stop(int pwmpin){
  digitalWrite(pwmpin, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
//  make_buzz(14, 500, 1000);
  buzz_stop(14);
}
