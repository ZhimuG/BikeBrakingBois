int magic_thresh = 210;
int previous_value = 100000;
int analogPinPhoto = 31;
elapsedMicros Time;
unsigned int max_time = 1; //[s]
bool debug = true;
#include <PWMServo.h>
PWMServo myservoF;  // create servo object to control a servo
PWMServo myservoB;  // create servo object to control a servo

void setup() {
 // put your setup code here, to run once:
 myservoF.attach(2);         // attaches the servo on pin 9 to the servo object
 myservoB.attach(3);
 myservoB.write(10);
 myservoF.write(170);
}

int get_rps_flag(int previous_value, unsigned long previous_time){
  int current_value = analogRead(analogPinPhoto);
  delayMicroseconds(20);
  current_value += analogRead(analogPinPhoto);
  current_value /= 2.0;
  if(current_value > magic_thresh && previous_value < magic_thresh){
//    if(debug){
//      Serial.println("Threshold crossed at ");
//      Serial.println(current_value);
//    }
    return 1;
 }
 else if((micros() - previous_time)*pow(10,-6) < max_time){
//   if(debug){
//    Serial.println("Still looking... ");
//    Serial.print(current_value);
//   }
   return get_rps_flag(current_value, previous_time);
 }
 else{
//  if(debug){
//    Serial.println("Mission Failed");
//  }
  return 0;
 }
 // add window threshold
 // add delay?
}
double get_rps(int max_hole_count){
 int hole_count = 0;
 unsigned long previous_time = micros();
 while(hole_count < max_hole_count){
  int temp = get_rps_flag(analogRead(analogPinPhoto), (unsigned int)Time);
  if(!temp){
    return 0.0;
  }
  else{
    hole_count += temp;
  }
//  if(debug){
//    Serial.println(hole_count);
//  }
 }
 unsigned long current_time = micros();
 return (double)max_hole_count/((current_time-previous_time)*pow(10,-6)*16.0);
}
void loop() {
 // put your main code here, to run repeatedly:
 double RPS = get_rps(2);
 if(debug){
//  Serial.println("RPS: ");
  Serial.println(RPS);
 }
//  Serial.println(RPS);
//  Serial.println(analogRead(31));
//  Serial.println(analogRead(32));
//  delay(20);
//    digitalWrite(14,HIGH);
}
