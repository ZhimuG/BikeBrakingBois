int magic_thresh = 212;
int window_thresh = 218;
//int previous_value = 100000;
int analogPinPhoto = 31;
elapsedMicros Time;
unsigned int max_time = 1; //[s]

double max_value = 0;
double min_value = 100000;

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

bool get_rps_flag(int hole_count, int max_count, int previous_value, unsigned int previous_time){
  int current_value = analogRead(analogPinPhoto);
  delayMicroseconds(100);
//  Serial.println(((unsigned int)Time - previous_time)*pow(10,-6));
//Serial.println((unsigned int)Time);
//Serial.println(previous_time);
//  Serial.println(hole_count);
//  Serial.println(current_value);
  if(hole_count >= max_count){
//    Serial.println("hello");
    return true;
  }
  else if(current_value > magic_thresh && previous_value <= magic_thresh){
//    Serial.println(current_value);
    int window_value = analogRead(analogPinPhoto);
    while(window_value < window_thresh){
      window_value = analogRead(analogPinPhoto);
    }
      
    hole_count++;
//    Serial.println(hole_count);
    return get_rps_flag(hole_count, max_count, current_value, previous_time);
  }
  else if(((unsigned int)Time - previous_time)*pow(10,-6) > max_time){
//    Serial.println("yoyo");
    return false;
  } 
  else{
    return get_rps_flag(hole_count, max_count, current_value, previous_time);
  }
  // add window threshold
  // add delay?  
}

double get_rps(int num_holes){
  bool rps_flag = false;
  unsigned int previous_time = (unsigned int)Time;
  int hole_count = 0;
  rps_flag = get_rps_flag(hole_count, num_holes, analogRead(analogPinPhoto), previous_time);
//  Serial.println(rps_flag);
  if(rps_flag){
    unsigned int current_time = (unsigned int)Time;
    return (double)num_holes/((current_time-previous_time)*pow(10,-6)*16.0);
  }
  else{
//    Serial.println("hello???");
    return 0.0;
  }
}

void get_minmax(){
  max_value = 0;
  min_value = 100000;
  for(int i=0; i<3500; i++){
    double current_value = analogRead(analogPinPhoto);
    if(current_value > max_value){
      max_value = current_value;
      }
    if(current_value < min_value){
      min_value = current_value;
    }
    delayMicroseconds(100);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  int num_holes = 2;
  double RPS = get_rps(num_holes);
  Serial.println(RPS);

//  Serial.println("begin");
//  int num_holes = 1;
//  double avg = 0;
//  for(int i=0; i<5; i++){
//    double rps = get_rps(num_holes);
//    avg += rps;
//  }
//  avg /= 5;
////  double rps = get_rps(num_holes);
//  Serial.println(avg);
//  Serial.println("done");
//  delay(10000);

//unsigned int previous_time = (unsigned int)Time;
//Serial.println(previous_time);

//  Serial.println(RPS);
  
//  Serial.println(analogRead(31));
//  Serial.println(analogRead(32));
//  delay(20);
//    digitalWrite(14,HIGH);

//  get_minmax();
//  Serial.println(min_value);
//  Serial.println(max_value);

//  Serial.println(analogRead(analogPinPhoto));
//  delayMicroseconds(100);
}
