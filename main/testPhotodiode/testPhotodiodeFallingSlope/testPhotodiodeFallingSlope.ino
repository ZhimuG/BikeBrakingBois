//int analogPinPhoto = 1;
//elapsedMillis current_max_time;
//elapsedMillis previous_max_time = 0;
//double threshold_middle = 250.0;
//double threshold_upper = 270.0;
//double threshold_lower = 230.0;
//bool flag_lower = false;
//bool flag_upper = false;
//double angle_btw_max = 3.14159/3;
//double current_value = 0;
//double previous_value = 0;
//double current_rpm;
//elapsedMillis time_btw_max;

int analogPinPhoto = 31;
double threshold1 = 0.1;
//double threshold2 = 0.1;
double current_value = 0;
int numDecreasingPoints = 4;
int count = 0;
double previous_value = 1;
//elapsedMicros current_time;
//elapsedMicros previous_time;
//elapsedMicros elapsed_time;
elapsedMicros Time;
unsigned int previous_time;
unsigned int elapsed_time;
unsigned int current_time;


double current_rps = 0;
double angle_btw_holes = 3.14159/3.0;
int i = 0;
double threshold_fall = 250.0;


void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
  current_value = analogRead(analogPinPhoto);

//  Serial.println(current_value);
  if(previous_value > threshold_fall && current_value < threshold_fall){
//    current_time = micros();
    current_time = (unsigned int)Time;
    elapsed_time = current_time - previous_time;
    current_rps = (angle_btw_holes/elapsed_time)*pow(10,6);
//    Serial.println(previous_time);
    previous_time = current_time;
//    Serial.println(current_time);
    Serial.println(current_rps);
  }
  previous_value = current_value;
  delayMicroseconds(100);
  
//
////  if(abs((previous_value - current_value) / previous_value) < threshold1){
//////    Serial.println("hello");
////    count = 0;
////    for(i=0; i<numDecreasingPoints; i++){
////      delay(50);
////      previous_value = current_value;
////      current_value = analogRead(analogPinPhoto);
////      if(current_value < previous_value){
////        count++;
////      }
////    }
////    if(count/numDecreasingPoints > 0.6){
////        current_time = millis();
////        elapsed_time = current_time - previous_time;
////        previous_time = current_time;
////        current_rpm = angle_btw_max/elapsed_time;
////        Serial.println(current_rpm);
////    }
////  }
//
//    if(abs((previous_value - current_value) / previous_value) > threshold1){
//        current_time = micros();
//        elapsed_time = (current_time - previous_time);
//        current_rps = angle_btw_holes/elapsed_time;
//        previous_time = current_time;
////        Serial.println(current_time);
////        Serial.println(previous_time);
////        Serial.println(elapsed_time); 
////        Serial.println(current_rps);
//      while(current_value < previous_value){
//        previous_value = current_value;
//        current_value = analogRead(analogPinPhoto);
////        delay(50);
//    }
//    }
//
//  previous_value = current_value;
  

//  if(flag_upper == true && previous_value > threshold_lower && current_value < threshold_lower){
//    flag_upper = false;
//    flag_lower = true;
//  }
//  if(flag_lower == true && previous_value < threshold_upper && current_value > threshold_upper){
//    flag_lower = false;
//    flag_upper = true;
//    current_max_time = millis();
//    time_btw_max = current_max_time - previous_max_time;
//    previous_max_time = current_max_time;
//    current_rpm = angle_btw_max/time_btw_max;
////    Serial.println(current_rpm);
//  }
//
//  previous_value = current_value;
//  
//  //if(current_value > previous_max){
//  //  current_max = current_value;
//  //}
  
}
