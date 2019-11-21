void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
  int analogPinPhoto1 = 31;
  int analogPinPhoto2 = 32;
  double current_value = 0;
  //int numDecreasingPoints = 4;
  //int count = 0;
  double previous_value = 1;
  elapsedMicros Time;
  unsigned int previous_time;
  unsigned int elapsed_time;
  unsigned int current_time;
  
  double current_rps = 0;
  double angle_btw_holes = 3.14159/8.0;
  int i = 0;
  double threshold_fall = 240.0;
  double threshold_lower = 220.0;
  int numHoles = 6;
  int noHoleCounter = 0;
  int thresh = 10000;

  while(i<numHoles){
    current_value = analogRead(analogPinPhoto1);
//    current_value = analogRead(analogPinPhoto2);
    Serial.println(current_value);
    if(previous_value > threshold_fall && current_value < threshold_fall){
      current_time = (unsigned int)Time;
      elapsed_time = current_time - previous_time;
      current_rps += (angle_btw_holes/elapsed_time)*pow(10,6);
      previous_time = current_time;
      i++;
      while(current_value > threshold_lower){
        current_value = analogRead(analogPinPhoto1);
        delayMicroseconds(20);
      }
    }
    previous_value = current_value;
    delayMicroseconds(100);
//    delayMicroseconds(3);
    if(noHoleCounter>thresh){
//      RPS[0] = 0;
      break;
    }
    noHoleCounter++;
  }
  current_rps /= numHoles;
//  if(current_rps<100){
//    Serial.println(current_rps);
//  }
  
//  RPS[0] = current_rps;

//  current_value = 0;
//  previous_value = 1;
//
//  i=0;
//  while(i<numHoles){
//  current_value = analogRead(analogPinPhoto2);
//  if(previous_value > threshold_fall && current_value < threshold_fall){
//    current_time = (unsigned int)Time;
//    elapsed_time = current_time - previous_time;
//    current_rps += (angle_btw_holes/elapsed_time)*pow(10,6);
//    previous_time = current_time;
//    i++;
//  }
//  previous_value = current_value;
//  delayMicroseconds(300);
//  } 
//  current_rps /= numHoles;
//  RPS[1] = current_rps;

//  RPS[1] = -20;
//  RPS[0] = -21;
}
//}
