#include <PWMServo.h>

#define NUM_POINTS 2000
#define BIG_NUMBER 1000000

PWMServo myservoF;
PWMServo myservoB;
bool debug = true;
int magic_thresh = 212;
int window_thresh = 218;
int analogPinPhotoB = 31;
int analogPinPhotoF = 32;

int* get_rps_data(unsigned int dt){
  static int rps[NUM_POINTS*2];
  for(int i=0;i<NUM_POINTS;i++){
    rps[i] = analogRead(analogPinPhotoF);
    rps[i+NUM_POINTS] = analogRead(analogPinPhotoB);
    delayMicroseconds(dt);
  }
  if(debug){
    Serial.println((sizeof(rps) / sizeof(rps[0])));
  }
  return rps;  
}

float* get_rps(){
  // Constants
  int i=0, j=NUM_POINTS;
  int* rpsData = get_rps_data(50);
  int count[6] = {0,0,BIG_NUMBER,0,0,BIG_NUMBER};
  static float rps[2] = {.0,.0};
  // Outer loop
  while(i<NUM_POINTS-1 || j<NUM_POINTS-1){
    // Front RPS part
    // Rising edge: next point above threshold, current point below
    if(rpsData[i+1] > magic_thresh && rpsData[i] <= magic_thresh){
      // Count the peaks and record locations
      ++count[0];
      if(count[0]==1){
        count[1] = i;
      }
      count[2] = i;
      // value between thresholds as window values
      int window_valueF = rpsData[i+1];
      // Skip rest of the same peak
      while(window_valueF < window_thresh){  
        ++i;
        if(i==NUM_POINTS-2){
          return rps;
        } 
        window_valueF = rpsData[i+1];
      }
    }
    ++i; // increment front counter
    // Back RPS part
    // Rising edge: next point above threshold, current point below
    if(rpsData[j+1] > magic_thresh && rpsData[j] <= magic_thresh){
      // Count the peaks and record locations
      ++count[3];
      if(count[3]==1){
        count[4] = j;
      }
      count[5] = j;
      // value between thresholds as window values
      int window_valueB = rpsData[j+1];
      // Skip rest of the same peak
      while(window_valueB < window_thresh){  
        ++j;
        if(j==NUM_POINTS-2){
          return rps;
        } 
        window_valueB = rpsData[j+1];
      }
    }
    ++j; // increment front counter
  }
  if(debug){
    Serial.print(count[0]);
    Serial.print("\t");
    Serial.println(count[2]-count[1]);
    Serial.print(count[3]);
    Serial.print("\t");
    Serial.println(count[5]-count[4]);
  }
  // Calculate RPS. If only one peak, set speed to 0
  if(count[0]==1){
    rps[0]=0;
  }else if(count[3]==1){
    rps[1]=0;
  }else{
    rps[0]=16.0/((float)(count[2]-count[1])*pow(10,-6));
    rps[1]=16.0/((float)(count[5]-count[4])*pow(10,-6));
  }
  return rps;
}

void setup() {
  // put your setup code here, to run once:
  myservoB.attach(3);         
  myservoF.attach(2);         
  myservoB.write(10);
  myservoF.write(170);
}


void loop() {
  // put your main code here, to run repeatedly:
  float* rps;
  rps = get_rps();
  if(debug){
    Serial.println((sizeof(rps) / sizeof(rps[0])));
    Serial.print(rps[0]);
    Serial.print(",");
    Serial.print(rps[1]);
    Serial.println();
  }
}
