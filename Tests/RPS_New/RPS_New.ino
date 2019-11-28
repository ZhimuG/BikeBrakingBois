#include <PWMServo.h>

#define NUM_POINTS 2000

PWMServo myservoF;
PWMServo myservoB;
int magic_thresh = 212;
int window_thresh = 218;
int analogPinPhotoB = 31;
int analogPinPhotoF = 32;
elapsedMicros Time;
unsigned int max_time = 500; //[ms]

int* get_rps_data(unsigned int dt){
  static int rps[NUM_POINTS*2];
  for(int i=0;i<NUM_POINTS;i++){
    rps[i] = analogRead(analogPinPhotoB);
    rps[i+NUM_POINTS] = analogRead(analogPinPhotoF);
    delayMicroseconds(dt);
  }
  return rps;  
}

float* get_rps(){
  int i=0, j=NUM_POINTS;
  int* rpsData = get_rps_data(50);
  int count[6] = {0,0,0,0,0,0};
  static float rps[2] = {.0,.0};
  while(i<NUM_POINTS || j<NUM_POINTS){
    if(rpsData[i+1] > magic_thresh && rpsData[i] <= magic_thresh){
      int window_valueF = rpsData[i+1];
      while(window_valueF < window_thresh){  
        ++i;
        if(i==NUM_POINTS-2){
          return rps;
        } 
        window_valueF = rpsData[i+1];
      }
      ++count[0];
      if(count[0]==1){
        count[1] = i;
      }
      count[2] = i;
    }

    if(rpsData[j+1] > magic_thresh && rpsData[j] <= magic_thresh){
      int window_valueB = rpsData[j+1];
      while(window_valueB < window_thresh){  
        ++j;
        if(j==NUM_POINTS-2){
          return rps;
        } 
        window_valueB = rpsData[j+1];
      }
      ++count[3];
      if(count[3]==1){
        count[4] = j;
      }
      count[5] = j;
    }
    ++i;++j;
  }
  if(count[1]==count[2]){
    rps[0]=0;
  }else if(count[4]==count[5]){
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
}
