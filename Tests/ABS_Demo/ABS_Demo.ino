#include <PWMServo.h>
#include "SdFat.h"
#include "BufferedPrint.h"

PWMServo myservoF;  // create servo object to control a servo
PWMServo myservoB;

#define NUM_POINTS 3000
#define BIG_NUMBER 1000000000
#define Pi 3.1415926

elapsedMicros Time;

// SD_FAT_TYPE = 0 for SdFat/File as defined in SdFatConfig.h,
// 1 for FAT16/FAT32, 2 for exFAT, 3 for FAT16/FAT32 and exFAT.
#define SD_FAT_TYPE 0
// SDCARD_SS_PIN is defined for the built-in SD on some boards.
#ifndef SDCARD_SS_PIN
const uint8_t SD_CS_PIN = SS;
#else  // SDCARD_SS_PIN
// Assume built-in SD is used.
const uint8_t SD_CS_PIN = SDCARD_SS_PIN;
#endif  // SDCARD_SS_PIN
// Try to select the best SD card configuration.
#if HAS_SDIO_CLASS
#define SD_CONFIG SdioConfig(FIFO_SDIO)
#elif ENABLE_DEDICATED_SPI
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI)
#else  // HAS_SDIO_CLASS
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI)
#endif  // HAS_SDIO_CLASS
#if SD_FAT_TYPE == 0
SdFat sd;
typedef File file_t;
#elif SD_FAT_TYPE == 1
SdFat32 sd;
typedef File32 file_t;
#elif SD_FAT_TYPE == 2
SdExFat sd;
typedef ExFile file_t;
#elif SD_FAT_TYPE == 3
SdFs sd;
typedef FsFile file_t;
#else  // SD_FAT_TYPE
#error Invalid SD_FAT_TYPE
#endif  // SD_FAT_TYPE
// number of lines to print
const uint16_t N_PRINT = 20000;

const int potpin = A3;

// For RPS
bool debug = true;
int magic_thresh = 212;
int window_high = 218;
int analogPinPhotoB = 31;
int analogPinPhotoF = 32;

int* get_rps_data(int dt){
  static int rps[NUM_POINTS*2];
  for(int i=0;i<NUM_POINTS;i++){
    rps[i] = analogRead(analogPinPhotoF);
    rps[i+NUM_POINTS] = analogRead(analogPinPhotoB);
    delayMicroseconds(dt);
  }
  return rps;  
}

float* get_rps(){
  // Constants
  int i=0, j=NUM_POINTS, dt=50;
  int* rpsData = get_rps_data(dt);
  unsigned int count[6] = {0,0,BIG_NUMBER,0,0,BIG_NUMBER};
  static float rps[2] = {.0,.0};
  // Outer loop
  while(i<NUM_POINTS-1){
    // Front RPS part
    // Rising edge: next point above threshold, current point below
    if(rpsData[i+1] > magic_thresh && rpsData[i] <= magic_thresh){
      // Count the peaks and record locations
      // value between thresholds as window values
      int window_valueF = rpsData[i+1];
      // Skip rest of the same peak
      while(window_valueF < window_high){  
        ++i;
        if(i==NUM_POINTS-2){
          return rps;
        } 
        window_valueF = rpsData[i+1];
      }
      count[0]++;
      if(count[0]==1){
        count[1] = i;
      }
      count[2] = i;
    }
    ++i; // increment front counter
  }
  while(j<NUM_POINTS*2-1){
    // Back RPS part
    // Rising edge: next point above threshold, current point below
    if(rpsData[j+1] > magic_thresh && rpsData[j] <= magic_thresh){
      // Count the peaks and record locations
      // value between thresholds as window values
      int window_valueB = rpsData[j+1];
      // Skip rest of the same peak
      while(window_valueB < window_high){  
        ++j;
        if(j==NUM_POINTS*2-2){
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
    ++j; // increment front counter
  }
  // Calculate RPS. If only one peak, set speed to 0
  if(count[0]>1){
    rps[0]=(float)(count[0]-1)/((float)(count[2]-count[1])*(float)dt*pow(10,-6)*16.0);
  }else{
    rps[0]=0;
  }
  if(count[3]>1){
    rps[1]=(float)(count[3]-1)/((float)(count[5]-count[4])*(float)dt*pow(10,-6)*16.0);
  }else{
    rps[1]=0;
  }
  return rps;
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

void MoveMotors(int* PWM){
  //Serial.println("success");
  myservoF.write(PWM[0]);
  myservoB.write(PWM[1]);
}

// Read input potentiometer value
int ReadPot(const int potpin){
  val=0;
  for(int i=0; i<5; i++){
    val += analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023)
    delay(5);
  }
  val = val/5;
  int rounder = 10;
   if(val>100){
    val=120;
  }
  val = map(val/rounder, 0, 12, 10, 170);     // scale it to use it with the servo (value between 0 and 180)
  return val;
}

float ReadLinSpeed(float wheelRadius){
  float* rps = get_rps();
  return (rps[0]*wheelRadius*Pi);
}

void absAlgorithm(int PWM[]) {
    // save the original PWM values
    double PWM1 = PWM[0];
    double PWM2 = PWM[1];
    float wheelRadius = 0.6985; //[m]
    // find the current wheel rotation wheed and linear speed of the bike  
    float* rps = get_rps();
    float wheelRotationSpeed = (rps[0]>rps[1])? rps[1] : rps[0];
    float linSpeed = ReadLinSpeed(wheelRadius);
    double slipRatio = 1 - (wheelRadius * wheelRotationSpeed * Pi / linSpeed);
    // not slipping at all
    if(slipRatio < 0.19)
      return;
    // slipping, run the algorithm until stop moving or stop braking
    while((PWM[0] > 0 || PWM[1] > 0) && linSpeed > 0) {
        // set braking force to zero until slipRatio < 0.19
        PWM[0] = 0;
        PWM[1] = 0;
        MoveMotors(PWM);
        unsigned long previous_time = (unsigned long)Time;
        while(slipRatio > 0.19) {
          unsigned long current_time1 = (unsigned long)Time;
          if((current_time1-previous_time) > 100) {
              float* rps = get_rps();
              float wheelRotationSpeed = (rps[0]>rps[1])? rps[1] : rps[0];
              float linSpeed = ReadLinSpeed(wheelRadius);
              slipRatio = 1 - (wheelRadius * wheelRotationSpeed / linSpeed);
          }
        }
        // start braking at original force again until slipRatio > 0.19
        PWM[0] = PWM1;
        PWM[1] = PWM2;
        MoveMotors(PWM);
        previous_time = (unsigned long)Time;
        while(slipRatio < 0.19) {
          unsigned long current_time2 = (unsigned long)Time;
          if((current_time2-previous_time) > 100) {
              float* rps = get_rps();
              float wheelRotationSpeed = (rps[0]>rps[1])? rps[1] : rps[0];
              float linSpeed = ReadLinSpeed(wheelRadius);
              slipRatio = 1 - (wheelRadius * wheelRotationSpeed / linSpeed);
          }
        }   
      }  
    return;
}

void setup_motors(){
  myservoB.attach(3);
  myservoF.attach(2);
  myservoB.write(7);
  myservoF.write(170);
}

void buzz_setup(int pwmpin){
  pinMode(pwmpin, OUTPUT);  
  digitalWrite(pwmpin, LOW);
}

void setup_SD(){
  if (!sd.begin(SD_CONFIG)) {
    sd.initErrorHalt(&Serial);
  }
}


void setup() {
  // put your setup code here, to run once:
  setup_motors();
  buzz_setup(14);
//  setup_SD();
}

void loop() {
  // put your main code here, to run repeatedly:
  int val = ReadPot(potpin);
  int PWM[2] = {170-val, val+7};
  MoveMotors(PWM);
  absAlgorithm(PWM);
}
