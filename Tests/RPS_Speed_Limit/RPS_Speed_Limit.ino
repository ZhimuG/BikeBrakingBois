#include <PWMServo.h>
#include "SdFat.h"
#include "BufferedPrint.h"

#define NUM_POINTS 3000
#define BIG_NUMBER 1000000000

// For SD write:
// SD_FAT_TYPE = 0 for SdFat/File as defined in SdFatConfig.h,
// 1 for FAT16/FAT32, 2 for exFAT, 3 for FAT16/FAT32 and exFAT.
#define SD_FAT_TYPE 0
/*
  Change the value of SD_CS_PIN if you are using SPI and
  your hardware does not use the default value, SS.
  Common values are:
  Arduino Ethernet shield: pin 4
  Sparkfun SD shield: pin 8
  Adafruit SD shields and modules: pin 10
*/

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

PWMServo myservoF;
PWMServo myservoB;
bool debug = true;
int magic_thresh = 212;
int window_high = 218;
int window_low = 208;
int analogPinPhotoB = 31;
int analogPinPhotoF = 32;

int loopCount = 0;

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

void writeSD_int(int data){
  file_t file;
  BufferedPrint<file_t, 64> bp;

  //--------------------------------------------
  // Change the file name if necessary
  char fileName[13] = "TestRPS.txt";
  //char fileName[10] = "TestPot.txt"; //filename;
  
  //--------------------------------------------
  
  if (!file.open(fileName, O_RDWR | O_APPEND)) {
      sd.errorHalt(&Serial, F("open failed"));
    }
  if (1) {
    bp.begin(&file);
  }

  //-----------------------------------------------
  // Change this section if input data type changes
  file.println();
  file.print(data);
  file.print(",");
  
  //-----------------------------------------------
  
  if (1) {
    bp.sync();
  }
  if (file.getWriteError()) {
    sd.errorHalt(&Serial, F("write failed"));
  }
  double s = file.fileSize();
  file.close();
}

void writeSD(int* data){
  file_t file;
  BufferedPrint<file_t, 64> bp;

  //--------------------------------------------
  // Change the file name if necessary
  char fileName[13] = "TestRPS.txt";
  //char fileName[10] = "TestPot.txt"; //filename;
  
  //--------------------------------------------
  
  if (!file.open(fileName, O_RDWR | O_APPEND)) {
      sd.errorHalt(&Serial, F("open failed"));
    }
  if (1) {
    bp.begin(&file);
  }

  //-----------------------------------------------
  // Change this section if input data type changes
  file.print("Trial #");
  file.print(loopCount);
  file.println();
  for(int i=0; i<NUM_POINTS*2; i++){
    file.print(data[i]);
    file.print(",");
  }
  file.println();
  
  //-----------------------------------------------
  
  if (1) {
    bp.sync();
  }
  if (file.getWriteError()) {
    sd.errorHalt(&Serial, F("write failed"));
  }
  double s = file.fileSize();
  file.close();
}

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

void buzz_setup(int pwmpin){
  pinMode(pwmpin, OUTPUT);  
  digitalWrite(pwmpin, LOW);
}

void setup() {
  // put your setup code here, to run once:
  myservoB.attach(3);         
  myservoF.attach(2);         
  myservoB.write(10);
  myservoF.write(170);
  buzz_setup(14);
}


void loop() {
  loopCount++;
  // put your main code here, to run repeatedly:
  float* rps;
  rps = get_rps();
  if(debug){
    Serial.print(rps[0]);
    Serial.print(",");
    Serial.print(rps[1]);
    Serial.println();
  }
  if(rps[0] > 20.0/3.6 || rps[1] > 20.0/3.6){
    make_buzz(14, 500, 200);
  }
}
