#include "SdFat.h"
#include "BufferedPrint.h"
#include <PWMServo.h>

PWMServo myservoF;  // create servo object to control a servo
PWMServo myservoB;  // create servo object to control a servo

#define NUM_POINTS 2000
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

const int potpin = A3;  // analog pin used to connect the potentiometer
int val;    // variable to read the value from the analog pin

// For RPS
bool debug = true;
int magic_thresh = 212;
int window_high = 218;
int analogPinPhotoB = 31;
int analogPinPhotoF = 32;

int loopCount = 0;

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
  myservoB.attach(3);         // attaches the servo on pin 9 to the servo object
  myservoF.attach(2);         // attaches the servo on pin 9 to the servo object
  //  Serial.begin(9600);
  myservoB.write(10);
  myservoF.write(170);
  if (!sd.begin(SD_CONFIG)) {
    sd.initErrorHalt(&Serial);
  }
  buzz_setup(14);
  make_buzz(14, 500, 1000);
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
   if(val>180){
    val=220;
  }
  val = map(val/rounder, 0, 22, 10, 170);     // scale it to use it with the servo (value between 0 and 180)
  return val;
}


void writeSD_cal(float* RPS_arr, float dt, int N, int PWM){
  file_t file;
  BufferedPrint<file_t, 64> bp;

  //--------------------------------------------
  // Change the file name if necessary
  char fileName[23] = "Calibration_wheel0.txt";
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
//  int data_arr_len = 6;
  file.print("Datapoint ");
  file.print(loopCount);
  file.println();
  file.print(PWM);
  file.print(", ");
  file.print(dt);
  file.print(", ");
  file.print(N);
  file.print(", ");
  for(int i=0; i<N; i++){
    file.print(RPS_arr[i]);
    file.print(" ");
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


void loop() {
  if(loopCount > 17){
    return;
  }
  buzz_stop(14);
  
  // Step 1: Reset motor positions to zero
  loopCount++;
  int stepSize = 10;
  int PWM[2] = {170, 10}; // PWM[0] = PWM_1, PWM[1] = PWM_2
  MoveMotors(PWM);
  // Step 2: Choose PWM 
  //PWM[0] = 100;
  PWM[1] = stepSize*loopCount;
  // Step 3: Spin wheel and start the program 
  // monitor the pot in a while loop to trigger program
  int pot = ReadPot(potpin);
  int thresh = 80;
  while(pot>thresh){
        pot = ReadPot(potpin);
        delay(1);
      }
  // Step 4: Apply brake at PWM for N measurements
//  unsigned int prev = (unsigned int)Time;
  int N = 100;
  float dt = 30; //[ms]
  float RPS_arr[N];
  float* RPS;
  MoveMotors(PWM);
  for(int i=0;i<N;i++){
    // Step 5: read the RPS every dt RPS_arr[i]
    RPS = get_rps();
    RPS_arr[i] = RPS[1];
    delay(dt);
  }
  // Step 6: Write RPS_arr, dt, N
  // can just write this to the serial monitor
  writeSD_cal(RPS_arr, dt, N, PWM[1]);
  make_buzz(14, 500, 1000);
}
