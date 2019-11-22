#include "SdFat.h"
#include "BufferedPrint.h"
#include <PWMServo.h>

PWMServo myservoF;  // create servo object to control a servo
PWMServo myservoB;  // create servo object to control a servo

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
char filename[12] = "TestPot.txt";

int loopCount = 0;

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
}

void writeSD(double* data){
  file_t file;
  BufferedPrint<file_t, 64> bp;

  //--------------------------------------------
  // Change the file name if necessary
  char fileName[13] = "TestPot.txt";
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
  int data_arr_len = 6;
  for(int i=0; i<data_arr_len; i++){
    file.print(data[i]);
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
//
//void ReadRPS(double* RPS){
//  int analogPinPhoto1 = 31;
//  int analogPinPhoto2 = 32;
//  double current_value = 0;
//  //int numDecreasingPoints = 4;
//  //int count = 0;
//  double previous_value = 1;
//  elapsedMicros Time;
//  unsigned int previous_time;
//  unsigned int elapsed_time;
//  unsigned int current_time;
//  
//  double current_rps = 0;
//  double angle_btw_holes = 3.14159/8.0;
//  int i = 0;
//  double threshold_fall = 100.0;
//  int numHoles = 6;
//  int noHoleCounter = 0;
//  int thresh = 10000;
//
//  while(i<numHoles){
//    current_value = analogRead(analogPinPhoto1);
////    Serial.println(current_value);
//    if(previous_value > threshold_fall && current_value < threshold_fall){
//      current_time = (unsigned int)Time;
//      elapsed_time = current_time - previous_time;
//      current_rps += (angle_btw_holes/elapsed_time)*pow(10,6);
//      previous_time = current_time;
//      i++;
//    }
//    previous_value = current_value;
//    delayMicroseconds(300);
////    delayMicroseconds(3);
//    if(noHoleCounter>thresh){
//      RPS[0] = 0;
//      break;
//    }
//    noHoleCounter++;
//  }
//  current_rps /= numHoles;
//  RPS[0] = current_rps;
//
////  current_value = 0;
////  previous_value = 1;
////
////  i=0;
////  while(i<numHoles){
////  current_value = analogRead(analogPinPhoto2);
////  if(previous_value > threshold_fall && current_value < threshold_fall){
////    current_time = (unsigned int)Time;
////    elapsed_time = current_time - previous_time;
////    current_rps += (angle_btw_holes/elapsed_time)*pow(10,6);
////    previous_time = current_time;
////    i++;
////  }
////  previous_value = current_value;
////  delayMicroseconds(300);
////  } 
////  current_rps /= numHoles;
//  RPS[1] = current_rps;
//
////  RPS[1] = -20;
////  RPS[0] = -21;
//}

void MoveMotors(int* PWM){
  //Serial.println("success");
  myservoF.write(PWM[0]);
  myservoB.write(PWM[1]);
}

// Read input potentiometer value
int ReadPot(const int potpin){
  int val = analogRead(potpin);
  return val;
}

void print_cal(double* RPS_arr, float dt, int N, int PWM){
  Serial.print(PSTR(PWM));
  Serial.print(PSTR(", "));
  Serial.print(PSTR(dt));
  Serial.print(PSTR(", "));
  Serial.print(PSTR(N));
  Serial.print(PSTR(", "));
  for(int i=0; i<N; i++){
    Serial.print(PSTR(RPS_arr[i]));
    Serial.print(PSTR(" "));
  }
  Serial.println();
}

void writeSD_cal(double* RPS_arr, float dt, int N, int PWM){
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


// For RPS
int magic_thresh = 212;
int window_thresh = 218;
int analogPinPhoto1 = 31;
int analogPinPhoto2 = 32;
elapsedMicros Time;
unsigned int max_time = 300; //[ms]

bool get_rps_flag(int hole_count, int max_count, int previous_value, unsigned int previous_time, int analogPinPhoto){
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
    return get_rps_flag(hole_count, max_count, current_value, previous_time, analogPinPhoto);
  }
  else if(((unsigned int)Time - previous_time)*pow(10,-3) > max_time){
//    Serial.println("yoyo");
    return false;
  } 
  else{
    return get_rps_flag(hole_count, max_count, current_value, previous_time, analogPinPhoto);
  }
  // add window threshold
  // add delay?  
}

double get_rps(int num_holes, int analogPinPhoto){
  bool rps_flag = false;
  unsigned int previous_time = (unsigned int)Time;
  int hole_count = 0;
  rps_flag = get_rps_flag(hole_count, num_holes, analogRead(analogPinPhoto), previous_time, analogPinPhoto);
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

void loop() {
  // put your main code here, to run repeatedly:
  double rps_back = get_rps(2, analogPinPhoto1);
  Serial.println(rps_back);
  delay(20);
  

// Calibration:  
//  // Step 1: Reset motor positions to zero
//  loopCount++;
//  int stepSize = 10;
//  int* PWM = (int*)malloc(sizeof(int)*2); // PWM[0] = PWM_1, PWM[1] = PWM_2
//  PWM[0] = 170;
//  PWM[1] = 10;
//  MoveMotors(PWM);
//  // Step 2: Choose PWM 
//  //PWM[0] = 100;
//  PWM[0] = 170;
//  PWM[1] = stepSize*loopCount + 30;
//  // Step 3: Spin wheel and start the program 
//  // monitor the pot in a while loop to trigger program
//  int pot = ReadPot(potpin);
//  int thresh = 100;
//  while(pot>thresh){
//        pot = ReadPot(potpin);
////        Serial.println(pot);
//        delay(1);
//      }
//  // Step 4: Apply brake at PWM for N measurements
////  unsigned int prev = (unsigned int)Time;
//  int N = 100;
//  float dt = 30; //[ms]
//  double RPS_arr[N];
//  double* RPS = (double*)malloc(sizeof(double)*2);
//  MoveMotors(PWM);
//  for(int i=0;i<N;i++){
//    // Step 5: read the RPS every dt RPS_arr[i]
////    ReadRPS(RPS);
////    RPS_arr[i] = RPS[0];
////    Serial.println(RPS[0]);
////    unsigned int prev = (unsigned int)Time;
//    RPS_arr[i] = get_rps(2, analogPinPhoto1);
//    delay(dt);
////    unsigned int curr = (unsigned int)Time;
////    if(RPS[0] == 0){
////      break;
////    }
////    Serial.println(((double)curr-(double)prev)*pow(10,-6));
//  }
//  // Step 6: Write RPS_arr, dt, N
////  unsigned int curr = (unsigned int)Time;
//  // can just write this to the serial monitor
//  print_cal(RPS_arr, dt, N, PWM[1]);
////  Serial.println(((double)curr-(double)prev)*pow(10,-6));
//  free(PWM);
//  free(RPS);
}
