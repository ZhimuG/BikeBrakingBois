// Controlling a servo position using a potentiometer (variable resistor)
// by Michal Rinott <http://people.interaction-ivrea.it/m.rinott>
// Adapted by OLBABS team

#include <PWMServo.h>

PWMServo myservoF;  // create servo object to control a servo
PWMServo myservoB;  // create servo object to control a servo

#include "SdFat.h"
#include "BufferedPrint.h"

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

char filename[12] = "move_servos_calibrated.txt";

void writeSD_setup(){
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
  unsigned long t = micros();
  file.print("Trial\t");
  file.print(t);
  file.println();
  if (1) {
    bp.sync();
  }
  if (file.getWriteError()) {
    sd.errorHalt(&Serial, F("write failed"));
  }
  double s = file.fileSize();
  file.close();
}

void writeSD_move_motors(int pot, int* PWM){
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
  file.print(pot);
  file.print(", ");
  file.print(PWM[0]);
  file.print(", ");
  file.print(PWM[1]);
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

void setup() {
  myservoF.attach(2);         // attaches the servo on pin 9 to the servo
  myservoB.attach(3);
  myservoB.write(10);
  myservoF.write(170);
  if (!sd.begin(SD_CONFIG)) {
    sd.initErrorHalt(&Serial);
  }
  writeSD_setup();
} 

void MoveMotors(int* PWM){
  //Serial.println("success");
  myservoF.write(10+PWM[0]);
  myservoB.write(170-PWM[1]);
}

const int potpin = A3;  // analog pin used to connect the potentiometer
int val;    // variable to read the value from the analog pin

// Read input potentiometer value
int ReadPot(const int potpin){
  val=0;
  for(int i=0; i<5; i++){
    val += analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023)
    delay(5);
  }
  val = val/5;
  Serial.println(val);
  int rounder = 10;
   if(val>180){
    val=200;
  }
  val = val/rounder;
//  Serial.println(val);
//  delay(20);
  val = map(val, 0, 20, 10, 170);     // scale it to use it with the servo (value between 0 and 180)
  return val;
}

void loop() {

  int* PWM = (int*)malloc(sizeof(int)*2); // PWM[0] = PWM_1, PWM[1] = PWM_2
  int val = ReadPot(potpin);
  PWM[0] = val;
  PWM[1] = val;
  MoveMotors(PWM);
  if(val<170){
    writeSD_move_motors(map(val, 10, 170, 0, 20), PWM);
  }
  // sets the servo position according to the scaled value
  delay(20);                           // waits for the servo to get there
}
