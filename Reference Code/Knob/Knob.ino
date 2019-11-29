// Controlling a servo position using a potentiometer (variable resistor)
// by Michal Rinott <http://people.interaction-ivrea.it/m.rinott>

// http://arduiniana.org/libraries/pwmservo/

//   Board                     SERVO_PIN_A   SERVO_PIN_B   SERVO_PIN_C
//   -----                     -----------   -----------   -----------
//   Arduino Uno, Duemilanove       9            10          (none)
//   Arduino Mega                  11            12            13
//   Sanguino                      13            12          (none)
//   Teensy 1.0                    17            18            15
//   Teensy 2.0                    14            15             4
//   Teensy++ 1.0 or 2.0           25            26            27
//   Teensy LC & 3.x                 (all PWM pins are usable)

#include <PWMServo.h>
#include "SdFat.h"
#include "BufferedPrint.h"

PWMServo myservoF;  // create servo object to control a servo
PWMServo myservoB;

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

void writeSD(int data){
  file_t file;
  BufferedPrint<file_t, 64> bp;

  //--------------------------------------------
  // Change the file name if necessary
  char fileName[13] = "Interference_Test.txt";
  
  //--------------------------------------------
  
  if (!file.open(fileName, O_RDWR | O_APPEND)) {
      sd.errorHalt(&Serial, F("open failed"));
    }
  if (1) {
    bp.begin(&file);
  }

  //-----------------------------------------------
  // Change this section if input data type changes
  file.println(data);
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

int ReadPot(const int potpin){
  val=0;
  for(int i=0; i<5; i++){
    val += analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023)
    delay(5);
  }
  val = val/5;
//  Serial.println(val);
  int rounder = 10;
   if(val>180){
    val=220;
  }
  val = val/rounder;
//  Serial.println(val);
//  delay(20);
  val = map(val, 0, 22, 10, 170);     // scale it to use it with the servo (value between 0 and 180)
  return val;
}

void MoveMotors(int PWM){
  //Serial.println("success");
  myservoF.write(170-PWM);
  myservoB.write(10+PWM);
}

void setup() {
  // put your setup code here, to run once:
  myservoB.attach(2);         // attaches the servo on pin 9 to the servo object
  myservoF.attach(3);         // attaches the servo on pin 9 to the servo object
  //  Serial.begin(9600);
  myservoB.write(10);
  myservoF.write(170);
}

void loop() {
  val = ReadPot(potpin);            // reads the value of the potentiometer (value between 0 and 1023)
//  Serial.println(val);
//  val = map(val, 0, 1023, 0, 179);     // scale it to use it with the servo (value between 0 and 180)
  MoveMotors(val);
  Serial.println(analogRead(31));
//  writeSD(analogRead(31));
  delay(15);                           // waits for the servo to get there
}

// Modular code starts here:
// Get potentiometer input
//int getPotentiometerInput(const int potpin){
//  int val = 0;
//  val = analogRead(potpin);
//  return val;
//}
//
//// Write PWM signal to servomotor
//void writeToMotor(int* forcePWM, int* servoPorts){
//  PWMServo myservoF, myservoB;
//  myservoF.attach(servoPorts[0]);
//  myservoB.attach(servoPorts[1]);
//  myservoF.write(forcePWM[0]);
//  myservoB.write(forcePWM[1]);
//}
