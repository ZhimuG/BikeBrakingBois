//=============================================================================================================
//======================================== Include Statements =================================================
#include <PWMServo.h>
#include "SdFat.h"
#include "BufferedPrint.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//=============================================================================================================

//=============================================================================================================
//========================================= Global Variables ==================================================
PWMServo myservoF;  // create servo object to control a servo
PWMServo myservoB;  // create servo object to control a servo

//----------------------------------------------- Constants ---------------------------------------------------
//------------------------------------------ For RunNoSlipNoFlipAlgo ------------------------------------------
float g = 9.81; //[m/s^2]
int p_i_max = 150; // Define the potentiometer position corresponding to a maximum braking force [10 170]
float mu_s = 0.4; // Similar to tire rubber on grass (underestimated for normal cycling conditions)
float d_C1_COM[3] = {0.5, 1, 0}; //[m] x,y,z components of distance from C1 to COM
float d_C1_C2[3] = {1.12, 0, 0}; //[m] x,y,z components of distance from C1 to C2
float M = 80; //[kg]
float SB1 = 0; 
float R = 0.66/2; //[m]
float r = 0.16/2; //[m]
float I_A2 = 0.9*R*R; //[kg*m^2]
float d_C2_COM[3] = {0.7, 1, 0}; //[m]
float SB2 = 0; 
float I_A1 = 0.9*R*R; //[kg*m^2]
float d_A1_COM[3] = {0.3, 0.6, 0}; //[m]

//------------------------------------------ For ReadPot ------------------------------------------------------
const int potpin = A3;  // analog pin used to connect the potentiometer

//------------------------------------------ For SD write -----------------------------------------------------
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

//---------------------------------------- For MPU6050 --------------------------------------------------------
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatusF;      // return status after each device operation (0 = success, !0 = error)
uint8_t devStatusB;
uint16_t packetSizeF;    // expected DMP packet size (default is 42 bytes)
uint16_t packetSizeB;
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
elapsedMicros Time;
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 linAccel;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
VectorFloat linSpeed = VectorFloat();
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//=============================================================================================================

////////////////////////////////////////////////////////////////////////////////////////
// Read input potentiometer value
int ReadPot(const int potpin){
  val=0;
  for(int i=0; i<5; i++){
    val += analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023)
    delay(5);
  }
  val = val/5;
  //Serial.println(val);
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

////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////// Need to write this
// Read the input angle of incline (in radians)
float ReadAngle(){
  float theta = 0.1; //[rad]
  return theta;
}
//////////////////////////////////////////////////////////////////////////////////////// 

// Helper Functions for NoSlipNOFlipAlgo()
float DesiredGroundFriction(float F_F_max, int p_i_max, int p_i){
  float F_F_desired = min(F_F_max, F_F_max*p_i/p_i_max);
  return F_F_desired;
}

float MaximumGroundFriction(float mu_s,float* d_C1_COM, float M, float theta, float* d_C1_C2){
  //   d_C1_COM is a 3D vector of the x, y, z components of the displacement from C1 to COM. 
  //   theta is the angle of incline in radians 
  //   d_C1_C2 is just the x-component since it is defined to point in the x-direction

  float F_N = M*g*(d_C1_COM[0]*cos(theta)-d_C1_COM[1]*sin(theta))/d_C1_C2[0];
  float F_F_max = mu_s*F_N;
  return F_F_max;
}

float ProvidedDiskBraking(float M, float R, float r, float I_A2, float F_F_desired, float F_F_provided){
  float F_b_provided = (F_F_provided*R - I_A2*F_F_desired/(R*M))/r;
  return F_b_provided;
}

float MaximumGroundFrictionNoseOver(float M, float theta,float* d_A1_COM){
  // Assume a_z=0
  // Assume back tire never leaves the ground (but normal force is zero, therefore friction is zero)
  // Assume no acceleration about front axle
  
  // Using hungarian equation:
  float a_x = g*(sin(theta)*d_A1_COM[1]-cos(theta)*d_A1_COM[0])/d_A1_COM[1];
  float F_F1_NO = -M*(a_x-g*sin(theta));
  return F_F1_NO;
}

void RunNoSlipNoFlipAlgo(float* F_b_out, float F_F_max,int p_i_max,int p_i, float mu_s,float* d_C1_COM,float M,float theta,float* d_C1_C2,float SB1,float R,float r,float I_A2,float* d_C2_COM,float SB2,float I_A1,float* d_A1_COM){
  // Step 1:
  float F_F_desired = DesiredGroundFriction(F_F_max, p_i_max, p_i);
  // Step 2: 
  float F_F2_max = MaximumGroundFriction(mu_s,d_C1_COM,M,theta,d_C1_C2);
  // Step 3: 
  float leftover1 = F_F_desired - (F_F2_max - SB1);
  float F_F2_provided, F_b2_provided;
  if(leftover1 <= 0){
    // Step 4:
    F_F2_provided = F_F_desired;
    F_b2_provided = ProvidedDiskBraking(M, R, r, I_A2, F_F_desired, F_F2_provided);
    F_b_out[0] = 0;
    F_b_out[1] = F_b2_provided;
  }
  else{
    // Step 5:
    F_F2_provided = F_F2_max - SB1;
    F_b2_provided = ProvidedDiskBraking(M, R, r, I_A2, F_F_desired, F_F2_provided);
    F_b_out[1] = F_b2_provided;
    
    // Step 6:
    // See value of d_A1_COM, it is purposely adjusted forwards
    float F_F1_NO = MaximumGroundFrictionNoseOver(M, theta, d_A1_COM);
    
    // Step 7:
    d_C2_COM[0] = d_C2_COM[0] - 0.2; // Adjust COM backwards
    float F_F1_Smax = MaximumGroundFriction(mu_s,d_C2_COM,M,theta,d_C1_C2);

    // Step 8:
    float F_F1_max = min(F_F1_NO,F_F1_Smax);
    float leftover2 = leftover1 - (F_F1_max - SB2);

    float F_F1_provided, F_b1_provided;
    if(leftover2 <= 0){
        // Step 9:
        F_F1_provided = leftover1;
        F_b1_provided = ProvidedDiskBraking(M, R, r, I_A1, F_F_desired, F_F1_provided);
        F_b_out[0] = F_b1_provided;
    }
    else{
        // Step 10:
        F_F1_provided = F_F1_max - SB2;
        F_b1_provided = ProvidedDiskBraking(M, R, r, I_A1, F_F_desired, F_F1_provided);
        F_b_out[0] = F_b1_provided;
    }
    
    //Serial.println(F_b_out[0]);
    //Serial.println(F_b_out[1]);
  }
  
  //return F_b_out;
}

float TheoreticalMaximumGroundFriction(float mu_s,float* d_C1_COM,float M,float theta,float* d_C1_C2,float SB1,float R,float r,float I_A2,float* d_C2_COM,float SB2,float I_A1,float* d_A1_COM){
  float F_F2_max = MaximumGroundFriction(mu_s,d_C1_COM,M,theta,d_C1_C2);
  float F_F1_NO = MaximumGroundFrictionNoseOver(M, theta, d_A1_COM);
  float F_F1_Smax = MaximumGroundFriction(mu_s,d_C2_COM,M,theta,d_C1_C2);
  float F_F1_max = min(F_F1_NO, F_F1_Smax);
  float F_F_max = F_F2_max + F_F1_max;
  return F_F_max;
}

//////////////////////////////////////////////////////////////////////////////////////// Need to write this (and run calibration test)
void ForceToPWM(int* PWM, float* F_b_out){
  //float PWM[2] = {20, 80};
  //return PWM;
//  float calibration1 = 1;
//  float calibration2 = 1;
//  F_b_out[0] = F_b_out[0]*calibration1;
//  F_b_out[1] = F_b_out[1]*calibration2;
//  return F_b_out;

  //float* PWM = malloc(sizeof(float)*2);
  PWM[0] = 20;
  PWM[1] = 80;
  //return PWM
}
////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////// 
void MoveMotors(int* PWM){
  //Serial.println("success");
  myservoF.write(10+PWM[0]);
  myservoB.write(170-PWM[1]);
}
////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////// Could use more sophisticated method here. Need to do this for both wheels
//void ReadRPS(double* RPS){
//  int analogPinPhoto1 = 1;
//  int analogPinPhoto2 = 2;
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
//  double angle_btw_holes = 3.14159/3.0;
//  int i = 0;
//  double threshold_fall = 300.0;
//  int numHoles = 6;
//
//  for(i=0; i<numHoles; i++){
//    current_value = analogRead(analogPinPhoto1);
//    if(previous_value > threshold_fall && current_value < threshold_fall){
//      current_time = (unsigned int)Time;
//      elapsed_time = current_time - previous_time;
//      current_rps += (angle_btw_holes/elapsed_time)*pow(10,6);
//      previous_time = current_time;
//    }
//    previous_value = current_value;
//    delayMicroseconds(300);
//  }
//  current_rps /= numHoles;
//  RPS[0] = current_rps;
//
//  current_value = 0;
//  previous_value = 1;
//  
//  for(i=0; i<numHoles; i++){
//  current_value = analogRead(analogPinPhoto2);
//  if(previous_value > threshold_fall && current_value < threshold_fall){
//    current_time = (unsigned int)Time;
//    elapsed_time = current_time - previous_time;
//    current_rps += (angle_btw_holes/elapsed_time)*pow(10,6);
//    previous_time = current_time;
//  }
//  previous_value = current_value;
//  delayMicroseconds(300);
//  } 
//  current_rps /= numHoles;
//  RPS[1] = current_rps;
//}
////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////// Need to write this
double ReadLinSpeed(){
  double linSpeed = 20; //[m/s]
  return linSpeed;
}
////////////////////////////////////////////////////////////////////////////////////////

//void writeSD(outputData* data){
//  file_t file;
//  BufferedPrint<file_t, 64> bp;
//
//  //--------------------------------------------
//  // Change the file name if necessary
//  char fileName[13] = "Test0.txt";
//  //--------------------------------------------
//  
//  if (!file.open(fileName, O_RDWR | O_CREAT | O_TRUNC)) {
//      sd.errorHalt(&Serial, F("open failed"));
//    }
//  if (1) {
//    bp.begin(&file);
//  }
//
//  //-----------------------------------------------
//  // Change this section if input data type changes
//  //file.println(i);
//
//  for(int j=0; j<3000; j++){
//        file.print(data[j].t);
//        file.print(data[j].pot);
//        for(int i = 0; i<3; i++){
//            file.print(data[j].meters[0].angle[i]);
//        }
//  for(int i = 0; i<3; i++){
//            file.print(data[j].meters[0].accel[i]);
//        }
//        for(int i = 0; i<3; i++){
//            file.print(data[j].meters[0].spd[i]);
//        }
//        for(int i = 0; i<3; i++){
//            file.print(data[j].meters[1].angle[i]);
//        }
//  for(int i = 0; i<3; i++){
//            file.print(data[j].meters[1].accel[i]);
//        }
//        for(int i = 0; i<3; i++){
//            file.print(data[j].meters[1].spd[i]);
//        }
//        file.print(data[j].PWM[0]);
//        file.print(data[j].PWM[1]);
//        file.print(data[j].RPS[0]);
//        file.print(data[j].RPS[1]);
//}
//  //-----------------------------------------------
//  
//  if (1) {
//    bp.sync();
//  }
//  if (file.getWriteError()) {
//    sd.errorHalt(&Serial, F("write failed"));
//  }
//  double s = file.fileSize();
//  file.close();
//}

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

//=====================================================================================================
//====================================== Set up functions =============================================
//=====================================================================================================
void setup_motors(){
  myservoB.attach(SERVO_PIN_B);         // attaches the servo on pin 9 to the servo object
  myservoF.attach(SERVO_PIN_A);         // attaches the servo on pin 9 to the servo object
  myservoB.write(10);
  myservoF.write(170);
}

void setup_SD(){
  if (!sd.begin(SD_CONFIG)) {
    sd.initErrorHalt(&Serial);
  }
}

void setup_mpu(){
  Wire.begin();
  Serial.begin(9600);
  mpuF.initialize();
  mpuB.initialize();
  Wire.setSDA(18);
  Wire.setSCL(19);
  Wire.setSDA(38);
  Wire.setSCL(37);
  mpuF.setXGyroOffset(0);
  mpuF.setYGyroOffset(0);
  mpuF.setZGyroOffset(0);
  mpuF.setZAccelOffset(1688);
  mpuB.setXGyroOffset(0);
  mpuB.setYGyroOffset(0);
  mpuB.setZGyroOffset(0);
  mpuB.setZAccelOffset(1688);
  devStatusF = mpuF.dmpInitialize();
  devStatusB = mpuB.dmpInitialize();
  if (devStatusF == 0 && devStatusB == 0) {
    mpuF.CalibrateAccel(6);
    mpuF.CalibrateGyro(6);
    mpuF.PrintActiveOffsets();
    mpuB.CalibrateAccel(6);
    mpuB.CalibrateGyro(6);
    mpuB.PrintActiveOffsets();
    mpuF.setDMPEnabled(true);
    mpuB.setDMPEnabled(true);
    dmpReady = true;
    packetSizeF = mpuF.dmpGetFIFOPacketSize();
    packetSizeB = mpuB.dmpGetFIFOPacketSize();
  }else{return;}
}
//=====================================================================================================

void setup() {
  setup_motors();
  //Serial.begin(9600);
  setup_SD();
  setup_mpu();
}

void loop() {
  // READ INPUTS
  int p_i = ReadPot(potpin); // Read the input potentiometer position
  float theta = ReadAngle(); // Read the gyroscope angle

  // PREVENTATIVE SYSTEM
  float* F_b_out = (float*)malloc(sizeof(float)*2); // F_b_out[0] = F_b1_out, F_b_out[1] = F_b2_out
  int* PWM = (int*)malloc(sizeof(int)*2); // PWM[0] = PWM_1, PWM[1] = PWM_2
  
  float F_F_max = TheoreticalMaximumGroundFriction(mu_s,d_C1_COM,M,0,d_C1_C2,SB1,R,r,I_A2,d_C2_COM,SB2,I_A1,d_A1_COM);
  RunNoSlipNoFlipAlgo(F_b_out,F_F_max,p_i_max,p_i,mu_s,d_C1_COM,M,theta,d_C1_C2,SB1,R,r,I_A2,d_C2_COM,SB2,I_A1,d_A1_COM); 

  ForceToPWM(PWM, F_b_out);
  MoveMotors(PWM);
  
  free(F_b_out);
  free(PWM);

  //////////////////////////////////////////////////////////////////////////////////////// Need to update this
  // REACTIVE SYSTEM
  double* RPS = (double*)malloc(sizeof(double)*2);
  
  // Infinite loop that only breaks with a significant change in potentiometer position
  ReadRPS(RPS);
  ////////////////////////////////////////////////////////////////////////////////////////

  // after loop:
  free(RPS);
  
  ///////////////////////////////////////////////////////////////////////////////////////
  // Need to write the data to the teensy at each time step
  ///////////////////////////////////////////////////////////////////////////////////////

  ///////////////////////////////////////////////////////////////////////////////////////
  // Need to write a function to activate the buzzer at a certain velocity
  ///////////////////////////////////////////////////////////////////////////////////////

//  // This is super weird we need to fix this (there will be a memory leak cuz there is no good time to free the data)
//  // Should probably just write to the file each time
//  outputData* data = (outputData*)malloc(sizeof(outputData)*3000);
//  free(data)

}
