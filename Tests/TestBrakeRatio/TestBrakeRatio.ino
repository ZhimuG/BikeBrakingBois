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
MPU6050 mpuF;
MPU6050 mpuB;

//----------------------------------------------- Constants ---------------------------------------------------
//------------------------------------------ For RunNoSlipNoFlipAlgo ------------------------------------------
float g = 9.81; //[m/s^2]
int p_i_max = 100; // Define the potentiometer position corresponding to a maximum braking force [10 170]
float mu_s = 0.4; // Similar to tire rubber on grass (underestimated for normal cycling conditions)
float d_C1_COM[3] = {0.69, 1, 0}; //[m] x,y,z components of distance from C1 to COM
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

//------------------------------------ For RPS ----------------------------------------------------------------
int magic_thresh = 212;
int window_thresh = 218;
int analogPinPhotoB = 31;
int analogPinPhotoF = 32;
unsigned int max_time = 300; //[ms]

//-------------------------------------- For ABS --------------------------------------------------------------
float wheelRadius = 0.6604; //[m]

//=============================================================================================================
//======================================= Modular Functions ===================================================
//------------------------------- Read input potentiometer value ----------------------------------------------
int ReadPot(const int potpin){
  int val=0;
  for(int i=0; i<5; i++){
    val += analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023)
    delay(5);
  }
  val = val/5;
  //Serial.println(val);
  int rounder = 10;
   if(val>100){
    val=120;
  }
  val = val/rounder;
//  Serial.println(val);
//  delay(20);
  val = map(val, 0, 12, 10, 170);     // scale it to use it with the servo (value between 0 and 180)
  return val;
}

//---------------------------- Read Linear Acceleration ------------------------------------------------------
void ReadLinAccel(MPU6050 mpu, uint16_t packetSize){
  fifoCount = mpu.getFIFOCount();
  if(fifoCount >= 1024){
    mpu.resetFIFO();
  }
  else if (_BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
    while(fifoCount >= packetSize){
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;
    }
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&linAccel, &aaReal, &q);
    linAccel.x = linAccel.x/2048.0*9.80665;
    linAccel.y = linAccel.y/2048.0*9.80665;
    linAccel.z = linAccel.z/2048.0*9.80665;
  }
}

//---------------------------- Read the angle of incline (in radians) -----------------------------------
void ReadAngle(MPU6050 mpu, uint16_t packetSize){
  fifoCount = mpu.getFIFOCount();
  if(fifoCount >= 1024){
    mpu.resetFIFO();
  }
  else if (_BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
    while(fifoCount >= packetSize){
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;
    }
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  }
}

//--------------------------------- Read Linear Speed ----------------------------------------------------
void ReadLinSpeed(MPU6050 mpu, uint16_t packetSize){
  unsigned int prev = (unsigned int)Time;
  ReadLinAccel(mpu, packetSize);
  unsigned int curr = (unsigned int)Time;
  linSpeed.x += linAccel.x*1.0*(curr-prev)*pow(10,-6);
    linSpeed.y += linAccel.y*1.0*(curr-prev)*pow(10,-6);
    linSpeed.z += linAccel.z*1.0*(curr-prev)*pow(10,-6);
}

//------------------------- Buzzer Functions -------------------------------------------------------------
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

//------------------------- Helper Functions for NoSlipNOFlipAlgo() --------------------------------------
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
//  Serial.println(F_F2_max);
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
    //d_C2_COM[0] = d_C2_COM[0] - 0.2; // Adjust COM backwards
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
  }
  if(F_b_out[0] < 0){
    F_b_out[0] = 0; 
  }
   if(F_b_out[1] < 0){
    F_b_out[1] = 0; 
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

//----------------------------------------- ABS Algorithm ----------------------------------------------------------
void absAlgorithm(int* PWM) {
    // save the original PWM values
    double PWM1 = PWM[0];
    double PWM2 = PWM[1];
    // find the current wheel rotation wheed and linear speed of the bike  
    double wheelRotationSpeedF = get_rps(2, analogPinPhotoF);
    double wheelRotationSpeedB = get_rps(2, analogPinPhotoB);
    double wheelRotationSpeed = (wheelRotationSpeedF>wheelRotationSpeedB)? wheelRotationSpeedB : wheelRotationSpeedF;
    ReadLinSpeed(mpuF, packetSizeF);
    double slipRatio = 1 - (wheelRadius * wheelRotationSpeed / linSpeed.x);
    // not slipping at all
    if(slipRatio < 0.19)
      return;
    // slipping, run the algorithm until stop moving or stop braking
    while((PWM[0] > 0 || PWM[1] > 0) && linSpeed.x > 0) {
        // set braking force to zero until slipRatio < 0.19
        PWM[0] = 0;
        PWM[1] = 0;
        MoveMotors(PWM);
        unsigned long previous_time = (unsigned long)Time;
        while(slipRatio > 0.19) {
          unsigned long current_time1 = (unsigned long)Time;
          if((current_time1-previous_time) > 100) {
              float wheelRotationSpeedF = get_rps(2, analogPinPhotoF);
              float wheelRotationSpeedB = get_rps(2, analogPinPhotoB);
              wheelRotationSpeed = wheelRotationSpeedF > wheelRotationSpeedB ? wheelRotationSpeedB : wheelRotationSpeedF;
              ReadLinSpeed(mpuF, packetSizeF);
              slipRatio = 1 - (wheelRadius * wheelRotationSpeed / linSpeed.x);
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
              float wheelRotationSpeedF = get_rps(2, analogPinPhotoF);
              float wheelRotationSpeedB = get_rps(2, analogPinPhotoB);
              wheelRotationSpeed = wheelRotationSpeedF > wheelRotationSpeedB ? wheelRotationSpeedB : wheelRotationSpeedF;
              ReadLinSpeed(mpuF, packetSizeF);
              slipRatio = 1 - (wheelRadius * wheelRotationSpeed / linSpeed.x);
          }
        }   
      }  
    return;
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
  myservoB.attach(3);         // attaches the servo on pin 9 to the servo object
  myservoF.attach(2);         // attaches the servo on pin 9 to the servo object
  myservoB.write(10);
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

void setup_mpu(){
  Wire.begin();
  Serial.begin(9600);
//  fifoCount = mpuF.getFIFOCount();
  mpuF.resetFIFO();
//  if(fifoCount >= 1024){
//    
//  }
  
  mpuF.initialize();
//  mpuB.initialize();
//  Wire.setSDA(18);
//  Wire.setSCL(19);
//  Wire.setSDA(38);
//  Wire.setSCL(37);
  mpuF.setXGyroOffset(0);
  mpuF.setYGyroOffset(0);
  mpuF.setZGyroOffset(0);
  mpuF.setZAccelOffset(1688);
//  mpuB.setXGyroOffset(0);
//  mpuB.setYGyroOffset(0);
//  mpuB.setZGyroOffset(0);
//  mpuB.setZAccelOffset(1688);
  devStatusF = mpuF.dmpInitialize();
//  devStatusB = mpuB.dmpInitialize();
  if (devStatusF == 0 && devStatusB == 0) {
    mpuF.CalibrateAccel(6);
    mpuF.CalibrateGyro(6);
    mpuF.PrintActiveOffsets();
//    mpuB.CalibrateAccel(6);
//    mpuB.CalibrateGyro(6);
//    mpuB.PrintActiveOffsets();
    mpuF.setDMPEnabled(true);
//    mpuB.setDMPEnabled(true);
    dmpReady = true;
    packetSizeF = mpuF.dmpGetFIFOPacketSize();
//    packetSizeB = mpuB.dmpGetFIFOPacketSize();
  }else{return;}
}
//=====================================================================================================

void setup() {
  setup_motors();
  //Serial.begin(9600);
//  setup_SD();
  setup_mpu();
  buzz_setup(14);
}

void loop() {
  // int p_i = ReadPot(potpin); // Read the input potentiometer position
  p_i = 50;
  ReadAngle(mpuF, packetSizeF); // Read the gyroscope angle
  ypr[1] = atan(ypr[1])*180;
  if(debug){
	Serial.println(p_i);
	Serial.println(ypr[1]);
  }
  float F_b_out[2] = {.0,.0}; 
  float F_F_max = TheoreticalMaximumGroundFriction(mu_s, d_C1_COM, M, 0, d_C1_C2, SB1, R, r, I_A2, d_C2_COM, SB2, I_A1, d_A1_COM);
  RunNoSlipNoFlipAlgo(F_b_out, F_F_max, p_i_max, p_i, mu_s, d_C1_COM, M, ypr[1], d_C1_C2, SB1, R, r, I_A2, d_C2_COM, SB2, I_A1, d_A1_COM); 
  F_b_out[0] = map(F_b_out[0], 0, 1400, 170, 10);
  F_b_out[1] = map(F_b_out[1], 0, 800, 10, 170);
  if(debug){
	Serial.print(F_b_out[0]);
	Serial.print(" ");
	Serial.print(F_b_out[1]);
	Serial.println();
  }
  delay(10);
}
