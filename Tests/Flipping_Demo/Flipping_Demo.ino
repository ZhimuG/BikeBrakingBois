#include <PWMServo.h>
#include "SdFat.h"  
#include "BufferedPrint.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define NUM_POINTS 3000
#define BIG_NUMBER 1000000000
#define Pi 3.1415926

PWMServo myservoF;  // create servo object to control a servo
PWMServo myservoB;  // create servo object to control a servo
MPU6050 mpu;

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

// For RPS
bool debug = true;
int magic_thresh = 212;
int window_high = 218;
int analogPinPhotoB = 31;
int analogPinPhotoF = 32;

const int potpin = A3;

void MoveMotors(int PWM){
  myservoF.write(PWM);
  myservoB.write(10);
}

int ReadPot(const int potpin){
  int val=0;
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

float getLinSpeed(float wheelRadius){
  float* rps = get_rps();
  return rps[1]*wheelRadius*2*Pi;
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

void antiFlipping(int PWM){
  int prevPWM = PWM;
  float wheelRadius = 0.6985 / 2;
  float linSpeed = getLinSpeed(wheelRadius);
  float pitch = 0;
  for (int i=0; i<10; i++){
    ReadAngle(mpu, packetSizeF);
    pitch += ypr[1]*2;
  }
  pitch /= 10;
  if(abs(pitch) <= 0.1){
    return;
  }
  // if the angle is ever larger than 0.1 we are flipping
  while(PWM<165 && linSpeed!=0){
    MoveMotors(10);
    delay(10);
    while(abs(pitch) > 0.1){
      for (int i=0; i<10; i++){
        ReadAngle(mpu, packetSizeF);
        pitch += ypr[1]*2;
      }
      pitch /= 10;
      linSpeed = getLinSpeed(wheelRadius);
    }
    MoveMotors(prevPWM);
    while(abs(pitch) <= 0.1){
      for (int i=0; i<10; i++){
        ReadAngle(mpu, packetSizeF);
        pitch += ypr[1]*2;
      }
      pitch /= 10;
      linSpeed = getLinSpeed(wheelRadius);
    }
  }
  return;
}


void setup_motors(){
  myservoB.attach(3);
  myservoF.attach(2);
  myservoB.write(10);
  myservoF.write(170);
}

void buzz_setup(int pwmpin){
  pinMode(pwmpin, OUTPUT);  
  digitalWrite(pwmpin, LOW);
}

void setup_mpu(){
  Wire.begin();
  mpu.initialize();
  mpu.setXGyroOffset(100);
  mpu.setYGyroOffset(100);
  mpu.setZGyroOffset(100);
  mpu.setZAccelOffset(1688);
  devStatusF = mpu.dmpInitialize();
  if (devStatusF == 0 && devStatusB == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    mpu.setDMPEnabled(true);
    dmpReady = true;
    packetSizeF = mpu.dmpGetFIFOPacketSize();
  }else{return;}
}

void setup() {
  // put your setup code here, to run once:
  setup_motors();
  buzz_setup(14);
  setup_mpu();
  make_buzz(14, 500, 500);
}

void loop() {
  // brake as normal
  int val = ReadPot(potpin);
  MoveMotors(val);
  // read the angle as a result of braking
  antiFlipping(val);
}
