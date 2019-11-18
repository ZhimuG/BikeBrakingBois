#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)

MPU6050 mpuF;
MPU6050 mpuB;

bool blinkState = false;

// MPU control/status variables
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatusF;      // return status after each device operation (0 = success, !0 = error)
uint8_t devStatusB;
uint16_t packetSizeF;    // expected DMP packet size (default is 42 bytes)
uint16_t packetSizeB;
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
elapsedMicros Time;

// orientation/motion variables
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 linAccel;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
VectorFloat linSpeed = VectorFloat();
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
 
// Read linear acceleration of the bike
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
    
    // recalculate the accelerations in m/s^2
    linAccel.x = linAccel.x/2048.0*9.80665;
    linAccel.y = linAccel.y/2048.0*9.80665;
    linAccel.z = linAccel.z/2048.0*9.80665;
	}
}

// Read linear speed of the bike
void ReadLinSpeed(MPU6050 mpu, uint16_t packetSize){
	unsigned int previousTime = (unsigned int)Time;
	ReadLinAccel(mpu, packetSize);
	unsigned int currentTime = (unsigned int)Time;
	linSpeed.x += linAccel.x * 1.0 * (currentTime - previousTime) * pow(10,-6);
  linSpeed.y += linAccel.y * 1.0 * (currentTime - previousTime) * pow(10,-6);
  linSpeed.z += linAccel.z * 1.0 * (currentTime - previousTime) * pow(10,-6);
}

// Read angles of the bike
void ReadAngles(MPU6050 mpu, uint16_t packetSize){
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

void loop() {
	ReadLinAccel(mpuB, packetSizeB);
}
