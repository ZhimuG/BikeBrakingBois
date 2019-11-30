#include <PWMServo.h>
#include "SdFat.h"
#include "BufferedPrint.h"

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // brake as normal
  int val = ReadPot(potpin);
  int PWM[2] = {val, val};
  MoveMotors(val);
  // read the angle as a result of braking
  ReadAngle(MPU6050 mpu, packetSize)
  angle = ypr[1];
  // if the angle is ever larger than 0.1 we are flipping
  if(angle > 0.1) {
    // release the brakes
    PWM[0] = 170;
    PWM[1] = 10;
    MoveMotors(PWM);
    // continue leave the brakes released until we are back on the ground
    while(angle > 0.1) {
      ReadAngle(MPU6050 mpu, packetSize)
      angle = ypr[1];
    }
  }
}
