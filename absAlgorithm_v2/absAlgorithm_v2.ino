void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:


  // version 2 of the abs algorithm

  // release the brakes by a lot and the slip ratio will go below 0.2
  // reapply the brakes when the slip ratio rises back up to 0.2
  // repeat until the bike stops

}

int* absAlgorithm(int* PWM) {
    // save the original PWM values
    double PWM1 = PWM[0];
    double PWM2 = PWM[1];
    // find the current wheel rotation wheed and linear speed of the bike  
    double wheelRotationSpeed = findRotation();
    double xSpeedOfBike = findSpeed(); 
    double slipRatio = 1 - (wheelRadius * wheelRotationSpeed / xSpeedOfBike);
    // not slipping at all
    if(slipRatio < 0.19)
      return PWM;
    // slipping, run the algorithm until stop moving or stop braking
    while((PWM[0] > 0 || PWM[1] > 0) && xSpeedOfBike > 0) {
        // set braking force to zero until slipRatio < 0.19
        PWM[0] = 0;
        PWM[1] = 0;
        moveMotors(PWM);
        elapsedMillis elapsedTime;
        while(slipRatio > 0.19) {
            if(elapsedTime > 100) {
                double wheelRotationSpeed = findRotation();
                double xSpeedOfBike = findSpeed();
                slipRatio = 1 - (wheelRadius * wheelRotationSpeed / xSpeedOfBike);
            }
        }
        // start braking at original force again until slipRatio > 0.19
        PWM[0] = PWM1;
        PWM[1] = PWM2;
        moveMotors(PWM);
        elapsedMillis elapsedTime;
        while(slipRatio < 0.19) {
            if(elapsedTime > 100) {
                double wheelRotationSpeed = findRotation();
                double xSpeedOfBike = findSpeed();
                slipRatio = 1 - (wheelRadius * wheelRotationSpeed / xSpeedOfBike);
            }
        }   
      }  
    return PWM;
}

