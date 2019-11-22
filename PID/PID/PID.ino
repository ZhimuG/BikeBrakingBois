void setup() {
  // put your setup code here, to run once:

}

void loop() {

double findSpeed() {
    elapsedMillis elapsedTime
    double xAcceleration1 = read(accelerometer1(1));
    // integrate to find the speed of the bike in the x-direction
    xSpeedOfBike = xAcceleration * t;
}

double findRotation() {
    wheelRotationSpeed = readLinearTachometer();
}



void absAlgorithm(int* PWM) {  
    // find the current wheel rotation wheed and linear speed of the bike  
    double wheelRotationSpeed1 = findRotation();
    double xSpeedOfBike1 = findSpeed(); 
    slipRatio1 = 1 - (wheelRadius * wheelRotationSpeed / xSpeedOfBike);
    // not slipping at all
    if(slipRatio1 < 0.19)
      return PWM;
    // slipping, release the brakes by 90%
    PWM[1] = PWM[1] * 0.1;
    PWM[2] = PWM[2] * 0.1;
    moveMotors(PWM);
    elapsedMillis elapsedTime;
    while(elapsedTime < 100) {
      double wheelRotationSpeed2 = findRotation();
      double xSpeedOfBike2 = findSpeed();
    }
    // calculate the new slip ratio
    double slipRatio2 = 1 - (wheelRadius * wheelRotationSpeed2 / xSpeedOfBike);
    // find the rate of change for 10% reduction in PWM
    int changePWM = PWM2 - PWM1;
    double slipRatioChange = slipRatio2 - slipRatio1;
    double rateOfChange = changePWM / slipRatioChange;

    // continue making adjustments until stop braking or stop moving
    while(PWM[1] > 0 && PWM[2] > 0 && xSpeedOfBike > 0) {
        // measure and find new slip ratio
        elapsedMillis elapsedTime;
        while(elapsedTime < 500) {
            double wheelRotationSpeed = findRotation();
            double xSpeedOfBike = findSpeed();
        }
        slipRatio = 1 - (wheelRadius * wheelRotationSpeed / xSpeedOfBike);
        // find how far we are off from the goal now
        double error = slipRatio - 0.19
        double PWMadjustment = error * rateOfChange
        PWM = PWM - PWMadjustment;
        // apply brakes at the new force
        moveMotors(PWM);
    }      
    return PWM;
}

}
