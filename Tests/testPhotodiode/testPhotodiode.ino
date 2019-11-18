unsigned long int test1;
//elapsedMillis test1;

float previousHoleTime = 0;

float previous = 1000000;

float threshold = 2.0;

int analogPinPhoto = 1;

float angleBtwConsecHoles = 3.14159/3;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  //test1 = millis();
  //float voltage = analogRead(analogPinPhoto);
  //Serial.println(voltage);
  
  float current = analogRead(analogPinPhoto);
  if((current-previous)>threshold){
    float currentTime = millis();
    float timeBtwConsecHoles = currentTime - previousHoleTime;
    previousHoleTime = currentTime;
    float rps = angleBtwConsecHoles/timeBtwConsecHoles;
    //Serial.println(rps);
  }
  previous = analogRead(analogPinPhoto);

  //Serial.println(current, previous);
  //Serial.println(current-previous);
  Serial.println(current);
}

//double readLineartachometer() {
//    int second1 = second();             // get starting time
//    int second2 = second();             // get temporary ending time
//    double difference = second2 - second1;
//    int count = 0;    // count for number of holes
//    while(difference < 1) {
//        voltage = analogRead(1);   // take analog reading of the photodiode at pin 1
//        if(voltage > 1)
//          count += 1
//        second2 = second();
//        difference = second2 - second1;
//    }
//    double radPerHole = 1;      // need to measure this value
//    double rotationSpeed = radPerHole * count;
//    return rotationSpeed;
//}
