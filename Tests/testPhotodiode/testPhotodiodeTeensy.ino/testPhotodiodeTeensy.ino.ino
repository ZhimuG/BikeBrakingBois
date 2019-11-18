int analogPinPhoto = 1;
elapsedMillis current_max_time;
elapsedMillis previous_max_time = 0;
double threshold_middle = 250.0;
double threshold_upper = 270.0;
double threshold_lower = 230.0;
bool flag_lower = false;
bool flag_upper = false;
double angle_btw_max = 3.14159/3;
double current_value = 0;
double previous_value = 0;
double current_rpm;
elapsedMillis time_btw_max;

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
  current_value = analogRead(analogPinPhoto);
  Serial.println(current_value);

  if(flag_upper == true && previous_value > threshold_lower && current_value < threshold_lower){
    flag_upper = false;
    flag_lower = true;
  }
  if(flag_lower == true && previous_value < threshold_upper && current_value > threshold_upper){
    flag_lower = false;
    flag_upper = true;
    current_max_time = millis();
    time_btw_max = current_max_time - previous_max_time;
    previous_max_time = current_max_time;
    current_rpm = angle_btw_max/time_btw_max;
//    Serial.println(current_rpm);
  }

  previous_value = current_value;
  
  //if(current_value > previous_max){
  //  current_max = current_value;  
  //}
  
  delayMicroseconds(200);
}
