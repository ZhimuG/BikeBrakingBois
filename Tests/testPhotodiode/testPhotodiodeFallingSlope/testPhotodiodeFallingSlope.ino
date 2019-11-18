//int analogPinPhoto = 1;
//elapsedMillis current_max_time;
//elapsedMillis previous_max_time = 0;
//double threshold_middle = 250.0;
//double threshold_upper = 270.0;
//double threshold_lower = 230.0;
//bool flag_lower = false;
//bool flag_upper = false;
//double angle_btw_max = 3.14159/3;
//double current_value = 0;
//double previous_value = 0;
//double current_rpm;
//elapsedMillis time_btw_max;

int analogPinPhoto = 31;
double dSlope = 0.1;
int numPoints = 4;
elapsedMicros Time;
bool debug = false;
//elapsedMicros current_time;
//elapsedMicros previous_time;
//elapsedMicros elapsed_time;
//double threshold2 = 0.1;


void setup() {
  // put your setup code here, to run once:

}

/* void loop() {
	// put your main code here, to run repeatedly:
	current_value = analogRead(analogPinPhoto);

 //  Serial.println(current_value);
	if(previous_value > threshold_fall && current_value < threshold_fall){
//    current_time = micros();
	current_time = (unsigned int)Time;
    elapsed_time = current_time - previous_time;
    current_rps = (angle_btw_holes/elapsed_time)*pow(10,6);
//    Serial.println(previous_time);
    previous_time = current_time;
//    Serial.println(current_time);
    Serial.println(current_rps);
	}
	previous_value = current_value;
	delayMicroseconds(100); 
  

	if(abs((previous_value - current_value) / previous_value) < dSlope){
		if(debug){
			Serial.println("I'm on the rising edge.");
		}
		count++;
		for(i=0; i<numDecreasingPoints; i++){
			delay(50);
			previous_value = current_value;
			current_value = analogRead(analogPinPhoto);
			if(current_value < previous_value){
				count++;
			}
		}
		if(count/numDecreasingPoints > 0.6){
			current_time = millis();
			elapsed_time = current_time - previous_time;
			previous_time = current_time;
			current_rpm = angle_btw_max/elapsed_time;
			Serial.println(current_rpm);
		}
	}

	if(abs((previous_value - current_value) / previous_value) > threshold1){
		current_time = micros();
		elapsed_time = (current_time - previous_time);
		current_rps = angle_btw_holes/elapsed_time;
		previous_time = current_time;
		Serial.println(current_time);
		Serial.println(previous_time);
		Serial.println(elapsed_time); 
		Serial.println(current_rps);
		while(current_value < previous_value){
			previous_value = current_value;
			current_value = analogRead(analogPinPhoto);
			delay(50);
		}
	}

	previous_value = current_value;
  

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
		Serial.println(current_rpm);
	}

	previous_value = current_value;
  
	if(current_value > previous_max){
		current_max = current_value;
	}
  
} */


//------------------------------------------------------------------------------------
//--------------------------- Rate of change -----------------------------------------
int readRPSRate(int count, double previous_value, int maxCount){
	double current_value = analogRead(analogPinPhoto);
	// To count every other point, repeat above line
	if(count == maxCount){
		if(debug){
			Serial.print(previous_time);
			Serial.print("\t");
			Serial.print(current_time);
			Serial.println();
		}
		return 1;
	}
	// falling edge: (current_value-previous_value) / previous_value < dSlope
	else if((current_value-previous_value) / previous_value > dSlope){
		if(debug){
			Serial.print(prev);
			Serial.print("\t");
			Serial.print(current_value);
			Serial.println();
		}
		count++;
		return readRPSRate(count, current_value, maxCount);
	}
	else{
		return readRPSRate(count, current_value, maxCount);
	}
}
//------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------
//-------------------------- Sign change ---------------------------------------------
double readRPSSign(int count, double previous_value, int maxCount, unsigned int previous_time){
	double current_value = analogRead(analogPinPhoto);
	// To count every other point, repeat above line
	unsigned int current_time = (unsigned int)Time;
	if(count == maxCount){
		if(debug){
			Serial.print(previous_time);
			Serial.print("\t");
			Serial.print(current_time);
			Serial.println();
		}
		return maxCount/((current_time-previous_time)*pow(10.0,-6)*16.0);
	}
	// Detects sign change from - to +
	else if(current_value > abs(previous_value)){
		if(debug){
			Serial.print(prev);
			Serial.print("\t");
			Serial.print(current_value);
			Serial.println();
		}
		if(previous_value < 0){
			count++;
		}
		return readRPSSign(count, current_value, maxCount, previous_time);
	}
	else{
		return readRPSSign(count, -current_value, maxCount, previous_time);
	}
}
//------------------------------------------------------------------------------------

void loop(){
  int count = 0;
  unsigned int previous_time = (unsigned int)Time;
	count += readRPSRate(0, analogRead(analogPinPhoto), 4);
  if(count == 4){
    unsigned int current_time = (unsigned int)Time;
    Serial.print(4.0/((current_time-previous_time)*pow(10.0,-6)*16.0));
    Serial.println();
  }
	// double RPS = readRPSSign(0, analogRead(analogPinPhoto), 4, (unsigned int)Time);
}
