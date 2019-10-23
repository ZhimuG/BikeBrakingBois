#include <Wire.h>
 
//IMU adress
#define MPU 0x68
 
//Used convertion ratio 
#define A_R 16384.0
#define G_R 131.0
 
//Rad to degree convertion ratio 180/PI
#define RAD_A_DEG = 57.295779
 
//MPU-6050 gives 16 bits integrer values 
//Start values
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
 
//Start values
float Acc[2]; //16 bits values of the acceleration
float Gy[2];  //16 bits values of the gyro
float Angle[2]; //16 bits values of the angle

void setup()
{
Wire.begin();
Wire.beginTransmission(MPU);
Wire.write(0x6B);  //i2c adress
Wire.write(0);
Wire.endTransmission(true);
Serial.begin(9600);
}

void loop()
{
   //We read the acceleration values from the module
   Wire.beginTransmission(MPU);
   Wire.write(0x3B); //we ask for 0x3B register - correspond to AcX
   Wire.endTransmission(false);
   Wire.requestFrom(MPU,6,true); //From 0x3B, we ask for 6 register
   
   AcX=Wire.read()<<8|Wire.read(); //Each values ahs 2 bytes
   AcY=Wire.read()<<8|Wire.read();
   AcZ=Wire.read()<<8|Wire.read();
 
    //We apply mathematics in order to calculate the acceleration
    //This are the applied formulas 
   Acc[1] = atan(-1*(AcX/A_R)/sqrt(pow((AcY/A_R),2) + pow((AcZ/A_R),2)))*RAD_TO_DEG;
   Acc[0] = atan((AcY/A_R)/sqrt(pow((AcX/A_R),2) + pow((AcZ/A_R),2)))*RAD_TO_DEG;
 
   //We read data from the gyto register
   Wire.beginTransmission(MPU);
   Wire.write(0x43);
   Wire.endTransmission(false);
   Wire.requestFrom(MPU,4,true); //In this case we only ask for 4 register
   GyX=Wire.read()<<8|Wire.read();
   GyY=Wire.read()<<8|Wire.read();
 
   //We calculate the angle
   Gy[0] = GyX/G_R;
   Gy[1] = GyY/G_R;
 
   //We apply the complementary filter
   Angle[0] = 0.98 *(Angle[0]+Gy[0]*0.010) + 0.02*Acc[0];
   Angle[1] = 0.98 *(Angle[1]+Gy[1]*0.010) + 0.02*Acc[1];
 
   //Printe the values using serial. Open serial monitor to see the values.
   //Uncoment the values you want to serial print
   Serial.print("Y angle: "); Serial.print(Angle[1]);
   //2//Serial.print("       ");   Serial.print("X angle: "); Serial.print(Angle[0]); 
   //3//Serial.print("       ");   Serial.print("X acceleration: "); Serial.print(AcY); 
   //4//Serial.print("       ");   Serial.print("Y acceleration: "); Serial.print(AcX);
   //5//Serial.print("       ");   Serial.print("Gyro X: "); Serial.print(GyX);
   //6//Serial.print("       ");   Serial.print("Gyro Y: "); Serial.print(GyY);
   //7//Serial.print("\n"); //jump a line
   
   //delay(1); //Use a delay if the serial is printing too fast
}
