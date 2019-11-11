#include <Wire.h>
short measure();
byte lidarliteAddress=0x62;
byte distance_data[2];
int busy_flag;

#include "SdFat.h"
SdFatSdio sd;
File file;
#define MaxCount 100000

void setup()
{
  Wire.begin();
  Serial.begin(115200);

  if (!sd.begin()) {
    sd.initErrorHalt();
  }
}

short measure ()
{
  Wire.beginTransmission(lidarliteAddress);
  Wire.write(0x00); 
  Wire.write(0x04); 
  Wire.endTransmission();
  
  Wire.beginTransmission(lidarliteAddress);
  Wire.write(0x01);
  Wire.endTransmission();
  do
  {
    Wire.requestFrom(lidarliteAddress,1);
    busy_flag= bitRead(Wire.read(),0);
  } while(busy_flag!=0);
  
  if(busy_flag==0)
  {
    Wire.beginTransmission(lidarliteAddress);
    Wire.write(0x8f);
    Wire.endTransmission();
    Wire.requestFrom(lidarliteAddress,2);
    for(int count=0; count<2; count++)
    {
      while(Wire.available());
      distance_data[count]=Wire.read();
    }
    short distance=((distance_data[0]<<8)+distance_data[1]); // why conversion from short to float?
    return distance;
  }
}

void doLogging(uint32_t timeStamp, short *data, int nb)
{
  // following is for logging
  static uint32_t ifl = 0;
  static uint32_t count = 0;
  char filename[32];

  if(!count)
  {
     //open file
     sprintf(filename,"Ranging_%04d.bin",ifl); ifl++;
     if (!file.open(filename, O_RDWR | O_CREAT)) {
       sd.errorHalt("open failed");
     count=1;
     }
  }
  //
  if(count>0)
  {
    // write to file
    file.print(timeStamp);
    for(int ii=0; ii<nb; ii++)
    { file.print(',');
      file.print(data[ii]) ;
    }
    file.println();

    count++;
    if(count>MaxCount) count=0;
  }
  //
  if(!count)
  {
     // close file
     file.close();
  }
}


void loop()
{
  uint32_t t0 = millis();
  short distance=measure();
  Serial.println(distance);

  doLogging(t0,&distance,1);
}
