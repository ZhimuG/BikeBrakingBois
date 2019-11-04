/*
   MPU6050_raw.ino : example of reading raw IMU data from MPU6050 using Teensy 3.X or Teensy LC

   This file is part of MPU6050.

   MPU6050 is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "MPU6050.h"

MPU6050 imu1;
MPU6050 imu2;

void setup()
{
    Serial.begin(115200);

    Wire.begin();
 
    if (!imu1.begin(AFS_2G, GFS_250DPS)) {
        Serial.println("MPU6050 1 is online...");
    }
    else {
        Serial.println("Failed to init MPU6050 1");
        while (true) 
            ;
    }

    if (!imu2.begin(AFS_2G, GFS_250DPS)) {
        Serial.println("MPU6050 2 is online...");
    }
    else {
        Serial.println("Failed to init MPU6050 2");
        while (true) 
            ;
    }
}

void loop()
{  
    int16_t ax1, ay1, az1, gx1, gy1, gz1;
    int16_t ax2, ay2, az2, gx2, gy2, gz2;

    if (imu1.getMotion6Counts(&ax1, &ay1, &az1, &gx1, &gy1, &gz1)) {
        Serial.print("Device 1:\n");
        Serial.print(ax1);
        Serial.print(" ");
        Serial.print(ay1);
        Serial.print(" ");
        Serial.print(az1);
        Serial.print(" ");
        Serial.print(gx1);
        Serial.print(" ");
        Serial.print(gy1);
        Serial.print(" ");
        Serial.print(gz1);
        Serial.println();
    }
    
    if (imu2.getMotion6Counts(&ax2, &ay2, &az2, &gx2, &gy2, &gz2)) {
        Serial.print("Device 2:\n");
        Serial.print(ax2);
        Serial.print(" ");
        Serial.print(ay2);
        Serial.print(" ");
        Serial.print(az2);
        Serial.print(" ");
        Serial.print(gx2);
        Serial.print(" ");
        Serial.print(gy2);
        Serial.print(" ");
        Serial.print(gz2);
        Serial.println();
    }
}
