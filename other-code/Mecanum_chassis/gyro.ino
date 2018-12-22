#include "MeOrion.h"
#include <Wire.h>

MeGyro gyro;

void gyroInit(){
  gyro.begin();
}

double readAngleOrGyroZ(int i){
  gyro.update();
  Serial.read();
  switch(i)
  {
    case 0 :
      return gyro.getAngleZ();
      break;
    case 1 :
//      return gyro.getGyroZ();
      break;
    default:
      break; 
  }  
}


