//https://blog.csdn.net/zhaoyuaiweide/article/details/75560257?locationNum=6&fps=1
//https://zhuanlan.zhihu.com/p/20282234?utm_source=qq&utm_medium=social

#include "MeOrion.h"
//#include <Arduino.h>
#include "SoftwareSerial.h"
#include <Wire.h>
#include "gyro.h"
#include "movefuc.h"
#include "IICMasterReader.h"

float pi = 3.14;

//定义麦克纳母轮底盘参数m
double half_width  = 0.048;  
double half_length = 0.14;
double wheelRadius = 0.152;

char iicData[20];//IIC传送的字符数组
float recData[3]; 

long timemark=0;

void setup()
{
  Serial.begin(9600);
  motorsInit();
  spdPIDInit();
  gyroInit();
  IIC_VelDataReader_Init();
  timemark=millis();
}

void loop()
{
    //delay(2);
    if ((millis() - timemark)>=200)
    {
       char return_data = read_Data(8, iicData, 20);
       if(return_data == 0){
        char2floatFunc(iicData,20);
        }
       else{
        setEachMotorSpeed(0,0,0,0);
        }
      }
   
    mecanum_reverse_kinematic(recData[0], recData[1], recData[2]);

//    setEachMotorSpeed(50,50,50,50);
//    setMotorSpdFuc(0, 30);   
//    setEachMotorPIDSpeed(fourMotorRpmArray[0], fourMotorRpmArray[0], fourMotorRpmArray[0], fourMotorRpmArray[0]);
//    Serial.println(readAngleOrGyroZ(1));
//      mecanum_reverse_kinematic(0, 0, 0);
//    setEachMotorPIDSpeed(50,-50,-50,-50);
    
//    delay(2);
//    spdDisplay();
//      setMotorSpdFuc(1, 50);
}



