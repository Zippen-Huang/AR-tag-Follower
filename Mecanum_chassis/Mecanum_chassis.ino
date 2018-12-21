//https://blog.csdn.net/zhaoyuaiweide/article/details/75560257?locationNum=6&fps=1
//https://zhuanlan.zhihu.com/p/20282234?utm_source=qq&utm_medium=social

#include "MeOrion.h"
//#include <Arduino.h>
//#include "SoftwareSerial.h"
#include <Wire.h>
#include "gyro.h"
#include "movefuc.h"

#include <SoftwareSerial.h>

char table[20] = {0};
SoftwareSerial softuart(13,12);

char iicData[20];//IIC传送的字符数组
float recData[3];

float pi = 3.14;

//定义麦克纳母轮底盘参数m
double half_width  = 0.415;  
double half_length = 0.485;
double wheelRadius = 0.152;

long timemark=0;

void char2floatFunc(char* iicdata,char iic_size);

void setup()
{
  Serial.begin(57600);
  softuart.begin(57600); 
  motorsInit();
  spdPIDInit();
}

void loop()
{
  int readdata = 0,i = 0,count = 0;
    if (softuart.available())
    {
//        Serial.print("   readdata:");
        while((readdata = softuart.read()) != (int)-1)
        {
            table[count] = readdata;
            count++;
            delay(1);
        }
        for(i = 0;i<20;i++)
        {
            iicData[i] = table[i];
        }
        char2floatFunc(iicData,20);
//        for(int j=0;j<3;j++){
//          Serial.print(recData[j]);
//          Serial.print(" ");
//      }
//        Serial.println("   stop rev");
    }

    mecanum_reverse_kinematic(recData[0], recData[1], recData[2]);
   
//    mecanum_reverse_kinematic(0, 0, 0);

//    setEachMotorSpeed(50,50,50,50);
//    setMotorSpdFuc(0, 30);   
//    setEachMotorPIDSpeed(fourMotorRpmArray[0], fourMotorRpmArray[0], fourMotorRpmArray[0], fourMotorRpmArray[0]);
//    Serial.println(readAngleOrGyroZ(1));
//      mecanum_reverse_kinematic(0, 0, 0);
//    setEachMotorPIDSpeed(50,-50,-50,-50);
}

//字符数组转浮点数组
void char2floatFunc(char* iicdata,char iic_size){
  char intervalArr[]="";
  int n=0;
  int cn=0;
  for(int i=0;i<iic_size;i++){
    if(n>3)
    {
      n=0;
      break;
      }
     
    if(iicData[i] == 'X'){
      recData[n]=atof(intervalArr);
      //Serial.print(recData[n]);
      n++;
      cn=i;
      //Serial.println(cn); 
    }
    else
    {
      if(n==0)
      {
        intervalArr[i]=iicData[i];
        }
      else
      {
        intervalArr[i-cn-1]=iicData[i];
        //Serial.print(intervalArr[i-cn]);
        }
      }
  }
//  Serial.println(recData[0]); 
}



