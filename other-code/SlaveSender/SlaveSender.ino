#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif


#include <ros.h>
#include <ArduinoHardware.h>
#include <geometry_msgs/Twist.h>
#include <Wire.h>

char *c;
char cstr[20];//临时用于储存的字符数组
float tf_data[3]={0,0,0};//需要通过IIC传输的数组
float vel_outbound[3]={1.0,1.0,5.0};
float vel_inbound[3]={0.1,0.1,0.5};

ros::NodeHandle nh;
//回调函数
void cmdVelCallback(const geometry_msgs::Twist& vel)
{
  tf_data[0] = vel.linear.x;
  tf_data[1] = vel.linear.y;
  tf_data[2] = vel.angular.z;
  for(int i=0;i<3;i++){
    if(tf_data[i]>vel_outbound[i]) tf_data[i]=vel_outbound[i];
    if(tf_data[i]<-vel_outbound[i]) tf_data[i]=-vel_outbound[i];
    if(-vel_inbound[i]<tf_data[i]<vel_inbound[i]) tf_data[i]=0;
  }
}
//设置订阅的消息类型和发布的主题
ros::Subscriber<geometry_msgs::Twist> sub_vel("/cmd_vel", &cmdVelCallback);

void setup() {
  Serial.begin(57600);
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onRequest(requestEvent); // register event
  nh.initNode();
  nh.loginfo("start sub vel");
  nh.subscribe(sub_vel);
  
}

void loop() {
  delay(10);
  nh.spinOnce();
}

void requestEvent() {
  char* s = arr_tf(tf_data,3);
  Wire.write(s,strlen(s)); 
}

char* arr_tf(float arr[],int n)//浮点数数组转字符数组
{
  int i;
  cstr[20]="";
  for(i=0;i<n;i++)
  {
    char intel_char[]="";
    dtostrf(arr[i],2,2,intel_char);
    if(i==0)
    {
      strcpy(cstr,intel_char);
      }
     else{
      strcat(cstr,intel_char);
     }
     strcat(cstr,"X");
  }
  return cstr;
}
