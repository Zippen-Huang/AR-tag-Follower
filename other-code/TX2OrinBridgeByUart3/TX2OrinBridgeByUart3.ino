#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

const uint8_t *c;
char cstr[20];//临时用于储存的字符数组
float tf_data[3]={0,0,0};//需要通过IIC传输的数组
float max_linear_speed = 1.0;
float min_linear_speed = 0.0;
float max_angular_speed = 6.0;
float min_angular_speed = 0.0;
float dead_strict = 0.1;


ros::NodeHandle nh;
//回调函数
void robo_cb(const geometry_msgs::Twist& vel)
{
//  tf_data[0] = -vel.linear.y;
//  tf_data[1] = vel.linear.x;
//  tf_data[2] = vel.angular.z;

  float vx = vel.linear.y;
  float vy = vel.linear.x;
  float wz = vel.angular.z;
  
  tf_data[0] = copysign(min(max_linear_speed,max(min_linear_speed, abs(vx))), vx);
  tf_data[1] = copysign(min(max_linear_speed,max(min_linear_speed, abs(vy))), vy);
  tf_data[2] = copysign(min(max_angular_speed,max(min_angular_speed, abs(wz))), wz);
  
  for(int i=0;i<3;i++)
  {
    if(abs(tf_data[i]) < dead_strict)
    tf_data[i] = 0.0;
  }
  
}
//设置订阅的消息类型和发布的主题
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", robo_cb);

void setup() {
  Serial3.begin(57600);
  //Serial.begin(57600);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  nh.spinOnce();
  const unsigned char* s = arr_tf(tf_data,3);
  
  Serial3.write(s,20);
  
  //Serial3.write('0');
  delay(100);
}

const unsigned char* arr_tf(float arr[],int n)//浮点数数组转字符数组
{
  int i;
  for(i=0;i<n;i++)
  {
    char intel_char[]="";
    dtostrf(double(arr[i]),2,2,intel_char);
    if(i==0)
    {
      strcpy(cstr,intel_char);
      }
     else{
      strcat(cstr,intel_char);
     }
     strcat(cstr,"X");
  }
  return (const unsigned char*)cstr;
}
