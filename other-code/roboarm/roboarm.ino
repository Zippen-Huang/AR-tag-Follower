#include <ros.h>
#include <geometry_msgs/Vector3.h>

#include <Servo.h>                //使用servo库
Servo leftArm, rightArm;    //创建2个servo对象，前臂和后臂
// 建立2个int型变量存储当前电机角度值
// 初始角度值为设备启动后初始状态所需要的电机角度数值
//int rightArmPos = 0;
//int leftArmPos = 90;

int initBaseAnge = 90;

//STEP MOTOR PIN
#define PUL 6
#define DIR 7
#define STEPEN 8

//DC MOTOR PIN
#define IN1 11
#define IN2 12
#define DCENA 13

//  dobot_inverse_kinematic_calculating-------------------------------
struct theta_dobot
{
  float theta1;
  float theta2;
  float theta3;
};
theta_dobot theta;

float pxx = 60;
float pyy = 35.62;
float pzz = 361.8;

struct theta_dobot thetaData = {0.0, 0.0, 0.0};

float max_theta1 = -180;
float min_theta1 = 180;
float max_theta2 = 90;
float min_theta2 = 45;
float max_theta3 = 90;
float min_theta3 = 45;


ros::NodeHandle nh;
//回调函数
void robo_cb(const geometry_msgs::Vector3& th)
{
  float t1 = th.x;
  float t2 = th.y;
  float t3 = th.z;

  thetaData.theta1 = copysign(min(max_theta1,max(min_theta1, abs(t1))), t1);
  thetaData.theta2 = copysign(min(max_theta2,max(min_theta2, abs(t2))), t2);
  thetaData.theta3 = copysign(min(max_theta3,max(min_theta3, abs(t3))), t3);
  
   _leftArm(thetaData.theta2);
   //delay(10);
   _rightArm(thetaData.theta2 , thetaData.theta3);
   //delay(10);
   stepMotorRun(thetaData.theta1,  40);
 
  
}
//设置订阅的消息类型和发布的主题
ros::Subscriber<geometry_msgs::Vector3> sub("/angle_msg", robo_cb);

void setup() {
  Serial.begin(9600);
  servosInit();
  stepMotorInit();
  DCMotorInit();

  nh.initNode();
  nh.subscribe(sub);
}
void loop()
{
//    Serial.print(leftArm.read());
//    Serial.print(" ");
//    Serial.println(rightArm.read());
  
//   _leftArm(thetaData.theta2);
//   delay(10);
//   _rightArm(thetaData.theta2 , thetaData.theta3);
//   delay(10);
//   stepMotorRun(thetaData.theta1,  400);
   nh.spinOnce();
   delay(1000);
}

//SERVO FUNCTION--------------------------------------------------------------------------
void servosInit() {
  rightArm.attach(10);     // rightArm 伺服舵机连接引脚10 舵机代号'r'
  delay(200);          // 稳定性等待
  leftArm.attach(9);      // leftArm 伺服舵机连接引脚9  舵机代号'l'
  delay(200);          // 稳定性等待 
  rightArm.write(60);
  leftArm.write(45);
}

void _leftArm(float theta2) {
  leftArm.attach(9);
  delay(200); 
  int _fromPos = leftArm.read();
  int theta22 = (int) (theta2*57.2);
  if (_fromPos <= theta22) { //如果“起始角度值”小于“目标角度值”
    for (int i = _fromPos; i <= theta22; i=i+3) {
      leftArm.write(i);
      delay (100);
    }
  } else {  //否则“起始角度值”大于“目标角度值”
    for (int i = _fromPos; i >= theta22; i=i-3) {
      leftArm.write(i);
      delay (100);
    }
  }
  leftArm.detach();
  delay(100); 
}

void _rightArm(float theta2 , float theta3) {
  rightArm.attach(10);
  delay(200); 
  int _fromPos = rightArm.read();
  int  theta33 = (int) ((theta3 - PI + theta2)*57.2);
  if (_fromPos <= theta33) { //如果“起始角度值”小于“目标角度值”
    for (int i = _fromPos; i <= theta33; i=i+3) {
      rightArm.write(i);
      delay (100);
    }
  } else {  //否则“起始角度值”大于“目标角度值”
    for (int i = _fromPos; i >= theta33; i=i-3) {
      rightArm.write(i);
      delay (100);
    }
  }
  rightArm.detach();
  delay(200); 
}



//STEP MOTOR FUNCTION--------------------------------------------------------------------------
void stepMotorInit() {
  pinMode(PUL, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(STEPEN, OUTPUT);
  digitalWrite(STEPEN, LOW);
}

void stepMotorCw(float _step, int _time) {
  digitalWrite(PUL, HIGH);
  digitalWrite(DIR, HIGH);
  digitalWrite(STEPEN, HIGH);
  //int _step = 5.7 * theta1 ;
  tone(PUL, _step);
  delay(_time);
  noTone(PUL);
}

void stepMotorCcw(float _step, int _time) {
  digitalWrite(PUL, HIGH);
  digitalWrite(DIR, LOW);
  digitalWrite(STEPEN, HIGH);
  //int _step = 5.7 * theta1 ;
  tone(PUL, _step);
  delay(_time);
  noTone(PUL);
}

void stepMotorRun(float theta1, int _time) {
  int _step = 5.7 * theta1*57.2;
  if (_step > initBaseAnge)
  {
    stepMotorCw(theta1, _time);
  }
  else {
    stepMotorCcw(theta1, _time);
  }
  initBaseAnge = _step;
}

void stepMotorStop() {
  noTone(PUL);
  digitalWrite(STEPEN, LOW);
}

//DC MOTOR FUNCTION--------------------------------------------------------------------------
void DCMotorInit() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(DCENA, OUTPUT);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

void DCMotorCw(int pwm, int _time) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(DCENA, pwm);
  delay(_time);
}

void DCMotorCcw(int pwm, int _time) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(DCENA, pwm);
  delay(_time);
}

void DCMotorRun(int pwm, int _time) {
  if (pwm >= 0)
  {
    DCMotorCw(pwm, _time );
  }
  else {
    DCMotorCcw(pwm, _time);
  }
}

void DCMotorStop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}
