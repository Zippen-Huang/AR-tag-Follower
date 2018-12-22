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


struct theta_dobot dobot_inverse_kinematic_calculating(float px, float py, float pz)
{
  struct theta_dobot t;
  int L1, L2, L3, L4;
  L1 = 138; L2 = 28; L3 = 176; L4 = 104;

  float theta_1;
  float k1, k2, c3, s3, theta_3;
  float c3_1, c3_2, c3_3;
  float k2_1, k2_2, s2, k2_3, c2, theta_2;

  theta_1 = atan(py / px);

  k1 = px * cos(theta_1) + py * sin(theta_1) - L2;
  k2 = pz - L1;
  //c3=(k1*k1+k2*k2-L3*L3-L4*L4)/(2*L3*L4);
  //in this equation , 2*L3*L4 out the scope,and the variant is out
  c3_1 = k1 * k1 + k2 * k2 - L3 * L3 - L4 * L4;
  c3_2 = L3 * L4;
  c3_3 = c3_1 / 2;
  c3 = c3_3 / c3_2;
  s3 = sqrt(1 - c3 * c3);

  theta_3 = atan(s3 / c3);

  k2_1 = (L3 + L4 * cos(theta_3)) * (pz - L1) - L4 * sin(theta_3) * (cos(theta_1) * px + sin(theta_1) * py - L2);
  k2_2 = (pz - L1) * (pz - L1) + (cos(theta_1) * px + sin(theta_1) * py - L2) * (cos(theta_1) * px + sin(theta_1) * py - L2);
  s2 = k2_1 / k2_2;
  k2_3 = L4 * sin(theta_3) * (pz - L1) + (L3 + L4 * cos(theta_3)) * (cos(theta_1) * px + sin(theta_1) * py - L2);
  c2 = k2_3 / k2_2;

  theta_2 = atan(s2 / c2);

  t.theta1 = theta_1;
  t.theta2 = theta_2;
  t.theta3 = theta_3;
//  Serial.print("*---**");
  return t;
}

struct theta_dobot testData = {0.0, 0.0, 0.0};

void setup() {
  Serial.begin(9600);
  servosInit();
  stepMotorInit();
  DCMotorInit();
}
void loop()
{
  //    stepMotorCw(float theta1,int _step, 2000);
  //    delay(100);
  //    stepMotorCcw(float theta1,int _step, 2000);
  //    delay(100);
  //    _leftArm(90,45);
  //    delay(100);
  //    _rightArm(45,30);
  //    delay(100);
  //    zhuashou(100);
  //    delay(100);
  //    zhuashouf(100);
  //    delay(100);

      testData = dobot_inverse_kinematic_calculating(pxx, pyy, pzz);
      delay(1000);
      _rightArm(testData.theta2 ,testData.theta3);
      delay(1000);
      _leftArm(testData.theta2);
      delay(1000);
      //stepMotorRun(testData.theta1, 300);

//      Serial.print(testData.theta1*57.2);
//      Serial.print(" ");
//      Serial.print(testData.theta2*57.2);
//      Serial.print("*******");
//      Serial.println(testData.theta3*57.2);

      while(1);

}

//SERVO FUNCTION--------------------------------------------------------------------------
void servosInit() {
  rightArm.attach(10);     // rightArm 伺服舵机连接引脚10 舵机代号'r'
  delay(200);          // 稳定性等待
  leftArm.attach(9);      // leftArm 伺服舵机连接引脚9  舵机代号'l'
  delay(200);          // 稳定性等待
}

void _leftArm(float theta2) {
  leftArm.attach(9);
  delay(200); 
  int _fromPos = leftArm.read();
  int theta22 = (int) (theta2*57.2);
  if (_fromPos <= theta22) { //如果“起始角度值”小于“目标角度值”
    for (int i = _fromPos; i <= theta22; i++) {
      leftArm.write(i);
      delay (300);
    }
  } else {  //否则“起始角度值”大于“目标角度值”
    for (int i = _fromPos; i >= theta22; i--) {
      leftArm.write(i);
      delay (300);
    }
  }
  leftArm.detach();
  delay(200); 
}

void _rightArm(float theta2 , float theta3) {
  rightArm.attach(10);
  delay(200); 
  int _fromPos = rightArm.read();
  int  theta33 = (int) ((theta3 - PI + theta2)*57.2);
  if (_fromPos <= theta33) { //如果“起始角度值”小于“目标角度值”
    for (int i = _fromPos; i <= theta33; i++) {
      rightArm.write(i);
      delay (500);
    }
  } else {  //否则“起始角度值”大于“目标角度值”
    for (int i = _fromPos; i >= theta33; i--) {
      rightArm.write(i);
      delay (500);
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
