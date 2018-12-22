#include <PID_v1.h>

double  lastTime;
//Define Variables we'll be connecting to
double Setpoint_0, Input_0, Output_0;
double Setpoint_1, Input_1, Output_1;
double Setpoint_2, Input_2, Output_2;
double Setpoint_3, Input_3, Output_3;

//Specify the links and initial tuning parameters
double Kp_0=2, Ki_0=0.4, Kd_0=0;
PID spdPID_0(&Input_0, &Output_0, &Setpoint_0, Kp_0, Ki_0, Kd_0, DIRECT);

double Kp_1=1.5, Ki_1=0.5, Kd_1=0;
PID spdPID_1(&Input_1, &Output_1, &Setpoint_1, Kp_1, Ki_1, Kd_1, DIRECT);

double Kp_2=2.4, Ki_2=1.3, Kd_2=0;
PID spdPID_2(&Input_2, &Output_2, &Setpoint_2, Kp_2, Ki_2, Kd_2, DIRECT);

double Kp_3=1.8, Ki_3=0.6, Kd_3=0;
PID spdPID_3(&Input_3, &Output_3, &Setpoint_3, Kp_3, Ki_3, Kd_3, DIRECT);

double fourMotorRpmArray[4];

MeEncoderNew M_LeftFront(0x09, SLOT1); //前左  1
MeEncoderNew M_RightFront(0x09, SLOT2); //前右  0 
MeEncoderNew M_LeftBack(0x0a, SLOT1); //后左   2
MeEncoderNew M_RightBack(0x0a, SLOT2); //后右   3

void motorsInit(){
  M_LeftFront.begin();
  M_RightFront.begin();
  M_LeftBack.begin();
  M_RightBack.begin();
  
  M_LeftFront.reset();
  M_RightFront.reset();
  M_LeftBack.reset();
  M_RightBack.reset();

//  M_LeftFront.setPulse(10);
}

void spdPIDInit(){
  spdPID_0.SetMode(AUTOMATIC);
  spdPID_1.SetMode(AUTOMATIC);
  spdPID_2.SetMode(AUTOMATIC);
  spdPID_3.SetMode(AUTOMATIC);

  spdPID_0.SetOutputLimits(-254, 254);
  spdPID_1.SetOutputLimits(-254, 254);
  spdPID_2.SetOutputLimits(-254, 254);
  spdPID_3.SetOutputLimits(-254, 254);
  
  spdPID_0.SetSampleTime(50);//pid采样时间
  spdPID_1.SetSampleTime(50);//pid采样时间
  spdPID_2.SetSampleTime(50);//pid采样时间
  spdPID_3.SetSampleTime(50);//pid采样时间
  
  spdPID_0.SetTunings(Kp_0, Ki_0, Kd_0);
  spdPID_1.SetTunings(Kp_1, Ki_1, Kd_1);
  spdPID_2.SetTunings(Kp_2, Ki_2, Kd_2);
  spdPID_3.SetTunings(Kp_3, Ki_3, Kd_3);
  
  lastTime = millis();
}

void spdDisplay(){
  for(int i = 0;i<4;i++){
      Serial.print(getMotorSpdFuc(i));
      Serial.print(" ");
  }
  Serial.println();

//  Serial.println(M_RightFront.getCurrentSpeed());
}

void posDisplay(){
    Serial.print(M_LeftFront.getCurrentPosition());
    Serial.print("  ");
    Serial.print(M_RightFront.getCurrentPosition());
    Serial.print("  ");
    Serial.print(M_LeftBack.getCurrentPosition());
    Serial.print("  ");
    Serial.println(M_RightBack.getCurrentPosition());
}

void setEachMotorSpeed(double speed0, double speed1, double speed2, double speed3)
{
    M_RightFront.runSpeed(-speed0);   //0
    M_LeftFront.runSpeed(speed1);      //1  
    M_LeftBack.runSpeed(speed2);      //2
    M_RightBack.runSpeed(-speed3);   //3
}

void setEachMotorPIDSpeed(double speed0, double speed1, double speed2, double speed3)
{
      double spdSetpointArr[4] = {speed0,speed1,speed2,speed3};
      
     if((millis()-lastTime)>50){
      Input_0 = getMotorSpdFuc(0);
      Setpoint_0 = spdSetpointArr[0];
      spdPID_0.Compute();
      setMotorSpdFuc(0, Output_0);
      
      Input_1 = getMotorSpdFuc(1);
      Setpoint_1 = spdSetpointArr[1];
      spdPID_1.Compute();
      setMotorSpdFuc(1, Output_1);

      Input_2 = getMotorSpdFuc(2);
      Setpoint_2 = spdSetpointArr[2];
      spdPID_2.Compute();
      setMotorSpdFuc(2, Output_2);

      Input_3 = getMotorSpdFuc(3);
      Setpoint_3 = spdSetpointArr[3];
      spdPID_3.Compute();
      setMotorSpdFuc(3, Output_3);
      
//      for(int i = 0;i<4;i++){
//          Input = getMotorSpdFuc(i);
//          Setpoint = spdSetpointArr[i];
//          spdPID.Compute();
//          setMotorSpdFuc(i,Output);
//      }
      
      lastTime = millis();
     }
}

void mecanum_reverse_kinematic(double vx, double vy, double w){
  double a = half_width;
  double b = half_length;
  double vArray[4];
  

  //将几何中心速度转化成四个轮子的线速度
  vArray[0] = -vx + vy + w*(a+b);
  vArray[1] =  vx + vy - w*(a+b);
  vArray[2] = -vx + vy - w*(a+b);
  vArray[3] =  vx + vy + w*(a+b);

  //将四个轮子的线速度转化成转速
  for(int i = 0;i<4;i++){
    fourMotorRpmArray[i] = radTorpm(vArray[i]/wheelRadius);
  }

  setEachMotorSpeed(fourMotorRpmArray[0],fourMotorRpmArray[1],fourMotorRpmArray[2],fourMotorRpmArray[3]);
  //setEachMotorPIDSpeed(fourMotorRpmArray[0],fourMotorRpmArray[1],fourMotorRpmArray[2],fourMotorRpmArray[3]);
}

double rpmTorad(double rpm){  //转速 转 角速度
  double rad = rpm*2*pi/60;
  return rad;
}

double radTorpm(double rad){  //角速度 转 转速
  double rpm = rad*30/pi;
  return rpm;
}

void setMotorSpdFuc(int i, double spd){
  switch(i)
  {
  case 0 :
        M_RightFront.runSpeed(-spd);
        break;
  case 1 :
        M_LeftFront.runSpeed(spd);
        break;
  case 2 :
        M_LeftBack.runSpeed(spd);
        break;
  case 3 :
        M_RightBack.runSpeed(-spd);
        break;
  default :
        //Serial.print("error");
        break;
    }
}

double getMotorSpdFuc(int i){
  switch(i)
  {
  case 0 :
        return -M_RightFront.getCurrentSpeed();   //0
        break;
  case 1 :
        return M_LeftFront.getCurrentSpeed();      //1
        break;
  case 2 :
        return M_LeftBack.getCurrentSpeed();       //2
        break;
  case 3 :
        return -M_RightBack.getCurrentSpeed();     //3
        break;
  default :
       // Serial.print("error");
        break;
    }
}
