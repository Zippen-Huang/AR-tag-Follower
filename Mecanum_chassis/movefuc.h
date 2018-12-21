
//函数接口
void motorsInit();
void spdDisplay();
void setEachMotorSpeed(double speed0, double speed1, double speed2, double speed3);
void setEachMotorPIDSpeed(double speed0, double speed1, double speed2, double speed3);
void mecanum_reverse_kinematic(double vx, double vy, double w);
double rpmTorad(double rpm);
double radTorpm(double rad);
void spdPIDInit();
void setMotorSpdFuc(int i, double spd);
double getMotorSpdFuc(int i);
