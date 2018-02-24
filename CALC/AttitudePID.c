#include "AttitudePid.h"

float Pitch_PID,Roll_PID,Yaw_PID;
float MOTOR1_Def,MOTOR2_Def,MOTOR3_Def,MOTOR4_Def;

PidObject pidPitchAngle;
PidObject pidRollAngle;
PidObject pidYawAngle;
PidObject pidPitchAngularRate;
PidObject pidRollAngularRate;
PidObject pidYawAngularRate;

//PidConfig pidPitchAngleConfig = 		{0,0.0,0};
//PidConfig pidRollAngleConfig = 			{-0,-0.0,-0.00};
//PidConfig pidYawAngleConfig = 			{0,0.0,0};
//PidConfig pidPitchAngularRateConfig =	{0.0,0.0,0.00};
//PidConfig pidRollAngularRateConfig = 	{0.0,0.0,0.00};
//PidConfig pidYawAngularRateConfig = 	{0,0,0.0};

PidConfig pidPitchAngleConfig = 		{15,0.2,0};
PidConfig pidRollAngleConfig = 			{6,0.1,0};
PidConfig pidYawAngleConfig = 			{0,0,0};
PidConfig pidPitchAngularRateConfig =	{0.135,0.09,0.0036};
PidConfig pidRollAngularRateConfig = 	{0.135,0.09,0.0036};
PidConfig pidYawAngularRateConfig = 	{0.18,0.018,0};


//PID OF PX4
//PidConfig pidPitchAngleConfig = 		{4.5,0,0};
//PidConfig pidRollAngleConfig = 			{4.5,0,0};
//PidConfig pidYawAngleConfig = 			{0,0,0};
//PidConfig pidPitchAngularRateConfig =	{0.135,0.09,0.0036};
//PidConfig pidRollAngularRateConfig = 	{0.135,0.09,0.0036};
//PidConfig pidYawAngularRateConfig = 	{0.18,0.018,0};



void InitSelfStability(void){
	InitPidObject(&pidPitchAngle,pidPitchAngleConfig,33,-33,dTime);
	InitPidObject(&pidRollAngle,pidRollAngleConfig,15,-15,dTime);
	InitPidObject(&pidYawAngle,pidYawAngleConfig,360,0,dTime);
	InitPidObject(&pidPitchAngularRate,pidPitchAngularRateConfig,35,-35,dTime);
	InitPidObject(&pidRollAngularRate,pidRollAngularRateConfig,10,-10,dTime);
	InitPidObject(&pidYawAngularRate,pidYawAngularRateConfig,360,-360,dTime);
}
void SelfStability(float Pitch_exp,float Roll_exp,float Yaw_exp){	
	calcPidSelfStability(Pitch_exp,Roll_exp,Yaw_exp,
						 Pitch_Sensor,Roll_Sensor,Yaw_Sensor,
						 GYRO_X-GYROxIdle,-(GYRO_Y-GYROyIdle),GYRO_Z-GYROzIdle,
						 &Pitch_PID,&Roll_PID,&Yaw_PID);
}

//void calcPid(PidObject *APidObject,PidConfig *APidConfig,float exp,float data_now);

void calcPidSelfStability(float Pitch_exp,float Roll_exp,float Yaw_exp,
						  float Pitch_now,float Roll_now,float Yaw_now,
						  float GYROx_now,float GYROy_now,float GYROz_now,
						  float *Pitch_Pid,float *Roll_Pid,float *Yaw_Pid) {	
	calcPid(&pidPitchAngle,Pitch_exp,Pitch_now);
	calcPid(&pidRollAngle,Roll_exp,Roll_now);
	calcPid(&pidYawAngle,Yaw_exp,Yaw_now);

					  
	calcPid(&pidPitchAngularRate,pidPitchAngle.output,GYROx_now);
	calcPid(&pidRollAngularRate,pidRollAngle.output,GYROy_now);
	calcPid(&pidYawAngularRate,pidYawAngle.output,GYROz_now);
							  
	*Pitch_Pid = pidPitchAngularRate.output;
	*Roll_Pid = pidRollAngularRate.output;
	*Yaw_Pid = pidYawAngularRate.output;
}
