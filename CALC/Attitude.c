#include "Attitude.h"

float Pitch_Sensor,Roll_Sensor,Yaw_Sensor;
float Pitch_Accel,Roll_Accel,Yaw_Accel;
float Pitch_GYRO,Roll_GYRO,Yaw_GYRO;

void CalcAttitude(unsigned char IfUseGYRO){
	CalcPitch(IfUseGYRO);
	CalcRoll(IfUseGYRO);
	CalcYaw(IfUseGYRO);
}

void CalcPitch(unsigned char IfUseGYRO){
	static unsigned char i = 0;
	static float Pitch_Acc[3] = { 0 };
	
	if(IfUseGYRO){
		Pitch_GYRO = (GYRO_X - GYROxIdle)*20/1000 + Pitch_Sensor;
		Pitch_Sensor = Pitch_GYRO * 0.98 + Pitch_Accel * 0.02;
	}
	else{
		Pitch_Acc[i] = (atan2(Acc_Y,Acc_Z)/PI)*180 - AccPitchIdle;
		if(++i > 2) i = 0;
		Pitch_Accel = (Pitch_Acc[0] + Pitch_Acc[1] + Pitch_Acc[2]) / 3;
	}
}

void CalcRoll(unsigned char IfUseGYRO){
	static unsigned char i = 0;
	static float Roll_Acc[3] = { 0 };
	
	if(IfUseGYRO){
		Roll_GYRO = (GYRO_Y - GYROyIdle)*20/1000 + Roll_Sensor;
		Roll_Sensor = Roll_GYRO * 0.85 + Roll_Accel * 0.15;
	}
	else{
		Roll_Acc[i] = (atan2(Acc_X,Acc_Z)/PI)*180 - AccRollIdle;
		if(++i > 2) i = 0;
		Roll_Accel = (Roll_Acc[0] + Roll_Acc[1] + Roll_Acc[2]) / 3;
	}
}

void CalcYaw(unsigned char IfUseGYRO){
	if(IfUseGYRO){
		Yaw_GYRO += (GYRO_Z - GYROzIdle)*20/1000;
		Yaw_Sensor = Yaw_GYRO;
		if(abs((int)Yaw_GYRO) >= 360)
			Yaw_GYRO = 0;
	}
	else{
		//NO ELSE!I DON'T WANT TO USE MAG AT ALL!
	}
}
