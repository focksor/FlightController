#ifndef __PID_H
#define __PID_H

#include "data.h"
#include "MPU6050.h"
#include "motor.h"
#include "rc_ppm.h"

typedef struct{
	float exp;
	float Err;
	float PreErr;
	float Integral;
	float derivative;
	float Kp;
	float Ki;
	float Kd;
	float Pout;
	float Iout;
	float Dout;
	float output;
	float IntegralUpper;
	float IntegralLower;//IntegralUpper == IntegralLower means no limit;
	float dt; 
}PidObject;

typedef struct{
	float Kp;
	float Ki;
	float Kd;
}PidConfig;

void InitPidObject(PidObject *APidObfect,PidConfig APidConfig,float IntegralUpper,float integralLower,float dt);
void calcPid(PidObject *APidObject,float exp,float data_now);
float DataLimit(float in,float Upper,float Lower);

#endif

