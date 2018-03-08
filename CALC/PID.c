#include "PID.h"

void InitPidObject(PidObject *APidObject,PidConfig APidConfig,float IntegralUpper,float integralLower,float dt){
	APidObject->exp = 0;
	APidObject->Err = 0;
	APidObject->PreErr = 0;
	APidObject->Integral = 0;
	APidObject->derivative = 0;
	APidObject->Kp = APidConfig.Kp;
	APidObject->Ki = APidConfig.Ki;
	APidObject->Kd = APidConfig.Kd;
	APidObject->Pout = 0;
	APidObject->Iout = 0;
	APidObject->Dout = 0;
	APidObject->output = 0;
	APidObject->IntegralUpper = IntegralUpper;
	APidObject->IntegralLower = integralLower;
	APidObject->dt = dt;
}

void calcPid(PidObject *APidObject,float exp,float data_now){
	float Integral_Temp = APidObject->Integral;
	
	APidObject->exp = exp;
	APidObject->Err = APidObject->exp - data_now;
	APidObject->Pout = APidObject->Kp * APidObject->Err;
	
	Integral_Temp += APidObject->Err * APidObject->dt;
	APidObject->Integral = DataLimit(Integral_Temp,APidObject->IntegralUpper,APidObject->IntegralLower);
	APidObject->Iout = APidObject->Ki * APidObject->Integral;
	
	APidObject->derivative = (APidObject->Err - APidObject->PreErr) / APidObject->dt;
	APidObject->Dout = APidObject->Kd * APidObject->derivative;
	
	APidObject->output = APidObject->Pout + APidObject->Iout + APidObject->Dout;	
	
	APidObject->PreErr = APidObject->Err;
}

float DataLimit(float in,float Upper,float Lower){
	if(Upper != Lower){
		if(in >= Upper)
			return Upper;
		else if(in <= Lower)
			return Lower;
	}
	return in;
}
