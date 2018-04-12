#ifndef __ATTITUDE_H
#define __ATTITUDE_H

#define PI 3.1415926535

#include "math.h"
#include "mpu9250.h"
#include "flight.h"

extern float PitchIdle,RollIdle;
extern float PitchIdle_All,RollIdle_All;
extern long PitchIdleCntr,RollIdleCntr;


void CalcAttitude(unsigned char IfUseGYRO);
void CalcPitch(unsigned char IfUseGYRO);
void CalcRoll(unsigned char IfUseGYRO);
void CalcYaw(unsigned char IfUseGYRO);

#endif
