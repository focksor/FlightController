#ifndef __ATTITUDEPID_H
#define __ATTITUDEPID_H

#include "PID.h"

#define dTime 0.02

extern PidObject pidPitchAngle;
extern PidObject pidRollAngle;
extern PidObject pidYawAngle;
extern PidObject pidPitchAngularRate;
extern PidObject pidRollAngularRate;
extern PidObject pidYawAngularRate;

void InitSelfStability(void);
void SelfStability(float Pitch_exp,float Roll_exp,float Yaw_exp);
void calcPidSelfStability(float Pitch_exp,float Roll_exp,float Yaw_exp,
						  float Pitch_now,float Roll_now,float Yaw_now,
						  float GYROx_now,float GYROy_now,float GYROz_now,
						  float *Pitch_PID,float *Roll_PID,float *Yaw_PID);
#endif
