#ifndef __DATA_H
#define __DATA_H

#include "PID.h"

extern unsigned short RM_CH_Length[];

extern float Acc_X,Acc_Y,Acc_Z;
extern float GYRO_X,GYRO_Y,GYRO_Z;
extern float MAG_X,MAG_Y,MAG_Z;

extern float Pitch_Sensor,Roll_Sensor,Yaw_Sensor;
extern float Pitch_Accel,Roll_Accel,Yae_Accel;
extern float Pitch_PID,Roll_PID,Yaw_PID;



extern float MOTOR1_Def,MOTOR2_Def,MOTOR3_Def,MOTOR4_Def;

extern float gx_idle, gy_idle, gz_idle;

#endif
