#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "config.h"
//#include "RM.h"

#define Motor_PWM_Period 2499		//PWM定时器溢出值
#define Motor_PWM_Idle   999-170		//PWM底值

extern bool WithoutDream;
extern bool IfUnlock;

void MOTOR_Init(void);
int MOTOR_Set(unsigned short MOTOR1_Def,unsigned short MOTOR2_Def,unsigned short MOTOR3_Def,unsigned short MOTOR4_Def);

#endif
