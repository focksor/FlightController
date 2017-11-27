#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "config.h"

#define Motor_PWM_Period 2499		//PWM定时器溢出值
#define Motor_PWM_Idle   999		//PWM底值


void MOTOR_Init(void);
int MOTOR_Write(int val_1, int val_2, int val_3, int val_4);

#endif
