#ifndef __TIMER_H
#define __TIMER_H

#include "stm32f10x.h"
#include "MOTOR.h"
#include "RM.h"
#include "AttitudePID.h"
#include "Attitude.h"

void Timer_Init(void);
void TIM2_IRQHandler(void);//T=2000us,TIM2 MAINLY USED FOR CONTROLLING THE UAV'S ATTITUDE
void TIM3_IRQHandler(void);//T=2000us
//TIM4 HAS BEEN USED FOR MOTOE,SEE IN ../MOTOE.C
void TIM5_IRQHandler(void);

#endif
