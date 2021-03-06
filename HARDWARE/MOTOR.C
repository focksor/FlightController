#include "motor.h"

bool WithoutDream = false;
bool IfUnlock = false;

void MOTOR_Init(){
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_BaseInitStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);//使能GPIOB时钟
	GPIO_InitStructure.GPIO_Pin = __MOTOR_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//AF_定时器引脚复用
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	// 定时器基础初始化
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//使能APB1外设时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);//使能复用时钟
	
	TIM_BaseInitStructure.TIM_Prescaler = 72 - 1;//设置TIM时钟分频值
	TIM_BaseInitStructure.TIM_Period = Motor_PWM_Period;//设置了在下一个更新事件装入活动的自动重装载寄存器周期的,周期为2500us
	TIM_BaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;//TIM向上计数模式
	TIM_BaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;//设置时钟分割_TDTS = Tck_tim 
	
	TIM_TimeBaseInit(TIM4, &TIM_BaseInitStructure);//根据 TIM_TimeBaseInitStruct 中指定的参数初始化TIMx的时间基数单位 
	
	// PWM初始化
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//设置定时器模式_TIM 脉冲宽度调制模式 1 (CCR>TIM_Pulse时为有效电平，模式2相反)
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//开启OCx输出到对应引脚
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//设置输出极性_TIM输出比较极性高
	TIM_OCInitStructure.TIM_Pulse = Motor_PWM_Idle;//设置了待装入捕获比较寄存器的脉冲值
	
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);//初始化TIM通道
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);//使能TIM在CCRx上的预装载寄存器
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);	
	
	TIM_Cmd(TIM4, ENABLE);//使能TIMx外设
}//MOTOR_Init();


int MOTOR_Set(unsigned short MOTOR1_Def,unsigned short MOTOR2_Def,unsigned short MOTOR3_Def,unsigned short MOTOR4_Def){
	if((!WithoutDream) && IfUnlock){
		TIM4->CCR1 = MOTOR1_Def;//TIM_SetComparex(TIM_TypeDef* TIMx, u16 Comparex)
		TIM4->CCR2 = MOTOR2_Def;
		TIM4->CCR3 = MOTOR3_Def;
		TIM4->CCR4 = MOTOR4_Def;
	}
	else{
		TIM4->CCR1 = Motor_PWM_Idle - 100;//TIM_SetComparex(TIM_TypeDef* TIMx, u16 Comparex)
		TIM4->CCR2 = Motor_PWM_Idle - 100;
		TIM4->CCR3 = Motor_PWM_Idle - 100;
		TIM4->CCR4 = Motor_PWM_Idle - 100;
	}
        
	return 0;
}//int MOTOR_Set(unsigned short MOTOR1_Def,unsigned short MOTOR2_Def,unsigned short MOTOR3_Def,unsigned short MOTOR4_Def);
