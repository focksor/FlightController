#include "timer.h"

__time_slice t_slice;
double       time_since_boot = 0.;      // 单位: s

void Timer_Heartbeat_Init(void){
	TIM_TimeBaseInitTypeDef TIM_BaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	// 定时器基础初始化
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);			//使能APB1外设时钟
	
	TIM_BaseInitStructure.TIM_Prescaler = 72-1;				//设置TIM时钟分频值
	TIM_BaseInitStructure.TIM_Period = 2500-1;		//设置了在下一个更新事件装入活动的自动重装载寄存器周期的
	TIM_BaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;	//TIM向上计数模式
	TIM_BaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;			//设置时钟分割_TDTS = Tck_tim 
	TIM_TimeBaseInit(TIM5, &TIM_BaseInitStructure);					//根据 TIM_TimeBaseInitStruct 中指定的参数初始化TIMx的时间基数单位 
	
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE); //使能指定的TIM2中断,允许更新中断

	//中断优先级NVIC设置
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器
	
	TIM_Cmd(TIM5, ENABLE);  //使能TIMx
}//void Timer_Init(void);

// 心跳函数见stm32f10x_it.c
//void TIM5_IRQHandler(void)

void update_TimeSlice(short heartbeat) {
    const float dt = 2.5e-3;    // s
    float t0 = dt * (float)heartbeat;
    
    time_since_boot += t0;      // 更新总时间
    
    if (t0 >= 5e-3)             // 5ms
        t_slice._200Hz = true;
    if (t0 >= 1e-2)             // 10ms
        t_slice._100Hz = true;
    if (t0 >= 2e-2)             // 20ms
        t_slice._50Hz  = true;
    if (t0 >= 4e-2)             // 40ms
        t_slice._25Hz  = true;
    if (t0 >= 1e-1)             // 100ms
        t_slice._10Hz  = true;
    if (t0 >= 2e-1)             // 200ms
        t_slice._5Hz   = true;
    if (t0 >= 1e0) {            // 1s
        t_slice._1Hz   = true;
    }
}
