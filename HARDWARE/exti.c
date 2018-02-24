#include "exti.h"

void EXTIX_Init(void){
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RM_Init();
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);	//使能复用功能时钟

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource6);
	EXTI_InitStructure.EXTI_Line = EXTI_Line6;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);	
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;		
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;	
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;				
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		
	NVIC_Init(&NVIC_InitStructure); 
}//void EXTIX_Init(void);

void EXTI9_5_IRQHandler(void){
	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6) == RESET){
		RM_CH_Length[CH_Def] = TIM3->CNT;
		CH_Def++;
		TIM_Cmd(TIM3,DISABLE);
		TIM3->CNT = 0;
	}
	else{
		TIM_Cmd(TIM3,ENABLE);
	}

	EXTI_ClearITPendingBit(EXTI_Line6); //清除LINE6上的中断标志位  
}//void EXTI9_5_IRQHandler(void);
