#include "MOTOR.H"
#include "stm32f10x.h"

void MOTOR_Init(){
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_BaseInitStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);			//ʹ��GPIOBʱ��
	GPIO_InitStructure.GPIO_Pin=__MOTOR_Pin;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;					//AF_��ʱ�����Ÿ���
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	// ��ʱ��������ʼ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);			//ʹ��APB1����ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);			//ʹ�ܸ���ʱ��
	
	TIM_BaseInitStructure.TIM_Prescaler     = 72 - 1;				//����TIMʱ�ӷ�Ƶֵ
	TIM_BaseInitStructure.TIM_Period        = Motor_PWM_Period;		//����������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�,����Ϊ2500us
	TIM_BaseInitStructure.TIM_CounterMode   = TIM_CounterMode_Up;	//TIM���ϼ���ģʽ
	TIM_BaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;			//����ʱ�ӷָ�_TDTS = Tck_tim 
	
	TIM_TimeBaseInit(TIM4, &TIM_BaseInitStructure);					//���� TIM_TimeBaseInitStruct ��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ 
	
	// PWM��ʼ��
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;				//���ö�ʱ��ģʽ_TIM ������ȵ���ģʽ 1 (CCR>TIM_PulseʱΪ��Ч��ƽ��ģʽ2�෴)
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	//����OCx�������Ӧ����
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;		//�����������_TIM����Ƚϼ��Ը�
	TIM_OCInitStructure.TIM_Pulse = Motor_PWM_Idle;					//�����˴�װ�벶��ȽϼĴ���������ֵ
	
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);						//��ʼ��TIMͨ��
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);				//ʹ��TIM��CCRx�ϵ�Ԥװ�ؼĴ���
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);	
	
	TIM_Cmd(TIM4, ENABLE);											//ʹ��TIMx����
	
}//MOTOR_Init()

int MOTOR_Write(int val_1, int val_2, int val_3, int val_4){
	TIM4->CCR1 = Motor_PWM_Idle + val_1;							//TIM_SetComparex(TIM_TypeDef* TIMx, u16 Comparex)
	TIM4->CCR2 = Motor_PWM_Idle + val_2;
	TIM4->CCR3 = Motor_PWM_Idle + val_3;
	TIM4->CCR4 = Motor_PWM_Idle + val_4;
	
	return 0;
	
	
}// int MOTOR_Write(u16 val_1, u16 val_2, u16 val_3, u16 val_4)