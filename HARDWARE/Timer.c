#include "timer.h"

__time_slice t_slice;
double       time_since_boot = 0.;      // ��λ: s

void Timer_Heartbeat_Init(void){
	TIM_TimeBaseInitTypeDef TIM_BaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	// ��ʱ��������ʼ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);			//ʹ��APB1����ʱ��
	
	TIM_BaseInitStructure.TIM_Prescaler = 72-1;				//����TIMʱ�ӷ�Ƶֵ
	TIM_BaseInitStructure.TIM_Period = 2500-1;		//����������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�
	TIM_BaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;	//TIM���ϼ���ģʽ
	TIM_BaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;			//����ʱ�ӷָ�_TDTS = Tck_tim 
	TIM_TimeBaseInit(TIM5, &TIM_BaseInitStructure);					//���� TIM_TimeBaseInitStruct ��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ 
	
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE); //ʹ��ָ����TIM2�ж�,��������ж�

	//�ж����ȼ�NVIC����
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //��ʼ��NVIC�Ĵ���
	
	TIM_Cmd(TIM5, ENABLE);  //ʹ��TIMx
}//void Timer_Init(void);

// ����������stm32f10x_it.c
//void TIM5_IRQHandler(void)

void update_TimeSlice(short heartbeat) {
    const float dt = 2.5e-3;    // s
    float t0 = dt * (float)heartbeat;
    
    time_since_boot += t0;      // ������ʱ��
    
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
