#include "rc_ppm.h"

__rc_channel rc = { 0 }, rc_last = { 0 };
unsigned short RM_CH_Length[9] = { 0 };
unsigned char CH_Def;

bool is_PPM_Frame_Over(void);
bool is_PPM_IO_Reset(void);

void RC_PPM_Init(void){
    
    short gpio_set_time   = 0;        // us
    short gpio_reset_time = 0;        // us
    
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef  GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_BaseInitStructure;
 	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);	//ʹ�ܸ��ù���ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);			//ʹ��APB1����ʱ��	
    
	GPIO_InitStructure.GPIO_Pin = __RM_Pin;			
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 		
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
    
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource6);
	EXTI_InitStructure.EXTI_Line = EXTI_Line6;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//	EXTI_Init(&EXTI_InitStructure);
    
    // ��ʱ��������ʼ��
	TIM_BaseInitStructure.TIM_Prescaler = 72-1;				//����TIMʱ�ӷ�Ƶֵ
	TIM_BaseInitStructure.TIM_Period = 2000-1;		//����������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�
	TIM_BaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;	//TIM���ϼ���ģʽ
	TIM_BaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;			//����ʱ�ӷָ�_TDTS = Tck_tim 
	TIM_TimeBaseInit(TIM3, &TIM_BaseInitStructure);					//���� TIM_TimeBaseInitStruct ��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ 
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;		
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;	
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;				
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		
//	NVIC_Init(&NVIC_InitStructure);
  
    // ��⣺�����ߵ�ƽ5ms������Ϊ��⵽��֡�������ź�
    do {
        delay_us(250);
        if (is_PPM_Frame_Over() == false) {
            gpio_set_time += 250;
        }
        else {
            gpio_set_time = 0;
        }
        
        if (is_PPM_IO_Reset() == true) {
            gpio_reset_time += 250;
        }
        else {
            gpio_reset_time = 0;
        }
        
    } while (gpio_set_time <= 5000 &&    // 5ms
             gpio_reset_time <= 20000);  // 20ms
                                         // ��һ��������ʱ�˳�
    
    EXTI_Init(&EXTI_InitStructure);
    NVIC_Init(&NVIC_InitStructure);
	//TIM_Cmd(TIM3, ENABLE);  //ʹ��TIMx		
    
	CH_Def = 1;
    
}//void EXTIX_Init(void)

bool is_PPM_Frame_Over(void) {
    
    // ֻ���ں��ʵ�ʱ����ⲿ�жϲ��������ɼ�����
    // ���һ��ͨ����������һ֡��ʼ����һ����ų���8ms�ĸߵ�ƽ����ʱ��
    if (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6) == SET)
        return false;
    else
        return true;
}

bool is_PPM_IO_Reset(void) {
    // ���ң�������ˣ�ң�����������źţ�������Ҫ�����ж�
    if (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6) == RESET)
        return true;
    else
        return false;
}

void refine_RC_PPM_Data(void) {
    
    const short rc_bias = 400;
    const float lpf_stick_k         = 0.66f;
    const float lpf_sw_k            = 0.45f;
    
    __rc_channel rc_raw;
    
    rc_raw.pitch    = RM_CH_Length[pitch_ch];
    rc_raw.roll     = RM_CH_Length[roll_ch];
    rc_raw.throttle = RM_CH_Length[throttle_ch];
    rc_raw.yaw      = RM_CH_Length[yaw_ch];
    rc_raw.ch_5     = RM_CH_Length[sw_5_ch];
    rc_raw.ch_6     = RM_CH_Length[sw_6_ch];
    rc_raw.ch_7     = RM_CH_Length[sw_7_ch];
    rc_raw.ch_8     = RM_CH_Length[sw_8_ch];
    
    // ��������
    rc_raw.pitch    += rc_bias;
    rc_raw.roll     += rc_bias;
    rc_raw.throttle += rc_bias;
    rc_raw.yaw      += rc_bias;

    rc_raw.ch_5     += rc_bias;
    rc_raw.ch_6     += rc_bias;
    rc_raw.ch_7     += rc_bias;
    rc_raw.ch_8     += rc_bias;    
    
    // һ�׵�ͨ�˲���
    rc_last = rc;
    rc.pitch    = lpf_stick_k * rc_raw.pitch    + (1.0f - lpf_stick_k) * rc_last.pitch;
    rc.roll     = lpf_stick_k * rc_raw.roll     + (1.0f - lpf_stick_k) * rc_last.roll;
    rc.throttle = lpf_stick_k * rc_raw.throttle + (1.0f - lpf_stick_k) * rc_last.throttle;
    rc.yaw      = lpf_stick_k * rc_raw.yaw      + (1.0f - lpf_stick_k) * rc_last.yaw;

    rc.ch_5     = lpf_sw_k * rc_raw.ch_5        + (1.0f - lpf_sw_k) * rc_last.ch_5;
    rc.ch_6     = lpf_sw_k * rc_raw.ch_6        + (1.0f - lpf_sw_k) * rc_last.ch_6;
    rc.ch_7     = lpf_sw_k * rc_raw.ch_7        + (1.0f - lpf_sw_k) * rc_last.ch_7;
    rc.ch_8     = lpf_sw_k * rc_raw.ch_8        + (1.0f - lpf_sw_k) * rc_last.ch_8;

}

//��ʱ��3�жϷ������
void EXTI9_5_IRQHandler(void){
    
    static u8 pos_edge_num = 0;
    static u8 neg_edge_num = 0;
    
	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6) == RESET){
        if (neg_edge_num == 0) {  // �½��ؿ�ʼ�ź�
            neg_edge_num++;
            EXTI_ClearITPendingBit(EXTI_Line6); //���LINE6�ϵ��жϱ�־λ
            return;
        }
        neg_edge_num++;
        
		RM_CH_Length[CH_Def] = TIM3->CNT;
		CH_Def++;
		TIM_Cmd(TIM3,DISABLE);
		TIM3->CNT = 0;

	}
	else{
        pos_edge_num++;
        
        TIM_Cmd(TIM3,ENABLE);
        
	}
    
    // ���ͨ�������ﵽ��9���������������������½�����������˵����⵽�˱�ʾ�����������أ���˵��������֡����
    if (pos_edge_num == neg_edge_num && CH_Def >= 9) {
        pos_edge_num = neg_edge_num = 0;
        
        TIM_Cmd(TIM3,DISABLE);
		TIM3->CNT = 0;
        CH_Def = 1;
    }
    
    

	EXTI_ClearITPendingBit(EXTI_Line6); //���LINE6�ϵ��жϱ�־λ  
}//void EXTI9_5_IRQHandler(void)
