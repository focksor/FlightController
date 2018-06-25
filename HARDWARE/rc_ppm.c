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
 	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);	//使能复用功能时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);			//使能APB1外设时钟	
    
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
    
    // 定时器基础初始化
	TIM_BaseInitStructure.TIM_Prescaler = 72-1;				//设置TIM时钟分频值
	TIM_BaseInitStructure.TIM_Period = 2000-1;		//设置了在下一个更新事件装入活动的自动重装载寄存器周期的
	TIM_BaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;	//TIM向上计数模式
	TIM_BaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;			//设置时钟分割_TDTS = Tck_tim 
	TIM_TimeBaseInit(TIM3, &TIM_BaseInitStructure);					//根据 TIM_TimeBaseInitStruct 中指定的参数初始化TIMx的时间基数单位 
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;		
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;	
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;				
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		
//	NVIC_Init(&NVIC_InitStructure);
  
    // 检测：持续高电平5ms可以认为检测到该帧结束的信号
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
                                         // 当一个不满足时退出
    
    EXTI_Init(&EXTI_InitStructure);
    NVIC_Init(&NVIC_InitStructure);
	//TIM_Cmd(TIM3, ENABLE);  //使能TIMx		
    
	CH_Def = 1;
    
}//void EXTIX_Init(void)

bool is_PPM_Frame_Over(void) {
    
    // 只有在合适的时候打开外部中断才能正常采集数据
    // 最后一个通道结束到下一帧开始，有一个大概长达8ms的高电平持续时间
    if (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6) == SET)
        return false;
    else
        return true;
}

bool is_PPM_IO_Reset(void) {
    // 如果遥控器关了，遥控器会拉低信号，所以需要额外判定
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
    
    // 油门修正
    rc_raw.pitch    += rc_bias;
    rc_raw.roll     += rc_bias;
    rc_raw.throttle += rc_bias;
    rc_raw.yaw      += rc_bias;

    rc_raw.ch_5     += rc_bias;
    rc_raw.ch_6     += rc_bias;
    rc_raw.ch_7     += rc_bias;
    rc_raw.ch_8     += rc_bias;    
    
    // 一阶低通滤波器
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

//定时器3中断服务程序
void EXTI9_5_IRQHandler(void){
    
    static u8 pos_edge_num = 0;
    static u8 neg_edge_num = 0;
    
	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6) == RESET){
        if (neg_edge_num == 0) {  // 下降沿开始信号
            neg_edge_num++;
            EXTI_ClearITPendingBit(EXTI_Line6); //清除LINE6上的中断标志位
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
    
    // 如果通道数都达到了9，而且上升沿数量等于下降沿数量，则说明检测到了表示结束的上升沿，则说明结束单帧采样
    if (pos_edge_num == neg_edge_num && CH_Def >= 9) {
        pos_edge_num = neg_edge_num = 0;
        
        TIM_Cmd(TIM3,DISABLE);
		TIM3->CNT = 0;
        CH_Def = 1;
    }
    
    

	EXTI_ClearITPendingBit(EXTI_Line6); //清除LINE6上的中断标志位  
}//void EXTI9_5_IRQHandler(void)
