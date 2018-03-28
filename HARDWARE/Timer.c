#include "timer.h"

void Timer_Init(void){
	TIM_TimeBaseInitTypeDef TIM_BaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	// 定时器基础初始化
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);			//使能APB1外设时钟
	
	TIM_BaseInitStructure.TIM_Prescaler = 72-1;				//设置TIM时钟分频值
	TIM_BaseInitStructure.TIM_Period = 2000-1;		//设置了在下一个更新事件装入活动的自动重装载寄存器周期的
	TIM_BaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;	//TIM向上计数模式
	TIM_BaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;			//设置时钟分割_TDTS = Tck_tim 
	TIM_TimeBaseInit(TIM2, &TIM_BaseInitStructure);					//根据 TIM_TimeBaseInitStruct 中指定的参数初始化TIMx的时间基数单位 
	
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE); //使能指定的TIM2中断,允许更新中断

	//中断优先级NVIC设置
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;  //从优先级4级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器
	
	TIM_Cmd(TIM2, ENABLE);  //使能TIMx
	

	// 定时器基础初始化
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);			//使能APB1外设时钟
	
	TIM_BaseInitStructure.TIM_Prescaler = 72-1;				//设置TIM时钟分频值
	TIM_BaseInitStructure.TIM_Period = 2000-1;		//设置了在下一个更新事件装入活动的自动重装载寄存器周期的
	TIM_BaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;	//TIM向上计数模式
	TIM_BaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;			//设置时钟分割_TDTS = Tck_tim 
	TIM_TimeBaseInit(TIM3, &TIM_BaseInitStructure);					//根据 TIM_TimeBaseInitStruct 中指定的参数初始化TIMx的时间基数单位 
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //使能指定的TIM3中断,允许更新中断
	
	//中断优先级NVIC设置
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器
	
	TIM_Cmd(TIM3, ENABLE);  //使能TIMx		
	
	
	// 定时器基础初始化
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);			//使能APB1外设时钟
	
	TIM_BaseInitStructure.TIM_Prescaler = 72-1;				//设置TIM时钟分频值
	TIM_BaseInitStructure.TIM_Period = 20000-1;		//设置了在下一个更新事件装入活动的自动重装载寄存器周期的
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

void TIM2_IRQHandler(void){//T=2000us
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET){
		static unsigned char Tim2Cntr;
		
		WithoutDream=!(RC_Com(7,600));//THE UAV WILL FALL LIKES A SALTED FISH WITHOUT DREAM
		if(RC_Com(3,600)){//ONLY WHEN THE THROTTLE ON THE BOTTOM THE UAV CAN BE UNLOCKED
			IfUnlock=RC_Com(6,1600);
			pidPitchAngle.Integral	= 0;
			pidRollAngle.Integral	= 0;
			pidYawAngle.Integral	= 0;
			pidPitchAngularRate.Integral	= 0;
			pidRollAngularRate.Integral		= 0;
			pidYawAngularRate.Integral		= 0;
		}
		
		if(++Tim2Cntr>=40)//CHANGE LED TO SHOW THE STATE OF TIM2
			Tim2Cntr=GPIO_WriteBit(GPIOE,GPIO_Pin_6,(BitAction)!(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_6)));
		
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);  //清除TIMx更新中断标志 
	}
}//void TIM2_IRQHandler(void);

void TIM3_IRQHandler(void){//T=2000us
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET){
		static unsigned char Tim3Cntr;
		
		CH_Def=0;
		RM_CH_Length[0]=1;//SIGNED THE RC HAS BEEN STARTED
		
		if(++Tim3Cntr>=50)//CHANGE LED TO SHOW THE STATE OF TIM3
			Tim3Cntr=GPIO_WriteBit(GPIOE,GPIO_Pin_5,(BitAction)!(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_5)));
		
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);  //清除TIMx更新中断标志 
	}
}//void TIM3_IRQHandler(void);

void TIM5_IRQHandler(void){//T=20000us,20ms
	if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET){
		short RC3=RM_CH_Length[3]-600;//CH3:600-1100-1600us
		READ_MPU9250_GYRO();
		CalcPitch(0);
		CalcRoll(0);
		CalcPitch(1);
		CalcRoll(1);
		CalcYaw(1);
		
		SelfStability(0,0,0);
		
		//设前倾，右滚，顺时针偏航为正方向。调整重心。
		MOTOR1_Def=Motor_PWM_Idle+RC3-Pitch_PID-Roll_PID+Yaw_PID;
		MOTOR2_Def=Motor_PWM_Idle+RC3+Pitch_PID+Roll_PID+Yaw_PID;
		MOTOR3_Def=Motor_PWM_Idle+RC3-Pitch_PID+Roll_PID-Yaw_PID;
		MOTOR4_Def=Motor_PWM_Idle+RC3+Pitch_PID-Roll_PID-Yaw_PID;
			
		MOTOR_Set(MOTOR1_Def,MOTOR2_Def,MOTOR3_Def,MOTOR4_Def);
		//MOTOR_Set(Motor_PWM_Idle+RC3,Motor_PWM_Idle+RC3,Motor_PWM_Idle+RC3,Motor_PWM_Idle+RC3);

		TIM_ClearITPendingBit(TIM5, TIM_IT_Update);  //清除TIMx更新中断标志 
	}
}//void TIM5_IRQHandler(void);
