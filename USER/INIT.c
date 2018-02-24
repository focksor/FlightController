#include "INIT.h"

void Init_All(void){
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	uart_init(115200);
	delay_init();
	LED_Init();
	MOTOR_Init();
	InitSelfStability();
	MPU_IIC_Init();
	MPU9250_Init();
	EXTIX_Init();
	Timer_Init();
}
