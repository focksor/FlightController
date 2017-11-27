#include "flight.h"


int main(void){
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	uart_init(115200);	 //串口初始化为115200
	delay_init();
	LCD_Init();		
	IIC_GPIO_Config();
	MPU9250_Init();
	show_Init();
	
	while(1){
		showData();
	}
}

/*
{
__电机驱动
	MOTOR_Init();
	MOTOR_Write(-100, -100, -100, -100);//电机初始化
	delay_ms(1500);
	 
	MOTOR_Write(100, 200, 300, 400);
}
*/

/*
{
__LCD驱动
	LCD_Init();
	
	POINT_COLOR=RED;
	LCD_ShowString(60,50,200,16,16,"WarShip STM32");	
}
*/

