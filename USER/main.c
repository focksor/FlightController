#include "flight.h"


int main(void){
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	uart_init(115200);	 //���ڳ�ʼ��Ϊ115200
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
__�������
	MOTOR_Init();
	MOTOR_Write(-100, -100, -100, -100);//�����ʼ��
	delay_ms(1500);
	 
	MOTOR_Write(100, 200, 300, 400);
}
*/

/*
{
__LCD����
	LCD_Init();
	
	POINT_COLOR=RED;
	LCD_ShowString(60,50,200,16,16,"WarShip STM32");	
}
*/

