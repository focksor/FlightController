#ifndef __MPUIIC_H
#define __MPUIIC_H
#include "sys.h"
#include "config.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK战舰STM32开发板V3
//MPU6050 IIC驱动 代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2015/1/17
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////


//IO方向设置
#define MPU_SDA_IN()  {GPIOB->CRH&=0XFF0FFFFF;GPIOB->CRH|=8<<20;}
#define MPU_SDA_OUT() {GPIOB->CRH&=0XFF0FFFFF;GPIOB->CRH|=3<<20;}

//IO操作函数	 
#define MPU_IIC_SCL    PBout(12) 		//SCL
#define MPU_IIC_SDA    PBout(13) 		//SDA	 
#define MPU_READ_SDA   PBin(13) 		//输入SDA 


#define SCL_read      GPIOB->IDR  & SCL_PIN /* GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_10) */
#define SDA_read      GPIOB->IDR  & SDA_PIN /* GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_11) */
#define SCL_PIN        GPIO_Pin_12
#define SDA_PIN        GPIO_Pin_13
//IIC所有操作函数
void MPU_IIC_Delay(void);				//MPU IIC延时函数
void MPU_IIC_Init(void);                //初始化IIC的IO口				 
void MPU_IIC_Start(void);				//发送IIC开始信号
void MPU_IIC_Stop(void);	  			//发送IIC停止信号
void MPU_IIC_Send_Byte(u8 txd);			//IIC发送一个字节
u8 MPU_IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
u8 MPU_IIC_Wait_Ack(void); 				//IIC等待ACK信号
void MPU_IIC_Ack(void);					//IIC发送ACK信号
void MPU_IIC_NAck(void);				//IIC不发送ACK信号

bool Single_Write(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data);
unsigned char Single_Read(unsigned char SlaveAddress,unsigned char REG_Address);

#endif
