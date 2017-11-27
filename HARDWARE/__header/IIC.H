#ifndef __IIC_H
#define __IIC_H

#include "config.h"
#include "stm32f10x.h"

//************************************
/*模拟IIC端口输出输入定义*/
#define SCL_H         GPIOB->BSRR = __IIC_SCL_Pin
#define SCL_L         GPIOB->BRR  = __IIC_SCL_Pin 
   
#define SDA_H         GPIOB->BSRR = __IIC_SDA_Pin
#define SDA_L         GPIOB->BRR  = __IIC_SDA_Pin

#define SCL_read      GPIOB->IDR  & __IIC_SCL_Pin
#define SDA_read      GPIOB->IDR  & __IIC_SDA_Pin

typedef enum {FALSE = 0, TRUE = !FALSE} bool;

void IIC_GPIO_Config(void);
void IIC_delay(void);
bool IIC_Start(void);
void IIC_Stop(void);
void IIC_Ack(void);
void IIC_NoAck(void);
bool IIC_WaitAck(void); 	 //返回为:=1有ACK,=0无ACK
void IIC_SendByte(u8 SendByte); //数据从高位到低位//
unsigned char IIC_RadeByte(void);  //数据从高位到低位//
bool Single_Write(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data);		     //void
unsigned char Single_Read(unsigned char SlaveAddress,unsigned char REG_Address);
void delay5ms(void);

#endif
