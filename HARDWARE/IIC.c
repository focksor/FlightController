#include "IIC.H"

char  test=0; 

void IIC_GPIO_Config(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure; 
 
  GPIO_InitStructure.GPIO_Pin =  __IIC_Pin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;  
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void IIC_delay(void){	
	u8 i=30; //这里可以优化速度	，经测试最低到5还能写入
	while(i) 
	{ 
	  i--; 
	}  	
}	

bool IIC_Start(void)
{
	SDA_H;
	SCL_H;
	IIC_delay();
	if(!SDA_read)return FALSE;	//SDA线为低电平则总线忙,退出
	SDA_L;
	IIC_delay();
	if(SDA_read) return FALSE;	//SDA线为高电平则总线出错,退出
	SDA_L;
	IIC_delay();
	return TRUE;
}

void IIC_Stop(void)
{
	SCL_L;
	IIC_delay();
	SDA_L;
	IIC_delay();
	SCL_H;
	IIC_delay();
	SDA_H;
	IIC_delay();
} 

void IIC_Ack(void)
{	
	SCL_L;
	IIC_delay();
	SDA_L;
	IIC_delay();
	SCL_H;
	IIC_delay();
	SCL_L;
	IIC_delay();
}	

void IIC_NoAck(void)
{	
	SCL_L;
	IIC_delay();
	SDA_H;
	IIC_delay();
	SCL_H;
	IIC_delay();
	SCL_L;
	IIC_delay();
} 

bool IIC_WaitAck(void) 	 //返回为:=1有ACK,=0无ACK
{
	SCL_L;
	IIC_delay();
	SDA_H;			
	IIC_delay();
	SCL_H;
	IIC_delay();
	if(SDA_read)
	{
		SCL_L;
	  IIC_delay();
		return FALSE;
	}
	SCL_L;
	IIC_delay();
	return TRUE;
}

void IIC_SendByte(u8 SendByte) //数据从高位到低位//
{
	 u8 i=8;
	 while(i--)
	 {
		  SCL_L;
		  IIC_delay();
		if(SendByte&0x80)
		  SDA_H;  
		else 
		  SDA_L;	
		  SendByte<<=1;
		  IIC_delay();
		SCL_H;
		  IIC_delay();
	 }
	 SCL_L;
} 

unsigned char IIC_RadeByte(void)  //数据从高位到低位//
{ 
	 u8 i=8;
	 u8 ReceiveByte=0;

	 SDA_H;				
	 while(i--)
	 {
		ReceiveByte<<=1;		
		SCL_L;
		IIC_delay();
	  SCL_H;
		IIC_delay();	
		if(SDA_read)
		{
		  ReceiveByte|=0x01;
		}
	 }
	 SCL_L;
	 return ReceiveByte;
} 

//单字节写入*******************************************
bool Single_Write(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data)			  //void
{
  	if(!IIC_Start())return FALSE;
	 IIC_SendByte(SlaveAddress);	//发送设备地址+写信号//IIC_SendByte(((REG_Address & 0x0700) >>7) | SlaveAddress & 0xFFFE);//设置高起始地址+器件地址 
	 if(!IIC_WaitAck()){IIC_Stop(); return FALSE;}
	 IIC_SendByte(REG_Address );	//设置低起始地址		
	 IIC_WaitAck();	
	 IIC_SendByte(REG_data);
	 IIC_WaitAck();	
	 IIC_Stop(); 
	 delay5ms();
	 return TRUE;
}

//单字节读取*****************************************
unsigned char Single_Read(unsigned char SlaveAddress,unsigned char REG_Address)
{	unsigned char REG_data;	  	
	if(!IIC_Start())return FALSE;
	 IIC_SendByte(SlaveAddress); //IIC_SendByte(((REG_Address & 0x0700) >>7) | REG_Address & 0xFFFE);//设置高起始地址+器件地址 
	 if(!IIC_WaitAck()){IIC_Stop();test=1; return FALSE;}
	 IIC_SendByte((u8) REG_Address);	//设置低起始地址		
	 IIC_WaitAck();
	 IIC_Start();
	 IIC_SendByte(SlaveAddress+1);
	 IIC_WaitAck();

	REG_data= IIC_RadeByte();
	 IIC_NoAck();
	 IIC_Stop();
	 //return TRUE;
	return REG_data;

}		

void delay5ms(void)
{
		
	int i=5000;  
	while(i) 
	{ 
	  i--; 
	}  
}
