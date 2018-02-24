#include "MPU9250.H"
#include "delay.h"
#include "stdlib.h"

unsigned char BUF[10];		 //接收数据缓存区
short T_X,T_Y,T_Z;	  //X,Y,Z轴
unsigned char str[30] = { 0 };
float Acc_X,Acc_Y,Acc_Z;
float GYRO_X,GYRO_Y,GYRO_Z;
float MAG_X,MAG_Y,MAG_Z;

void MPU9250_Init(void){  
	delay_init();
	Single_Write(GYRO_ADDRESS,PWR_MGMT_1, 0x00);  //解除休眠状态
	Single_Write(GYRO_ADDRESS,SMPLRT_DIV, 0x07);
	Single_Write(GYRO_ADDRESS,CONFIG, 0x06);
	Single_Write(GYRO_ADDRESS,GYRO_CONFIG, 0x18);
	Single_Write(GYRO_ADDRESS,ACCEL_CONFIG, 0x00);
}//void MPU9250_Init(void);

void READ_MPU9250_ACCEL(void){
	BUF[0]=Single_Read(ACCEL_ADDRESS,ACCEL_XOUT_L); 
	BUF[1]=Single_Read(ACCEL_ADDRESS,ACCEL_XOUT_H);
	T_X= (BUF[1]<<8)|BUF[0];//BUF[1] AS HIGH POSITION AND BUF[0] AS LOW POSITION
	Acc_X = 2 * 9.8f * (float)T_X / 32768;
		
	BUF[2]=Single_Read(ACCEL_ADDRESS,ACCEL_YOUT_L);
	BUF[3]=Single_Read(ACCEL_ADDRESS,ACCEL_YOUT_H);
	T_Y= (BUF[3]<<8)|BUF[2];
	Acc_Y = 2 * 9.8f * (float)T_Y / 32768;
	
	BUF[4]=Single_Read(ACCEL_ADDRESS,ACCEL_ZOUT_L);
	BUF[5]=Single_Read(ACCEL_ADDRESS,ACCEL_ZOUT_H);
	T_Z= (BUF[5]<<8)|BUF[4];
	Acc_Z = 2 * 9.8f * (float)T_Z / 32768; 
}//void READ_MPU9250_ACCEL(void);

void READ_MPU9250_GYRO(void){ 	
	BUF[0]=Single_Read(GYRO_ADDRESS,GYRO_XOUT_L); 
	BUF[1]=Single_Read(GYRO_ADDRESS,GYRO_XOUT_H);
	T_X= (BUF[1]<<8)|BUF[0];
	GYRO_X=T_X/16.4;//读取计算X轴数据

	BUF[2]=Single_Read(GYRO_ADDRESS,GYRO_YOUT_L);
	BUF[3]=Single_Read(GYRO_ADDRESS,GYRO_YOUT_H);
	T_Y= (BUF[3]<<8)|BUF[2];
	GYRO_Y=T_Y/16.4;//读取计算Y轴数据
	BUF[4]=Single_Read(GYRO_ADDRESS,GYRO_ZOUT_L);
	BUF[5]=Single_Read(GYRO_ADDRESS,GYRO_ZOUT_H);
	T_Z= (BUF[5]<<8)|BUF[4];
	GYRO_Z=2000*(float)T_Z / 32768;
	GYRO_Z=T_Z/16.4;//读取计算Z轴数据
}//void READ_MPU9250_GYRO(void);

void READ_MPU9250_MAG(void){ 
	Single_Write(GYRO_ADDRESS,0x37,0x02);//turn on Bypass Mode 
	delay_ms(7);//MUST TAKE MORE THAN 7ms.
	Single_Write(MAG_ADDRESS,0x0A,0x01);
	delay_ms(7);
	BUF[0]=Single_Read(MAG_ADDRESS,MAG_XOUT_L);
	BUF[1]=Single_Read(MAG_ADDRESS,MAG_XOUT_H);
	T_X=(BUF[1]<<8)|BUF[0];
	MAG_X=T_X;

	BUF[2]=Single_Read(MAG_ADDRESS,MAG_YOUT_L);
	BUF[3]=Single_Read(MAG_ADDRESS,MAG_YOUT_H);
	T_Y= (BUF[3]<<8)|BUF[2];
	MAG_Y=T_Y;
	
	BUF[4]=Single_Read(MAG_ADDRESS,MAG_ZOUT_L);
	BUF[5]=Single_Read(MAG_ADDRESS,MAG_ZOUT_H);
	T_Z= (BUF[5]<<8)|BUF[4];
	MAG_Z=T_Z;
}//void READ_MPU9250_MAG(void);