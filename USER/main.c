#include "flight.h"

int main(void){	
	Init_All();
	
	printf("FlightController is running.\r\nUAV is always ready to fall like a salted fish without dream.\n");
	
	while(1){
		READ_MPU9250_ACCEL();
//		READ_MPU9250_GYRO();
//		READ_MPU9250_MAG();

		printf("{B%d:%d:%d:%d:%d}$",(int)(Pitch_Sensor*1000),(int)(Pitch_PID*1000),(int)(Roll_Sensor*1000),(int)(Roll_PID*1000),(int)(Yaw_PID*1000));
//		printf("{B%d:%d:%d:%d:%d}$",(int)(Pitch_Sensor*1000),(int)(Roll_Sensor*1000),(int)(Yaw_Sensor*1000),(int)(Pitch_PID*1000),(int)(Roll_PID*1000));
//		printf("{B%d:%d:%d:%d:%d}$",(int)(MOTOR1_Def),(int)(MOTOR2_Def),(int)(MOTOR3_Def),(int)(MOTOR4_Def),(int)(MOTOR4_Def));
		delay_ms(50);
	}
}
