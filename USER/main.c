#include "flight.h"

extern float Roll_PID;
void get_GyroIdle(void);

int main(void){	
	Init_All();
    TIM_Cmd(TIM5, DISABLE);
	get_GyroIdle();
    TIM_Cmd(TIM5, ENABLE);
    
	printf("FlightController is running.\r\nUAV is always ready to fall like a salted fish without dream.\n");
	
	while(1){
		READ_MPU9250_ACCEL();
//		READ_MPU9250_GYRO();
//		READ_MPU9250_MAG();

<<<<<<< HEAD
		printf("{B%d:%d:%d:%d:%d}$",(int)(Pitch_Sensor*1000),(int)(Pitch_PID*1000),(int)(Roll_Sensor*1000),(int)(Roll_PID*1000),(int)(Yaw_PID*1000));
=======
        printf("{B%d:%d:%d:%d:%d}$",(int)(Roll_Sensor*1000), (int)(Roll_PID*1000), 0, 0, 0);
//		printf("{B%d:%d:%d:%d:%d}$",(int)(Pitch_Accel*1000),(int)(Pitch_Sensor*1000),(int)(Roll_Accel*1000),(int)(Roll_Sensor*1000),0);
>>>>>>> e334d5f022fbf2d0af8dc30df98d1e3fac9d684d
//		printf("{B%d:%d:%d:%d:%d}$",(int)(Pitch_Sensor*1000),(int)(Roll_Sensor*1000),(int)(Yaw_Sensor*1000),(int)(Pitch_PID*1000),(int)(Roll_PID*1000));
//		printf("{B%d:%d:%d:%d:%d}$",(int)(MOTOR1_Def),(int)(MOTOR2_Def),(int)(MOTOR3_Def),(int)(MOTOR4_Def),(int)(MOTOR4_Def));
		delay_ms(50);
	}
}

void get_GyroIdle(void) {
    
    int i = 0;
    float gx_temp = 0.0f, gy_temp = 0.0f, gz_temp = 0.0f;
    
    while (i < 10) {
    
        gx_temp += GYRO_X;
        gy_temp += GYRO_Y;
        gz_temp += GYRO_Z;
        
        i++;
        delay_ms(50);
    }
    
    gx_idle = gx_temp / 10;
    gy_idle = gy_temp / 10;
    gz_idle = gz_temp / 10;
    
}// void get_GyroIdle(void)
