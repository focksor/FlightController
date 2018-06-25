#include "flight.h"

int main(void){

	init_All();
	
	printf("FlightController is running.\r\nUAV is always ready to fall like a salted fish without dream.\n");
	    
	while(true) {
        
        // 软实时任务表，可以将任务添加在下列if框内
        if (t_slice._200Hz) {       // 5ms
            
            
            
            t_slice._200Hz = false;
        }
        if (t_slice._100Hz) {       // 10ms
              
            display_Debug_Info();

            
            t_slice._100Hz = false;
        }            
        if (t_slice._50Hz) {        // 20ms
        
            
            
            t_slice._50Hz  = false;
        }
        if (t_slice._25Hz) {        // 40ms
        
            
            
            t_slice._25Hz  = false;
        }
        if (t_slice._10Hz) {        // 100ms
        
            
            
            t_slice._10Hz  = false;
        }
        if (t_slice._5Hz) {         // 200ms
        
            
            
            t_slice._5Hz  = false;
        }
        if (t_slice._1Hz) {         // 1s
        
            
            
            t_slice._1Hz  = false;
        }               
	}
}

void init_All(void) {
    
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	uart_init(115200);
	delay_init();
	LED_Init();
	MOTOR_Init();
	MPU_IIC_Init();
	delay_ms(200);
    
	MPU_Init();
    while (mpu_dmp_init()) {
        delay_ms(50);
    }
    mpu_set_lpf(188);
    mpu_set_sample_rate(400);
    mpu_set_gyro_fsr(1000);
    mpu_set_accel_fsr(4);
	delay_ms(200);
    
    // 校准陀螺仪静偏
    //calc_Gyro_Offset();
    
    // 初始化控制PID
    init_Control();
    
    // 开启遥控器，开启控制环
    RC_PPM_Init();
	Timer_Heartbeat_Init();
        
}

void display_Debug_Info(void) {
    
    // 其实如果需要调试，可以把printf写进需要检查的位置，作为断点使用
    
//  printf("{B%d:%d:%d:%d:%d}$",(int)(Pitch_Sensor*1000),(int)(Pitch_PID*1000),(int)(Roll_Sensor*1000),(int)(Roll_PID*1000),(int)(Yaw_PID*1000));
//  printf("{B%d:%d:%d:%d:%d}$",(int)(Pitch_Sensor*1000),(int)(Roll_Sensor*1000),(int)(Yaw_Sensor*1000),(int)(Pitch_PID*1000),(int)(Roll_PID*1000));
//  printf("{B%d:%d:%d:%d:%d}$",(int)(MOTOR1_Def),(int)(MOTOR2_Def),(int)(MOTOR3_Def),(int)(MOTOR4_Def),(int)(MOTOR4_Def));

//  printf("{B%d:%d:%d:%d:%d}$",(int)rc.pitch,(int)rc.roll,(int)rc.ch_5,(int)rc.ch_5,(int)rc.ch_5);
    
}
