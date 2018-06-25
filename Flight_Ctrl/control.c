#include "include/control.h"

// 自稳控制: 俯仰、横滚角度，偏航角速度
__imu target_stabilized;
float target_yaw_rate;

const float d_time_attitude = 2.5e-3;

void init_Control(void) {
    
    init_Ctrl_Acro();
    init_Ctrl_Stabilized();
    
}

void control_All(__flight_mode_t flight_mode,
                 short *motor_1, short *motor_2, short *motor_3, short *motor_4) {
    
    bool is_in_safe;
    short motor_out_throttle = 0;
    float roll_out = 0.0f, pitch_out = 0.0f, yaw_out = 0.0f;
    
    is_in_safe = control_Safety();
                         
    if (is_in_safe) {
        // 安全开关打开
        // 则切断所有控制，关闭马达，阻止起飞
        *motor_1 = *motor_2 = *motor_3 = *motor_4 = 0;
        
        return;
    }
    
    switch (flight_mode) {
        
        case mode_stabilized : {

            calc_Target_Stabilized(rc);
            calc_Throttle_Stabilized(rc, &motor_out_throttle);
            control_PID_Stabilized(gyro.x,   gyro.y,     gyro.z,
                                   IMU.roll, IMU.pitch,  IMU.yaw,
                                   &roll_out, &pitch_out, &yaw_out);
            
        } break;
        
        case mode_acro : {
                        
            calc_Target_Acro(rc);
            calc_Throttle_Acro(rc, &motor_out_throttle);
            control_PID_Acro(gyro.x, gyro.y, gyro.z,
                             &roll_out, &pitch_out, &yaw_out);
                    
        } break;
        
        default : break;
    }
    
    *motor_1 = motor_out_throttle - pitch_out - roll_out + yaw_out;
    *motor_2 = motor_out_throttle + pitch_out + roll_out + yaw_out;
    *motor_3 = motor_out_throttle - pitch_out + roll_out - yaw_out;
    *motor_4 = motor_out_throttle + pitch_out - roll_out - yaw_out;
    
}

/*
安全开关相关，应该在主程序内被执行:

*/
bool control_Safety(void) {    

    if (rc.ch_7 >= 1500) {
        // 紧急停止打开，不能起飞
        WithoutDream = true;
        return true;
    } else {
        
        WithoutDream = false;
        if (rc.ch_7 < 1500 && rc.ch_6 < 1500) {    
            // 紧急停止关闭但马达没解锁，不能起飞
            IfUnlock = false;
            return true;
        } else if (rc.ch_6 >= 1500) {
            // 飞行器安全锁没有开启，可以起飞
            IfUnlock = true;
            return false;
        }
    }
    return true;
}
