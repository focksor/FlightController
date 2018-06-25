#include "include/control.h"

// 内环
PidObject pid_r_rate_stab;          // roll  rate
PidObject pid_p_rate_stab;          // pitch rate
PidObject pid_y_rate_stab;          // yaw   rate
// 外环
PidObject pid_r_angle_stab;         // roll
PidObject pid_p_angle_stab;         // pitch
//PidObject pid_y_angle_stab;       // yaw

__imu target_rate_stab;             // 角速度内环期望，由PID外环得到
__imu target_angle_stab;            // 角度外环期望，由遥控器得到

bool init_Ctrl_Stabilized(void) {

    const float integrate_limit_pitch_roll_rate = 50;
    const float integrate_limit_yaw_rate        = 50;
    
    const float integrate_limit_pitch_roll = 25;
    const float integrate_limit_yaw = 25;

    // 内环
    PidConfig pid_r_rate_stab_cfg  =     {  1.35,  0.002,  0.0060 };
    PidConfig pid_p_rate_stab_cfg  =     { -1.35, -0.002, -0.0060 };
    PidConfig pid_y_rate_stab_cfg  =     { -2.05, -0.001, -0.0075 };
    
    PidConfig pid_r_angle_stab_cfg =     {  1.75,  0.000,  0.0045 };
    PidConfig pid_p_angle_stab_cfg =     {  1.75,  0.000,  0.0045 };
//  PidConfig pid_y_angle_stab_cfg =     { -0.00, -0.000, -0.0000 };
    
    // 内环
    InitPidObject(&pid_r_rate_stab, pid_r_rate_stab_cfg, 
                  integrate_limit_pitch_roll_rate, -integrate_limit_pitch_roll_rate, d_time_attitude);
    
    InitPidObject(&pid_p_rate_stab, pid_p_rate_stab_cfg, 
                  integrate_limit_pitch_roll_rate, -integrate_limit_pitch_roll_rate, d_time_attitude);
    
    InitPidObject(&pid_y_rate_stab, pid_y_rate_stab_cfg, 
                  integrate_limit_yaw_rate, -integrate_limit_yaw_rate, d_time_attitude);
    
    // 外环
    InitPidObject(&pid_r_angle_stab, pid_r_angle_stab_cfg, 
                  integrate_limit_pitch_roll, -integrate_limit_pitch_roll, d_time_attitude);
    
    InitPidObject(&pid_p_angle_stab, pid_p_angle_stab_cfg, 
                  integrate_limit_pitch_roll, -integrate_limit_pitch_roll, d_time_attitude);
    
//  InitPidObject(&pid_y_angle_stab, pid_y_angle_stab_cfg, 
//                integrate_limit_yaw, -integrate_limit_yaw, d_time_attitude);
    
    return true;
}

void calc_Target_Stabilized(__rc_channel _rc) {
    
    const float rc_pitch_roll_sensitivity = 15.0f;
    const float rc_yaw_sensitivity        = 0.33f;
    
    // 俯仰、横滚通过角度控制
    // 偏航通过角速度控制
    target_angle_stab.pitch = (rc.pitch - 1500) / rc_pitch_roll_sensitivity;
    target_angle_stab.roll  = (rc.roll  - 1500) / rc_pitch_roll_sensitivity;
    
    target_rate_stab.yaw = (1500 - rc.yaw) * rc_yaw_sensitivity;
    
    // 方向修正
    
}

void calc_Throttle_Stabilized(__rc_channel _rc, short *motor_out) {
        
    // 油门最小调整值限制在1000，即输入在1000以下不调整
    // 油门最大限制值在1800，超过这个值不允许油门再增加
    // 需要根据飞机载重调整这个函数
    
    if (_rc.throttle <= 1000)
        *motor_out = _rc.throttle;
    else if (_rc.throttle >= 1800)
        *motor_out = 1800;
    else
        *motor_out = (_rc.throttle - 1000) * 0.66f + 1000;       // 油门将1000以上的段落取原来值的0.66，使得操作更柔和
        
}

void control_PID_Stabilized(float g_roll, float g_pitch, float g_yaw,
                            float roll,   float pitch,   float yaw,
                            float *roll_out, float *pitch_out, float *yaw_out) {
    // 特技控制
    // 传感器输入:  角度、角速度
    // 目标输入:    角度
    // 输出:        电机

    // 角度外环
    calcPid(&pid_r_angle_stab, target_angle_stab.roll,   roll);
    calcPid(&pid_p_angle_stab, target_angle_stab.pitch,  pitch);                                

    target_rate_stab.roll  = pid_r_angle_stab.output;
    target_rate_stab.pitch = pid_p_angle_stab.output;           // 外环输出作为内环输入
    
    // 角速度内环
    calcPid(&pid_r_rate_stab, target_rate_stab.roll,  g_roll);
    calcPid(&pid_p_rate_stab, target_rate_stab.pitch, g_pitch);
    
    // 单独计算偏航角，偏航角仅使用角速度环
    calcPid(&pid_y_rate_stab, target_rate_stab.yaw,   g_yaw);

    *roll_out  = pid_r_rate_stab.output;
    *pitch_out = pid_p_rate_stab.output;
    *yaw_out   = pid_y_rate_stab.output;                          

    printf("{B%d:%d:%d:%d:%d}$",(int)g_yaw,(int)g_yaw,(int)g_yaw,(int)g_yaw,(int)g_yaw);                          
}

