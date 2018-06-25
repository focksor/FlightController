#include "include/control.h"

PidObject pid_r_rate_acro;          // roll  rate
PidObject pid_p_rate_acro;          // pitch rate
PidObject pid_y_rate_acro;          // yaw   rate

__imu target_rate_acro;

bool init_Ctrl_Acro(void) {

    const float integrate_limit_pitch_roll = 50;
    const float integrate_limit_yaw        = 50;
    
    PidConfig pid_r_rate_acro_cfg =     {  1.25,  0.002,  0.0060 };
    PidConfig pid_p_rate_acro_cfg =     { -1.25, -0.002, -0.0060 };
    PidConfig pid_y_rate_acro_cfg =     { -2.05, -0.001, -0.0075 };
    
    InitPidObject(&pid_r_rate_acro, pid_r_rate_acro_cfg, 
                  integrate_limit_pitch_roll, -integrate_limit_pitch_roll, d_time_attitude);
    
    InitPidObject(&pid_p_rate_acro, pid_p_rate_acro_cfg, 
                  integrate_limit_pitch_roll, -integrate_limit_pitch_roll, d_time_attitude);
    
    InitPidObject(&pid_y_rate_acro, pid_y_rate_acro_cfg, 
                  integrate_limit_yaw, -integrate_limit_yaw, d_time_attitude);
    
    return true;
}

void calc_Target_Acro(__rc_channel _rc) {
    
    const float rc_pitch_roll_rate_sensitivity = 0.33f;
    const float rc_yaw_rate_sensitivity        = 0.33f;
    
    target_rate_acro.pitch = (_rc.pitch - 1500) * rc_pitch_roll_rate_sensitivity;
    target_rate_acro.roll  = (_rc.roll  - 1500) * rc_pitch_roll_rate_sensitivity;
    target_rate_acro.yaw   = (_rc.yaw   - 1500) * rc_yaw_rate_sensitivity;
 
    // ��������
    
}

void calc_Throttle_Acro(__rc_channel _rc, short *motor_out) {
        
    // ������С����ֵ������1000����������1000���²�����
    // �����������ֵ��1800���������ֵ����������������
    // ��Ҫ���ݷɻ����ص����������
    
    if (_rc.throttle <= 1000)
        *motor_out = _rc.throttle;
    else if (_rc.throttle >= 1800)
        *motor_out = 1800;
    else
        *motor_out = (_rc.throttle - 1000) * 0.66f + 1000;   // ���Ž�1000���ϵĶ���ȡԭ��ֵ��0.66��ʹ�ò��������
        
}

void control_PID_Acro(float g_roll, float g_pitch, float g_yaw,
                      float *roll_out, float *pitch_out, float *yaw_out) {
    // �ؼ�����
    // ����������: ������
    // Ŀ������:   ���ٶ�
    // ���:       ���
                          
    calcPid(&pid_r_rate_acro, target_rate_acro.roll,  g_roll);
    calcPid(&pid_p_rate_acro, target_rate_acro.pitch, g_pitch);
    calcPid(&pid_y_rate_acro, target_rate_acro.yaw,   g_yaw);

    *roll_out  = pid_r_rate_acro.output;
    *pitch_out = pid_p_rate_acro.output;
    *yaw_out   = pid_y_rate_acro.output;                          

    printf("{B%d:%d:%d:%d:%d}$",(int)g_yaw,(int)g_yaw,(int)g_yaw,(int)g_yaw,(int)g_yaw);                          
}

