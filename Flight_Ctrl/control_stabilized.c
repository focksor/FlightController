#include "include/control.h"

// �ڻ�
PidObject pid_r_rate_stab;          // roll  rate
PidObject pid_p_rate_stab;          // pitch rate
PidObject pid_y_rate_stab;          // yaw   rate
// �⻷
PidObject pid_r_angle_stab;         // roll
PidObject pid_p_angle_stab;         // pitch
//PidObject pid_y_angle_stab;       // yaw

__imu target_rate_stab;             // ���ٶ��ڻ���������PID�⻷�õ�
__imu target_angle_stab;            // �Ƕ��⻷��������ң�����õ�

bool init_Ctrl_Stabilized(void) {

    const float integrate_limit_pitch_roll_rate = 50;
    const float integrate_limit_yaw_rate        = 50;
    
    const float integrate_limit_pitch_roll = 25;
    const float integrate_limit_yaw = 25;

    // �ڻ�
    PidConfig pid_r_rate_stab_cfg  =     {  1.35,  0.002,  0.0060 };
    PidConfig pid_p_rate_stab_cfg  =     { -1.35, -0.002, -0.0060 };
    PidConfig pid_y_rate_stab_cfg  =     { -2.05, -0.001, -0.0075 };
    
    PidConfig pid_r_angle_stab_cfg =     {  1.75,  0.000,  0.0045 };
    PidConfig pid_p_angle_stab_cfg =     {  1.75,  0.000,  0.0045 };
//  PidConfig pid_y_angle_stab_cfg =     { -0.00, -0.000, -0.0000 };
    
    // �ڻ�
    InitPidObject(&pid_r_rate_stab, pid_r_rate_stab_cfg, 
                  integrate_limit_pitch_roll_rate, -integrate_limit_pitch_roll_rate, d_time_attitude);
    
    InitPidObject(&pid_p_rate_stab, pid_p_rate_stab_cfg, 
                  integrate_limit_pitch_roll_rate, -integrate_limit_pitch_roll_rate, d_time_attitude);
    
    InitPidObject(&pid_y_rate_stab, pid_y_rate_stab_cfg, 
                  integrate_limit_yaw_rate, -integrate_limit_yaw_rate, d_time_attitude);
    
    // �⻷
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
    
    // ���������ͨ���Ƕȿ���
    // ƫ��ͨ�����ٶȿ���
    target_angle_stab.pitch = (rc.pitch - 1500) / rc_pitch_roll_sensitivity;
    target_angle_stab.roll  = (rc.roll  - 1500) / rc_pitch_roll_sensitivity;
    
    target_rate_stab.yaw = (1500 - rc.yaw) * rc_yaw_sensitivity;
    
    // ��������
    
}

void calc_Throttle_Stabilized(__rc_channel _rc, short *motor_out) {
        
    // ������С����ֵ������1000����������1000���²�����
    // �����������ֵ��1800���������ֵ����������������
    // ��Ҫ���ݷɻ����ص����������
    
    if (_rc.throttle <= 1000)
        *motor_out = _rc.throttle;
    else if (_rc.throttle >= 1800)
        *motor_out = 1800;
    else
        *motor_out = (_rc.throttle - 1000) * 0.66f + 1000;       // ���Ž�1000���ϵĶ���ȡԭ��ֵ��0.66��ʹ�ò��������
        
}

void control_PID_Stabilized(float g_roll, float g_pitch, float g_yaw,
                            float roll,   float pitch,   float yaw,
                            float *roll_out, float *pitch_out, float *yaw_out) {
    // �ؼ�����
    // ����������:  �Ƕȡ����ٶ�
    // Ŀ������:    �Ƕ�
    // ���:        ���

    // �Ƕ��⻷
    calcPid(&pid_r_angle_stab, target_angle_stab.roll,   roll);
    calcPid(&pid_p_angle_stab, target_angle_stab.pitch,  pitch);                                

    target_rate_stab.roll  = pid_r_angle_stab.output;
    target_rate_stab.pitch = pid_p_angle_stab.output;           // �⻷�����Ϊ�ڻ�����
    
    // ���ٶ��ڻ�
    calcPid(&pid_r_rate_stab, target_rate_stab.roll,  g_roll);
    calcPid(&pid_p_rate_stab, target_rate_stab.pitch, g_pitch);
    
    // ��������ƫ���ǣ�ƫ���ǽ�ʹ�ý��ٶȻ�
    calcPid(&pid_y_rate_stab, target_rate_stab.yaw,   g_yaw);

    *roll_out  = pid_r_rate_stab.output;
    *pitch_out = pid_p_rate_stab.output;
    *yaw_out   = pid_y_rate_stab.output;                          

    printf("{B%d:%d:%d:%d:%d}$",(int)g_yaw,(int)g_yaw,(int)g_yaw,(int)g_yaw,(int)g_yaw);                          
}

