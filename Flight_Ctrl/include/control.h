#ifndef __Control_H
#define __Control_H

#include <stm32f10x.h>
#include <config.h>
#include <imu.h>

#include <motor.h>

void init_Control(void);
void control_All(__flight_mode_t flight_mode,
                 short *motor_1, short *motor_2, short *motor_3, short *motor_4);

bool control_Safety(void);

// Acro 特技模式
bool init_Ctrl_Acro(void);
void calc_Target_Acro(__rc_channel _rc);
void calc_Throttle_Acro(__rc_channel _rc, short *motor_out);
void control_PID_Acro(float g_roll, float g_pitch, float g_yaw,
                      float *roll_out, float *pitch_out, float *yaw_out);

// Stabilized 自稳模式
bool init_Ctrl_Stabilized(void);
void calc_Target_Stabilized(__rc_channel _rc);
void calc_Throttle_Stabilized(__rc_channel _rc, short *motor_out);
void control_PID_Stabilized(float g_roll, float g_pitch, float g_yaw,
                            float roll,   float pitch,   float yaw,
                            float *roll_out, float *pitch_out, float *yaw_out);

extern const float d_time_attitude;

#endif  // __Control_H
