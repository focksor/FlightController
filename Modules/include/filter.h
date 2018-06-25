#ifndef __Algorithm_filter_H
#define	__Algorithm_filter_H

#include "stm32f10x.h"

double IIR_I_Filter(double InData, double *x, double *y, double *b, short nb, double *a, short na);
float LPF_1st(float oldData, float newData, float lpf_factor);
#endif /* __Algorithm_filter_H */

#ifndef __FILTER_H
#define __FILTER_H


//float Moving_Average(u8 item,u8 width_num,float in);
void Moving_Average(float in,float moavarray[],u16 len ,u16 fil_cnt[2],float *out);
float Moving_Median(u8 item,u8 width_num,float in);
float kalmanUpdate(const float gyro_m,const float incAngle);

#endif
