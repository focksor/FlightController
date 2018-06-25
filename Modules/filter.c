/******************* (C) COPYRIGHT 2016 Hentotech Team ***************************
 * 文件名   ：Algorithm_filter.c
 * 描述     ：波形函数      
 * 实验平台 ：HT-Hawk开源飞控
 * 库版本   ：ST3.5.0
 * 作者     ：Hentotech Team 
 * 官网     ：http://www.hentotech.com 
 * 论坛     ：http://bbs.hentotech.com
 * 商城     ：http://hentotech.taobao.com   
*********************************************************************************/
#include "config.h"
#include "include/filter.h"



/*====================================================================================================*/
/*====================================================================================================*
** 函数名称: IIR_I_Filter
** 功能描述: IIR直接I型滤波器
** 输    入: InData 为当前数据
**           *x     储存未滤波的数据
**           *y     储存滤波后的数据
**           *b     储存系数b
**           *a     储存系数a
**           nb     数组*b的长度
**           na     数组*a的长度
**           LpfFactor
** 输    出: OutData         
** 说    明: 无
** 函数原型: y(n) = b0*x(n) + b1*x(n-1) + b2*x(n-2) -
                    a1*y(n-1) - a2*y(n-2)
**====================================================================================================*/
/*====================================================================================================*/
double IIR_I_Filter(double InData, double *x, double *y, double *b, short nb, double *a, short na)
{
  double z1,z2;
  short i;
  double OutData;
  
  for(i=nb-1; i>0; i--)
  {
    x[i]=x[i-1];
  }
  
  x[0] = InData;
  
  for(z1=0,i=0; i<nb; i++)
  {
    z1 += x[i]*b[i];
  }
  
  for(i=na-1; i>0; i--)
  {
    y[i]=y[i-1];
  }
  
  for(z2=0,i=1; i<na; i++)
  {
    z2 += y[i]*a[i];
  }
  
  y[0] = z1 - z2; 
  OutData = y[0];
    
  return OutData;
}

/*====================================================================================================*/
/*====================================================================================================*
**函数 : KalmanFilter
**功能 : 卡尔曼滤波
**输入 :  
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
double KalmanFilter(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   static double x_mid ;//= x_last
   static double x_now;
   static double p_mid ;
   static double p_now;
   static double kg;        
   static double x_last,p_last;//原来是在形式参数里的
   x_mid=x_last;          //x_last=x(k-1|k-1),x_mid=x(k|k-1)
   p_mid=p_last+Q;        //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
   kg=p_mid/(p_mid+R);    //kg为kalman filter，R为噪声
   x_now=x_mid+kg*(ResrcData-x_mid);//估计出的最优值
                
   p_now=(1-kg)*p_mid;   //最优值对应的covariance       
   p_last = p_now;       //更新covariance值
   x_last = x_now;       //更新系统状态值
   return x_now;                
}
/*====================================================================================================*/
/*====================================================================================================*
**函数 : LPF_1st
**功能 : 一阶低通滤波
**输入 :  
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
float LPF_1st(float oldData, float newData, float lpf_factor)
{
	return oldData * (1 - lpf_factor) + newData * lpf_factor;
}
// #define WIDTH_NUM 101
// #define FIL_ITEM  10

void Moving_Average(float in,float moavarray[],u16 len ,u16 fil_cnt[2],float *out)
{
	u16 width_num;
	
	width_num = len ;
	
	if( ++fil_cnt[0] > width_num )	
	{
		fil_cnt[0] = 0; //now
		fil_cnt[1] = 1; //old
	}
	else
	{
		fil_cnt[1] = (fil_cnt[0] == width_num)? 0 : (fil_cnt[0] + 1);
	}
	
	moavarray[ fil_cnt[0] ] = in;
	*out += ( in - ( moavarray[ fil_cnt[1] ]  ) )/(float)( width_num ) ;
	
}

#define MED_WIDTH_NUM 11
#define MED_FIL_ITEM  4

float med_filter_tmp[MED_FIL_ITEM][MED_WIDTH_NUM ];
float med_filter_out[MED_FIL_ITEM];

u8 med_fil_cnt[MED_FIL_ITEM];

float Moving_Median(u8 item,u8 width_num,float in)
{
	u8 i,j;
	float t;
	float tmp[MED_WIDTH_NUM];
	
	if(item >= MED_FIL_ITEM || width_num >= MED_WIDTH_NUM )
	{
		return 0;
	}
	else
	{
		if( ++med_fil_cnt[item] >= width_num )	
		{
			med_fil_cnt[item] = 0;
		}
		
		med_filter_tmp[item][ med_fil_cnt[item] ] = in;
		
		for(i=0;i<width_num;i++)
		{
			tmp[i] = med_filter_tmp[item][i];
		}
		
		for(i=0;i<width_num-1;i++)
		{
			for(j=0;j<(width_num-1-i);j++)
			{
				if(tmp[j] > tmp[j+1])
				{
					t = tmp[j];
					tmp[j] = tmp[j+1];
					tmp[j+1] = t;
				}
			}
		}		
		return ( tmp[(u16)width_num/2] );
	}
}

//float gyro_m：陀螺仪测得的量（角速度）  
//float incAngle:加计测得的角度值  
#define dt                  0.0025//卡尔曼滤波采样频率   
#define R_angle          0.69 //测量噪声的协方差（即是测量偏差） 
#define Q_angle          0.005//过程噪声的协方差   
#define Q_gyro         0.001 //过程噪声的协方差  过程噪声协方差为一个一行两列矩阵 
float kalmanUpdate(const float gyro_m,const float incAngle)    
{  
	  float K_0;//含有卡尔曼增益的另外一个函数，用于计算最优估计值         
		float K_1;//含有卡尔曼增益的函数，用于计算最优估计值的偏差         
		float Y_0;         
		float Y_1;            
			float Rate;//去除偏差后的角速度           
		float Pdot[4];//过程协方差矩阵的微分矩阵          
		float angle_err;//角度偏量         
		float E;//计算的过程量          
		static float angle = 0;            //下时刻最优估计值角度          
		static float q_bias = 0;        //陀螺仪的偏差                          
		static float P[2][2] = {{ 1, 0 }, { 0, 1 }};//过程协方差矩阵         
		Rate = gyro_m - q_bias;          //计算过程协方差矩阵的微分矩阵               
		Pdot[0] = Q_angle - P[0][1] - P[1][0];//？？？？？？                  
		Pdot[1] = - P[1][1];                                   
		Pdot[2] = - P[1][1];                                           
		Pdot[3] = Q_gyro;//？？？？？？                                  
		angle += Rate * dt; //角速度积分得出角度         
		P[0][0] += Pdot[0] * dt; //计算协方差矩阵         
		P[0][1] += Pdot[1] * dt;          
		P[1][0] += Pdot[2] * dt;          
		P[1][1] += Pdot[3] * dt;            
		angle_err = incAngle - angle; //计算角度偏差          
		E = R_angle + P[0][0];          
		K_0 = P[0][0] / E; //计算卡尔曼增益         
		K_1 = P[1][0] / E;           
		Y_0 = P[0][0];            
		Y_1 = P[0][1];             
		P[0][0] -= K_0 * Y_0; //跟新协方差矩阵         
		P[0][1] -= K_0 * Y_1;          
		P[1][0] -= K_1 * Y_0;           
		P[1][1] -= K_1 * Y_1;          
		angle += K_0 * angle_err; //给出最优估计值         
		q_bias += K_1 * angle_err;//跟新最优估计值偏差          
		return angle;
}
