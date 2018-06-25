/******************* (C) COPYRIGHT 2016 Hentotech Team ***************************
 * �ļ���   ��Algorithm_filter.c
 * ����     �����κ���      
 * ʵ��ƽ̨ ��HT-Hawk��Դ�ɿ�
 * ��汾   ��ST3.5.0
 * ����     ��Hentotech Team 
 * ����     ��http://www.hentotech.com 
 * ��̳     ��http://bbs.hentotech.com
 * �̳�     ��http://hentotech.taobao.com   
*********************************************************************************/
#include "config.h"
#include "include/filter.h"



/*====================================================================================================*/
/*====================================================================================================*
** ��������: IIR_I_Filter
** ��������: IIRֱ��I���˲���
** ��    ��: InData Ϊ��ǰ����
**           *x     ����δ�˲�������
**           *y     �����˲��������
**           *b     ����ϵ��b
**           *a     ����ϵ��a
**           nb     ����*b�ĳ���
**           na     ����*a�ĳ���
**           LpfFactor
** ��    ��: OutData         
** ˵    ��: ��
** ����ԭ��: y(n) = b0*x(n) + b1*x(n-1) + b2*x(n-2) -
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
**���� : KalmanFilter
**���� : �������˲�
**���� :  
**ݔ�� : None
**��ע : None
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
   static double x_last,p_last;//ԭ��������ʽ�������
   x_mid=x_last;          //x_last=x(k-1|k-1),x_mid=x(k|k-1)
   p_mid=p_last+Q;        //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=����
   kg=p_mid/(p_mid+R);    //kgΪkalman filter��RΪ����
   x_now=x_mid+kg*(ResrcData-x_mid);//���Ƴ�������ֵ
                
   p_now=(1-kg)*p_mid;   //����ֵ��Ӧ��covariance       
   p_last = p_now;       //����covarianceֵ
   x_last = x_now;       //����ϵͳ״ֵ̬
   return x_now;                
}
/*====================================================================================================*/
/*====================================================================================================*
**���� : LPF_1st
**���� : һ�׵�ͨ�˲�
**���� :  
**ݔ�� : None
**��ע : None
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

//float gyro_m�������ǲ�õ��������ٶȣ�  
//float incAngle:�ӼƲ�õĽǶ�ֵ  
#define dt                  0.0025//�������˲�����Ƶ��   
#define R_angle          0.69 //����������Э������ǲ���ƫ� 
#define Q_angle          0.005//����������Э����   
#define Q_gyro         0.001 //����������Э����  ��������Э����Ϊһ��һ�����о��� 
float kalmanUpdate(const float gyro_m,const float incAngle)    
{  
	  float K_0;//���п��������������һ�����������ڼ������Ź���ֵ         
		float K_1;//���п���������ĺ��������ڼ������Ź���ֵ��ƫ��         
		float Y_0;         
		float Y_1;            
			float Rate;//ȥ��ƫ���Ľ��ٶ�           
		float Pdot[4];//����Э��������΢�־���          
		float angle_err;//�Ƕ�ƫ��         
		float E;//����Ĺ�����          
		static float angle = 0;            //��ʱ�����Ź���ֵ�Ƕ�          
		static float q_bias = 0;        //�����ǵ�ƫ��                          
		static float P[2][2] = {{ 1, 0 }, { 0, 1 }};//����Э�������         
		Rate = gyro_m - q_bias;          //�������Э��������΢�־���               
		Pdot[0] = Q_angle - P[0][1] - P[1][0];//������������                  
		Pdot[1] = - P[1][1];                                   
		Pdot[2] = - P[1][1];                                           
		Pdot[3] = Q_gyro;//������������                                  
		angle += Rate * dt; //���ٶȻ��ֵó��Ƕ�         
		P[0][0] += Pdot[0] * dt; //����Э�������         
		P[0][1] += Pdot[1] * dt;          
		P[1][0] += Pdot[2] * dt;          
		P[1][1] += Pdot[3] * dt;            
		angle_err = incAngle - angle; //����Ƕ�ƫ��          
		E = R_angle + P[0][0];          
		K_0 = P[0][0] / E; //���㿨��������         
		K_1 = P[1][0] / E;           
		Y_0 = P[0][0];            
		Y_1 = P[0][1];             
		P[0][0] -= K_0 * Y_0; //����Э�������         
		P[0][1] -= K_0 * Y_1;          
		P[1][0] -= K_1 * Y_0;           
		P[1][1] -= K_1 * Y_1;          
		angle += K_0 * angle_err; //�������Ź���ֵ         
		q_bias += K_1 * angle_err;//�������Ź���ֵƫ��          
		return angle;
}
