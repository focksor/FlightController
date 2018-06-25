#include <imu.h>

const float r2d = 180.0f / PI;      // 弧度转角度
const float d2r = PI / 180.0f;      // 角度转弧度

const float acc_range  = 4 * 9.8;   // m * s^-2
const float gyro_range = 1000;      // dps 

double b_IIR[ACC_IIR_ORDER+1] ={0.0008f, 0.0032f, 0.0048f, 0.0032f, 0.0008f};  //系数b
double a_IIR[ACC_IIR_ORDER+1] ={1.0000f, -3.0176f, 3.5072f, -1.8476f, 0.3708f};//系数a
double InPut_IIR[3][ACC_IIR_ORDER+1] = {0};
double OutPut_IIR[3][ACC_IIR_ORDER+1] = {0};

__imu IMU, IMU_last;
__imu imu_dmp;

__vector3f acc_raw,  acc,  acc_last;        // m * s^-2
__vector3f gyro_raw, gyro, gyro_last;       // dps, degree per second
__vector3f gyro_idle = { 0 };               // 陀螺仪静态偏置, dps

bool update_IMU_Data(void) {
        
    short ax, ay, az, ax_iir, ay_iir, az_iir;
    short gx, gy, gz;
    
    if (mpu_dmp_get_data(&imu_dmp.roll, &imu_dmp.pitch, &imu_dmp.yaw) == 0) {   // 这里MPU6050的安装方向有点毛病，所以要对调Pitch和Roll
        
        MPU_Get_Accelerometer(&ax, &ay, &az);	// 得到加速度传感器数据
        MPU_Get_Gyroscope(&gx, &gy, &gz);	    // 得到陀螺仪数据
                       
        // 加速度计IIR滤波
        ax_iir = IIR_I_Filter((double)ax, InPut_IIR[0], OutPut_IIR[0], b_IIR, ACC_IIR_ORDER+1, a_IIR, ACC_IIR_ORDER+1);
        ay_iir = IIR_I_Filter((double)ay, InPut_IIR[1], OutPut_IIR[1], b_IIR, ACC_IIR_ORDER+1, a_IIR, ACC_IIR_ORDER+1);
        az_iir = IIR_I_Filter((double)az, InPut_IIR[2], OutPut_IIR[2], b_IIR, ACC_IIR_ORDER+1, a_IIR, ACC_IIR_ORDER+1);
	
        // 加速度计AD转换
        acc_raw.x  = (float)ax_iir / 32768. * acc_range;
        acc_raw.y  = (float)ay_iir / 32768. * acc_range;
        acc_raw.z  = (float)az_iir / 32768. * acc_range;
        
        acc_last = acc;
        acc = acc_raw;                          // 更新数据
        
        
        // 陀螺仪AD转换
        gyro_raw.x = (float)gx / 32768. * gyro_range - gyro_idle.x;
        gyro_raw.y = (float)gy / 32768. * gyro_range - gyro_idle.y;
        gyro_raw.z = (float)gz / 32768. * gyro_range - gyro_idle.z;
                
        // 陀螺仪一阶低通滤波
        gyro_last = gyro;                       // 更新数据
        gyro.x = LPF_1st(gyro_last.x, gyro_raw.x, 0.386f);
        gyro.y = LPF_1st(gyro_last.y, gyro_raw.y, 0.386f);      
        gyro.z = LPF_1st(gyro_last.z, gyro_raw.z, 0.386f);	
        
        // 旋转修正
        fix_IMU_Orentation();
        
        // 更新IMU角度输出
        // 这里为了方便，暂时直接使用DMP输出
        IMU_last = IMU;
        IMU = imu_dmp;
                
        return true;
    }
    
    return false;
}

void fix_IMU_Orentation(void) {
    float temp;
    
    // 陀螺仪roll, pitch对掉
    temp   = gyro.x;
    gyro.x = gyro.y;
    gyro.y = temp;
    
    // 这里有个很奇怪的现象，如果从改参数极性和改PID极性选择，这里是改PID极性会让飞机更加稳定，所以以下修改不采用
    // 陀螺仪pitch极性调整
    //gyro.y = -gyro.y;
    
    // 陀螺仪yaw极性调整
    //gyro.z = -gyro.z;
    
}

void calc_Gyro_Offset(void) {
    
    const float gyro_gather = 150;                 // 30
    
	static u8 over_flag=0;
	u8  i,cnt_g = 0;

    int16_t gx, gy, gz;
    int16_t gx_last=0,gy_last=0,gz_last=0;
	int16_t Integral[3] = {0,0,0};
	int32_t tempg[3]={0,0,0};
    
    over_flag=0;//因为定义的是static，如果不自己赋值，下次进来时over_flag就不会被赋值0了，保持为上一次校准完时赋值的1
	delay_ms(1500);
	
	while(!over_flag)	//此循环是确保四轴处于完全静止状态
	{
		if(cnt_g < 200)
		{
            MPU_Get_Gyroscope(&gx, &gy, &gz);	    // 得到陀螺仪数据

			tempg[0] += gx;
			tempg[1] += gy;
			tempg[2] += gz;

			Integral[0] += abs(gx_last - gx);
			Integral[1] += abs(gy_last - gy);
			Integral[2] += abs(gz_last - gz);

			gx_last = gx;
			gy_last = gy;
			gz_last = gz;
            
            delay_ms(1);
		}
		else{
			// 未校准成功
			if(Integral[0]>= gyro_gather || Integral[1]>= gyro_gather || Integral[2]>= gyro_gather){
				cnt_g = 0;
				for(i=0;i<3;i++){
					tempg[i]=Integral[i]=0;
				}
			}
			// 校准成功 
			else{				
                gyro_idle.x = (float)tempg[0]/cnt_g;
                gyro_idle.y = (float)tempg[1]/cnt_g;
                gyro_idle.z = (float)tempg[2]/cnt_g;
                
                gyro_idle.x = gyro_idle.x / 32768. * gyro_range;
                gyro_idle.y = gyro_idle.y / 32768. * gyro_range;
                gyro_idle.z = gyro_idle.z / 32768. * gyro_range;
                
                over_flag = 1;
			}
		}
		cnt_g++;
	}

}
