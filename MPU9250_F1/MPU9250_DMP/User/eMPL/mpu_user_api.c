#include "mpu_user_api.h"
#include "stm32_iic.h"

int16_t Direction;
float Pitch,Roll,Yaw;
unsigned char BUF[10];       //接收数据缓存区
short T_X,T_Y,T_Z;		 //X,Y,Z轴，温度
 
//声明相关变量
unsigned long sensor_timestamp;
short gyro[3], accel[3], sensors;
unsigned char more;
long quat[4];
//误差纠正
#define  Pitch_error  1.0
#define  Roll_error   -2.0
#define  Yaw_error    0.0


static struct hal_s hal = {0};
volatile unsigned char rx_new;
float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};

																					 
																					 
																					 

/* These next two functions converts the orientation matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from Invensense's MPL.
 */
static  unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}



static  unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}



 /*自检函数*/
static void run_self_test(void)
{
    int result;

    long gyro[3], accel[3];

    result = mpu_run_self_test(gyro, accel);
    if (result == 0x7) 
    {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
	//	printf("setting bias succesfully ......\n");
    }
	else
	{
	//	printf("bias has not been modified ......\n");
	}
    
}
 

void Delayms(vu32 m)
{
  u32 i;
  
  for(; m != 0; m--)	
       for (i=0; i<50000; i++);
}

void Mpu_Init(u8 mode)
{
		u8 result = mpu_init();
		i2cWrite(GYRO_ADDRESS,0x37,0x02);//turn on Bypass Mode 
		Delayms(10);
    if(!result)   //返回0代表初始化成功
    {   
        //printf("mpu initialization complete......\n ");
        
        //mpu_set_sensor
        if(!mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL))
        {
            //printf("mpu_set_sensor complete ......\n");
        }
        else
        {
            //printf("mpu_set_sensor come across error ......\n");
        }
        
        //mpu_configure_fifo
        if(!mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL))
        {
            //printf("mpu_configure_fifo complete ......\n");
        }
        else
        {
            //printf("mpu_configure_fifo come across error ......\n");
        }
        
        //mpu_set_sample_rate
        if(!mpu_set_sample_rate(DEFAULT_MPU_HZ))
        {
            //printf("mpu_set_sample_rate complete ......\n");
        }
        else
        {
            //printf("mpu_set_sample_rate error ......\n");
        }
        
        //dmp_load_motion_driver_firmvare
        if(!dmp_load_motion_driver_firmware())
        {
            //printf("dmp_load_motion_driver_firmware complete ......\n");
        }
        else
        {
            //printf("dmp_load_motion_driver_firmware come across error ......\n");
        }
        
        //dmp_set_orientation
        if(!dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)))
        {
            //printf("dmp_set_orientation complete ......\n");
        }
        else
        {
            //printf("dmp_set_orientation come across error ......\n");
        }
        
        //dmp_enable_feature
        if(!dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
            DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
            DMP_FEATURE_GYRO_CAL))
        {
            //printf("dmp_enable_feature complete ......\n");
        }
        else
        {
            //printf("dmp_enable_feature come across error ......\n");
        }
        
        //dmp_set_fifo_rate
        if(!dmp_set_fifo_rate(DEFAULT_MPU_HZ))
        {
           //printf("dmp_set_fifo_rate complete ......\n");
        }
        else
        {
            //printf("dmp_set_fifo_rate come across error ......\n");
        }
        
        //不开自检，以水平作为零度
        //开启自检以当前位置作为零度
				if(mode==1)
					run_self_test();
        
        if(!mpu_set_dmp_state(1))
        {
            //printf("mpu_set_dmp_state complete ......\n");
        }
        else
        {
            //printf("mpu_set_dmp_state come across error ......\n");
        }
        
    }
}

u8 Update_attitude_Angle(void)
{
	//float Yaw,Roll,Pitch;
        dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);	 
        /* Gyro and accel data are written to the FIFO by the DMP in chip
        * frame and hardware units. This behavior is convenient because it
        * keeps the gyro and accel outputs of dmp_read_fifo and
        * mpu_read_fifo consistent.
        */
        /*     if (sensors & INV_XYZ_GYRO )
        send_packet(PACKET_TYPE_GYRO, gyro);
        if (sensors & INV_XYZ_ACCEL)
        send_packet(PACKET_TYPE_ACCEL, accel); */
        /* Unlike gyro and accel, quaternions are written to the FIFO in
        * the body frame, q30. The orientation is set by the scalar passed
        * to dmp_set_orientation during initialization.
        */
				/*四元数解姿态*/
        if (sensors & INV_WXYZ_QUAT )
        {
            q0 = quat[0] / q30;
            q1 = quat[1] / q30;
            q2 = quat[2] / q30;
            q3 = quat[3] / q30;
            
            Pitch  = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3 + Pitch_error; // pitch
            Roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3 + Roll_error; // roll
            Yaw = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3 + Yaw_error;
						return 0;
        }
				return 1;

}
void Update_Magnetometer(void)
{
//		 i2cWrite(GYRO_ADDRESS,PWR_MGMT_1, 0x00);	//解除休眠状态
//	i2cWrite(GYRO_ADDRESS,SMPLRT_DIV, 0x07);
//	i2cWrite(GYRO_ADDRESS,CONFIG, 0x06);
//	i2cWrite(GYRO_ADDRESS,GYRO_CONFIG, 0x18);
//	i2cWrite(GYRO_ADDRESS,ACCEL_CONFIG, 0x01);
   
   i2cWrite(MAG_ADDRESS,0x0A,0x01);
   Delayms(10);	
   i2cRead (MAG_ADDRESS,MAG_XOUT_L,1,BUF);
   i2cRead (MAG_ADDRESS,MAG_XOUT_H,1,BUF+1);
   T_X=(BUF[1]<<8)|BUF[0];

   i2cRead(MAG_ADDRESS,MAG_YOUT_L,1,BUF+2);
   i2cRead(MAG_ADDRESS,MAG_YOUT_H,1,BUF+3);
   T_Y=	(BUF[3]<<8)|BUF[2];
   						   //读取计算Y轴数据
	 
   i2cread(MAG_ADDRESS,MAG_ZOUT_L,1,BUF+4);
   i2cread(MAG_ADDRESS,MAG_ZOUT_H,1,BUF+5);
   T_Z=	(BUF[5]<<8)|BUF[4];
 					       //读取计算Z轴数据

	 Direction=	atan2( 
									(double) (   (int16_t)   (T_X +0)   ),
									(double) (   (int16_t)   (T_Y -0)  )
								)*(180/3.14159265)+180;

}

