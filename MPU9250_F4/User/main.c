
#include "stm32f4xx.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "inv_mpu.h"
#include "math.h"
#include "usart.h"
#include "stm32_iic.h"
#include "stdio.h"
#include "mpu_user_api.h"



int main(void)
{  
    USART1_Config();  //串口初始化
    i2cInit();      //IIC总线的初始化
    delay_ms(10);
		Mpu_Init(1);


    while(1)
    {
        
        Update_attitude_Angle();

            printf("Pitch:");
            printf("%f ",Pitch);


            printf("Roll:");
            printf("%f ",Roll);     


            printf("Yaw:");        
            printf("%f ",Yaw);
  
				
						Update_Magnetometer();
						printf("M:%d \n",Direction);
    }
}

 

