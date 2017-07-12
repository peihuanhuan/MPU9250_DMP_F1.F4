#ifndef __STM32_I2C_H
#define __STM32_I2C_H

#include "stm32f4xx.h"
#include "Systick.h"

/*****************You only need to change the following four macro definitions************/

#define I2C_PORT GPIOB
#define I2C_SCL GPIO_Pin_6
#define I2C_SDA GPIO_Pin_7
#define RCC_I2C_PORT RCC_AHB1Periph_GPIOB

/*****************All you need to do is change the four macro definitions above************/





#define SCL_L (I2C_PORT->BSRRH|=I2C_SCL)
#define SCL_H (I2C_PORT->BSRRL|=I2C_SCL)
#define SDA_L (I2C_PORT->BSRRH|=I2C_SDA)
#define SDA_H (I2C_PORT->BSRRL|=I2C_SDA)

#define  SCL_read     (I2C_PORT->IDR&I2C_SCL) 
#define  SDA_read     (I2C_PORT->IDR&I2C_SDA)

#define CLI()      __set_PRIMASK(1)  
#define SEI()      __set_PRIMASK(0)

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

#define true 1
#define false 0 
#define bool  uint8_t

#define TRUE  0
#define FALSE -1


//0表示写
#define	I2C_Direction_Transmitter   0
//１表示读
#define	I2C_Direction_Receiver      1	 





bool i2cWriteBuffer(uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data);
bool i2cWrite(uint8_t addr_, uint8_t reg_, uint8_t data);
bool i2cRead(uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf);
void i2cInit(void);

uint16_t i2cGetErrorCounter(void);
static void i2cUnstick(void);

int8_t i2cwrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t * data);
int8_t i2cread(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);




#endif


