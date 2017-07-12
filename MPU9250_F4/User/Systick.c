#include "Systick.h"

//void delay_init(u8 SYSCLK)
//{
//	SysTick->CTRL&=0xfffffffb;//bit2清空,选择外部时钟  HCLK/8
//	fac_us=SYSCLK/8;		    
//	fac_ms=(u16)fac_us*1000;
//}		

void delay_ms(u32 ms)//毫秒级别延时最大798
{
	u32 tmp;
	SysTick->LOAD=21*ms*1000; //寄存器24位，最大 16777215
	SysTick->VAL=0x00;
	SysTick->CTRL=0x01;
	do{
		tmp=SysTick->CTRL;
	}while((tmp&0x01)&&!(tmp&(1<<16)));
	SysTick->LOAD=0x00;
	SysTick->VAL=0x00;
}

void delay_us(u32 us)//微秒级别延时
{
	u32 tmp;
	SysTick->LOAD=21*us;
	SysTick->VAL=0x00;//清空计数器
	SysTick->CTRL=0x01;//使能定时器
	do{
		tmp=SysTick->CTRL;
	}while((tmp&0x01)&&!(tmp&(1<<16)));
	SysTick->LOAD=0x00;
	SysTick->VAL=0x00;//清空计数器
}

