#ifndef _LED_H
#define _LED_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK NANO STM32开发板
//LED驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2018/3/27
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2018-2028
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////  	

#define LEDR PCout(0)   	//LEDR
#define LEDG PCout(1)   	//LEDG
#define LEDB PCout(2)   	//LEDB

#define LED_RED_ON		LEDR=0
#define LED_RED_OFF		LEDR=1
#define LED_BLUE_ON		LEDB=0
#define LED_BLUE_OFF	LEDB=1
#define LED_GREEN_ON	LEDG=0
#define LED_GREEN_OFF	LEDG=1

void LED_Init(void);
#endif
