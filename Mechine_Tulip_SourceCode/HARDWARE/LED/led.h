#ifndef _LED_H
#define _LED_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK NANO STM32������
//LED��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2018/3/27
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2018-2028
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
