#include "paj7620u2.h"
#include "paj7620u2_cfg.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "key.h"
#include "servo.h"

extern void system_motion(void);
extern TaskTypeDef Task;

//选择PAJ7620U2 BANK区域
void paj7620u2_selectBank(bank_e bank)
{
	switch(bank)
	{
		case BANK0: GS_Write_Byte(PAJ_REGITER_BANK_SEL,PAJ_BANK0);break;//BANK0寄存器区域
		case BANK1: GS_Write_Byte(PAJ_REGITER_BANK_SEL,PAJ_BANK1);break;//BANK1寄存器区域
	}
}

//PAJ7620U2唤醒
u8 paj7620u2_wakeup(void)
{ 
	u8 data=0x0a;
	GS_WakeUp();//唤醒PAJ7620U2
	delay_ms(5);//唤醒时间>400us
	GS_WakeUp();//唤醒PAJ7620U2
	delay_ms(5);//唤醒时间>400us
	paj7620u2_selectBank(BANK0);//进入BANK0寄存器区域
	data = GS_Read_Byte(0x00);//读取状态
	if(data!=0x20) return 0; //唤醒失败
	
	return 1;
}

//PAJ7620U2初始化
//返回值：0:失败 1:成功
u8 paj7620u2_init(void)
{
	u8 i;
	u8 status;
	
	GS_i2c_init();//IIC初始化
    status = paj7620u2_wakeup();//唤醒PAJ7620U2
	if(!status) return 0;
	paj7620u2_selectBank(BANK0);//进入BANK0寄存器区域
	for(i=0;i<INIT_SIZE;i++)
	{
		GS_Write_Byte(init_Array[i][0],init_Array[i][1]);//初始化PAJ7620U2
	}
    paj7620u2_selectBank(BANK0);//切换回BANK0寄存器区域
	
	return 1;
}


//手势识别测试
void Gesture_Detection(void)
{
	u8 i;
    u8 status;
	u8 data[2]={0x00};
	u16 gesture_data;
	paj7620u2_selectBank(BANK0);//进入BANK0寄存器区域
	for(i=0;i<GESTURE_SIZE;i++)
	{
		GS_Write_Byte(gesture_arry[i][0],gesture_arry[i][1]);//手势识别模式初始化
	}
	paj7620u2_selectBank(BANK0);//切换回BANK0寄存器区域
	i=0;
	while(1)
	{
        status = GS_Read_nByte(PAJ_GET_INT_FLAG1,2,&data[0]);//读取手势状态			
		if(!status)
		{
			gesture_data =(u16)data[1]<<8 | data[0];
			if(gesture_data) 
			{
				switch(gesture_data)
				{
					case GES_UP:
					{
						Task=GS_UP;
						LED_BLUE_ON;
						break;//向上
					}
					case GES_DOWM:
					{
						
						Task=GS_DOWN;
						LED_BLUE_OFF;
						break;//向下
					}
					case GES_LEFT:
					{
						
						Task=GS_LEFT;
						LED_RED_ON;
						break; //向左
					}
					case GES_RIGHT:
					{
						
						Task=GS_RIGHT;
						LED_RED_OFF;
						break; //向右
					}
					case GES_FORWARD:
					{
						break;//向内
					}
					case GES_BACKWARD:
					{
						break;//向外
					}
					case GES_CLOCKWISE:
					{
						
						Task=GS_CLOCKWISE;
						LED_GREEN_ON;
						break;//顺时针
					}
					case GES_COUNT_CLOCKWISE:
					{
						
						Task=GS_ANTICLOCKWISE;
						LED_GREEN_OFF;
						break;//逆时针
					}
					case GES_WAVE:
					{
						break;//挥手
					}
					default:
					{
						
					}
				}				
			}
		}
		delay_ms(50);
		i++;
		if(i==5)
		{
			i=0;
		}		   
		system_motion();
	}
}

