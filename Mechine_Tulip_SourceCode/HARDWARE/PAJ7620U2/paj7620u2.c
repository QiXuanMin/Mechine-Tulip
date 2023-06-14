#include "paj7620u2.h"
#include "paj7620u2_cfg.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "key.h"
#include "servo.h"

extern void system_motion(void);
extern TaskTypeDef Task;

//ѡ��PAJ7620U2 BANK����
void paj7620u2_selectBank(bank_e bank)
{
	switch(bank)
	{
		case BANK0: GS_Write_Byte(PAJ_REGITER_BANK_SEL,PAJ_BANK0);break;//BANK0�Ĵ�������
		case BANK1: GS_Write_Byte(PAJ_REGITER_BANK_SEL,PAJ_BANK1);break;//BANK1�Ĵ�������
	}
}

//PAJ7620U2����
u8 paj7620u2_wakeup(void)
{ 
	u8 data=0x0a;
	GS_WakeUp();//����PAJ7620U2
	delay_ms(5);//����ʱ��>400us
	GS_WakeUp();//����PAJ7620U2
	delay_ms(5);//����ʱ��>400us
	paj7620u2_selectBank(BANK0);//����BANK0�Ĵ�������
	data = GS_Read_Byte(0x00);//��ȡ״̬
	if(data!=0x20) return 0; //����ʧ��
	
	return 1;
}

//PAJ7620U2��ʼ��
//����ֵ��0:ʧ�� 1:�ɹ�
u8 paj7620u2_init(void)
{
	u8 i;
	u8 status;
	
	GS_i2c_init();//IIC��ʼ��
    status = paj7620u2_wakeup();//����PAJ7620U2
	if(!status) return 0;
	paj7620u2_selectBank(BANK0);//����BANK0�Ĵ�������
	for(i=0;i<INIT_SIZE;i++)
	{
		GS_Write_Byte(init_Array[i][0],init_Array[i][1]);//��ʼ��PAJ7620U2
	}
    paj7620u2_selectBank(BANK0);//�л���BANK0�Ĵ�������
	
	return 1;
}


//����ʶ�����
void Gesture_Detection(void)
{
	u8 i;
    u8 status;
	u8 data[2]={0x00};
	u16 gesture_data;
	paj7620u2_selectBank(BANK0);//����BANK0�Ĵ�������
	for(i=0;i<GESTURE_SIZE;i++)
	{
		GS_Write_Byte(gesture_arry[i][0],gesture_arry[i][1]);//����ʶ��ģʽ��ʼ��
	}
	paj7620u2_selectBank(BANK0);//�л���BANK0�Ĵ�������
	i=0;
	while(1)
	{
        status = GS_Read_nByte(PAJ_GET_INT_FLAG1,2,&data[0]);//��ȡ����״̬			
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
						break;//����
					}
					case GES_DOWM:
					{
						
						Task=GS_DOWN;
						LED_BLUE_OFF;
						break;//����
					}
					case GES_LEFT:
					{
						
						Task=GS_LEFT;
						LED_RED_ON;
						break; //����
					}
					case GES_RIGHT:
					{
						
						Task=GS_RIGHT;
						LED_RED_OFF;
						break; //����
					}
					case GES_FORWARD:
					{
						break;//����
					}
					case GES_BACKWARD:
					{
						break;//����
					}
					case GES_CLOCKWISE:
					{
						
						Task=GS_CLOCKWISE;
						LED_GREEN_ON;
						break;//˳ʱ��
					}
					case GES_COUNT_CLOCKWISE:
					{
						
						Task=GS_ANTICLOCKWISE;
						LED_GREEN_OFF;
						break;//��ʱ��
					}
					case GES_WAVE:
					{
						break;//����
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

