#include "led.h"
#include "delay.h"
#include "sys.h"
#include "key.h"
#include "paj7620u2.h"
#include "LCD12864.h"
#include "relay.h"
#include "bmplib.h"
#include "ws2812b.h"
#include "servo.h"
#include "pwm.h"

unsigned char zibi[]="�ֺ�����...";
unsigned char huakai[]="������";
unsigned char xideng[]="Ϩ����";
unsigned char kaideng[]="������";

unsigned char jinxiangei[]="���׸�";
unsigned char wozuiaideren[]="�������";
unsigned char yongjiubaoxiu[]="���ñ���";
unsigned char dianhua[]="�绰18267316310";
unsigned char zhizuozhe[]="������";
unsigned char minqixuan[]="������";
unsigned char yongyouzhe[]="ӵ����";
unsigned char hetiantian[]="�Μ���";
unsigned char shijian[]="2020��5��20��";
unsigned char yuanwoneng[]="Ը����";
unsigned char xiangzheduohuayiban[]="����仨һ��";
unsigned char zhaoliangni[]="������";

TaskTypeDef Task=WAIT;

void OLED_Display()
{
	OLED_ShowText(42,2,yuanwoneng,0);
	OLED_ShowText(18,4,xiangzheduohuayiban,0);
	OLED_ShowText(42,6,zhaoliangni,0);
}


void hardware_init()
{
    HAL_Init();                    	//��ʼ��HAL��    
    Stm32_Clock_Init(RCC_PLL_MUL9); //����ʱ��,72M
    delay_init(72);                 //��ʼ����ʱ����
	LED_Init();						//��ʼ��RGB�ʵ�
	LCD_Init();						//OLED��ʼ��
	relay_init();					//�̵�����ʼ��
	ws2812b_init();					//��о�Ƴ�ʼ��
	servo_init();					//�����ʼ��
	paj7620u2_init();				//���ƴ�������ʼ��
	OLED_Display();
	delay_ms(3000);
	OLED_Refresh_Gram();						//OLED����
}





int display_order=0;

//ϵͳ��������
void system_motion()
{
	switch(Task)
	{
		case WAIT:
		{
			if(display_order==0)
			{
				//�ȴ�״̬��ʾ����ǩ��
				dis_bmp(64,111,gImage_leslie,99);
			}
			else if(display_order==3)
			{
				OLED_ShowText(30,2,yongjiubaoxiu,0);
				OLED_ShowText(0,4,dianhua,0);
			}
			else if(display_order==2)
			{
				OLED_ShowText(40,2,jinxiangei,0);
				OLED_ShowText(22,4,wozuiaideren,0);
			}
			else if(display_order==1)
			{
				OLED_ShowText(0,2,yongyouzhe,0);
				OLED_ShowText(60,2,hetiantian,0);
				OLED_ShowText(0,4,zhizuozhe,0);
				OLED_ShowText(60,4,minqixuan,0);
				OLED_ShowText(10,6,shijian,0);

			}
			break;
		}
		case GS_CLOCKWISE:
		{
			OLED_Refresh_Gram();						//OLED����
			dis_bmp(64,72,gImage_clockwise,99);			//����ʾ����
			delay_ms(500);								//��ʱ500ms
			OLED_Refresh_Gram();						//OLED����
			OLED_ShowText(40,4,kaideng,0);				//OLED��ʾ����������
			delay_ms(1000);
			RELAY_ON;									//����
			OLED_Refresh_Gram();						//OLED����
			//ws2812b_show_rainbow();
			Task=WAIT;
			break;
		}
		case GS_ANTICLOCKWISE:
		{
		
			OLED_Refresh_Gram();						//OLED����
			dis_bmp(64,72,gImage_anticlockwise,99);		//����ʾ����
			delay_ms(500);								//��ʱ500ms
			OLED_Refresh_Gram();						//OLED����
			OLED_ShowText(40,4,xideng,0);				//OLED��ʾ��Ϩ������
			delay_ms(1000);
			RELAY_OFF;									//Ϩ��
			OLED_Refresh_Gram();						//OLED����
			Task=WAIT;
			break;
		}
		case GS_UP:
		{
		
			OLED_Refresh_Gram();						//OLED����
			dis_bmp(64,72,gImage_up,99);				//����ʾ����
			delay_ms(500);								//��ʱ500ms
			OLED_Refresh_Gram();						//OLED����
			OLED_ShowText(35,4,huakai,0);				//OLED��ʾ����������
			bloom();									//����
			OLED_Refresh_Gram();						//OLED����
			Task=WAIT;
			break;
		}
		case GS_DOWN:
		{
		
			OLED_Refresh_Gram();						//OLED����
			dis_bmp(64,72,gImage_down,99);				//����ʾ����
			delay_ms(500);								//��ʱ500ms
			OLED_Refresh_Gram();						//OLED����
			OLED_ShowText(20,4,zibi,0);					//OLED��ʾ���ֺ����ˡ�
			fade();										//�ջ�
			OLED_Refresh_Gram();						//OLED����
			Task=WAIT;
			break;
		}
		case GS_LEFT:
		{
			OLED_Refresh_Gram();						//OLED����
			dis_bmp(64,72,gImage_left,99);				//����ʾ����
			delay_ms(500);								//��ʱ500ms
			OLED_Refresh_Gram();						//OLED����
														//OLED��ʾ���л���ɫ�С�
			display_order--;							//�л���ʾͼƬ
			OLED_Refresh_Gram();						//OLED����
			Task=WAIT;
			break;
		}
		case GS_RIGHT:
		{
			OLED_Refresh_Gram();						//OLED����
			dis_bmp(64,72,gImage_right,99);				//����ʾ����
			delay_ms(500);								//��ʱ500ms
			OLED_Refresh_Gram();						//OLED����
														//OLED��ʾ���л���ɫ�С�
			display_order++;							//�л���ʾͼƬ
			OLED_Refresh_Gram();						//OLED����
			Task=WAIT;
			break;
		}
		default:
		{
		}
	}
	if(display_order==4)
	{
		display_order=0;
	}
	if(display_order==-1)
	{
		display_order=3;
	}
}






//DMA�жϺʹ󲿷ֺ�������ͬʱʹ�ã�ע���ڵ������������ʱ��ر�DMA�жϣ����л���ɫʱ�����жϡ�
int main(void)
{
	hardware_init();
	while(1)
	{
		Gesture_Detection();
		
	}

}

