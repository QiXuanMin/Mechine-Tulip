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

unsigned char zibi[]="又合上了...";
unsigned char huakai[]="花开啦";
unsigned char xideng[]="熄灯啦";
unsigned char kaideng[]="开灯啦";

unsigned char jinxiangei[]="仅献给";
unsigned char wozuiaideren[]="我最爱的人";
unsigned char yongjiubaoxiu[]="永久保修";
unsigned char dianhua[]="电话18267316310";
unsigned char zhizuozhe[]="制作者";
unsigned char minqixuan[]="闵启玄";
unsigned char yongyouzhe[]="拥有者";
unsigned char hetiantian[]="何湉湉";
unsigned char shijian[]="2020年5月20日";
unsigned char yuanwoneng[]="愿我能";
unsigned char xiangzheduohuayiban[]="像这朵花一般";
unsigned char zhaoliangni[]="照亮你";

TaskTypeDef Task=WAIT;

void OLED_Display()
{
	OLED_ShowText(42,2,yuanwoneng,0);
	OLED_ShowText(18,4,xiangzheduohuayiban,0);
	OLED_ShowText(42,6,zhaoliangni,0);
}


void hardware_init()
{
    HAL_Init();                    	//初始化HAL库    
    Stm32_Clock_Init(RCC_PLL_MUL9); //设置时钟,72M
    delay_init(72);                 //初始化延时函数
	LED_Init();						//初始化RGB彩灯
	LCD_Init();						//OLED初始化
	relay_init();					//继电器初始化
	ws2812b_init();					//花芯灯初始化
	servo_init();					//舵机初始化
	paj7620u2_init();				//手势传感器初始化
	OLED_Display();
	delay_ms(3000);
	OLED_Refresh_Gram();						//OLED清屏
}





int display_order=0;

//系统动作函数
void system_motion()
{
	switch(Task)
	{
		case WAIT:
		{
			if(display_order==0)
			{
				//等待状态显示哥哥的签名
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
			OLED_Refresh_Gram();						//OLED清屏
			dis_bmp(64,72,gImage_clockwise,99);			//先显示手势
			delay_ms(500);								//延时500ms
			OLED_Refresh_Gram();						//OLED清屏
			OLED_ShowText(40,4,kaideng,0);				//OLED显示“开灯啦”
			delay_ms(1000);
			RELAY_ON;									//开灯
			OLED_Refresh_Gram();						//OLED清屏
			//ws2812b_show_rainbow();
			Task=WAIT;
			break;
		}
		case GS_ANTICLOCKWISE:
		{
		
			OLED_Refresh_Gram();						//OLED清屏
			dis_bmp(64,72,gImage_anticlockwise,99);		//先显示手势
			delay_ms(500);								//延时500ms
			OLED_Refresh_Gram();						//OLED清屏
			OLED_ShowText(40,4,xideng,0);				//OLED显示“熄灯啦”
			delay_ms(1000);
			RELAY_OFF;									//熄灯
			OLED_Refresh_Gram();						//OLED清屏
			Task=WAIT;
			break;
		}
		case GS_UP:
		{
		
			OLED_Refresh_Gram();						//OLED清屏
			dis_bmp(64,72,gImage_up,99);				//先显示手势
			delay_ms(500);								//延时500ms
			OLED_Refresh_Gram();						//OLED清屏
			OLED_ShowText(35,4,huakai,0);				//OLED显示“花开啦”
			bloom();									//开花
			OLED_Refresh_Gram();						//OLED清屏
			Task=WAIT;
			break;
		}
		case GS_DOWN:
		{
		
			OLED_Refresh_Gram();						//OLED清屏
			dis_bmp(64,72,gImage_down,99);				//先显示手势
			delay_ms(500);								//延时500ms
			OLED_Refresh_Gram();						//OLED清屏
			OLED_ShowText(20,4,zibi,0);					//OLED显示“又合上了”
			fade();										//闭花
			OLED_Refresh_Gram();						//OLED清屏
			Task=WAIT;
			break;
		}
		case GS_LEFT:
		{
			OLED_Refresh_Gram();						//OLED清屏
			dis_bmp(64,72,gImage_left,99);				//先显示手势
			delay_ms(500);								//延时500ms
			OLED_Refresh_Gram();						//OLED清屏
														//OLED显示“切换颜色中”
			display_order--;							//切换显示图片
			OLED_Refresh_Gram();						//OLED清屏
			Task=WAIT;
			break;
		}
		case GS_RIGHT:
		{
			OLED_Refresh_Gram();						//OLED清屏
			dis_bmp(64,72,gImage_right,99);				//先显示手势
			delay_ms(500);								//延时500ms
			OLED_Refresh_Gram();						//OLED清屏
														//OLED显示“切换颜色中”
			display_order++;							//切换显示图片
			OLED_Refresh_Gram();						//OLED清屏
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






//DMA中断和大部分函数不能同时使用，注意在调用其他任务的时候关闭DMA中断，在切换颜色时开启中断。
int main(void)
{
	hardware_init();
	while(1)
	{
		Gesture_Detection();
		
	}

}

