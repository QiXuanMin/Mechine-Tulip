#include "servo.h"
#include "pwm.h"
#include "delay.h"

#define ARR	1999
#define PSC 719



//初始化舵机控制pwm频率为50hz
void servo_init()
{
	PWM_Init();
}


//舵机控制
//给定值pulse为高电平持续时间单位us
void servo_control(u32 pulse)
{
	TIM3->CCR3=pulse; 
}

u32 flower_pulse=1650;
/**花开
*/
void bloom()
{
	while(flower_pulse>1000)
	{
		flower_pulse=flower_pulse-5;
		delay_ms(50);
		servo_control(flower_pulse);
	}
}


void fade()
{
	while(flower_pulse<=1650)
	{
		flower_pulse=flower_pulse+5;
		delay_ms(50);
		servo_control(flower_pulse);
	}
}
































