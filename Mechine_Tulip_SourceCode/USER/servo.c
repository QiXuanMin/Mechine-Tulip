#include "servo.h"
#include "pwm.h"
#include "delay.h"

#define ARR	1999
#define PSC 719



//��ʼ���������pwmƵ��Ϊ50hz
void servo_init()
{
	PWM_Init();
}


//�������
//����ֵpulseΪ�ߵ�ƽ����ʱ�䵥λus
void servo_control(u32 pulse)
{
	TIM3->CCR3=pulse; 
}

u32 flower_pulse=1650;
/**����
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
































