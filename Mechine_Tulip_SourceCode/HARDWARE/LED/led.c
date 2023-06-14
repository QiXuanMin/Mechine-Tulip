#include "led.h"
#include "stdbool.h"


//LED IO��ʼ��
void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;

    __HAL_RCC_GPIOC_CLK_ENABLE();           	//����GPIOCʱ��
	
    GPIO_Initure.Pin=GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  	//�������
    GPIO_Initure.Pull=GPIO_PULLUP;          	//����
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;    	 	//����
	
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2,GPIO_PIN_SET);	//Ĭ�ϳ�ʼ�������
	
    HAL_GPIO_Init(GPIOC,&GPIO_Initure);
	
}


