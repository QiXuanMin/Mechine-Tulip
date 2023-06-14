#ifndef __WS2812B_H
#define __WS2812B_H

#include "sys.h"
#include "LCD12864.h"


/* ��ɫ�� */
#define LightPink			0xB6FFC1	//ǳ��
#define Pink				0xC0FFCB	//��ɫ
#define Crimson				0x14FF93	//�ɺ�
#define Orange				0xA5FF00	//��ɫ
#define Yellow				0xFFFF00	//��ɫ
#define Lime				0xFF0000	//������
#define DarkCyan			0x8B008B	//����ɫ
#define RoyalBlue			0x6941E1	//�ʼ���
#define Violet				0x82EEEE	//������
#define LavenderBlush		0xF0FFF5	//ǳ�۵���
#define MediumOrchid		0x55BAD3	//���е�������
#define GhostWhite			0xF8F8FF	//�����
#define CornflowerBlue		0x9564ED	//ʸ������
#define LightSkyBlue		0xCE87FA	//ǳ����
#define PaleGreen			0xFB9898	//�԰׵���ɫ







extern void ws2812b_init(void);
extern void ws2812b_show_rainbow(void);
extern void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle);
extern void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle);


#endif
