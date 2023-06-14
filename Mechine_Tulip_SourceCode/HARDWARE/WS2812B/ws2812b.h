#ifndef __WS2812B_H
#define __WS2812B_H

#include "sys.h"
#include "LCD12864.h"


/* 颜色表 */
#define LightPink			0xB6FFC1	//浅粉
#define Pink				0xC0FFCB	//粉色
#define Crimson				0x14FF93	//猩红
#define Orange				0xA5FF00	//橙色
#define Yellow				0xFFFF00	//黄色
#define Lime				0xFF0000	//酸柠绿
#define DarkCyan			0x8B008B	//深青色
#define RoyalBlue			0x6941E1	//皇家蓝
#define Violet				0x82EEEE	//紫罗兰
#define LavenderBlush		0xF0FFF5	//浅粉淡紫
#define MediumOrchid		0x55BAD3	//适中的兰花紫
#define GhostWhite			0xF8F8FF	//幽灵白
#define CornflowerBlue		0x9564ED	//矢车菊蓝
#define LightSkyBlue		0xCE87FA	//浅天蓝
#define PaleGreen			0xFB9898	//苍白的绿色







extern void ws2812b_init(void);
extern void ws2812b_show_rainbow(void);
extern void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle);
extern void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle);


#endif
