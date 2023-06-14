#ifndef __LCD12864_H
#define __LCD12864_H

#include "sys.h"

#define uint16 u16
#define uint8  u8

#define SLCD_SCL PAout(4)=1  //OLED D0
#define RLCD_SCL PAout(4)=0
#define SLCD_SDA PAout(5)=1  //OLED D1
#define RLCD_SDA PAout(5)=0
#define SLCD_RST PAout(6)=1  //OLED RST
#define RLCD_RST PAout(6)=0
#define SLCD_DC  PAout(7)=1  //OLED DC
#define RLCD_DC  PAout(7)=0

typedef struct
{
	char name[3];
	char dat[32];
}chinese;

extern void LCD_Init(void);
extern void LCD_CLS(void);
extern void LCD_P6x8Str(unsigned char x,unsigned char y,unsigned char ch[]);
extern void LCD_P8x16Str(unsigned char x,unsigned char y,unsigned char ch[]);
extern void LCD_P14x16Str(unsigned char x,unsigned char y,unsigned char ch[]);
extern void LCD_Print(unsigned char x, unsigned char y, unsigned char ch[]);
extern void LCD_PutPixel(unsigned char x,unsigned char y);
extern void LCD_Rectangle(unsigned char x1,unsigned char y1,unsigned char x2,unsigned char y2,unsigned char gif);
extern void Draw_LQLogo(void);
extern void Draw_LibLogo(void);
extern void Draw_BMP(unsigned char x0,unsigned char y0,unsigned char x1,unsigned char y1,const unsigned char * bmp);
extern void Draw_Road(void);
extern void dis_bmp(uint16 high, uint16 width,const unsigned char *p,uint8 value);
extern void LCD_PrintU16(unsigned char x,unsigned char y,unsigned int num);
extern void LCD_Print16(unsigned char x,unsigned char y,int num);
extern void OLED_Refresh_Gram(void);
extern void LCD_Show_Number3 (uint8 X,uint8 Y,uint16 number) ;
extern void OLED_PrintFloat(unsigned char x , unsigned char y , float num) ;
extern void OLED_ShowText(u8 x,u8 y,u8* str,u8 flag);
#endif

