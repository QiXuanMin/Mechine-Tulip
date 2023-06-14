#ifndef __SERVO_H
#define __SERVO_H



#include "sys.h"





extern void servo_init(void);
extern void servo_control(u32 pulse);
extern void bloom(void);
extern void fade(void);



#endif
