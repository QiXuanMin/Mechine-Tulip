#ifndef __RELAY_H
#define __RELAY_H

#include "stdbool.h"
#include "sys.h"




#define RELAY_OFF	PBout(9)=1
#define RELAY_ON	PBout(9)=0

void relay_init(void);

#endif
