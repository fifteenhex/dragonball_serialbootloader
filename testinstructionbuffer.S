#define __ASSEMBLY__
#include "../headers/gpio.h"

lea.l	PDDATA, %a4
mov.b	#1, %d7
mov.b	%d7, (%a4)
