#define __ASSEMBLY__
#include "../headers/uart.h"

lea.l	UTX1, %a4
waitfortx:
mov.w	(%a4), %d7
btst	#10, %d7
beq	waitfortx
mov.b	#'A', (%a4)
