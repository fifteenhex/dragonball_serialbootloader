#define __ASSEMBLY__
#include "../headers/uart.h"

mov.w	#0xffff, %d7
lea.l	0xAAAAAAAA, %a6
sendbyte:
mov.b	(%a6)+, UTX1 + 1
waitfortx:
btst.b	#2, UTX1
jne	waitfortx
sub.w	#1, %d7
jne	sendbyte
jmp	0xffffff5a
