#define __ASSEMBLY__
#include "../headers/uart.h"

lea.l	0xAAAAAAAA, %a5
lea.l	0xAAAAAAAA, %a6
recvbyte:
waitforrx:
nop
nop
btst.b	#5, URX1
jeq	waitforrx
mov.b	URX1 + 1, (%a5)+
//mov.b	(%a5)+, UTX1 + 1
cmp	%a5, %a6
jne	recvbyte
