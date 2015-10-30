#include "stm32f10x.h"

#ifndef __USART_H
#define __USART_H

#define    ENTER	USART1_SendByte(13);USART1_SendByte(10);
//定义命令结束符  10--换行 13--回车
#define		CMDEND		10

extern char RxBuffer[];
extern u16 RxCounter;
extern u8 Rx_Flag;

extern void USART1_SendByte(u8 Data);
extern void prints(char* data);
extern void printRX(char* data,u16 nub);
extern void printv(int v);
extern void USART_OUT(USART_TypeDef* USARTx, uint8_t *Data,...);
void USART1_Configuration(void);


#define dbgPrintf(arg...)  USART_OUT(USART1, arg)	

#endif
