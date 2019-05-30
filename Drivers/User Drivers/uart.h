#ifndef __UART_H
#define __UART_H

#include "stm32l0xx_hal.h"
#include "stm32l0xx_hal_dma.h"

#define BufSize 20



typedef struct
{
	__IO uint8_t RxComplete:1;
	__IO uint8_t TxComplete:1;
	uint8_t RxLen;
	uint8_t Buf[BufSize];
}USART_TRANSTYPE;

void uart_rx_idle(UART_HandleTypeDef *huart);

#endif

