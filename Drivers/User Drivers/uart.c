#include "uart.h"


extern DMA_HandleTypeDef hdma_usart2_rx;
extern USART_TRANSTYPE Usart_TransType;

void uart_rx_idle(UART_HandleTypeDef *huart)
{
	if((__HAL_UART_GET_FLAG(huart,UART_FLAG_IDLE) != RESET))
	{ 
		__HAL_UART_CLEAR_IDLEFLAG(huart);
		HAL_UART_DMAStop(huart);
	  Usart_TransType.RxLen = BufSize - hdma_usart2_rx.Instance->CNDTR;
		Usart_TransType.RxComplete = 1;
	}
}