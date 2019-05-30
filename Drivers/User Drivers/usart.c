/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

//#include "gpio.h"
//#include "dma.h"
//#include "user.h"

/* USER CODE BEGIN 0 */

#include <stdarg.h>
//#include <string.h>


USART_RECEIVETYPE Usart1_RX;
USART_RECEIVETYPE Usart2_RX; 
USART_RECEIVETYPE LPUsart1_RX;


/* USER CODE END 0 */

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

UART_HandleTypeDef hlpuart1;

/* LPUART1 init function */

void MX_LPUART1_UART_Init(void)
{
	LPUART1_UART_Init();
}
/* USART1 init function */

void LPUART1_UART_Init(void)
{


  hlpuart1.Instance        = LPUART1;
  hlpuart1.Init.BaudRate   = 9600;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits   = UART_STOPBITS_1;
  hlpuart1.Init.Parity     = UART_PARITY_NONE;
  hlpuart1.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  hlpuart1.Init.Mode       = UART_MODE_TX_RX;

  if(HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
      
  }
  
  __HAL_UART_DISABLE(&hlpuart1);
  LPUART1->RQR = 1<<3;//clears the RXNE flag, and overrun flag
  LPUART1->CR1 |= 1<<5;//enable the RXNE interrupt
  LPUART1->CR1 |= 1<<4;//enable the IDLE interrupt

  __HAL_UART_ENABLE(&hlpuart1);//使能USART	//LPUART1->CR1 |= 1<<0;

  HAL_Delay(10);

  __HAL_UART_CLEAR_FLAG(&hlpuart1,UART_FLAG_TC);                 //清除发送标志
  __HAL_UART_CLEAR_FLAG(&hlpuart1,UART_FLAG_RXNE);               //清除接收中断标志	
   __HAL_UART_CLEAR_FLAG(&hlpuart1,UART_FLAG_TXE);               //清除接收中断标志	
  __HAL_UART_CLEAR_FLAG(&hlpuart1,UART_FLAG_IDLE);               //清除空闲中断标志	

  //MODIFY_REG(LPUART1->CR3, USART_CR3_WUS, (USART_CR3_WUS_0 | USART_CR3_WUS_1));

}



/* USART1 init function */

void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

    HAL_Delay(10);
  __HAL_UART_CLEAR_FLAG(&huart1,UART_FLAG_IDLE);               //清除空闲中断标志	
  __HAL_UART_CLEAR_FLAG(&huart1,UART_FLAG_ORE);               //清除空闲中断标志	
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);                 //使能空闲中断
  HAL_UART_Receive_DMA(&huart1, Usart1_RX.RX_Buf, 1024); //开启DMA接收 
  Usart1_RX.receive_flag = 0;
}


void MX_USART1_UART_Init_CU(uint32_t baud)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = baud;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

    HAL_Delay(10);
  __HAL_UART_CLEAR_FLAG(&huart1,UART_FLAG_IDLE);               //清除空闲中断标志	
  __HAL_UART_CLEAR_FLAG(&huart1,UART_FLAG_ORE);               //清除空闲中断标志	
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);                 //使能空闲中断
  HAL_UART_Receive_DMA(&huart1, Usart1_RX.RX_Buf, 1024); //开启DMA接收 
  Usart1_RX.receive_flag = 0;
}



/* USART2 init function */

void MX_USART2_UART_Init(void)
{
	

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_Delay(10);
  __HAL_UART_CLEAR_FLAG(&huart2,UART_FLAG_IDLE);               //清除空闲中断标志	
  __HAL_UART_CLEAR_FLAG(&huart2,UART_FLAG_ORE);               //清除空闲中断标志	
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);                 //使能空闲中断
  HAL_UART_Receive_DMA(&huart2, Usart2_RX.RX_Buf, 1024); //开启DMA接收 
  Usart2_RX.receive_flag = 0;

}

void MX_USART2_UART_Init_115200(void)
{
	

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_Delay(10);
  __HAL_UART_CLEAR_FLAG(&huart2,UART_FLAG_IDLE);               //清除空闲中断标志	
  __HAL_UART_CLEAR_FLAG(&huart2,UART_FLAG_ORE);               //清除空闲中断标志	
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);                 //使能空闲中断
  HAL_UART_Receive_DMA(&huart2, Usart2_RX.RX_Buf, 1024); //开启DMA接收 
  Usart2_RX.receive_flag = 0;

}


void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();
  
    /**USART2 GPIO Configuration    
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	//SET_BIT(USART2->CR2,USART_CR2_SWAP);

    /* Peripheral DMA init*/
  
    hdma_usart2_rx.Instance = DMA1_Channel5;
    hdma_usart2_rx.Init.Request = DMA_REQUEST_4;
    hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_rx.Init.Mode = DMA_NORMAL;
    hdma_usart2_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart2_rx);

  /* USER CODE BEGIN USART2_MspInit 1 */
  
    HAL_NVIC_SetPriority(USART2_IRQn, 2, 1);
	HAL_NVIC_EnableIRQ(USART2_IRQn);

  /* USER CODE END USART2_MspInit 1 */
  }
  else if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();
  
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Peripheral DMA init*/
	//SET_BIT(USART1->CR2,USART_CR2_SWAP);
  
    hdma_usart1_rx.Instance = DMA1_Channel3;
    hdma_usart1_rx.Init.Request = DMA_REQUEST_3;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_NORMAL;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart1_rx);

    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(USART1_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==LPUART1)
  {
  /* USER CODE BEGIN LPUART1_MspInit 0 */

  /* USER CODE END LPUART1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_LPUART1_CLK_ENABLE();
  
    /**LPUART1 GPIO Configuration    
    PB10     ------> LPUART1_TX
    PB11     ------> LPUART1_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_LPUART1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN LPUART1_MspInit 1 */
  	HAL_NVIC_SetPriority(LPUART1_IRQn, 0, 1);//设置USART中断优先级:抢占优先级3；子优先级0（M0+内核无子优先级）
 	HAL_NVIC_EnableIRQ(LPUART1_IRQn);//使能USART中断

  /* USER CODE END LPUART1_MspInit 1 */
 
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();
  
    /**USART2 GPIO Configuration    
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

    /* Peripheral DMA DeInit*/
    HAL_DMA_DeInit(uartHandle->hdmarx);
  }
  else if(uartHandle->Instance==USART1)
  {
	 __HAL_RCC_USART1_CLK_DISABLE();
  
    /**LPUART1 GPIO Configuration    
    PB10     ------> LPUART1_TX
    PB11     ------> LPUART1_RX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);
  }
  else if(uartHandle->Instance==LPUART1)
  {
  /* USER CODE BEGIN LPUART1_MspDeInit 0 */

  /* USER CODE END LPUART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_LPUART1_CLK_DISABLE();
  
    /**LPUART1 GPIO Configuration    
    PB10     ------> LPUART1_TX
    PB11     ------> LPUART1_RX 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_11);

  /* USER CODE BEGIN LPUART1_MspDeInit 1 */

  /* USER CODE END LPUART1_MspDeInit 1 */
  
  }
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
} 

/* USER CODE BEGIN 1 */

//--------------------LPUART1---------------------------------


void LPUsart1SendData(uint8_t *pdata, uint16_t Length)  
{  
    uint32_t i = 0;
		
	for (i = 0; i < Length; i++)
	{
		LPUART1_SendByte(pdata[i]);
	}
}

void LPUART1_SendByte(uint8_t data)
{
  while((LPUART1->ISR&UART_FLAG_BUSY)==UART_FLAG_BUSY);//等待串口空闲
  while((LPUART1->ISR&UART_FLAG_TXE)!=UART_FLAG_TXE);//等待发送寄存器为空
  LPUART1->TDR = data;
}
void LPUART1_SendString(char *str)
{
  while((*str)!=0)
  {
    while((LPUART1->ISR&UART_FLAG_BUSY)==UART_FLAG_BUSY);//等待串口空闲
    while((LPUART1->ISR&UART_FLAG_TXE)!=UART_FLAG_TXE);//等待发送寄存器为空
    LPUART1->TDR = *str++;       
  }
}


void LPUsart1Receive_IDLE(uint8_t *temp,uint16_t lenth)  
{    
    if((__HAL_UART_GET_FLAG(&hlpuart1,UART_FLAG_IDLE) != RESET))  
    {   
        __HAL_UART_CLEAR_IDLEFLAG(&hlpuart1);  
		for(uint16_t i =0;i<lenth;i++)
		{
			LPUsart1_RX.RX_Buf[i] = temp[i];
		}
        LPUsart1_RX.rx_len =  lenth;   
        LPUsart1_RX.receive_flag=1;  
    }  
}


void Usart2SendData(uint8_t *pdata, uint16_t Length)  
{  
     uint32_t i = 0;
		
	for (i = 0; i < Length; i++)
	{
		UART2_SendByte(pdata[i]);
	}
}  

void UART2_SendByte(uint8_t data)
{
  while((USART2->ISR&UART_FLAG_TXE)!=UART_FLAG_TXE);//等待发送寄存器为空
  USART2->TDR = data;
}

void USART2_SendString(char *str)
{
  while((*str)!=0)
  {
    while((USART2->ISR&UART_FLAG_TXE)!=UART_FLAG_TXE);//等待发送寄存器为空
    USART2->TDR = *str++;       
  }
}


void DEBUG_Printf(char *fmt, ...)
{		
	#ifdef DEBUG
	char buf[128];
    va_list va_args;

    // Start the varargs processing.
    va_start(va_args, fmt);

    vsnprintf((char *)buf, sizeof(buf), fmt, va_args);

    // End the varargs processing.
    va_end(va_args);

    /*
     * 真正的打印输出函数，不同平台修改
     */
//    Serial_PutString(buf);
	 USART2_SendString(buf);
	#endif
}

void Usart2Receive_IDLE(void)  
{  
    uint32_t temp;  
	
    if((__HAL_UART_GET_FLAG(&huart2,UART_FLAG_IDLE) != RESET))  
    {   
        __HAL_UART_CLEAR_IDLEFLAG(&huart2);  
        HAL_UART_DMAStop(&huart2);  
        temp = huart2.hdmarx->Instance->CNDTR;  
        Usart2_RX.rx_len =  1024 - temp;   
        Usart2_RX.receive_flag=1;  
        HAL_UART_Receive_DMA(&huart2,Usart2_RX.RX_Buf,1024);  
    }  
  	

}



//--------------------UART1---------------------------------

void Usart1Receive_IDLE(void)  
{  
    uint32_t temp;  
  
    if((__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE) != RESET))  
    {   
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);  
        HAL_UART_DMAStop(&huart1);  
        temp = huart1.hdmarx->Instance->CNDTR;  
        Usart1_RX.rx_len =  1024 - temp;   
        Usart1_RX.receive_flag=1;  
        HAL_UART_Receive_DMA(&huart1,Usart1_RX.RX_Buf,1024);
    }  
}

void Usart1SendData(uint8_t *pdata, uint16_t Length)  
{  
     uint32_t i = 0;
		
	for (i = 0; i < Length; i++)
	{
		UART1_SendByte(pdata[i]);
	}
}  

void UART1_SendByte(uint8_t data)
{
  while((USART1->ISR&UART_FLAG_BUSY)==UART_FLAG_BUSY);//等待串口空闲
  while((USART1->ISR&UART_FLAG_TXE)!=UART_FLAG_TXE);//等待发送寄存器为空
  USART1->TDR = data;
}

void USART1_SendString(char *str)
{
  while((*str)!=0)
  {
  	while((USART1->ISR&UART_FLAG_BUSY)==UART_FLAG_BUSY);//等待串口空闲
    while((USART1->ISR&UART_FLAG_TXE)!=UART_FLAG_TXE);//等待发送寄存器为空
    USART1->TDR = *str++;       
  }
}




char *StringStr(char *str, char *dest)
{
#define STR_BUFF_LEN	0x100
	int i = STR_BUFF_LEN;
	char *cp = str;
	char *s1, *s2;
	
	if (*dest == 0)
	{
		return str;
	}
	
//	while(*str)
	while(i--)
	{		
		s1 = cp;
		s2 = dest;
		
		while((*s1 == *s2) && *s1 && *s2)
		{
			s1++;
			s2++;
		}
		if(!*s2)
			return cp;
		cp++;
	}
	
	return NULL;
}

/**
  * @brief  Convert an Integer to a string
  * @param  str: The string
  * @param  intnum: The integer to be converted
  * @retval None
  */
void Int2Str(uint8_t* str, int32_t intnum)
{
  uint32_t i, Div = 1000000000, j = 0, Status = 0;
 
  if(intnum < 0)
  {
	intnum = intnum*(-1);
	str[j++] = '-';
  }
  
  for (i = 0; i < 10; i++)
  {
    str[j++] = (intnum / Div) + 48;	/* '0' */

    intnum = intnum % Div;
    Div /= 10;
    if ((str[j-1] == '0') & (Status == 0))
    {
      j = 0;
    }
    else
    {
      Status++;
    }
  }
}


/*
**	string concat
*/
uint8_t *StringCat(char *str, const uint8_t *string)
{
	uint8_t *s = str;
	
	while(*s)
	{
		s++;
	}
	
	while(*string)
	{
		*s++ = *string++;
	}
	
	*s++ = '\r';
	*s++ = '\n';
	*s = '\0';
			
	return str;		
}


void ByteToHexStr(uint8_t *source, char* dest, int sourceLen)
{
    short i;
    uint8_t highByte, lowByte;


    for (i = 0; i < sourceLen; i++)
    {
        highByte = source[i] >> 4;
        lowByte = source[i] & 0x0f ;


        highByte += 0x30;


        if (highByte > 0x39)
                dest[i * 2] = highByte + 0x07;
        else
                dest[i * 2] = highByte;


        lowByte += 0x30;
        if (lowByte > 0x39)
            dest[i * 2 + 1] = lowByte + 0x07;
        else
            dest[i * 2 + 1] = lowByte;
    }
    return ;
}


uint8_t *StringCat2(char *str, const uint8_t *string)
{
	uint8_t *s = str;
	
	while(*s)
	{
		s++;
	}
	
	while(*string)
	{
		*s++ = *string++;
	}
			
	return str;		
}



uint8_t Time_Out_Break(uint32_t MAX_time , uint8_t *Sign)
{
	static uint32_t time_start = 0;
	static uint32_t time_new = 0;
	uint32_t temp=0;
	uint8_t TimeOut_Sign = *Sign;

	if(TimeOut_Sign == 0)
	{
		*Sign = 1;
		time_start = HAL_GetTick();
	}
	if(TimeOut_Sign == 1)
	{
		time_new = HAL_GetTick();

		if(time_new < time_start)
		{
			time_new = (time_new + (0xffffffff - time_start));
			time_start = 0;
		}
		temp = time_new - time_start;
		if(temp > MAX_time)
		{
			return 1;
		}
		else
			{return 0;}
	}
	return 0;
}



/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
