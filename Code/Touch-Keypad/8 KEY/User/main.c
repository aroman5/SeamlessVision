/*********************************************************************************************************
*
* File                : main.c
* Hardware Environment: 
* Build Environment   : RealView MDK-ARM  Version: 4.20
* Version             : V1.0
* By                  : 
*
*                                  (c) Copyright 2005-2011, WaveShare
*                                       http://www.waveshare.net
*                                          All Rights Reserved
*
*********************************************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include <stdio.h>
#include <string.h>
#include "usart.h"

void GPIO_Configuration(void);
void Delay (uint32_t nCount);
void NVIC_Configuration(void);

int main(void)
{
    SystemInit();
	GPIO_Configuration();
    usart_Configuration();
    NVIC_Configuration();
  	while (1)
	{
        switch(GPIO_ReadInputData(GPIOA)&0xC03f)
        {
            case 0x0001:printf("KEY0 PRESS\r\n");break;
            case 0x0002:printf("KEY1 PRESS\r\n");break;
            case 0x0004:printf("KEY2 PRESS\r\n");break;
            case 0x0008:printf("KEY3 PRESS\r\n");break;
            case 0x0010:printf("KEY4 PRESS\r\n");break;
            case 0x0020:printf("KEY5 PRESS\r\n");break;
            case 0x4000:printf("KEY6 PRESS\r\n");break;
            case 0x8000:printf("KEY7 PRESS\r\n");break;
            default:printf("NO KEY PRESS\r\n");break;
        }
        Delay(5000000);
  	}

}

void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;	
  
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0000);

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
  NVIC_Init(&NVIC_InitStructure);	
}

void GPIO_Configuration(void)
{
	  GPIO_InitTypeDef GPIO_InitStructure;
	  
	  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA , ENABLE); 						 
	 					 
	  GPIO_InitStructure.GPIO_Pin =GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_14|GPIO_Pin_15;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
	  GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void  Delay (uint32_t nCount)
{
  	for(; nCount != 0; nCount--);
}

/*************************************** END OF FILE *************************************/
