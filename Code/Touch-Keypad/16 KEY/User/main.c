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
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "myiic.h"
#include "TTP229.h"

u8 per;

void key_separation(u16 key_value)
{
	key_value&=0xffff;
	switch(key_value)
	{	
		case 0x0001:printf("KEY7 PRESS\r\n");break;
		case 0x0002:printf("KEY6 PRESS\r\n");break;
		case 0x0004:printf("KEY5 PRESS\r\n");break;
		case 0x0008:printf("KEY4 PRESS\r\n");break;
		case 0x0010:printf("KEY3 PRESS\r\n");break;
		case 0x0020:printf("KEY2 PRESS\r\n");break;
		case 0x0040:printf("KEY1 PRESS\r\n");break;
		case 0x0080:printf("KEY0 PRESS\r\n");break;
		case 0x0100:per=100;printf("100%%\r\n");break;
        case 0x0300:per=93;printf("93%%\r\n");break;
		case 0x0200:per=86;printf("86%%\r\n");break;
        case 0x0600:per=79;printf("79%%\r\n");break;
		case 0x0400:per=71;printf("71%%\r\n");break;
        case 0x0C00:per=64;printf("64%%\r\n");break;
		case 0x0800:per=57;printf("57%%\r\n");break;
        case 0x1800:per=50;printf("50%%\r\n");break;
		case 0x1000:per=43;printf("43%%\r\n");break;
        case 0x3000:per=36;printf("36%%\r\n");break;
		case 0x2000:per=29;printf("29%%\r\n");break;
        case 0x6000:per=21;printf("21%%\r\n");break;
		case 0x4000:per=14;printf("14%%d\r\n");break;
        case 0xC000:per=7;printf("7%%d\r\n");break;
		case 0x8000:per=0;printf("0%%d\r\n");break;
		default:printf("No KEY Press  per=%d\r\n",per);break;	
	}	
}

int main(void)
{
	delay_init();	    	 	  
	NVIC_Configuration(); 	 
	uart_init(115200);	 	
    IIC_Init();
  	while (1)
	{
		key_separation(TTP229_ReadOneByte());
		delay_ms(200);
  	}

}

/*************************************** END OF FILE *************************************/
