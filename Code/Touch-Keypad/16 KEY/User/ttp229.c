#include "ttp229.h" 
#include "delay.h"

u16 TTP229_ReadOneByte(void)
{				  
	u8 high=0,low=0;
	u16 key_value=0;		  	    																 
    IIC_Start();  
	IIC_Send_Byte(0XAF);	   
	IIC_Wait_Ack();
    low=IIC_Read_Byte(1);
	high=IIC_Read_Byte(0);
	key_value=(high<<8)|low;		   
    IIC_Stop();	    
	return key_value;
} 












