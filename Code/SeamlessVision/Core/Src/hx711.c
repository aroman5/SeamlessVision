#include "hx711.h"
#include "stm32fxxx_hal.h"
#include "tm_stm32_delay.h"

#define HX_SHIFT_BITS_CHA	7
#define HX_SHIFT_BITS_CHB	0

void HX711_Init(HX711 data)
{
	HAL_GPIO_WritePin(data.gpioSck, data.pinSck, GPIO_PIN_SET);
	HAL_Delay(50);
	HAL_GPIO_WritePin(data.gpioData, data.pinSck, GPIO_PIN_RESET);
}

uint32_t HX711_AverageValue(HX711 data, uint8_t times)
{
    uint32_t sum = 0;
    for (uint8_t i = 0; i < times; i++)
    {
        sum += HX711_Value(data);
    }

    return sum / times;
}


uint32_t HX711_Value(HX711 data)
{
    uint32_t buffer;
    buffer = 0;

    while (HAL_GPIO_ReadPin(data.gpioData, data.pinData)==1)
    {

    }

    for (uint8_t i = 0; i < 24; i++)
    {
    	HAL_GPIO_WritePin(data.gpioSck, data.pinSck, GPIO_PIN_SET);
    	Delay_us(10);

        buffer = buffer << 1 ;

        if (HAL_GPIO_ReadPin(data.gpioData, data.pinData))
        {
            buffer ++;
        }

        HAL_GPIO_WritePin(data.gpioSck, data.pinSck, GPIO_PIN_RESET);
        Delay_us(10);
    }

    for (uint32_t i = 0; i < data.gain; i++)
    {
    	HAL_GPIO_WritePin(data.gpioSck, data.pinSck, GPIO_PIN_SET);
    	Delay_us(10);
    	HAL_GPIO_WritePin(data.gpioSck, data.pinSck, GPIO_PIN_RESET);
    	Delay_us(10);
    }

    buffer = buffer >> 1;	//25 bits to 24 bits
    buffer = buffer & 0x00FFFFFF;	// clear unvalid bits (24 - 32 bit)

    // output data is in two's complement. DATASHEET:	When input differential signal goes out of
	// the 24 bit range, the output data will be saturated at 800000h (MIN) or 7FFFFFh (MAX).
	// To printout and keep two's complement of 24bit data in 32bit variable, shift it left and than right.
	// the sign of 24bit initial value will remain as it was originally.
    buffer = buffer << 8;
    buffer = buffer >> 8;
	//OR
	//buffer ^= 0x00800000;
	//buffer -= 8388608;

    buffer = buffer >> HX_SHIFT_BITS_CHA;

    return buffer;
}

HX711 HX711_Tare(HX711 data, uint8_t times)
{
    uint32_t sum = HX711_AverageValue(data, times);
    data.offset = sum;
    return data;
}

