/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define BYTE_AT(num, at)		((uint8_t)(((num) >> ((at) << 3)) & 0xFF))

// packing / unpacking is network-order (big endian)
#define PACK_8(dst,src)				(*dst++) = (uint8_t)(src)
#define PACK_16(dst,src)			(*dst++) = BYTE_AT(src,1); (*dst++) = BYTE_AT(src,0)
#define PACK_32(dst,src)			(*dst++) = BYTE_AT(src,3); (*dst++) = BYTE_AT(src,2); (*dst++) = BYTE_AT(src,1); (*dst++) = BYTE_AT(src,0)
#define PACK_ARRAY(dst,src,size)	std::memcpy(dst,src,size); dst += (size)
#define PACK_STRUCT(dst,src)		std::memcpy(dst,&src,sizeof(src)); dst += sizeof(src)

#define UNPACK_U8(src) 				(*src++)
#define UNPACK_U16(src) 			((uint16_t)((*src++) << 8) | (uint16_t)(*src++))
#define UNPACK_U32(src) 			(uint32_t)(((uint32_t)(*src++) << 24) | ((uint32_t)(*src++) << 16) | ((uint32_t)(*src++) << 8) | ((uint32_t)(*src++)))

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void UART_TX_DATA(uint8_t type, uint8_t *data, uint8_t size);

void logI(const char *fmt, ...);
void logE(const char *fmt, ...);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define M_1_Pin GPIO_PIN_0
#define M_1_GPIO_Port GPIOF
#define M_2_Pin GPIO_PIN_1
#define M_2_GPIO_Port GPIOF
#define LED_1_Pin GPIO_PIN_0
#define LED_1_GPIO_Port GPIOA
#define LED_2_Pin GPIO_PIN_1
#define LED_2_GPIO_Port GPIOA
#define HX711OUTPUT_Pin GPIO_PIN_4
#define HX711OUTPUT_GPIO_Port GPIOA
#define HX711SCK_Pin GPIO_PIN_5
#define HX711SCK_GPIO_Port GPIOA
#define SW_SELECT_UP_Pin GPIO_PIN_6
#define SW_SELECT_UP_GPIO_Port GPIOA
#define SW_L_UP_Pin GPIO_PIN_7
#define SW_L_UP_GPIO_Port GPIOA
#define SW_L_DOWN_Pin GPIO_PIN_0
#define SW_L_DOWN_GPIO_Port GPIOB
#define SW_VOICE_Pin GPIO_PIN_1
#define SW_VOICE_GPIO_Port GPIOB
#define IRQ_Pin GPIO_PIN_8
#define IRQ_GPIO_Port GPIOA
#define IRQ_EXTI_IRQn EXTI4_15_IRQn
#define SW_SELECT_Pin GPIO_PIN_11
#define SW_SELECT_GPIO_Port GPIOA
#define SW_SELECT_DOWN_Pin GPIO_PIN_12
#define SW_SELECT_DOWN_GPIO_Port GPIOA
#define TOUCH_1_Pin GPIO_PIN_15
#define TOUCH_1_GPIO_Port GPIOA
#define LED_3_Pin GPIO_PIN_3
#define LED_3_GPIO_Port GPIOB
#define SW_HOME_Pin GPIO_PIN_4
#define SW_HOME_GPIO_Port GPIOB
#define SW_R_DOWN_Pin GPIO_PIN_5
#define SW_R_DOWN_GPIO_Port GPIOB
#define SW_R_UP_Pin GPIO_PIN_6
#define SW_R_UP_GPIO_Port GPIOB
#define SW_SOS_Pin GPIO_PIN_7
#define SW_SOS_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
