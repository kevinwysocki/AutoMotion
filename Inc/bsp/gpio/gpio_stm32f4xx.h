/**
 * @file		gpio_stm32f4.c
 * @author		Kevin Wysocki
 * @brief		GPIO implementation for STM32F4xx
 * @copyright	MIT License
 *
 * Call create_GPIO_STM32F4() to create a GPIO instance
 * uid must be between 0 and GPIO_STM32F4_MAX_GPIO as :
 * PA0 = 0  .... PA15 = 15
 * PB0 = 16 .... PB15 = 31
 * .......................
 * PK0 = 160 ... PK15 = 175
 */

#ifndef BSP_GPIO_GPIO_STM32F4XX_H_
#define BSP_GPIO_GPIO_STM32F4XX_H_

#include "bsp/gpio/gpio.h"
#include "stm32f4xx.h"

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/* Class declaration	                                                      */
/*----------------------------------------------------------------------------*/
struct _gpio_stm32f4
{
	struct _gpio 	_base;

	GPIO_TypeDef* 		_port;
	uint16_t			_pin;
	GPIO_InitTypeDef 	_config;
};

GPIO create_GPIO_STM32F4 (uint32_t uid, enum GPIO_Type type);
void destroy_GPIO_STM32F4 (uint32_t uid);


#endif /* BSP_GPIO_GPIO_STM32F4XX_H_ */
