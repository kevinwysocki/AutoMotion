/**
 * @file	gpio.h
 * @author	kwysocki
 * @date	5 oct. 2018
 * @brief	
 *
 *
 */

#ifndef BSP_GPIO_GPIO_H_
#define BSP_GPIO_GPIO_H_

#include "../common.h"

/*----------------------------------------------------------------------------*/
/* Definitions		                                                          */
/*----------------------------------------------------------------------------*/

#define GPIO_ERR_WRONG_TYPE				(-1)
#define GPIO_ERR_IRQ_ALREADY_USED		(-2)

/*----------------------------------------------------------------------------*/
/* Enums		                                                              */
/*----------------------------------------------------------------------------*/

enum GPIO_State
{
	GPIO_State_Low = 0,
	GPIO_State_High,
	GPIO_State_Max
};

enum GPIO_Type
{
	GPIO_Type_Input = 0,
	GPIO_Type_Output,
	GPIO_Type_Bidirectionnal,
	GPIO_Type_Max
};

enum GPIO_IRQ_Type
{
	GPIO_IRQ_Type_Falling = 0,
	GPIO_IRQ_Type_Rising,
	GPIO_IRQ_Type_BothEdge,
	GPIO_IRQ_Type_Max
};

/*----------------------------------------------------------------------------*/
/* Class declaration	                                                      */
/*----------------------------------------------------------------------------*/

typedef struct _gpio * GPIO;

struct _gpio
{
	int32_t 		_uid;
	enum GPIO_Type	_type;
	Callback 		_irq_handler;

	int32_t (*init)				(GPIO this, enum GPIO_Type type);

	int32_t (*set)				(GPIO this, enum GPIO_State state);
	int32_t (*get)				(GPIO this, enum GPIO_State * state);
	int32_t (*toggle)			(GPIO this);

	int32_t (*irq_config)		(GPIO this, enum GPIO_IRQ_Type irq_type);
	int32_t (*irq_set_handler)	(GPIO this, Callback irq_handler);
	int32_t (*irq_set_state)	(GPIO this, State irq_state);
};

#endif /* BSP_GPIO_GPIO_H_ */
