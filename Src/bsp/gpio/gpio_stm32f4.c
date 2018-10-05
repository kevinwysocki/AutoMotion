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

#include <stddef.h>
#include <stdlib.h>

#include "bsp/gpio/gpio_stm32f4xx.h"

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

#define GPIO_STM32F4_MAX_PORT			(11)
#define GPIO_STM32F4_MAX_PIN_PER_PORT	(16)
#define GPIO_STM32F4_MAX_GPIO			(GPIO_STM32F4_MAX_PORT * GPIO_STM32F4_MAX_PIN_PER_PORT)

/*----------------------------------------------------------------------------*/
/* Types		                                                              */
/*----------------------------------------------------------------------------*/

typedef struct _gpio_stm32f4* GPIO_STM32F4;

/*----------------------------------------------------------------------------*/
/* Structs		                                                              */
/*----------------------------------------------------------------------------*/

struct _irq_status
{
	bool_t			free;
	GPIO_STM32F4 	gpio;
};

/*----------------------------------------------------------------------------*/
/* Private Members                                                            */
/*----------------------------------------------------------------------------*/

static GPIO_STM32F4 _gpio_list[GPIO_STM32F4_MAX_GPIO] = { NULL };

static GPIO_TypeDef* _gpio_port[] = { GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH, GPIOI, GPIOJ, GPIOK };

static uint16_t _gpio_pin[] = { GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3, GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_7,
								GPIO_PIN_8,	GPIO_PIN_9, GPIO_PIN_10, GPIO_PIN_11, GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15 };

static struct _irq_status _irq_stat[GPIO_STM32F4_MAX_PIN_PER_PORT] = { {.free = true, .gpio = NULL}, {.free = true, .gpio = NULL}, {.free = true, .gpio = NULL}, {.free = true, .gpio = NULL},
																	   {.free = true, .gpio = NULL}, {.free = true, .gpio = NULL}, {.free = true, .gpio = NULL}, {.free = true, .gpio = NULL},
																	   {.free = true, .gpio = NULL}, {.free = true, .gpio = NULL}, {.free = true, .gpio = NULL}, {.free = true, .gpio = NULL},
																	   {.free = true, .gpio = NULL}, {.free = true, .gpio = NULL}, {.free = true, .gpio = NULL}, {.free = true, .gpio = NULL} };

/*----------------------------------------------------------------------------*/
/* Private Functions                                                          */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/* Interrupt Handler                                                          */
/*----------------------------------------------------------------------------*/

void HAL_GPIO_EXTI_Callback (uint16_t pin_index)
{
	assert_param(pin_index < GPIO_STM32F4_MAX_PIN_PER_PORT);

	if(_irq_stat[pin_index].free == false)
	{
		if(_irq_stat[pin_index].gpio->_base._irq_handler != NULL)
		{
			_irq_stat[pin_index].gpio->_base._irq_handler((Object)&_irq_stat[pin_index].gpio->_base);
		}
	}
}

/*----------------------------------------------------------------------------*/
/* Class Implementation	                                                      */
/*----------------------------------------------------------------------------*/

/**
 * @brief Initialize GPIO
 * @param base : Base class
 * @param type : GPIO type
 * @return NO_ERROR or GPIO_ERR_xx
 */
static int32_t _init (GPIO base, enum GPIO_Type type)
{
	GPIO_STM32F4 this = (GPIO_STM32F4)base;
	int32_t ret = NO_ERROR;

	assert_param(this != NULL);

	this->_config.Pin		=	(uint32_t)this->_pin;
	this->_config.Mode		=	(this->_base._type == GPIO_Type_Input) ? GPIO_MODE_INPUT : GPIO_MODE_OUTPUT_PP;
	this->_config.Pull		=	GPIO_NOPULL;
	this->_config.Speed		=	GPIO_SPEED_FREQ_VERY_HIGH;
	this->_config.Alternate	=	0;

	HAL_GPIO_Init(this->_port, &this->_config);

	return ret;
}

/**
 * @brief Set GPIO state
 * @param base : Base class
 * @param state : GPIO state
 * @return NO_ERROR or GPIO_ERR_xx
 */
static int32_t _set (GPIO base, enum GPIO_State state)
{
	GPIO_STM32F4 this = (GPIO_STM32F4)base;
	int32_t ret = NO_ERROR;

	assert_param(this != NULL);

	if(this->_base._type == GPIO_Type_Output)
	{
		HAL_GPIO_WritePin(this->_port, this->_pin, (state == GPIO_State_Low) ? GPIO_PIN_RESET : GPIO_PIN_SET);
	}
	else
	{
		ret = GPIO_ERR_WRONG_TYPE;
	}

	return ret;
}

/**
 * @brief Get GPIO State
 * @param base : Base class
 * @param state : GPIO state
 * @return NO_ERROR or GPIO_ERR_xx
 */
static int32_t _get (GPIO base, enum GPIO_State * state)
{
	GPIO_STM32F4 this = (GPIO_STM32F4)base;
	int32_t ret = NO_ERROR;
	GPIO_PinState pin_state = GPIO_PIN_RESET;

	assert_param(this != NULL);

	pin_state = HAL_GPIO_ReadPin(this->_port, this->_pin);

	*state = (pin_state == GPIO_PIN_RESET) ? GPIO_State_Low : GPIO_State_High;

	return ret;
}

/**
 * @brief Toggle GPIO
 * @param base : Base class
 * @return NO_ERROR or GPIO_ERR_xx
 */
static int32_t _toggle (GPIO base)
{
	GPIO_STM32F4 this = (GPIO_STM32F4)base;
	int32_t ret = NO_ERROR;

	assert_param(this != NULL);

	if(this->_base._type == GPIO_Type_Output)
	{
		HAL_GPIO_TogglePin(this->_port, this->_pin);
	}
	else
	{
		ret = GPIO_ERR_WRONG_TYPE;
	}

	return ret;
}

/**
 * @brief Configure GPIO IRQ
 * @param this : Base class
 * @param irq_type : IRQ type
 * @return NO_ERROR or GPIO_ERR_xx
 */
static int32_t _irq_config (GPIO base, enum GPIO_IRQ_Type irq_type)
{
	GPIO_STM32F4 this = (GPIO_STM32F4)base;
	int32_t ret = NO_ERROR;
	uint8_t irq_index = 0;

	assert_param(this != NULL);
	assert_param(irq_type < GPIO_IRQ_Type_Max);

	if(this->_base._type == GPIO_Type_Input)
	{
		irq_index = this->_base._uid % GPIO_STM32F4_MAX_PORT;

		if(_irq_stat[irq_index].free == true)
		{
			_irq_stat[irq_index].free = false;
			_irq_stat[irq_index].gpio = this;

			switch(irq_type)
			{
			case GPIO_IRQ_Type_Falling:
				this->_config.Mode = GPIO_MODE_IT_FALLING;
				break;
			case GPIO_IRQ_Type_Rising:
				this->_config.Mode = GPIO_MODE_IT_RISING;
				break;
			case GPIO_IRQ_Type_BothEdge:
				this->_config.Mode = GPIO_MODE_IT_RISING_FALLING;
				break;
			default:
				break;
			}

			HAL_GPIO_Init(this->_port, &this->_config);
		}
		else
		{
			ret = GPIO_ERR_IRQ_ALREADY_USED;
		}
	}
	else
	{
		ret = GPIO_ERR_WRONG_TYPE;
	}

	return ret;
}

/**
 * @brief Set GPIO IRQ handler
 * @param base : Base class
 * @param irq_handler : IRQ handler
 * @return NO_ERROR or GPIO_ERR_xx
 */
static int32_t _irq_set_handler (GPIO base, Callback irq_handler)
{
	GPIO_STM32F4 this = (GPIO_STM32F4)base;
	int32_t ret = NO_ERROR;

	assert_param(this != NULL);
	assert_param(irq_handler != NULL);

	this->_base._irq_handler = irq_handler;

	return ret;
}

/**
 * @brief Set GPIO IRQ state
 * @param base : Base class
 * @param irq_state : IRQ state (Enabled or Disabled)
 * @return NO_ERROR or GPIO_ERR_xx
 */
static int32_t _irq_set_state (GPIO base, State irq_state)
{
	GPIO_STM32F4 this = (GPIO_STM32F4)base;
	int32_t ret = NO_ERROR;

	assert_param(this != NULL);

	if(irq_state == enabled)
	{

	}
	else
	{

	}

	return ret;
}

/**
 * @brief Create a GPIO for STM32F4
 * @param uid : GPIO unique ID
 * @param type : GPIO type
 * @return GPIO instance
 */
GPIO create_GPIO_STM32F4 (uint32_t uid, enum GPIO_Type type)
{
	GPIO_STM32F4 this = NULL;

	assert_param(uid < GPIO_STM32F4_MAX_GPIO);
	assert_param((type == GPIO_Type_Input) || (type == GPIO_Type_Output));

	_gpio_list[uid] = (GPIO_STM32F4)malloc(sizeof(struct _gpio_stm32f4));

	if(_gpio_list[uid] != NULL)
	{
		this = _gpio_list[uid];

		// Set attributes
		this->_base._uid			=	uid;
		this->_base._type			=	type;
		this->_base._irq_handler	=	NULL;

		this->_port =	_gpio_port[this->_base._uid / GPIO_STM32F4_MAX_PORT];
		this->_pin	= 	_gpio_pin[this->_base._uid % GPIO_STM32F4_MAX_PORT];

		// Set methods
		this->_base.init			=	&_init;
		this->_base.set				=	&_set;
		this->_base.get				=	&_get;
		this->_base.toggle			=	&_toggle;
		this->_base.irq_config		=	&_irq_config;
		this->_base.irq_set_handler = 	&_irq_set_handler;
		this->_base.irq_set_state	=	&_irq_set_state;

		// Init GPIO before using it
		this->_base.init(&this->_base, type);
	}

	return &this->_base;
}

/**
 * @brief Destroy GPIO instance
 * @param uid : GPIO unique ID
 */
void destroy_GPIO_STM32F4 (uint32_t uid)
{
	if(_gpio_list[uid] != NULL)
	{
		free(_gpio_list[uid]);
	}
}
