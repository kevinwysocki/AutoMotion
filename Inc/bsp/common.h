/**
 * @file	common.h
 * @author	kwysocki
 * @date	5 oct. 2018
 * @brief	
 *
 *
 */

#ifndef BSP_COMMON_H_
#define BSP_COMMON_H_

#include <stdint.h>

/*----------------------------------------------------------------------------*/
/* Definitions	                                                              */
/*----------------------------------------------------------------------------*/

#define NO_ERROR	(0)

/*----------------------------------------------------------------------------*/
/* Types		                                                              */
/*----------------------------------------------------------------------------*/

typedef void * Object;

typedef int32_t (*Callback)	(Object obj);

typedef enum
{
	disabled = 0,
	enabled
}State;

typedef enum
{
	false = 0,
	true = !false
}bool_t;

#endif /* BSP_COMMON_H_ */
