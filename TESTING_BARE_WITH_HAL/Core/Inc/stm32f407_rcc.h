/*
 * stm32f407_rcc.h
 *
 *  Created on: 04-Aug-2023
 *      Author: wardawg
 */

#ifndef INC_STM32F407_RCC_H_
#define INC_STM32F407_RCC_H_


#include "stm32f407xx.h"

//This returns the APB1 clock value
uint32_t RCC_GetPCLK1Value(void);

//This returns the APB2 clock value
uint32_t RCC_GetPCLK2Value(void);


uint32_t  RCC_GetPLLOutputClock(void);

#endif /* INC_STM32F407_RCC_H_ */
