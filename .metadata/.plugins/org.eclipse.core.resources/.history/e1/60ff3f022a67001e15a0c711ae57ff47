/*
 * it.c
 *
 *  Created on: Sep 24, 2023
 *      Author: wardawg
 */

/*
 * here we add all the timer interrupt API handler calls.
 */

#include"main.h"

extern TIM_HandleTypeDef htimer2;
extern TIM_HandleTypeDef htimer3;
extern TIM_HandleTypeDef htimer4;
extern TIM_HandleTypeDef htimer9;

void SysTick_Handler (void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}

void TIM3_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&htimer3);
}

void TIM2_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&htimer2);
}

void TIM4_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&htimer4);
}

void TIM1_BRK_TIM9_IRQHandler (void){
	HAL_TIM_IRQHandler(&htimer9);
}
