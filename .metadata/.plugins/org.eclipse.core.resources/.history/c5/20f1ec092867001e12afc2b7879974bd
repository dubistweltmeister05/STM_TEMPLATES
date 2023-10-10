/*
 * msp.c
 *
 *  Created on: Sep 24, 2023
 *      Author: wardawg
 */


#include "main.h"
#include "stm32f4xx.h"
#include "stm32f407xx.h"
void HAL_MspInit(void)
{
	//Here will do low level processor specific inits.
	//1. Set up the priority grouping of the arm cortex mx processor
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

	//2. Enable the required system exceptions of the arm cortex mx processor
	SCB->SHCSR |= 0x7 << 16; //usage fault, memory fault and bus fault system exceptions

	//3. configure the priority for the system exceptions
	HAL_NVIC_SetPriority(MemoryManagement_IRQn,0,0);
	HAL_NVIC_SetPriority(BusFault_IRQn,0,0);
	HAL_NVIC_SetPriority(UsageFault_IRQn,0,0);
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{

	//1. enable the clock for the peripherals needed

	/*
	 * CLOCKS FOR THE TIMERS
	 */
	__HAL_RCC_TIM2_CLK_ENABLE();
	__HAL_RCC_TIM3_CLK_ENABLE();
	__HAL_RCC_TIM4_CLK_ENABLE();
	__HAL_RCC_TIM9_CLK_ENABLE();

	/*
	 * CLOCKS FOR THE GPIOS
	 */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();


	//2. configure the GPIO pins for acting as channels of each timer

	/* TIMER 2
	 *
	 * PA2  -->channel 3
	 * PA3  -->channel 4
	 */
	GPIO_InitTypeDef timer2PWM_gpio;
	timer2PWM_gpio.Mode = GPIO_MODE_AF_PP;
	timer2PWM_gpio.Pull = GPIO_NOPULL;
	timer2PWM_gpio.Speed = GPIO_SPEED_FREQ_LOW;
	timer2PWM_gpio.Alternate = GPIO_AF1_TIM2;

	timer2PWM_gpio.Pin = GPIO_PIN_2;
	HAL_GPIO_Init(GPIOA,&timer2PWM_gpio);

	timer2PWM_gpio.Pin = GPIO_PIN_3;
	HAL_GPIO_Init(GPIOA,&timer2PWM_gpio);


	/* TIMER 3
	 *
	 * PB4  -->channel 1
	 * PB5  -->channel 2
	 */
	GPIO_InitTypeDef timer3PWM_gpio;
	timer3PWM_gpio.Mode = GPIO_MODE_AF_PP;
	timer3PWM_gpio.Pull = GPIO_NOPULL;
	timer3PWM_gpio.Speed = GPIO_SPEED_FREQ_LOW;
	timer3PWM_gpio.Alternate = GPIO_AF2_TIM3;

	timer3PWM_gpio.Pin = GPIO_PIN_4;
	HAL_GPIO_Init(GPIOB, &timer3PWM_gpio);

	timer3PWM_gpio.Pin = GPIO_PIN_5;
	HAL_GPIO_Init(GPIOB, &timer3PWM_gpio);

	/* TIMER 4
	 *
	 * PB6  -->channel 1
	 * PB7  -->channel 2
	 */
	GPIO_InitTypeDef timer4PWM_gpio;
	timer4PWM_gpio.Mode = GPIO_MODE_AF_PP;
	timer4PWM_gpio.Pull = GPIO_NOPULL;
	timer4PWM_gpio.Speed = GPIO_SPEED_FREQ_LOW;
	timer4PWM_gpio.Alternate = GPIO_AF2_TIM4;

	timer4PWM_gpio.Pin = GPIO_PIN_12;
	HAL_GPIO_Init(GPIOD, &timer4PWM_gpio);

	timer4PWM_gpio.Pin = GPIO_PIN_13;
	HAL_GPIO_Init(GPIOD, &timer4PWM_gpio);

	timer4PWM_gpio.Pin = GPIO_PIN_14;
	HAL_GPIO_Init(GPIOD, &timer4PWM_gpio);

	timer4PWM_gpio.Pin = GPIO_PIN_15;
	HAL_GPIO_Init(GPIOD, &timer4PWM_gpio);

	/* TIMER 9
	 *
	 * PE5  -->channel 1
	 * PE6   -->channel 2
	 */
	GPIO_InitTypeDef timer9PWM_gpio;
	timer9PWM_gpio.Mode = GPIO_MODE_AF_PP;
	timer9PWM_gpio.Pull = GPIO_NOPULL;
	timer9PWM_gpio.Speed = GPIO_SPEED_FREQ_LOW;
	timer9PWM_gpio.Alternate = GPIO_AF3_TIM9;

	timer9PWM_gpio.Pin = GPIO_PIN_5;
	HAL_GPIO_Init(GPIOE,&timer9PWM_gpio);

	timer9PWM_gpio.Pin = GPIO_PIN_6;
	HAL_GPIO_Init(GPIOE,&timer9PWM_gpio);

	//3. Configure the NVIC settings
	HAL_NVIC_SetPriority(TIM2_IRQn, 15,0);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);

	HAL_NVIC_SetPriority(TIM3_IRQn, 15,0);
	HAL_NVIC_EnableIRQ(TIM3_IRQn);

	HAL_NVIC_SetPriority(TIM4_IRQn, 15,0);
	HAL_NVIC_EnableIRQ(TIM4_IRQn);

	HAL_NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 15,0);
	HAL_NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);

}
