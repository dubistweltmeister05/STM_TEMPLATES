/*
 * stm32f407_gpio_driver.c
 *
 *  Created on: 23-Jul-2023
 *      Author: wardawg
 */
#include "stm32f407xx.h"
#include "stm2f407_gpio_driver.h"
/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_DI();
		}
	}

}




/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             - THis is used to initialize the GPIO parameters to the GPIO port in Question
 *
 * @param[in]         - Handler of the GPIO Port to be used, which holds the base address of the GPIO port and the parameters for the configuration
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - void
 *
 * @Note              -

 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp=0; //temp. register

	//enable the peripheral clock

	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	/*
	 * Mode Configuration
	 */

	//1 . configure the mode of gpio pin

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//the non interrupt mode

		//writing the pinmode at appropriate bits of MODER
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ) );

		//clearing any pre-set bits
		pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

		//setting the value already loaded in temp.
		pGPIOHandle->pGPIOx->MODER |= temp;

	}else
	{
		//this part is the interrupt mode, so special coding is needed

		//1. Select the triggering mechanism
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode ==GPIO_MODE_IT_FT ) //Falling edge triggering
		{
			//1. configure the FTSR
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			//2.Clear the corresponding RTSR bit
			EXTI->RTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode ==GPIO_MODE_IT_RT ) //Rising edge triggering
		{
			//1.configure the RTSR
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			//2.Clear the corresponding RTSR bit
			EXTI->FTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT )//Dual Triggering
		{
			//1. configure both FTSR and RTSR
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			//2.Clear the corresponding RTSR bit
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//2. configure the GPIO port selection in SYSCFG_EXTICR

		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4 ;
		//this tells us which CR of the SYSCFG_EXTI block we have to access

		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		// this tells us the bit field to access

		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << ( temp2 * 4);

		//3 . enable the exti interrupt delivery using IMR
		EXTI->IMR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	}

	/*
	 * SPEED Configuration
	 */

	//2. configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->OSPEEDR &= ~( 0x3 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

    temp=0;

    /*
	 * PULL Configuration
	 */

	//3. configure the pupd settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;

	/*
	 * OUTPUT TYPE Configuration
	 */

	//4. configure the optype
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	temp=0;

	/*
	 * ALTERNATE MODE Configuration
	 */


	//5. configure the alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		//configure the alt function registers.
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		//this is done to decide which register to use, AFRL or AFRH

		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber  % 8;
		//this done to tell us the bit field to be configured (AFRx) in the Low or High Registers.

		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << ( 4 * temp2 ) );
		//clearing; (temp2 * 4) tells us teh starting bit position of the bit field that needs configuring

		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << ( 4 * temp2 ) );
	}

}


/*********************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             - This is used to de-initialize any Port that is not in use. Useful for saving power
 *
 * @param[in]         - Base address of the GPIO
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - void
 *
 * @Note              -

 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}else if (pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}else if (pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}else if (pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}else if (pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}else if (pGPIOx == GPIOI)
	{
		GPIOI_REG_RESET();
	}

}



/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPin
 *
 * @brief             - Used to read the input data at any pin of the MCU
 *
 * @param[in]         - Base Address of the GPIO port to be read from
 * @param[in]         - The Pin Number of the GPIO Port that receives the signal
 * @param[in]         -
 *
 * @return            -   0 or 1
 *
 * @Note              -

 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;

	value = (uint8_t )((pGPIOx->IDR  >> PinNumber) & 0x00000001 ) ;

	return value;
}


/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPort
 *
 * @brief             - Used to read the input data at any port of the MCU
 *
 * @param[in]         - Base address of the port from which the data needs to be read from
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - A 16 bit number where each bit holds the value at the corresponding pin
 *
 * @Note              -

 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;

	value = (uint16_t)pGPIOx->IDR;

	return value;
}


/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPin
 *
 * @brief             - Used to write some data at any pin of the MCU
 *
 * @param[in]         - Base Address of the GPIO port to be written to
 * @param[in]         - The Pin Number of the GPIO Port that sends the signal
 * @param[in]         - The value to be sent (0 or 1)
 *
 * @return            - void
  *
 * @Note              -

 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{

	if(Value == GPIO_PIN_SET)
	{
		//write 1 to the output data register at the bit field corresponding to the pin number
		pGPIOx->ODR |= ( 1 << PinNumber);
	}else
	{
		//write 0
		pGPIOx->ODR &= ~( 1 << PinNumber);
	}
}


/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPort
 *
 * @brief             - Used to write some data at any pin of the MCU
 *
 * @param[in]         - Base Address of the GPIO port to be written to
 * @param[in]         - The value to be sent (0 or 1)
 * @param[in]         -
 *
 * @return            - void
 *
 * @Note              -

 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR  = Value;
}


/*********************************************************************
 * @fn      		  - GPIO_ToggleOutputPin
 *
 * @brief             - Toggles the value at any pin of any port of the MCU
 *
 * @param[in]         - Base address of the port
 * @param[in]         - Pin Number that needs to be toggled
 * @param[in]         -
 *
 * @return            - void
 *
 * @Note              -

 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR  ^= ( 1 << PinNumber);
}



/*********************************************************************
 * @fn      		  - GPIO_IRQConfig
 *
 * @brief             - Used to set up the interrupt that is received over any GPIO Pin
 *
 * @param[in]         - IRQ Number of the GPIO Port
 * @param[in]         - Enable or disable, the controlling choice for the delivery of the IRQ
 * @param[in]         -
 *
 * @return            - void
 *
 * @Note              -

 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register, which is used to enable the interrupts at EXTI lines 0-31
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
		{
			//program ISER1 register, which is used to enable the interrupts at EXTI lines 32-63
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ISER2 register, which is used to enable the interrupts at EXTI lines 64-95
			*NVIC_ISER2 |= ( 1 << (IRQNumber % 64) );
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register, which is used to clear the interrupts at lines 0-31
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			//program ICER1 register, which is used to clear the interrupts at lines 32-63
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ICER2 register, which is used to clear the interrupts at lines 64-95
			*NVIC_ICER2 |= ( 1 << (IRQNumber % 64) );
		}
	}

}



/*********************************************************************
 * @fn      		  - GPIO_IRQPriorityConfig
 *
 * @brief             - Used to set u[ the priority of the IRQ Being delivered, useful in the case of MULTIPLE IRQ
 *
 * @param[in]         - The IRQ number over which it is being delivered
 * @param[in]         - The Priority of the signal. The lower, the more prior it is
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	//1. first lets find out the ipr register

	uint8_t iprx = IRQNumber / 4;
	//tells us about the ipr register to write the value to

	uint8_t iprx_section  = IRQNumber %4 ;
	//tells us the bit field of the IPR register selected that we need to write to

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );
	// LHS is the address of the IPRx that needs to be written to

}
/*********************************************************************
 * @fn      		  - GPIO_IRQHandling
 *
 * @brief             - on the delivery of the interrupt, the Pending register is to be cleared by writing 1 to it via software
 *
 * @param[in]         - THe Pin Number on which the IRQ is received
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear the exti pr register corresponding to the pin number
	if(EXTI->PR & ( 1 << PinNumber))
	{
		//clear
		EXTI->PR |= ( 1 << PinNumber);
	}

}
