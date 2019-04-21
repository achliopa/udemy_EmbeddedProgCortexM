#include <stdint.h>
#include "stm32f446xx.h"
#include "led.h"
#include "my_board.h"


void button_init(void)
{
	/* Enable GPIOC clock */
	/* because BUTTON is connected to GPIOC */
	RCC->AHB1ENR |= 0x04;

	//set the mode 
	GPIOC->MODER &= ~0x30000000;
	GPIOC->PUPDR  &= ~0x3000000;

	//enable clock for RCC
	RCC->APB2ENR |= 0x00004000;

	//configure the interrupt 
	EXTI->IMR |= 0x01;
	//	EXTI->RTSR |= 0X01;
	EXTI->FTSR |= 0X01;

	//nvic configuration 
	/* if BASEPRI = 0X80, and making IP[EXTI0_IRQn] = 0X90 will block the interrupt 
	   if BASEPRI = 0X80, and making IP[EXTI0_IRQn] = 0X00 will allow the interrupt 
	*/
	NVIC->IP[EXTI0_IRQn] = 0x00;
	NVIC_EnableIRQ(EXTI0_IRQn);
	
	
}

								 


/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{

	led_init();

	/* Configure EXTI Line0 (connected to PC13 pin) in interrupt mode */
	button_init();

#if 0
	 __set_PRIMASK(1);
#endif
	
#if 0
	/* 0x80 is the priority level, any interrupt which is same or lower priority will be blocked */
	__set_BASEPRI(0X80);
#endif 
	
	
	/* Infinite loop */
	while (1)
	{
	}
}



/* ISR to handle BUTTON interrupt */

void EXTI0_IRQHandler(void)
{
	led_toggle(LED_2);
	
	/*clear the exti0 interrupt pending bit */
	if( (EXTI->PR & 0x01) )
	{
		EXTI->PR &= 0x01;
	
	}
}



