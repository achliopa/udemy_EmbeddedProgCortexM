#include <stdint.h>
#include "stm32f407xx.h"
#include "led.h"
#include "my_board.h"


void button_init(void)
{
	/*1. Enable GPIOC clock */
	/* because BUTTON is connected to GPIOC PIN13 */
	//RCC_AHB1ENR
	RCC->AHB1ENR |= 0x04; //Enables the clock
	

	/* 2. set the mode of GPIOC pin13 to "INPUT" */
	GPIOC->MODER &= ~0x0C000000;


	
	/*3. set the interrupt triggering level for pin13 (bit13)*/
	//(EXTI_FTSR
	EXTI->FTSR |= 0x00002000;	

	/*4. enable the interrupt over EXTI13 */
	EXTI->IMR |= 0x00002000;


	/*5. ENable the interrupt on NVIC for IRQ40 */
	NVIC->ISER[1] |= (1 << (EXTI15_10_IRQn - 32));
	
}

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{

	led_init();

	/* Configure EXTI Line0 (connected to PA0 pin) in interrupt mode */
	button_init();
	
	/* Infinite loop */
	while (1)
	{
	}
}

/* ISR for the button interrupt */
void EXTI15_10_IRQHandler(void)
{
	/*clear the pending bit for exti13 */
		if( (EXTI->PR & 0x00002000) )
	{
		EXTI->PR = 0x00002000;//Writing 1 , clears the pending bit for exti13
	
	}
	led_toggle(LED_2);
	/*process only which is important and trigger the pendsv to 
	handle rest in the pendsv handler  */
	SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
}


/* PendSV Exception handler */
void PendSV_Handler(void)
{
		// bottom half work can be done here. 
	 led_toggle2(LED_2);
}



