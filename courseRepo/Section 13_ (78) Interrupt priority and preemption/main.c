#include <stdint.h>
#include "stm32f407xx.h"
#include "led.h"
#include "my_board.h"

/* highest */
#define HIGH_PRIORITY_VAL_1   0X00 
/*lowest*/
#define LOW_PRIORITY_VAL_1    0XF0 

#define HIGH_PRIORITY_VAL_2   0X10 
#define LOW_PRIORITY_VAL_2    0XE0  

void button_init(void)
{
	/* Enable GPIOA clock */
	/* because BUTTON is connected to GPIOC */
	RCC->AHB1ENR |= 0x04;

	//set the mode 
	GPIOA->MODER &= ~0x30000000;
	GPIOA->PUPDR  &= ~0x3000000;

	//enable clock for RCC
	RCC->APB2ENR |= 0x00004000;

	//configure the interrupt 
	EXTI->IMR |= 0x01;
	//	EXTI->RTSR |= 0X01;
	EXTI->FTSR |= 0X01;
	
	
		//nvic configuration 
	/* button is irq number 6. which is connected over EXTI0 line in stm32f4xx */
	//NVIC->IP[EXTI0_IRQn] = 0Xf0;// (low priority )
	NVIC->IP[EXTI0_IRQn] = 0X00;// (high priority )
	NVIC_EnableIRQ(EXTI0_IRQn);
	
	

	
}


/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
	uint32_t val;
	led_init();

	/*configure systick timing */
	//2000 ticks
	SysTick_Config(2000);//125 micro seconds
	
		/*configure Systick priority and enable it */
	//SCB->SHP[0x0B] = 0X00;// ( high priority)
	SCB->SHP[0x0B] = 0Xf0;// ( low priority)
	NVIC_EnableIRQ(SysTick_IRQn);
	


	/* Configure EXTI Line0 (connected to PA0 pin) in interrupt mode */
	button_init();


	/* Infinite loop */
	while (1)
	{
		
	}
}

/* ISR for button interrupt */
void EXTI0_IRQHandler(void)
{
	
int i=0;
/* clear the interrupt pending bit of exti0*/
if( (EXTI->PR & 0x01) )
{
	EXTI->PR &= 0x01;

}
  //PD12
	led_on(LED_2);
	for(i=0;i<50000;i++); //do some work
	led_off(LED_2);
}

/* SysTick Exception handler */
void SysTick_Handler(void)
{
	
	//PD13
	 led_on(LED_2);
	 led_off(LED_2);
	
}

