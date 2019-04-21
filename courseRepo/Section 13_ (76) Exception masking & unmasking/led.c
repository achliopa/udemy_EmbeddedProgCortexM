
#include "my_board.h"
#include "stm32f407xx.h"


void led_init()	
{
	
	RCC->AHB1ENR |= (0x01 );
	
	//configure LED_2 . GREEN 
	GPIOA->MODER  |= (0x01 << (LED_2 * 2));
	GPIOA->OTYPER |= ( 0 << LED_2);
	GPIOA->PUPDR   |= (0x00 << (LED_2 * 2));
	GPIOA->OSPEEDR |= (0X00 << (LED_2 * 2));	
}


void led_on(uint8_t led_no)
{
	GPIOA->BSRR = ( 1 << led_no );
}

void led_off(uint8_t led_no)
{
	GPIOA->BSRR = ( 1 << (led_no+16) );
}


void led_toggle(uint8_t led_no)
{
	if(GPIOA->ODR & (1 << led_no) )
	{
		led_off(led_no);
	}else
	{
		led_on(led_no);
	}
	
}