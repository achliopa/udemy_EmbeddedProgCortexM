#include <stdint.h>
#include "stm32f446xx.h"

#define RCC_AHB1ENR_OFFSET 	0x30
#define RCC_BASE_ADDR 			0x40023800
#define RCC_AHB1ENR_ADDR 		*((volatile unsigned long *)(RCC_BASE_ADDR+RCC_AHB1ENR_OFFSET))

#define GPIOx_MODER_OFFSET	0x00
#define GPIOA_BASE_ADDR			0x40020000
#define GPIOA_MODER_ADDR		*((volatile unsigned long *)(GPIOA_BASE_ADDR+GPIOx_MODER_OFFSET))

#define GPIOx_ODR_OFFSET		0x14
#define GPIOA_ODR_ADDR			*((volatile unsigned long *)(GPIOA_BASE_ADDR+GPIOx_ODR_OFFSET))

#define GPIOx_IDR_OFFSET		0x10
#define GPIOA_IDR_ADDR			*((volatile unsigned long *)(GPIOA_BASE_ADDR+GPIOx_IDR_OFFSET))
	
void toggle_led(void) {
	if(GPIOA_IDR_ADDR & (1<<5)) {
		GPIOA_ODR_ADDR &= ~(1 << 5);
	} else {
		GPIOA_ODR_ADDR |= (1 << 5);
	}
}

int main(void) {
	uint32_t i = 0;
	/* 1. Enable the clock for GPIO port */
	RCC_AHB1ENR_ADDR |= 1; //bit0
	/* 2. Configure the GPIO PIN to output mode using MODE reg */
	GPIOA_MODER_ADDR |= (1<<(5*2)); //set 10 and 11bit to 0x01 to make 5 pin output
	/* 3. use the DATA REGISTER of GPIOA port to write to or read from LED */
	GPIOA_ODR_ADDR |= (1 << 5); //write 0x01 to 5 bit
	
	while(1) {
		toggle_led();
		for(i=0;i<1500000;i++);
		toggle_led();
		for(i=0;i<500000;i++);
	}
	
	return 0;
}
