#include <stdint.h>
#include "Board_LED.h"
#include "stm32f446xx.h"

// we have to implement the watchdog interrupt handler
void WWDG_IRQHandler(void) {
	LED_Off(0);
}

void delay(void) {
	uint32_t i = 0;
	for(i=0;i<100000;i++);
}

int main(void) {
	// change the stack pointer to PSP
	uint32_t value = __get_CONTROL();
	value |= (1<<1);
	__set_CONTROL(value);
	
	//iitialise the PSP
	__set_PSP(SRAM2_BASE);
	
	LED_Initialize();

	while(1){
		LED_On(0);
		
		//Enable and Pend the Watchdog Interrupt here
		NVIC_EnableIRQ(WWDG_IRQn);
		NVIC_SetPendingIRQ(WWDG_IRQn);
		
		delay();
		
		LED_On(0);
		
		delay();
	}
	return 0;
}
