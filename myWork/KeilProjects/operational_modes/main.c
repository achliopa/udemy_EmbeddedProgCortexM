#include "stm32f446xx.h"

// we have to implement the watchdog interrupt handler
void WWDG_IRQHandler(void) {
	for (int i=0;i<50;i++);
}

void generate_interrupt(void) {
	// lets simulate the watchdog interrupt
	NVIC_EnableIRQ(WWDG_IRQn);
	NVIC_SetPendingIRQ(WWDG_IRQn);
}

int main(void){
	generate_interrupt();
	//void (*jump_addr) (void) = &generate_interrupt;
	//void (*jump_addr) (void) = (void *) 0x08000374+1;
	//jump_addr();
	while(1);
	return 0;
}
