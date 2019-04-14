#include "stm32f446xx.h"
#include <stdint.h>

// we have to implement the watchdog interrupt handler
void WWDG_IRQHandler(void) {
	for (int i=0;i<50;i++);
	
	/* lets try to move the processor to PAL */
	uint32_t value = __get_CONTROL();
	value &= ~0x01;
	__set_CONTROL(value);
}

void generate_interrupt(void) {
	// lets simulate the watchdog interrupt
	NVIC_EnableIRQ(WWDG_IRQn);
	// WILL NEVER GET THERE!!!!!!! WE CANNOT SET SYSTEM REGS WHILE IN NPAL!!!
	NVIC_SetPendingIRQ(WWDG_IRQn);
}

void call_application_Task(void) {
	/* This is user application task */
	
	/* lets try to move the processor to PAL */
	uint32_t value = __get_CONTROL();
	value &= ~0x01;
	__set_CONTROL(value);

	generate_interrupt();
}

void RTOS_Init(void) {
	/* does RTOS related low leve inits  */
	
	/* before calling application task, change the priviledge level from PAL to NPAL */
	uint32_t value = __get_CONTROL();
	value |= 0x01;
	__set_CONTROL(value);
	
	call_application_Task();
	
}

int main(void){
	RTOS_Init();
	while(1);
	return 0;
}
