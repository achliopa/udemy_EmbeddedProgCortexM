#include <stdint.h>
#include "Board_LED.h"
#include "Board_Buttons.h"
#include "cmsis_armcc.h"
#include "stm32f446xx.h"

void delay(void) {
	uint32_t i = 0;
	for(i=0;i<100000;i++);
}

// we have to implement the watchdog interrupt handler
void WWDG_IRQHandler(void) {
	/* lets change the current SP to PSP in Handler Mode. it should NOT work */
	uint32_t value= __get_CONTROL();
	value |= (1 << 1);
	__set_CONTROL(value);
	/* */
	for (int i=0;i<50;i++);
}

void generate_interrupt(void) {
	// lets simulate the watchdog interrupt
	NVIC_EnableIRQ(WWDG_IRQn);
	NVIC_SetPendingIRQ(WWDG_IRQn);
}

int main(void) {
	/* lets move the top of the stack to end of SRAM1 */
	/* Lets program 0x2001BFFF+1 in to MSP */
	__set_MSP(0x2001BFFF+1);
	/* lets change the current SP to PSP */
	uint32_t value= __get_CONTROL();
	value |= (1 << 1);
	__set_CONTROL(value);
	/* lets initialize the PSP first before using it */
	__set_PSP(0x2001BFFF+1);
	generate_interrupt();
	LED_Initialize();
	Buttons_Initialize();
	while(1){
		if (Buttons_GetState() == 1){
					LED_On(0);
					delay();
					LED_Off(0);
					delay();
		}
	}
	return 0;
}
