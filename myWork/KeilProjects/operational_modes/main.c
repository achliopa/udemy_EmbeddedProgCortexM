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

void generate_NMI_interrupt(void) {
	/* This code WONT do the trick canno use it ofr exceptions IRQn <0 only for external interrupts */
	//NVIC_EnableIRQ(NonMaskableInt_IRQn);
	//NVIC_SetPendingIRQ(NonMaskableInt_IRQn);
	/* This code allow us to set NMI */
	SCB_Type *pSCB;
	pSCB = SCB;
	pSCB->ICSR |= SCB_ICSR_NMIPENDSET_Msk;
}

int main(void){
	/* PRIMASK example setting it to 1 (basically interrupts are disabled)*/
	//__set_PRIMASK(1);
	/* PRIMASK example setting it to 1 (exceptions and interrupts except from NMI are disabled)*/
	__set_FAULTMASK(1);
	//generate_interrupt();
	/* EPSR T-bit example*/
	//void (*jump_addr) (void) = &generate_interrupt;
	//void (*jump_addr) (void) = (void *) 0x08000374;
	//jump_addr();
	/* Generate NMI interrupt to test FAULTMASK limitations */
	generate_NMI_interrupt();
	while(1);
	return 0;
}
