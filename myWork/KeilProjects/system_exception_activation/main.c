#include "stm32f446xx.h"
#include <stdint.h>

void MemManage_Handler(void){
	
}

void BusFault_Handler(void){
	
}

void UsageFault_Handler(void){
	
}

void SVC_Handler(void){
	
}

void PendSV_Handler(void){
	
}

int divide_numbers(int x,int y) {
	return x/y;
}

int main(void) {
	void (*go_address) (void);
	SCB_Type *pSCB=SCB;

	go_address = 0x00000000;

	SCB->SHCSR = SCB->SHCSR | (1 << 16); //enable MemManage Exception
	SCB->SHCSR = SCB->SHCSR | (1 << 17); //enable BusFault Exception
	SCB->SHCSR = SCB->SHCSR | (1 << 18); //enable UsageFault Exception
	
	/* lets enable DIV_0_TRP bit in CCR */
	SCB->CCR = SCB->CCR | (1 << 4);
	/* lets enable UNALIGN_TRP bit in CCR */
	SCB->CCR = SCB->CCR | (1 << 3);
	//unaligned data access
	uint32_t *p = (uint32_t*) 0x20000001;
	uint32_t var = *p;
	var++;
	
	/* lets divide by zero to trigger UsageFault */
	divide_numbers(var,0);
	//__set_FAULTMASK(1);
	go_address();
	/* activate exceptions: MemManage, BusFault, UsageFault, SVC, PendSV */
	return 0;
}
