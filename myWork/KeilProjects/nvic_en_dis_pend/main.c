/* Enable and Pend the USART3 interrupt */

#include <stdint.h>
#include "stm32f446xx.h"

/* USING DIRECT REG ACCESS - NON INTERVENDOR PORTABLE */
/*

#define USART3_IRQ_NUM 39

void USART3_IRQHandler(void) {
	uint32_t i = 0;
	i++;
}

int main(void) {
	// Enable the USART3 IRQ num 39
	NVIC_Type	*pNVIC = NVIC;
	pNVIC->ISER[1] |= (1 << 7);
	
	// Lets pend the interrupt
	pNVIC->ISPR[1] |= (1 << 7);
	
	while(1);
	
	return 0;
}
*/

/* USING CMSIS API - INTERVENDOR PORTABLE */

void USART3_IRQHandler(void) {
	uint32_t i = 0;
	i++;
}

int main(void) {
	// Enable the USART3 IRQ num 39
	NVIC_EnableIRQ(USART3_IRQn);
	
	// Lets pend the interrupt
	NVIC_SetPendingIRQ(USART3_IRQn);
	
	while(1);
	
	return 0;
}
