#include "stm32f446xx.h"

void WWDG_IRQHandler(void) {
	NVIC_SetPendingIRQ(USART3_IRQn);
	while(1);
}

void USART3_IRQHandler() {
	int i = 0;
	i++;
}

int main(void) {
	// lets enable  WDOG and USART3 IRQ
	NVIC_EnableIRQ(WWDG_IRQn);
	NVIC_EnableIRQ(USART3_IRQn);
	
	// priority change
	NVIC_SetPriority(WWDG_IRQn,5);
	NVIC_SetPriority(USART3_IRQn,0);
	
	// lets pend  WDOG and USART3 IRQ
	NVIC_SetPendingIRQ(WWDG_IRQn);
	//NVIC_SetPendingIRQ(USART3_IRQn);
	
	while(1);
	
	return 0;
}
