#include <stdint.h>
#include "system_stm32f4xx.h"

extern int __main(void);

int main(void) {

	
	return 0;
}

void Reset_Handler(void) {
SystemInit();
__main();
}
