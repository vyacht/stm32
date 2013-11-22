#include "stm32f10x.h"
#include "core_cm3.h"
#include "main.h"
#define BOOTLOADER 1

#ifdef BOOTLOADER
void bootload(void) {

	uint32_t medium_density= 0x1FFFF000;

	// plus an offset of 4, first word is stackpointer value, next is reset vector
	void (*SysMemBootJump)(void) = (void (*)(void)) (*((uint32_t *) 0x1FFFF004));

	__set_PRIMASK(1); // disable all interrupts

	// shutdown all tasks running
	RCC_DeInit();
	
	// reset systicktimer
	SysTick->CTRL = 0;
	SysTick->LOAD = 0;
	SysTick->VAL = 0;

	RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI); // select HSI

	__set_MSP(0x20001000); // set main stack pointer to default
	// __set_MSP(0x1FFFF000); // set main stack pointer to default

	SysMemBootJump();

	while(1) {}
}

#endif
