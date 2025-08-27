#include "main.h"

#define GPIOA_BASE 0x40020000
#define GPIO_IDR 0x10
#define GPIO_MODER 0x00

#define RCC_BASE 0x40023800
#define RCC_AHB1ENR 0x30
#define RCC_APB2ENR 0x44

void BTN_Init();
char BTN_Read();

int main() {
	BTN_Init();
	while(1) {
		uint32_t* ptr = 0;
		void (*func_ptr)();
		uint32_t* VTOR = (uint32_t*)0xE000ED08;
		if (BTN_Read()) {
			ptr = (uint32_t)0x8008004;
			*VTOR = 0x8008000;
		} else {
			ptr = (uint32_t)0x8010004;
			*VTOR = 0x8010000;
		}
		func_ptr = (void(*)())*ptr;
		func_ptr();
	}
	return 0;
}

void BTN_Init() {
	uint32_t* AHB1ENR = (uint32_t*)(RCC_BASE + RCC_AHB1ENR);
	*AHB1ENR |= (0b01 << 0);
	uint32_t* GPIOA_MODER = (uint32_t*)(GPIOA_BASE + GPIO_MODER);
	*GPIOA_MODER &= ~(0b11 << 0);
}

char BTN_Read() {
	uint32_t* GPIOA_IDR = (uint32_t*)(GPIOA_BASE + GPIO_IDR);
	return !((*GPIOA_IDR >> 0) & 1);
}
