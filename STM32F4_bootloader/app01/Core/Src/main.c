#include "main.h"
#include <string.h>
#include <stdarg.h>

#define GPIOB_BASE 0x40020400
#define GPIOD_BASE 0x40020C00
#define GPIO_IDR 0x10
#define GPIO_ODR 0x14
#define GPIO_MODER 0x00
#define GPIO_AFRL 0x20

#define RCC_BASE 0x40023800
#define RCC_AHB1ENR 0x30
#define RCC_APB2ENR 0x44

#define UART1_BASE 0x40011000
#define UART_DR 0x04
#define UART_SR 0x00
#define UART_BRR 0x08
#define UART_CR1 0x0C
#define UART_CR3 0x14

#define TIM1_BASE 0x40010000
#define TIM_PSC 0x28
#define TIM_ARR 0x2C
#define TIM_SR 0x10
#define TIM_DIER 0x0C
#define TIM_CR1 0x00

#define ADC1_BASE 0x40012000
#define ADC_SMPR1 0x0C
#define ADC_JSQR 0x38
#define ADC_JDR 0x3C
#define ADC_CCR 0x304
#define ADC_CR2 0x08
#define ADC_SR 0x00

void LED_Init();
void LED_Blink(uint8_t pin, uint8_t state);

void UART_Init();
char UART_Read();
void UART_Write(uint8_t data);
void UART_Write_String(char* format, ...);

void TIMER_Init();
void delay(int t);

void ADC_Init();
float ADC_Measure();
float temp_measure();

char cmd[100];
int idx, time_cnt = 0;
float temperature;

int main() {
	LED_Init();
	UART_Init();
	TIMER_Init();
	ADC_Init();
	while(1) {
		temperature = temp_measure();
		UART_Write_String("Current temperature: %.2f\n", temperature);
		delay(1000);
	}
	return 0;
}

void LED_Init() {
	uint32_t* AHB1ENR = (uint32_t*)(RCC_BASE + RCC_AHB1ENR);
	*AHB1ENR |= (0b01 << 3);
	uint32_t* GPIOD_MODER = (uint32_t*)(GPIOD_BASE + GPIO_MODER);
	*GPIOD_MODER &= ~(0xFF << 24);
	*GPIOD_MODER |= (0x55 << 24);
}

void LED_Blink(uint8_t pin, uint8_t state) {
	uint32_t* GPIOD_ODR = (uint32_t*)(GPIOD_BASE + GPIO_ODR);
	if (state) {
		*GPIOD_ODR |= (0b01 << pin);
	} else {
		*GPIOD_ODR &= ~(0b01 << pin);
	}
}

void UART_Init() {
	uint32_t* AHB1ENR = (uint32_t*)(RCC_BASE + RCC_AHB1ENR);
	*AHB1ENR |= (0b01 << 1);
	uint32_t* APB2ENR = (uint32_t*)(RCC_BASE + RCC_APB2ENR);
	*APB2ENR |= (0b01 << 4);
	uint32_t* GPIOB_MODER = (uint32_t*)(GPIOB_BASE + GPIO_MODER);
	*GPIOB_MODER &= ~(0b1111 << 12);
	*GPIOB_MODER |= (0b1010 << 12);
	uint32_t* GPIOB_AFRL = (uint32_t*)(GPIOB_BASE + GPIO_AFRL);
	*GPIOB_AFRL &= ~(0xFF << 24);
	*GPIOB_AFRL |= (0x77 << 24);
	uint32_t* UART1_BRR = (uint32_t*)(UART1_BASE + UART_BRR);
	*UART1_BRR = (104 << 4) | (3 << 0);
	uint32_t* UART1_CR1 = (uint32_t*)(UART1_BASE + UART_CR1);
	*UART1_CR1 |= (0b01 << 13) | (0b01 << 12) | (0b01 << 10) | (0b01 << 5) | (0b01 << 3) | (0b01 << 2);
	uint32_t* NVIC_ISER1 = (uint32_t*)0xE000E104;
	*NVIC_ISER1 |= (0b01 << 5);
}

char UART_Read() {
	uint32_t* UART1_DR = (uint32_t*)(UART1_BASE + UART_DR);
	uint32_t* UART1_SR = (uint32_t*)(UART1_BASE + UART_SR);
	while(((*UART1_SR >> 5) & 1) == 0);
	char data = *UART1_DR;
	return data;
}

void UART_Write(uint8_t data) {
	uint32_t* UART1_DR = (uint32_t*)(UART1_BASE + UART_DR);
	uint32_t* UART1_SR = (uint32_t*)(UART1_BASE + UART_SR);
	while(((*UART1_SR >> 7) & 1) == 0);
	*UART1_DR = data;
	while(((*UART1_SR >> 6) & 1) == 0);
}

void UART_Write_String(char* format, ...) {
	va_list arg;
	va_start(arg, format);
	char txbuff[128] = {0};
	vsprintf(txbuff, format, arg);
	va_end(arg);
	int l = strlen(txbuff);
	for (int i = 0; i < l; i++) {
		UART_Write(txbuff[i]);
	}
}

void USART1_IRQHandler() {
	cmd[idx++] = UART_Read();
	if (strstr(cmd, "\n")) {
		if (strstr(cmd, "led on")) {
			LED_Blink(12, 1);
			LED_Blink(13, 1);
			LED_Blink(14, 1);
			LED_Blink(15, 1);
			UART_Write_String("Turning LEDs on...\n");
		} else if (strstr(cmd, "led off")) {
			LED_Blink(12, 0);
			LED_Blink(13, 0);
			LED_Blink(14, 0);
			LED_Blink(15, 0);
			UART_Write_String("Turning LEDs off...\n");
		} else {
			UART_Write_String("Command undefined, please try again.\n");
		}
		memset(cmd, 0, sizeof(cmd));
		idx = 0;
	}
}

void TIMER_Init() {
	uint32_t* APB2ENR = (uint32_t*)(RCC_BASE + RCC_APB2ENR);
	*APB2ENR |= (0b01 << 0);
	uint16_t* TIM1_PSC = (uint16_t*)(TIM1_BASE + TIM_PSC);
	*TIM1_PSC = 16000 - 1;
	uint16_t* TIM1_ARR = (uint16_t*)(TIM1_BASE + TIM_ARR);
	*TIM1_ARR = 1;
	uint16_t* TIM1_DIER = (uint16_t*)(TIM1_BASE + TIM_DIER);
	*TIM1_DIER |= (0b01 << 0);
	uint32_t* NVIC_ISER0 = (uint32_t*)0xE000E100;
	*NVIC_ISER0 |= (0b01 << 25);
	uint16_t* TIM1_CR1 = (uint16_t*)(TIM1_BASE + TIM_CR1);
	*TIM1_CR1 |= (0b01 << 0);
}

void delay(int t) {
	time_cnt = 0;
	while(time_cnt < t);
}

void TIM1_UP_TIM10_IRQHandler() {
	time_cnt++;
	uint16_t* TIM1_SR = (uint16_t*)(TIM1_BASE + TIM_SR);
	*TIM1_SR &= ~(0b01 << 0);
}

void ADC_Init() {
	uint32_t* APB2ENR = (uint32_t*)(RCC_BASE + RCC_APB2ENR);
	*APB2ENR |= (0b01 << 8);
	uint32_t* ADC1_SMPR1 = (uint32_t*)(ADC1_BASE + ADC_SMPR1);
	*ADC1_SMPR1 &= ~(0b111 << 18);
	*ADC1_SMPR1 |= (0b111 << 18);
	uint32_t* ADC1_JSQR = (uint32_t*)(ADC1_BASE + ADC_JSQR);
	*ADC1_JSQR &= ~(0b11111 << 15) & ~(0b11 << 20);
	*ADC1_JSQR |= (16 << 15);
	uint32_t* ADC1_CCR = (uint32_t*)(ADC1_BASE + ADC_CCR);
	*ADC1_CCR |= (0b01 << 23);
	uint32_t* ADC1_CR2 = (uint32_t*)(ADC1_BASE + ADC_CR2);
	*ADC1_CR2 |= (0b01 << 0);
}

float ADC_Measure() {
	uint32_t* ADC1_CR2 = (uint32_t*)(ADC1_BASE + ADC_CR2);
	*ADC1_CR2 |= (0b01 << 22);
	uint32_t* ADC1_SR = (uint32_t*)(ADC1_BASE + ADC_SR);
	while(((*ADC1_SR >> 2) & 1) == 0);
	*ADC1_SR &= ~(0b01 << 2);
	uint32_t* ADC1_JDR = (uint32_t*)(ADC1_BASE + ADC_JDR);
	return 3.0 * (*ADC1_JDR)/4095.0;
}

float temp_measure() {
	float vsense = ADC_Measure();
	const float v25 = 0.76;
	const float avg_slope = 2.5/1000;
	return (vsense - v25)/avg_slope + 25;
}
