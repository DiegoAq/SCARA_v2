/**
 * @file systick.c
 * @brief Base de tiempo de 1 ms (CMSIS, sin HAL).
 */
#include "systick.h"

#include "stm32f4xx.h"

#define SYSTICK_HZ	1000U

/* Compartido entre la ISR y el loop. 32 bits: en el M4 se lee de forma atomica. */
static volatile uint32_t g_ms;

bool SysTick_Init(void)
{
	g_ms = 0U;
	/* SysTick_Config devuelve 0 si la recarga entra en los 24 bits del contador. */
	return SysTick_Config(SystemCoreClock / SYSTICK_HZ) == 0U;
}

uint32_t Millis(void)
{
	return g_ms;
}

/** ISR del SysTick (nombre fijado por la tabla de vectores del startup). */
void SysTick_Handler(void)
{
	g_ms++;
}
