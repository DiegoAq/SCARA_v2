/**
 * @file gpio.c
 * @brief LED de estado de la Black Pill (CMSIS, sin HAL).
 */
#include "gpio.h"

#include "stm32f4xx.h"

/* LED integrado de la Black Pill: PC13, activo en bajo. */
#define LED_PUERTO	GPIOC
#define LED_PIN		13U

void GPIO_LedInit(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	(void)RCC->AHB1ENR;		/* el periferico tarda un ciclo en quedar habilitado */

	/* Apagado antes de pasar el pin a salida, para que no parpadee al arrancar. */
	LED_PUERTO->BSRR = 1UL << LED_PIN;
	LED_PUERTO->MODER = (LED_PUERTO->MODER & ~(3UL << (LED_PIN * 2U))) | (1UL << (LED_PIN * 2U));
}

void GPIO_LedEscribir(bool encendido)
{
	/* BSRR: escritura atomica, sin leer-modificar-escribir sobre ODR. */
	LED_PUERTO->BSRR = encendido ? (1UL << (LED_PIN + 16U)) : (1UL << LED_PIN);
}

void GPIO_LedConmutar(void)
{
	GPIO_LedEscribir((LED_PUERTO->ODR & (1UL << LED_PIN)) != 0U);
}
