/**
 * @file clock.h
 * @brief Arbol de relojes del STM32F411: SYSCLK a 96 MHz desde el PLL.
 */
#ifndef CLOCK_H
#define CLOCK_H

#include <stdbool.h>
#include <stdint.h>

#define CLOCK_SYSCLK_HZ		96000000UL	/**< SYSCLK = HCLK */
#define CLOCK_PCLK1_HZ		48000000UL	/**< APB1 (I2C, USART2) */
#define CLOCK_PCLK2_HZ		96000000UL	/**< APB2 (USART1) */
/** Reloj de TIM2..TIM5: el prescaler de APB1 distinto de 1 duplica la frecuencia. */
#define CLOCK_TIM_APB1_HZ	96000000UL

/**
 * @brief Configura SYSCLK = 96 MHz: HSE 25 MHz -> PLL (M=25, N=192, P=2).
 *
 * Flash con 3 wait states + prefetch + caches, regulador en escala 1, APB1 /2, APB2 /1.
 * Actualiza SystemCoreClock. Todas las esperas tienen timeout.
 *
 * Si el cristal no arranca, usa el HSI de 16 MHz como entrada del PLL: el micro queda
 * igual a 96 MHz (la consola y los timers siguen andando), pero con la precision del
 * oscilador interno (~1 %).
 *
 * @return true si quedo a 96 MHz desde el HSE; false si hubo que usar el HSI o si el
 *         PLL no engancho (en ese caso sigue a 16 MHz; consultar SystemCoreClock).
 */
bool Clock_Init(void);

#endif /* CLOCK_H */
