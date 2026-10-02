/**
 * @file systick.h
 * @brief Base de tiempo de 1 ms con el SysTick del Cortex-M4.
 */
#ifndef SYSTICK_H
#define SYSTICK_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Arranca el SysTick con interrupcion cada 1 ms.
 *
 * Usa SystemCoreClock: llamar despues de Clock_Init().
 * @return false si SystemCoreClock no permite un periodo de 1 ms.
 */
bool SysTick_Init(void);

/**
 * @brief Milisegundos desde SysTick_Init().
 *
 * Desborda cada ~49,7 dias: comparar siempre por diferencia,
 * `(uint32_t)(Millis() - t0) >= periodo`, nunca `Millis() >= t_fin`.
 */
uint32_t Millis(void);

#endif /* SYSTICK_H */
