/**
 * @file gpio.h
 * @brief Entradas/salidas digitales simples. Por ahora: LED de estado (PC13).
 */
#ifndef GPIO_H
#define GPIO_H

#include <stdbool.h>

/**
 * @brief Configura PC13 como salida y deja el LED apagado.
 */
void GPIO_LedInit(void);

/**
 * @brief Enciende o apaga el LED de estado.
 * @param encendido true = LED encendido (el pin va a nivel bajo: el LED es activo en bajo).
 */
void GPIO_LedEscribir(bool encendido);

/**
 * @brief Invierte el estado del LED.
 */
void GPIO_LedConmutar(void);

#endif /* GPIO_H */
